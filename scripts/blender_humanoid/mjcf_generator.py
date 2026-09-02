"""mjcf_generator.py -- the physics outlet of the single source of truth.

Generates assets/humanoid/humanoid.mjcf from humanoid_spec (the SAME
procedural parameters that build_humanoid.py uses for the .blend: bone tree,
probed bone axes, part anchors, explicit chibi masses). The MJCF is never
reverse-engineered from the .blend; both outlets are siblings under
humanoid_spec, per tech-insight-report-p2-physics-001 ("单一事实源双出口").

MJCF content:
  * one free-floating body per FK-contract bone (19 bodies, same names),
    each with three position-controlled hinge joints (.z/.y/.x) whose
    composition reproduces Blender XYZ rotation_euler exactly
    (R = Rz @ Ry @ Rx, verified against posed-Blender probes);
  * inertials via explicit per-body masses (BODY_MASS + boot ballast) and
    geom-derived inertia (density scaled so each body integrates to its
    budgeted mass);
  * collision PRIMITIVES only (head sphere, torso/limb capsules, foot boxes)
    -- never the render meshes;
  * tuned joint damping / frictionloss / servo kp / torque limits
    (JOINT_TUNING) so contract poses hold without oscillation.

Usage:
    python scripts/blender_humanoid/mjcf_generator.py [out_path]
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import humanoid_spec as hs  # noqa: E402

MJCF_DEFAULT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(_HERE)), "assets", "humanoid",
    "humanoid.mjcf")

# The free-floating base spawns a few mm above its nominal rest height so the
# feet settle onto the floor under gravity instead of starting in exact
# coplanar contact (which makes the contact solver explode at t=0). After the
# ~5 mm drop-settle the robot stands exactly in the spec rest pose.
SPAWN_LIFT = 0.005

ANGLE_FMT = "%.6f"


def fmt(v):
    return ANGLE_FMT % v


def vfmt(v):
    return " ".join(fmt(c) for c in v)


def body_relative_transform(parent_name, child_name):
    """Child body pos/quat relative to the parent body frame (rest pose)."""
    p_frame, p_head = hs.BONES[parent_name]["frame"], hs.BONES[parent_name]["head"]
    c_frame, c_head = hs.BONES[child_name]["frame"], hs.BONES[child_name]["head"]
    pt = hs.mat_transpose(p_frame)
    pos = hs.mat_apply(pt, hs.vsub(c_head, p_head))
    quat = hs.quat_from_mat(hs.mat_mul(pt, c_frame))
    return pos, quat


def quat_align_z(direction):
    """Quaternion rotating local +Z onto `direction` (unit vector)."""
    dx, dy, dz = direction
    if dz > 1.0 - 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    if dz < -1.0 + 1e-12:
        return (0.0, 1.0, 0.0, 0.0)  # 180 deg about X
    axis = (-dy, dx, 0.0)  # cross((0,0,1), dir)
    norm = math.sqrt(axis[0] ** 2 + axis[1] ** 2)
    axis = (axis[0] / norm, axis[1] / norm, 0.0)
    ang = math.acos(max(-1.0, min(1.0, dz)))
    s = math.sin(ang / 2.0)
    return (math.cos(ang / 2.0), axis[0] * s, axis[1] * s, 0.0)


def geom_to_local(bone_name, geom):
    """Map a world-space collider into the bone-local body frame."""
    frame, head = hs.BONES[bone_name]["frame"], hs.BONES[bone_name]["head"]
    ft = hs.mat_transpose(frame)
    out = dict(geom)
    if geom["type"] == "sphere":
        out["pos"] = hs.mat_apply(ft, hs.vsub(geom["center"], head))
    elif geom["type"] == "capsule":
        # MuJoCo capsules are pos + quat + size "radius halflength" with the
        # capsule axis along the geom's local Z
        f, t = geom["from"], geom["to"]
        axis = hs.vsub(t, f)
        length = math.sqrt(sum(v * v for v in axis))
        mid = tuple((a + b) / 2.0 for a, b in zip(f, t))
        dir_body = hs.mat_apply(ft, tuple(v / length for v in axis))
        out["pos"] = hs.mat_apply(ft, hs.vsub(mid, head))
        out["quat"] = quat_align_z(dir_body)
        out["halflength"] = length / 2.0
    elif geom["type"] == "box":
        out["pos"] = hs.mat_apply(ft, hs.vsub(geom["center"], head))
        # the box is world-aligned; the body frame is generally tilted, so
        # the geom carries the inverse body orientation
        out["quat"] = hs.quat_from_mat(ft)
    return out


def body_density(bone_name):
    """Density that makes the body's colliders integrate to BODY_MASS.

    Ballast geoms are excluded here; they carry BALLAST_MASS explicitly.
    """
    geoms = [g for g in hs.COLLIDERS[bone_name] if not g.get("ballast")]
    vol = sum(hs.geom_volume(g) for g in geoms)
    return hs.BODY_MASS[bone_name] / vol


def geom_xml(bone_name, idx, geom, density):
    """One <geom .../> line in bone-local coordinates."""
    g = geom_to_local(bone_name, geom)
    name = "%s_c%d" % (bone_name, idx)
    rgba = "0.85 0.25 0.20 1" if not geom.get("ballast") else "0.10 0.35 0.45 1"
    if geom.get("ballast"):
        # the counterweight carries BALLAST_MASS exactly, not the body density
        density = hs.BALLAST_MASS / hs.geom_volume(geom)
    common = 'name="%s" density="%s" rgba="%s"' % (name, fmt(density), rgba)
    if geom.get("ballast"):
        # counterweight: contributes mass/inertia, never collides
        common += ' contype="0" conaffinity="0" group="3"'
    if g["type"] == "sphere":
        return '<geom %s type="sphere" pos="%s" size="%s"/>' % (
            common, vfmt(g["pos"]), fmt(g["r"]))
    if g["type"] == "capsule":
        return ('<geom %s type="capsule" pos="%s" quat="%s" size="%s %s"/>'
                % (common, vfmt(g["pos"]), vfmt(g["quat"]), fmt(g["r"]),
                   fmt(g["halflength"])))
    if g["type"] == "box":
        return '<geom %s type="box" pos="%s" quat="%s" size="%s"/>' % (
            common, vfmt(g["pos"]), vfmt(g["quat"]), vfmt(g["half"]))
    raise ValueError("unknown collider type: %r" % g["type"])


# JOINT_TUNING["range"] is ordered (X, Y, Z) like rotation_euler; joints are
# emitted in the order z, y, x, so map axis name -> range index explicitly.
_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def joints_xml(bone_name, indent):
    """Three hinges reproducing Blender XYZ euler: order z, y, x (outermost
    first) with unit axes, because each body frame IS its bone frame."""
    tuning = hs.JOINT_TUNING[bone_name]
    if tuning is None:
        return ['%s<freejoint name="%s.free"/>' % (indent, bone_name)]
    lines = []
    for axis_name, axis in hs.JOINT_AXIS_ORDER:
        rng = tuning["range"][_AXIS_INDEX[axis_name]]
        lines.append(
            '%s<joint name="%s.%s" type="hinge" axis="%s" pos="0 0 0"\n'
            '%s       range="%s %s" damping="%s" frictionloss="%s"\n'
            '%s       armature="0.02"/>'
            % (indent, bone_name, axis_name, vfmt(axis), indent,
               fmt(rng[0]), fmt(rng[1]), fmt(tuning["damping"]),
               fmt(tuning["frictionloss"]), indent))
    return lines


def root_spawn_pos():
    """Free-base spawn position = spec root head + drop-settle clearance."""
    head = hs.BONES["root"]["head"]
    return (head[0], head[1], head[2] + SPAWN_LIFT)


def body_xml(bone_name, indent):
    """Recursively emit one body (joints + colliders + children)."""
    bone = hs.BONES[bone_name]
    parent = bone["parent"]
    if parent is None:
        pos, quat = root_spawn_pos(), hs.quat_from_mat(bone["frame"])
    else:
        pos, quat = body_relative_transform(parent, bone_name)
    lines = ['%s<body name="%s" pos="%s" quat="%s">'
             % (indent, bone_name, vfmt(pos), vfmt(quat))]
    inner = indent + "  "
    lines.extend(joints_xml(bone_name, inner))
    density = body_density(bone_name)
    for idx, geom in enumerate(hs.COLLIDERS[bone_name]):
        lines.append("%s%s" % (inner, geom_xml(bone_name, idx, geom, density)))
    for child in hs.BONE_ORDER:
        if hs.BONES[child]["parent"] == bone_name:
            lines.extend(body_xml(child, inner))
    lines.append("%s</body>" % indent)
    return lines


def actuators_xml():
    """Position servos (one per hinge) = the robot's 'muscles'; the
    physics_adapter drives them with the twin-control contract poses."""
    lines = ["  <actuator>"]
    for bone_name in hs.BONE_ORDER:
        tuning = hs.JOINT_TUNING[bone_name]
        if tuning is None:
            continue
        for axis_name, _axis in hs.JOINT_AXIS_ORDER:
            rng = tuning["range"][_AXIS_INDEX[axis_name]]
            lines.append(
                '    <position name="servo_%s.%s" joint="%s.%s" kp="%s"\n'
                '              ctrlrange="%s %s" forcelimited="true"\n'
                '              forcerange="-%s %s"/>'
                % (bone_name, axis_name, bone_name, axis_name,
                   fmt(tuning["kp"]), fmt(rng[0]), fmt(rng[1]),
                   fmt(tuning["tau"]), fmt(tuning["tau"])))
    lines.append("  </actuator>")
    return lines


def contact_xml():
    """Exclude body pairs whose simplified colliders overlap by construction
    (see humanoid_spec.CONTACT_EXCLUDES), so rest pose is penetration-free."""
    if not hs.CONTACT_EXCLUDES:
        return []
    lines = ["  <contact>"]
    for a, b in hs.CONTACT_EXCLUDES:
        lines.append('    <exclude body1="%s" body2="%s"/>' % (a, b))
    lines.append("  </contact>")
    return lines


def keyframe_xml():
    """qpos key 'home': upright rest pose (floating base at rest transform,
    all hinges at zero)."""
    root = hs.BONES["root"]
    qpos = list(root_spawn_pos()) + list(hs.quat_from_mat(root["frame"]))
    n_hinges = 3 * (len(hs.BONE_ORDER) - 1)
    qpos.extend([0.0] * n_hinges)
    return ("  <keyframe>\n"
            '    <key name="home" qpos="%s"/>\n'
            "  </keyframe>") % vfmt(qpos)


def generate_mjcf():
    """Render the whole MJCF document as a deterministic string."""
    lines = [
        '<mujoco model="aBot humanoid (chibi, 1.22 m, single-source MJCF)">',
        "  <!-- Generated by scripts/blender_humanoid/mjcf_generator.py from",
        "       humanoid_spec.py (single procedural source of truth shared with",
        "       build_humanoid.py). Do NOT hand-edit: regenerate instead. -->",
        '  <compiler angle="radian" autolimits="true"/>',
        '  <option timestep="0.002" integrator="implicitfast"/>',
        "  <default>",
        '    <geom condim="3" friction="1.0 0.005 0.0001" margin="0.002"/>',
        "  </default>",
        "  <worldbody>",
        '    <geom name="floor" type="plane" size="3 3 0.05" rgba="0.45 0.16 0.02 1"',
        '          friction="1.2 0.005 0.0001"/>',
    ]
    lines.extend(body_xml("root", "    "))
    lines.append("  </worldbody>")
    lines.extend(contact_xml())
    lines.extend(actuators_xml())
    lines.append(keyframe_xml())
    lines.append("</mujoco>")
    return "\n".join(lines) + "\n"


def write_mjcf(path=None):
    path = path or MJCF_DEFAULT_PATH
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    xml = generate_mjcf()
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(xml)
    return path


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else None
    written = write_mjcf(out)
    # sanity: axis assertions of the source spec must hold before export
    bad = [c for c in hs.assert_axis_conventions() if not c[1]]
    if bad:
        for name, _ok, detail in bad:
            print("FAIL %s | %s" % (name, detail))
        raise SystemExit("axis assertions failed; MJCF not exported")
    com, mass = hs.body_center_of_mass()
    print("MJCF_WRITTEN %s (total_mass=%.3f kg, COM=(%.4f, %.4f, %.4f))"
          % (written, mass, com[0], com[1], com[2]))
