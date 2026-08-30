"""Procedurally build a chibi robot 3D character in Blender (headless).

Design decisions
----------------
- Blender 5.1 has no bundled human generator addon, so the character is built
  from primitives via the bpy Python API.
- Restyled after assets/humanoid/reference_target.png: a chibi toy robot with
  an oversized helmet (metallic red + cream trim), a dark rounded visor with
  two emissive green triangle eyes, ear pods + antenna nubs, a small armored
  torso with an emissive chest emblem, segmented arms and chunky boots.
- Chibi proportions: ~1.2 m tall, helmet spans roughly the upper half.
- Primitive convention: spheres/cylinders/cones are created with radius 1 and
  depth 2, cubes with size 2, so object scale equals the part's half-extents
  (radii). Subdivision surfaces are APPLIED per part before joining, so the
  joined mesh keeps final geometry (a live subsurf on the joined mesh would
  shrink every disconnected island).
- Metallic / emissive Principled materials: robot red, maroon, cream trim,
  dark joints, near-black visor, emissive green eyes, emissive emblem, cyan
  accents. View transform 'Standard' for the saturated toy look.
 - The same FK armature layout (root -> spine -> chest -> neck -> head, plus
   L/R shoulder/upper_arm/forearm/hand/thigh/shin/foot) is kept so the twin
   control chain (humanoid_control.py / twin_server.py) keeps working.
 - Skinning (2026-08-30 action-evidence session): rigid per-part weights —
   every connected mesh island follows its nearest bone segment. Automatic
   (bone-heat) weights leaked torso verts onto the hanging arm bones, which
   stretched the body when the arms were raised (wave/tpose); the robot is a
   segmented mechanical figure, so rigid islands are both correct and clean.
   A Gaussian distance-falloff fallback still fills bones without an island.
 - Output paths can be redirected with ABOT_HUMANOID_OUT_DIR (used by the
   reproducibility acceptance test); default is assets/humanoid.

 Round-2 fixes after the visual analyst's gap list (2026-08-30):
 - Hands: rounded palm + three fingers + thumb per side (no polyhedral block).
 - Feet: the dark gray sole plates are gone; boots get a rounded gold sole,
   a cream collar and a cyan cable loop on the outer side (replaces the
   cyan spike accents).
 - Visor: curved rounded-rect screen built as a spherical shell (boolean
   intersect of a slightly larger sphere with a beveled prism) so it follows
   the helmet curvature, plus a gold/cream rim shell behind it, a procedural
   scanline (wave->ramp->bump+color) in the visor material, and emissive
   triangle eyes rebuilt as curved shell outlines hugging the visor (no
   floating in side view).
  - Hard-surface detail: helmet rivets, two horizontal panel-line seams, a
    cream waist belt (mechanical trim density closer to the reference).

Baked keyframe Actions (2026-08-30): after skinning, every key action is
baked into the .blend as a named Action (ActionRelax/ActionTPose/ActionAPose/
ActionIdle/ActionWave/ActionWalk/ActionNod/ActionLook/ActionRun) of pure
pose-bone rotation keyframes (walk/run additionally key the FK root-bob
location) — no vertex/shape animation. The motion parameters come straight
from humanoid_control.ACTION_SPECS / MOTION_DRIVERS so the baked clips and
the runtime FK chain share one contract. ActionIdle stays attached to the
armature, so opening the blend in the Blender GUI and pressing play shows
joint motion immediately; all nine Actions are selectable in the Action
Editor. The preview renders detach the Action first so they show the neutral
rest pose.

Run headless:
    blender -b -P scripts/blender_humanoid/build_humanoid.py
"""

import math
import os
import sys

import bpy
from mathutils import Vector

# humanoid_control lives next to this script (shared bake contract)
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import humanoid_control as hc  # noqa: E402

# ---------------------------------------------------------------------------
# Output paths (ABOT_HUMANOID_OUT_DIR overrides for reproducible test builds)
# ---------------------------------------------------------------------------
OUT_DIR = os.environ.get("ABOT_HUMANOID_OUT_DIR", r"D:\Projects\aBot\assets\humanoid")
BLEND_OUT = os.path.join(OUT_DIR, "humanoid.blend")
RENDER_FRONT = os.path.join(OUT_DIR, "preview_front.png")
RENDER_THREEQUARTER = os.path.join(OUT_DIR, "preview_3quarter.png")

# ---------------------------------------------------------------------------
# Scene helpers
# ---------------------------------------------------------------------------
def clear_scene():
    """Remove all objects, meshes and materials, keep only default collection."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    for mat in list(bpy.data.materials):
        bpy.data.materials.remove(mat)
    # remove leftover empty collections
    for col in list(bpy.data.collections):
        if col.users == 0:
            bpy.data.collections.remove(col)


def make_material(name, color, roughness=0.45, metallic=0.0, emission=None,
                  emission_strength=1.0):
    """Create a Principled BSDF material."""
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    if emission is not None:
        bsdf.inputs["Emission Color"].default_value = (*emission, 1.0)
        bsdf.inputs["Emission Strength"].default_value = emission_strength
    return mat


def make_primitive(kind, name, location, scale, rotation=(0.0, 0.0, 0.0),
                   segments=32, material=None, subdiv=2):
    """Create a primitive whose object scale equals its half-extents.

    Subdivision (if any) is applied immediately so joined meshes keep their
    final shape.
    """
    ops = {
        "sphere": lambda: bpy.ops.mesh.primitive_uv_sphere_add(
            radius=1.0, segments=segments, ring_count=max(segments // 2, 8),
            location=location, rotation=rotation),
        "cylinder": lambda: bpy.ops.mesh.primitive_cylinder_add(
            radius=1.0, depth=2.0, vertices=segments,
            location=location, rotation=rotation),
        "cube": lambda: bpy.ops.mesh.primitive_cube_add(
            size=2.0, location=location, rotation=rotation),
        "cone": lambda: bpy.ops.mesh.primitive_cone_add(
            radius1=1.0, radius2=0.7, depth=2.0, vertices=segments,
            location=location, rotation=rotation),
    }
    ops[kind]()
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = scale
    if material is not None:
        obj.data.materials.append(material)
    bpy.ops.object.shade_smooth()
    if subdiv and kind in ("sphere", "cylinder", "cone", "cube"):
        mod = obj.modifiers.new("Subdivision", "SUBSURF")
        mod.levels = subdiv
        mod.render_levels = subdiv
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
    return obj


def make_torus(name, major, minor, material):
    """Create a torus at the origin (ring in local XY plane), shade smooth."""
    bpy.ops.mesh.primitive_torus_add(major_radius=major, minor_radius=minor,
                                     major_segments=64, minor_segments=12,
                                     location=(0.0, 0.0, 0.0))
    obj = bpy.context.active_object
    obj.name = name
    if material is not None:
        obj.data.materials.append(material)
    bpy.ops.object.shade_smooth()
    return obj


def delete_verts_below(obj, axis="x", value=0.0):
    """Delete mesh vertices whose local coordinate on `axis` is < value."""
    import bmesh
    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.co[idx] < value],
                     context="VERTS")
    bm.to_mesh(me)
    bm.free()


def apply_modifier(obj, mod):
    """Apply one modifier on obj (headless-safe)."""
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)


def boolean_apply(obj, cutter, operation):
    """Apply a boolean `operation` with `cutter`, then delete the cutter."""
    mod = obj.modifiers.new("Bool", "BOOLEAN")
    mod.operation = operation
    mod.solver = "EXACT"
    mod.object = cutter
    apply_modifier(obj, mod)
    bpy.data.objects.remove(cutter, do_unlink=True)


def make_rounded_prism(name, half_x, half_z, half_y, bevel):
    """Rounded-rectangle prism, long axis Y (used as boolean cutter)."""
    bpy.ops.mesh.primitive_cube_add(size=2.0, location=(0.0, 0.0, 0.0))
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (half_x, half_y, half_z)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    b = obj.modifiers.new("Bevel", "BEVEL")
    b.width = bevel
    b.segments = 6
    apply_modifier(obj, b)
    return obj


def make_tri_ring_prism(name, side, thickness, depth):
    """Triangular outline prism (upright triangle), long axis Y.

    Built from two 3-vert cones (outer minus inner) so a boolean intersect
    with a sphere yields a curved triangle outline.
    """
    from mathutils import Matrix

    def cone(r):
        bpy.ops.mesh.primitive_cone_add(radius1=r, radius2=r, depth=depth,
                                        vertices=3, location=(0.0, 0.0, 0.0))
        return bpy.context.active_object

    outer = cone(side / math.sqrt(3.0))
    inner = cone(max((side - 2.2 * thickness) / math.sqrt(3.0), 0.01))
    # orient: apex of the triangle to +Y, then prism axis Z -> -Y so the
    # triangle stands upright in X-Z facing -Y
    v0 = outer.data.vertices[0].co
    theta0 = math.atan2(v0.y, v0.x)
    orient = Matrix.Rotation(math.radians(90), 4, "X") @ \
        Matrix.Rotation(math.radians(90) - theta0, 4, "Z")
    outer.matrix_world = orient
    inner.matrix_world = orient
    boolean_apply(outer, inner, "DIFFERENCE")
    outer.name = name
    return outer


def make_shell(name, radii, center, cutter, material, cutter_offset,
               segments=96):
    """Spherical shell patch = sphere(radii at `center`) INTERSECT cutter.

    The cutter is placed at `center` + `cutter_offset`.  Result hugs the
    sphere surface, i.e. follows the head curvature when radii ~ helmet
    radii + small offset.  The object keeps `center` as its location so the
    later join bakes it into the right world position.
    """
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0, segments=segments,
                                         ring_count=segments // 2,
                                         location=center)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = radii
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    cutter.location = (center[0] + cutter_offset[0],
                       center[1] + cutter_offset[1],
                       center[2] + cutter_offset[2])
    boolean_apply(obj, cutter, "INTERSECT")
    if material is not None:
        # boolean leaves a leftover empty slot; force a single clean slot so
        # every face actually uses the intended material
        obj.data.materials.clear()
        obj.data.materials.append(material)
        for poly in obj.data.polygons:
            poly.material_index = 0
    bpy.ops.object.shade_smooth()
    return obj


def add_scanlines(mat):
    """Procedural horizontal scanlines on a visor material (bump + tint)."""
    nt = mat.node_tree
    bsdf = nt.nodes.get("Principled BSDF")
    tex = nt.nodes.new("ShaderNodeTexCoord")
    mapping = nt.nodes.new("ShaderNodeMapping")
    mapping.inputs["Rotation"].default_value[1] = math.radians(90)
    wave = nt.nodes.new("ShaderNodeTexWave")
    wave.wave_type = "BANDS"
    wave.inputs["Scale"].default_value = 250.0
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.45
    ramp.color_ramp.elements[1].position = 0.55
    mix = nt.nodes.new("ShaderNodeMixRGB")
    mix.inputs["Color1"].default_value = (0.010, 0.010, 0.012, 1.0)
    mix.inputs["Color2"].default_value = (0.030, 0.030, 0.034, 1.0)
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.15
    lk = nt.links
    lk.new(tex.outputs["Object"], mapping.inputs["Vector"])
    lk.new(mapping.outputs["Vector"], wave.inputs["Vector"])
    lk.new(wave.outputs["Fac"], ramp.inputs["Fac"])
    lk.new(ramp.outputs["Color"], mix.inputs["Fac"])
    lk.new(mix.outputs["Color"], bsdf.inputs["Base Color"])
    lk.new(ramp.outputs["Color"], bump.inputs["Height"])
    lk.new(bump.outputs["Normal"], bsdf.inputs["Normal"])


def add_triangle_outline(parts, kind_name, center, side, thickness, material):
    """Add three thin cylinders forming an upright triangle outline.

    `center` is (x, y, z); the triangle lies in the X-Z plane facing -Y.
    """
    h = side * 0.5
    cz = center[2]
    top = (0.0, h)
    left = (-side * 0.5, -h * 0.72)
    right = (side * 0.5, -h * 0.72)
    edges = ((left, right), (left, top), (top, right))
    for i, (a, b) in enumerate(edges):
        mx = (a[0] + b[0]) / 2.0
        mz = (a[1] + b[1]) / 2.0
        dx = b[0] - a[0]
        dz = b[1] - a[1]
        length = math.sqrt(dx * dx + dz * dz) * 0.95
        ang = math.atan2(dx, dz)  # rotation about Y tilts the cylinder axis
        obj = make_primitive("cylinder", "%s_e%d" % (kind_name, i),
                             Vector((center[0] + mx, center[1], cz + mz)),
                             Vector((thickness, thickness, length / 2.0)),
                             rotation=(0.0, ang, 0.0), material=material,
                             subdiv=0)
        parts.append(obj)


# ---------------------------------------------------------------------------
# Materials (robot palette after reference_target.png)
# ---------------------------------------------------------------------------
ROBOT_RED = (0.42, 0.06, 0.05)
ROBOT_MAROON = (0.20, 0.03, 0.04)
CREAM = (0.80, 0.62, 0.30)
DARK_JOINT = (0.04, 0.04, 0.05)
VISOR = (0.01, 0.01, 0.012)
EYE_GREEN = (0.02, 0.05, 0.02)
EMBLEM = (0.05, 0.05, 0.02)
CYAN = (0.0, 0.55, 0.65)

# ---------------------------------------------------------------------------
# Build the chibi robot
# ---------------------------------------------------------------------------
def build_humanoid():
    clear_scene()

    mat_red = make_material("RobotRed", ROBOT_RED, roughness=0.30, metallic=0.85)
    mat_maroon = make_material("RobotMaroon", ROBOT_MAROON, roughness=0.40,
                               metallic=0.80)
    mat_cream = make_material("CreamTrim", CREAM, roughness=0.30, metallic=0.70)
    mat_dark = make_material("DarkJoint", DARK_JOINT, roughness=0.50, metallic=0.60)
    mat_visor = make_material("Visor", VISOR, roughness=0.15, metallic=0.10)
    add_scanlines(mat_visor)
    mat_eye = make_material("EyeGlow", EYE_GREEN, roughness=0.30,
                            emission=(0.0, 1.0, 0.05), emission_strength=4.0)
    mat_emblem = make_material("EmblemGlow", EMBLEM, roughness=0.30,
                               emission=(0.7, 1.0, 0.0), emission_strength=3.0)
    mat_cyan = make_material("CyanAccent", CYAN, roughness=0.30, metallic=0.50)

    parts = []

    def add(kind, name, loc, scale, material, **kw):
        obj = make_primitive(kind, name, Vector(loc), Vector(scale),
                             material=material, **kw)
        parts.append(obj)
        return obj

    # ---- Helmet (oversized head) ------------------------------------------
    add("sphere", "Helmet", (0.0, 0.0, 0.92), (0.30, 0.30, 0.28), mat_red, subdiv=2)
    # cream stripe band over the crown (front-to-back)
    stripe = make_torus("HelmetStripe", 0.27, 0.03, mat_cream)
    delete_verts_below(stripe, axis="x", value=0.0)
    stripe.rotation_euler = (0.0, math.radians(-90), 0.0)
    stripe.location = (0.0, 0.0, 0.92)
    parts.append(stripe)
    # curved rounded-rect visor screen hugging the helmet (spherical shell)
    HC = (0.0, 0.0, 0.92)
    visor_cut = make_rounded_prism("VisorCut", 0.19, 0.145, 0.5, 0.05)
    parts.append(make_shell("Visor", (0.312, 0.312, 0.292), HC,
                            visor_cut, mat_visor, (0.0, -0.5, 0.0)))
    # gold/cream rim shell behind the visor border
    rim_cut = make_rounded_prism("RimCut", 0.215, 0.165, 0.5, 0.05)
    parts.append(make_shell("VisorRim", (0.306, 0.306, 0.286), HC,
                            rim_cut, mat_cream, (0.0, -0.5, 0.0)))
    # emissive green triangle eyes as curved outlines hugging the visor
    for side in (-1, 1):
        eye_cut = make_tri_ring_prism("EyeCut%+d" % side, 0.11, 0.014, 0.6)
        parts.append(make_shell("Eye%+d" % side, (0.317, 0.317, 0.297), HC,
                                eye_cut, mat_eye,
                                (side * 0.105, -0.25, 0.01)))
    # ear pods + cream caps
    for side in (-1, 1):
        add("cylinder", "EarPod%+d" % side, (side * 0.30, 0.0, 0.92),
            (0.10, 0.10, 0.05), mat_dark, rotation=(0.0, math.radians(90), 0.0),
            subdiv=1)
        add("cylinder", "EarCap%+d" % side, (side * 0.335, 0.0, 0.92),
            (0.06, 0.06, 0.02), mat_cream, rotation=(0.0, math.radians(90), 0.0),
            subdiv=1)
        # antenna nubs on the upper sides
        add("cylinder", "Antenna%+d" % side, (side * 0.19, 0.0, 1.13),
            (0.03, 0.03, 0.06), mat_cream,
            rotation=(0.0, side * math.radians(25), 0.0), subdiv=1)

    # ---- Hard-surface detail: rivets + panel seams -------------------------
    def helmet_pt(dx, dy, dz, lift=0.0):
        d = Vector((dx, dy, dz)).normalized()
        return (d.x * (0.30 + lift), d.y * (0.30 + lift),
                0.92 + d.z * (0.28 + lift))

    for i, dirv in enumerate(((0.62, -0.72, 0.30), (-0.62, -0.72, 0.30),
                              (0.62, -0.72, -0.30), (-0.62, -0.72, -0.30))):
        add("sphere", "Rivet%d" % i, helmet_pt(*dirv, lift=-0.004),
            (0.011, 0.011, 0.011), mat_dark, subdiv=1)
    for i, dirv in enumerate(((0.30, -0.85, 0.45), (-0.30, -0.85, 0.45))):
        add("sphere", "Bolt%d" % i, helmet_pt(*dirv, lift=-0.004),
            (0.012, 0.012, 0.012), mat_cream, subdiv=1)
    for j, zz in enumerate((1.06, 0.78)):
        rr = 0.30 * math.sqrt(max(1.0 - ((zz - 0.92) / 0.28) ** 2, 0.05))
        seam = make_torus("Seam%d" % j, rr, 0.0035, mat_maroon)
        seam.location = (0.0, 0.0, zz)
        parts.append(seam)

    # ---- Torso -------------------------------------------------------------
    add("cylinder", "Neck", (0.0, 0.0, 0.62), (0.05, 0.05, 0.07), mat_dark, subdiv=1)
    add("sphere", "Chest", (0.0, 0.0, 0.50), (0.16, 0.12, 0.12), mat_red, subdiv=2)
    # emissive chest emblem (small upright triangle)
    add_triangle_outline(parts, "Emblem", (0.0, -0.115, 0.50), 0.06, 0.008,
                         mat_emblem)
    add("sphere", "Abdomen", (0.0, 0.0, 0.385), (0.11, 0.09, 0.08), mat_maroon,
        subdiv=2)
    belt = make_torus("Belt", 0.105, 0.008, mat_cream)
    belt.location = (0.0, 0.0, 0.385)
    parts.append(belt)
    add("sphere", "Pelvis", (0.0, 0.0, 0.285), (0.12, 0.10, 0.08), mat_maroon,
        subdiv=2)

    # ---- Arms (pauldron -> upper arm -> elbow -> forearm -> hand) ----------
    for side, sx in (("L", -1), ("R", 1)):
        x = sx * 0.185
        add("sphere", f"Pauldron{side}", (sx * 0.17, 0.0, 0.55),
            (0.08, 0.08, 0.08), mat_red, subdiv=2)
        add("cylinder", f"UpperArm{side}", (x, 0.0, 0.48),
            (0.045, 0.045, 0.09), mat_red, subdiv=2)
        add("sphere", f"Elbow{side}", (x, 0.0, 0.415),
            (0.042, 0.042, 0.042), mat_dark, subdiv=2)
        add("cylinder", f"Forearm{side}", (x, 0.0, 0.35),
            (0.040, 0.040, 0.08), mat_red, subdiv=2)
        add("cylinder", f"Cuff{side}", (x, 0.0, 0.30),
            (0.046, 0.046, 0.02), mat_cream, subdiv=1)
        # rounded palm + mechanical fingers (no polyhedral block)
        add("sphere", f"Palm{side}", (x, -0.005, 0.245),
            (0.030, 0.026, 0.034), mat_maroon, subdiv=1)
        for fi, dx in enumerate((-0.016, 0.0, 0.016)):
            add("cylinder", f"Finger{side}{fi}", (x + dx, -0.012, 0.205),
                (0.008, 0.008, 0.022), mat_red,
                rotation=(math.radians(12), 0.0, 0.0), subdiv=1)
        add("cylinder", f"Thumb{side}", (x - sx * 0.030, -0.012, 0.235),
            (0.008, 0.008, 0.018), mat_red,
            rotation=(math.radians(15), 0.0, sx * math.radians(35)), subdiv=1)

    # ---- Legs (thigh -> knee -> shin -> boot) -------------------------------
    for side, sx in (("L", -1), ("R", 1)):
        x = sx * 0.095
        add("cylinder", f"Thigh{side}", (x, 0.0, 0.215),
            (0.062, 0.062, 0.08), mat_red, subdiv=2)
        add("sphere", f"Knee{side}", (x, 0.0, 0.15),
            (0.05, 0.05, 0.05), mat_dark, subdiv=2)
        add("cylinder", f"Shin{side}", (x, 0.0, 0.10),
            (0.052, 0.052, 0.07), mat_red, subdiv=2)
        add("cube", f"Boot{side}", (x, -0.02, 0.09),
            (0.09, 0.12, 0.075), mat_red, subdiv=2)
        add("sphere", f"ToeCap{side}", (x, -0.13, 0.045),
            (0.06, 0.05, 0.045), mat_cream, subdiv=2)
        # rounded gold sole puck (replaces the gray flat plate)
        add("sphere", f"Sole{side}", (x, -0.045, 0.026),
            (0.078, 0.125, 0.028), mat_cream, subdiv=1)
        # cream boot collar + cyan cable loop on the outer side
        collar = make_torus(f"BootCollar{side}", 0.056, 0.010, mat_cream)
        collar.location = (x, 0.0, 0.155)
        parts.append(collar)
        cable = make_torus(f"BootCable{side}", 0.035, 0.006, mat_cyan)
        cable.rotation_euler = (0.0, math.radians(90), 0.0)
        cable.location = (sx * 0.16, -0.02, 0.09)
        parts.append(cable)

    # ---- Join all parts into a single mesh ---------------------------------
    bpy.ops.object.select_all(action="DESELECT")
    for obj in parts:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = parts[0]
    bpy.ops.object.join()
    body = bpy.context.active_object
    body.name = "Humanoid_Body"
    # Bake the inherited object transform so the mesh lives in world space
    # with unit scale (a non-uniform object scale would distort armature
    # deformation when the twin is posed).
    bpy.ops.object.select_all(action="DESELECT")
    body.select_set(True)
    bpy.context.view_layer.objects.active = body
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return body


# ---------------------------------------------------------------------------
# Armature + auto-weight parenting (same bone names as twin control chain)
# ---------------------------------------------------------------------------
def add_armature(body):
    arm_data = bpy.data.armatures.new("HumanoidRig")
    arm = bpy.data.objects.new("HumanoidRig", arm_data)
    bpy.context.collection.objects.link(arm)

    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    eb = arm_data.edit_bones

    root = eb.new("root")
    root.head = (0.0, 0.0, 0.02)
    root.tail = (0.0, 0.0, 0.06)

    spine = eb.new("spine")
    spine.head = (0.0, 0.0, 0.26)
    spine.tail = (0.0, 0.0, 0.42)
    spine.parent = root

    chest = eb.new("chest")
    chest.head = (0.0, 0.0, 0.42)
    chest.tail = (0.0, 0.0, 0.60)
    chest.parent = spine

    neck = eb.new("neck")
    neck.head = (0.0, 0.0, 0.60)
    neck.tail = (0.0, 0.0, 0.66)
    neck.parent = chest

    head = eb.new("head")
    head.head = (0.0, 0.0, 0.66)
    head.tail = (0.0, 0.0, 1.05)
    head.parent = neck

    for side, sx in (("L", -1), ("R", 1)):
        shoulder = eb.new(f"shoulder.{side}")
        shoulder.head = (sx * 0.04, 0.0, 0.55)
        shoulder.tail = (sx * 0.16, 0.0, 0.55)
        shoulder.parent = chest

        uarm = eb.new(f"upper_arm.{side}")
        uarm.head = (sx * 0.185, 0.0, 0.55)
        uarm.tail = (sx * 0.185, 0.0, 0.415)
        uarm.parent = shoulder

        farm = eb.new(f"forearm.{side}")
        farm.head = (sx * 0.185, 0.0, 0.415)
        farm.tail = (sx * 0.185, 0.0, 0.30)
        farm.parent = uarm

        hand = eb.new(f"hand.{side}")
        hand.head = (sx * 0.185, 0.0, 0.30)
        hand.tail = (sx * 0.185, 0.0, 0.23)
        hand.parent = farm

        thigh = eb.new(f"thigh.{side}")
        thigh.head = (sx * 0.095, 0.0, 0.28)
        thigh.tail = (sx * 0.095, 0.0, 0.15)
        thigh.parent = root

        shin = eb.new(f"shin.{side}")
        shin.head = (sx * 0.095, 0.0, 0.15)
        shin.tail = (sx * 0.095, 0.0, 0.05)
        shin.parent = thigh

        foot = eb.new(f"foot.{side}")
        foot.head = (sx * 0.095, 0.0, 0.05)
        foot.tail = (sx * 0.095, -0.10, 0.02)
        foot.parent = shin

    bpy.ops.object.mode_set(mode="OBJECT")

    # Rigid per-part skinning: each connected island follows its nearest bone
    # (no auto-weight leakage), then a Gaussian fallback fills empty groups.
    assign_rigid_weights(arm, body)
    fix_weights(arm, body)
    mod = body.modifiers.new("Armature", "ARMATURE")
    mod.object = arm
    body.parent = arm
    return arm


def assign_rigid_weights(arm_obj, body):
    """Rigid skinning: bind every connected mesh island to its nearest bone.

    The chibi robot is built from many disjoint primitive shells (helmet,
    visor, arm segments, boot parts, ...).  Binding each island rigidly to
    the bone segment closest to its centroid gives clean segmented-robot
    deformation and cannot leak torso verts onto arm bones the way bone-heat
    automatic weights did (which stretched the body in raised-arm poses).
    """
    import bmesh

    mesh = body.data
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.verts.ensure_lookup_table()
    seen = set()
    islands = []
    for v in bm.verts:
        if v.index in seen:
            continue
        comp = []
        stack = [v]
        seen.add(v.index)
        while stack:
            cur = stack.pop()
            comp.append(cur.index)
            for e in cur.link_edges:
                o = e.other_vert(cur)
                if o.index not in seen:
                    seen.add(o.index)
                    stack.append(o)
        islands.append(comp)
    bm.free()

    bones = [(b.name, Vector(b.head_local), Vector(b.tail_local))
             for b in arm_obj.data.bones]

    def seg_dist(p, p0, p1):
        seg = p1 - p0
        l2 = seg.length_squared
        t = 0.0 if l2 < 1e-9 else max(0.0, min(1.0, (p - p0).dot(seg) / l2))
        return (p - (p0 + seg * t)).length

    body.vertex_groups.clear()
    groups = {name: body.vertex_groups.new(name=name) for name, _, _ in bones}
    for comp in islands:
        c = Vector((0.0, 0.0, 0.0))
        for vi in comp:
            c += mesh.vertices[vi].co
        c /= len(comp)
        best = min(bones, key=lambda b: seg_dist(c, b[1], b[2]))
        groups[best[0]].add(comp, 1.0, "REPLACE")
    return len(islands)


def fix_weights(arm_obj, body):
    """Distance-falloff weight fallback for bones without any weights.

    Auto-weights (bone heat) can fail to find a solution for some bones when
    the mesh is built from many overlapping primitives. For any bone whose
    vertex group is missing or has zero weight, build a group using a Gaussian
    falloff from the bone segment.
    """
    import math
    from mathutils import Vector

    mesh = body.data
    vgs = body.vertex_groups
    sigma = 0.045

    for bone in arm_obj.data.bones:
        name = bone.name
        g = vgs.get(name)
        existing_ok = False
        if g is not None:
            for v in mesh.vertices:
                try:
                    if g.weight(v.index) > 0.001:
                        existing_ok = True
                        break
                except RuntimeError:
                    continue  # vertex not in this group; keep scanning
        if existing_ok:
            continue

        if g is not None:
            vgs.remove(g)
        g = vgs.new(name=name)

        p0 = Vector(bone.head_local)
        p1 = Vector(bone.tail_local)
        seg = p1 - p0
        seg_len2 = seg.length_squared

        for v in mesh.vertices:
            d = v.co - p0
            t = 0.0
            if seg_len2 > 1e-9:
                t = max(0.0, min(1.0, d.dot(seg) / seg_len2))
            closest = p0 + seg * t
            dist = (v.co - closest).length
            w = math.exp(-(dist * dist) / (2.0 * sigma * sigma))
            if w > 0.001:
                g.add([v.index], w, "REPLACE")


# ---------------------------------------------------------------------------
# Baked keyframe Actions (GUI-playable, joint-rotation-only)
# ---------------------------------------------------------------------------
def _action_fcurves(act):
    """All F-curves of an Action, compatible with Blender 4.4+ slotted
    Actions (layers/strips/channelbags) and legacy flat actions."""
    if hasattr(act, "fcurves"):
        try:
            return list(act.fcurves)
        except (AttributeError, RuntimeError):
            pass
    curves = []
    for layer in getattr(act, "layers", ()):
        for strip in layer.strips:
            for slot in getattr(act, "slots", ()):
                bag = strip.channelbag(slot)
                if bag is not None:
                    curves.extend(bag.fcurves)
    return curves


def _driven_channels(arm, fn, dur):
    """Names of pose bones whose rotation/location the motion ever drives.

    The driver is sampled across its whole duration; only bones that actually
    move get keyframes, so static actions stay tiny and un-driven bones keep
    following the rest pose.
    """
    rot_bones, loc_bones = set(), set()
    samples = [0.0] if dur is None else [dur * i / 8.0 for i in range(9)]
    for t in samples:
        fn(arm, t)  # every driver resets the pose itself
        for pb in arm.pose.bones:
            if any(abs(v) > 1e-6 for v in pb.rotation_euler):
                rot_bones.add(pb.name)
            if any(abs(v) > 1e-6 for v in pb.location):
                loc_bones.add(pb.name)
    hc.reset_pose(arm)
    return sorted(rot_bones), sorted(loc_bones)


def bake_actions(arm):
    """Bake every key action into a named Action of bone-rotation keyframes.

    Pure joint animation: every F-curve targets a pose-bone rotation_euler
    (walk/run additionally key the FK root-bob location); no vertex/shape-key
    animation is introduced anywhere. Keyframes are sampled straight from
    humanoid_control's drivers at FPS resolution over one (or more) full
    motion cycles, so looped playback is seamless. The armature is left with
    ActionIdle attached so pressing play in the GUI moves the robot right
    away; all nine Actions stay selectable in the Action Editor.
    """
    scene = bpy.context.scene
    scene.render.fps = hc.FPS
    ad = arm.animation_data_create()
    baked = {}
    for name, action_name, dur in hc.ACTION_SPECS:
        fn = hc.MOTION_DRIVERS[name]
        rot_bones, loc_bones = _driven_channels(arm, fn, dur)
        act = bpy.data.actions.new(action_name)
        act.use_fake_user = True  # keep the clip alive in the .blend
        ad.action = act           # keyframe_insert writes into this action
        n_frames = 60 if dur is None else int(round(dur * hc.FPS)) + 1
        key_frames = (1, n_frames) if dur is None else range(1, n_frames + 1)
        for f in key_frames:
            t = 0.0 if dur is None else (f - 1) / hc.FPS
            fn(arm, t)
            for bn in rot_bones:
                arm.pose.bones[bn].keyframe_insert("rotation_euler", frame=f)
            for bn in loc_bones:
                arm.pose.bones[bn].keyframe_insert("location", frame=f)
        # joint-only contract, enforced at bake time
        curves = _action_fcurves(act)
        if not curves:
            raise RuntimeError("baked %s has no F-curves" % action_name)
        for fc in curves:
            if not (fc.data_path.startswith("pose.bones[")
                    and (fc.data_path.endswith("rotation_euler")
                         or fc.data_path.endswith("location"))):
                raise RuntimeError("non-joint F-curve baked: %s" % fc.data_path)
        baked[name] = (act, n_frames)
        print("BAKED %s -> %s: %d frames, rot=%s loc=%s"
              % (name, action_name, n_frames, rot_bones, loc_bones))
    default_act, default_end = baked["idle"]
    ad.action = default_act
    scene.frame_start = 1
    scene.frame_end = default_end
    return baked


# ---------------------------------------------------------------------------
# Camera / lights / render
# ---------------------------------------------------------------------------
def setup_scene_and_render(arm):
    scene = bpy.context.scene
    # Previews show the neutral rest pose: detach the baked Action, otherwise
    # the depsgraph would evaluate it over the pose at render time.
    ad = arm.animation_data if arm is not None else None
    stashed_action = ad.action if ad is not None else None
    if ad is not None:
        ad.action = None
    # Engine: EEVEE (Blender 5.x)
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 1280
    # Saturated toy look (AgX would wash out the neon eyes)
    scene.view_settings.view_transform = "Standard"

    # Warm orange studio backdrop like the reference image
    world = bpy.data.worlds.get("World") or bpy.data.worlds.new("World")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = (0.85, 0.35, 0.05, 1.0)
    bg.inputs["Strength"].default_value = 1.0

    # Ground / subtle backdrop
    ground = make_primitive("cube", "Ground", (0.0, 0.0, -0.01),
                            (3.0, 3.0, 0.02), material=None, subdiv=0)
    gmat = make_material("Ground", (0.45, 0.16, 0.02), roughness=0.8)
    ground.data.materials.append(gmat)

    # Lights
    key = bpy.data.objects.new("KeyLight", bpy.data.lights.new("KeyLight", "AREA"))
    key.location = (1.8, -1.6, 2.4)
    key.rotation_euler = (math.radians(45), 0, math.radians(40))
    key.data.energy = 200
    scene.collection.objects.link(key)

    fill = bpy.data.objects.new("FillLight", bpy.data.lights.new("FillLight", "AREA"))
    fill.location = (-1.6, -1.2, 1.2)
    fill.rotation_euler = (math.radians(20), 0, math.radians(-60))
    fill.data.energy = 100
    scene.collection.objects.link(fill)

    rim = bpy.data.objects.new("RimLight", bpy.data.lights.new("RimLight", "AREA"))
    rim.location = (0.0, 2.2, 1.6)
    rim.rotation_euler = (math.radians(30), 0, math.radians(180))
    rim.data.energy = 70
    scene.collection.objects.link(rim)

    cam = bpy.data.objects.new("Camera", bpy.data.cameras.new("Camera"))
    scene.collection.objects.link(cam)
    scene.camera = cam

    # Track-to target
    empty = bpy.data.objects.new("Target", None)
    empty.location = (0.0, 0.0, 0.60)
    scene.collection.objects.link(empty)
    track = cam.constraints.new("TRACK_TO")
    track.target = empty
    track.track_axis = "TRACK_NEGATIVE_Z"
    track.up_axis = "UP_Y"

    def render_angle(name, cam_loc, out_path):
        cam.location = cam_loc
        scene.render.filepath = out_path
        bpy.ops.render.render(write_still=True)
        print(f"RENDERED {name}: {out_path}")

    render_angle("front", (0.0, -2.3, 0.62), RENDER_FRONT)
    render_angle("3quarter", (1.65, -1.65, 0.95), RENDER_THREEQUARTER)

    # closeups for the visual analyst (visor + hand)
    empty.location = (0.0, -0.28, 0.92)
    render_angle("closeup_visor", (0.45, -0.85, 1.05),
                 os.path.join(OUT_DIR, "preview_closeup_visor.png"))
    empty.location = (0.185, -0.01, 0.24)
    render_angle("closeup_hand", (0.55, -0.55, 0.35),
                 os.path.join(OUT_DIR, "preview_closeup_hand.png"))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("== building humanoid ==")
    body = build_humanoid()
    print("== adding armature ==")
    arm = add_armature(body)
    print("== baking key actions ==")
    bake_actions(arm)
    print("== scene + render ==")
    setup_scene_and_render(arm)
    # Re-attach the default baked Action so opening the saved blend in the
    # GUI and pressing play shows joint motion immediately (setup_scene_and_
    # render detached it only so the previews show the neutral rest pose).
    if arm.animation_data is not None:
        arm.animation_data.action = bpy.data.actions.get("ActionIdle")
    bpy.ops.wm.save_as_mainfile(filepath=BLEND_OUT)
    print(f"SAVED: {BLEND_OUT}")


if __name__ == "__main__":
    main()
