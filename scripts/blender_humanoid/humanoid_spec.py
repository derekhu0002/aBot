"""humanoid_spec.py -- the single procedural source of truth for the aBot twin.

P2 physics milestone (2026-09-02): the chibi humanoid used to be defined only
inside build_humanoid.py (bpy). To give the digital twin a real physics body
(MuJoCo) while Blender stays the render layer, every structural parameter now
lives in THIS pure-Python module (no bpy import), consumed by both outlets:

  * build_humanoid.py   -> humanoid.blend (armature bone tree + the physical
                           part anchors read from here; render styling stays
                           in the build script)
  * mjcf_generator.py   -> humanoid.mjcf (same bone tree/axes/part geometry/
                           masses; never reverse-engineered from the .blend)

Axis convention (measured 2026-09-02 by probing HumanoidRig bone frames in
the committed humanoid.blend; re-asserted by assert_axis_conventions() and by
tests/acceptance/test_humanoid_physics.py against fresh Blender probes):

  * '.L'-suffixed bones sit at world -X, '.R' at +X; the robot's anatomical
    RIGHT side is world -X (i.e. the '.L' chain), anatomical LEFT is the '.R'
    chain. The robot faces world -Y.
  * Vertical bones (root/spine/chest/neck/head): local X = world X (pitch),
    local Y = world Z (up, bone axis), local Z = world -Y (forward).
  * Limb bones (upper_arm/forearm/hand/thigh/shin): local X = world X,
    local Y = head->tail (down), local Z = world +Y (forward).
  * shoulder.L: X=(0,1,0) Y=(-1,0,0) Z=(0,0,1); shoulder.R mirrored.
  * foot.L/R: local X = world X (toe pitch), local Y along head->tail
    (down-forward), local Z = cross(X, Y).

FK semantics (matches Blender pose bones exactly, verified against posed
world-position probes, tolerance 1e-4):

  * rotation_euler (x, y, z) composes as R_local = Rz(z) @ Ry(y) @ Rx(x),
    expressed in the bone's local rest frame (columns of BONES[name]["frame"]).
  * chain: T_b = T_parent @ (rest_parent^-1 @ rest_b) @ [Trans(loc) @ Rot]
    (pose location, used only by the walk/run root bob, is applied in the
    bone-local frame; root rotation is never posed by the contract drivers).
"""

import math

# ---------------------------------------------------------------------------
# Linear algebra helpers (pure python, 3x3 matrices as 3 column vectors)
# ---------------------------------------------------------------------------
def mat_cols(x, y, z):
    """Rotation matrix stored as its three column (basis) vectors."""
    return (tuple(x), tuple(y), tuple(z))


def mat_apply(m, v):
    """m @ v where m is column-vector form."""
    return (
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    )


def mat_transpose(m):
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def mat_mul(a, b):
    """a @ b, both in column-vector form."""
    return mat_cols(mat_apply(a, b[0]), mat_apply(a, b[1]), mat_apply(a, b[2]))


def rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return mat_cols((1.0, 0.0, 0.0), (0.0, c, s), (0.0, -s, c))


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return mat_cols((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return mat_cols((c, s, 0.0), (-s, c, 0.0), (0.0, 0.0, 1.0))


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def quat_from_mat(m):
    """(w, x, y, z) quaternion from a rotation matrix in column form."""
    # rows of the matrix
    r00, r01, r02 = m[0][0], m[1][0], m[2][0]
    r10, r11, r12 = m[0][1], m[1][1], m[2][1]
    r20, r21, r22 = m[0][2], m[1][2], m[2][2]
    tr = r00 + r11 + r22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    elif (r00 > r11) and (r00 > r22):
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s
    return (w, x, y, z)


# ---------------------------------------------------------------------------
# Bone tree (the twin-control FK contract: 19 bones, same names as the rig)
# head/tail are armature-space rest coordinates (from add_armature).
# "frame" columns = bone local +X/+Y/+Z expressed in world, probed from the
# committed blend (roll-0 frames) on 2026-09-02.
# ---------------------------------------------------------------------------
_VERT = mat_cols((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0))
_LIMB = mat_cols((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
_SHOULDER_L = mat_cols((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
_SHOULDER_R = mat_cols((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
_FOOT = mat_cols((1.0, 0.0, 0.0), (0.0, -0.957826, -0.287348),
                 (0.0, 0.287348, -0.957826))

BONES = {
    "root":        {"parent": None,          "head": (0.0, 0.0, 0.02),  "tail": (0.0, 0.0, 0.06),  "frame": _VERT},
    "spine":       {"parent": "root",        "head": (0.0, 0.0, 0.26),  "tail": (0.0, 0.0, 0.42),  "frame": _VERT},
    "chest":       {"parent": "spine",       "head": (0.0, 0.0, 0.42),  "tail": (0.0, 0.0, 0.60),  "frame": _VERT},
    "neck":        {"parent": "chest",       "head": (0.0, 0.0, 0.60),  "tail": (0.0, 0.0, 0.66),  "frame": _VERT},
    "head":        {"parent": "neck",        "head": (0.0, 0.0, 0.66),  "tail": (0.0, 0.0, 1.05),  "frame": _VERT},
    "shoulder.L":  {"parent": "chest",       "head": (-0.04, 0.0, 0.55), "tail": (-0.16, 0.0, 0.55), "frame": _SHOULDER_L},
    "upper_arm.L": {"parent": "shoulder.L",  "head": (-0.185, 0.0, 0.55), "tail": (-0.185, 0.0, 0.415), "frame": _LIMB},
    "forearm.L":   {"parent": "upper_arm.L", "head": (-0.185, 0.0, 0.415), "tail": (-0.185, 0.0, 0.30), "frame": _LIMB},
    "hand.L":      {"parent": "forearm.L",   "head": (-0.185, 0.0, 0.30), "tail": (-0.185, 0.0, 0.23), "frame": _LIMB},
    "shoulder.R":  {"parent": "chest",       "head": (0.04, 0.0, 0.55), "tail": (0.16, 0.0, 0.55), "frame": _SHOULDER_R},
    "upper_arm.R": {"parent": "shoulder.R",  "head": (0.185, 0.0, 0.55), "tail": (0.185, 0.0, 0.415), "frame": _LIMB},
    "forearm.R":   {"parent": "upper_arm.R", "head": (0.185, 0.0, 0.415), "tail": (0.185, 0.0, 0.30), "frame": _LIMB},
    "hand.R":      {"parent": "forearm.R",   "head": (0.185, 0.0, 0.30), "tail": (0.185, 0.0, 0.23), "frame": _LIMB},
    "thigh.L":     {"parent": "root",        "head": (-0.095, 0.0, 0.28), "tail": (-0.095, 0.0, 0.15), "frame": _LIMB},
    "shin.L":      {"parent": "thigh.L",     "head": (-0.095, 0.0, 0.15), "tail": (-0.095, 0.0, 0.05), "frame": _LIMB},
    "foot.L":      {"parent": "shin.L",      "head": (-0.095, 0.0, 0.05), "tail": (-0.095, -0.10, 0.02), "frame": _FOOT},
    "thigh.R":     {"parent": "root",        "head": (0.095, 0.0, 0.28), "tail": (0.095, 0.0, 0.15), "frame": _LIMB},
    "shin.R":      {"parent": "thigh.R",     "head": (0.095, 0.0, 0.15), "tail": (0.095, 0.0, 0.05), "frame": _LIMB},
    "foot.R":      {"parent": "shin.R",      "head": (0.095, 0.0, 0.05), "tail": (0.095, -0.10, 0.02), "frame": _FOOT},
}

BONE_ORDER = [
    "root", "spine", "chest", "neck", "head",
    "shoulder.L", "upper_arm.L", "forearm.L", "hand.L",
    "shoulder.R", "upper_arm.R", "forearm.R", "hand.R",
    "thigh.L", "shin.L", "foot.L",
    "thigh.R", "shin.R", "foot.R",
]

# Bone CREATION order used by build_humanoid.add_armature -- must stay the
# exact historical order so Blender's roll-0 frame construction reproduces
# the probed frames bit-for-bit (kinematic order above is parent-first for
# FK / MJCF and independent of this).
BONE_CREATE_ORDER = [
    "root", "spine", "chest", "neck", "head",
    "shoulder.L", "upper_arm.L", "forearm.L", "hand.L",
    "thigh.L", "shin.L", "foot.L",
    "shoulder.R", "upper_arm.R", "forearm.R", "hand.R",
    "thigh.R", "shin.R", "foot.R",
]

# Head-bone-local landmarks of the FACE (visor center / ears), used by the
# axis assertions to pin "facing -Y" and "'.L' = anatomical right" beyond
# pure frame math (values probed from the committed blend, 2026-09-02).
FACE_VISOR_LOCAL = (0.0, 0.26, 0.312)   # visor center: local +Z = front
EAR_ANATOMICAL_RIGHT_LOCAL = (-0.30, 0.26, 0.0)  # world -X ear pod

# ---------------------------------------------------------------------------
# Physical part anchors shared with the render build (same numbers the .blend
# uses; decorative details stay in build_humanoid.py).
# ---------------------------------------------------------------------------
PHYSICAL_PARTS = (
    # name, kind, (x, y, z), (half extents / radii), subdivision levels
    ("Helmet",    "sphere",    (0.0, 0.0, 0.92),    (0.30, 0.30, 0.28), 2),
    ("Neck",      "cylinder",  (0.0, 0.0, 0.62),    (0.05, 0.05, 0.07), 1),
    ("Chest",     "sphere",    (0.0, 0.0, 0.50),    (0.16, 0.12, 0.12), 2),
    ("Abdomen",   "sphere",    (0.0, 0.0, 0.385),   (0.11, 0.09, 0.08), 2),
    ("Pelvis",    "sphere",    (0.0, 0.0, 0.285),   (0.12, 0.10, 0.08), 2),
    ("Pauldron",  "sphere",    (0.17, 0.0, 0.55),   (0.08, 0.08, 0.08), 2),
    ("UpperArm",  "cylinder",  (0.185, 0.0, 0.48),  (0.045, 0.045, 0.09), 2),
    ("Elbow",     "sphere",    (0.185, 0.0, 0.415), (0.042, 0.042, 0.042), 2),
    ("Forearm",   "cylinder",  (0.185, 0.0, 0.35),  (0.040, 0.040, 0.08), 2),
    ("Palm",      "sphere",    (0.185, -0.005, 0.245), (0.030, 0.026, 0.034), 1),
    ("Thigh",     "cylinder",  (0.095, 0.0, 0.215), (0.062, 0.062, 0.08), 2),
    ("Knee",      "sphere",    (0.095, 0.0, 0.15),  (0.05, 0.05, 0.05), 2),
    ("Shin",      "cylinder",  (0.095, 0.0, 0.10),  (0.052, 0.052, 0.07), 2),
    ("Boot",      "cube",      (0.095, -0.02, 0.09), (0.09, 0.12, 0.075), 2),
)

# Which render parts belong to which physics body (for documentation / future
# inertia refinement; colliders below are authoritative for physics).
PART_BODY_MAP = {
    "Helmet": "head", "Neck": "neck", "Chest": "chest", "Abdomen": "spine",
    "Pelvis": "root", "Pauldron": "shoulder", "UpperArm": "upper_arm",
    "Elbow": "upper_arm", "Forearm": "forearm", "Palm": "hand",
    "Thigh": "thigh", "Knee": "shin", "Shin": "shin", "Boot": "foot",
}

# ---------------------------------------------------------------------------
# T2: collision primitives (world-space rest coordinates; the generator maps
# them into bone-local frames with the probed frames above).
# Render meshes are NEVER used as colliders: head = sphere, torso/limbs =
# capsules, feet = boxes (plan abot-p2-physics-plan-001 / insight report
# tech-insight-report-p2-physics-001).
# ---------------------------------------------------------------------------
def _limb_colliders(sx):
    """Mirrored arm/leg colliders for sx=-1 ('.L', world -X) / +1 ('.R').

    Radii match the render primitives (PHYSICAL_PARTS) closely; capsule/box
    pairs that still interpenetrate at rest by construction are listed in
    CONTACT_EXCLUDES instead of colliding.
    """
    x_arm = sx * 0.185
    x_leg = sx * 0.095
    side = "L" if sx < 0 else "R"
    return {
        "shoulder.%s" % side: [
            {"type": "sphere", "center": (sx * 0.17, 0.0, 0.55), "r": 0.07},
        ],
        "upper_arm.%s" % side: [
            {"type": "capsule", "from": (x_arm, 0.0, 0.545),
             "to": (x_arm, 0.0, 0.42), "r": 0.045},
        ],
        "forearm.%s" % side: [
            {"type": "capsule", "from": (x_arm, 0.0, 0.415),
             "to": (x_arm, 0.0, 0.305), "r": 0.04},
        ],
        "hand.%s" % side: [
            {"type": "capsule", "from": (x_arm, 0.0, 0.295),
             "to": (x_arm, 0.0, 0.235), "r": 0.03},
        ],
        "thigh.%s" % side: [
            {"type": "capsule", "from": (x_leg, 0.0, 0.275),
             "to": (x_leg, 0.0, 0.155), "r": 0.06},
        ],
        "shin.%s" % side: [
            {"type": "capsule", "from": (x_leg, 0.0, 0.145),
             "to": (x_leg, 0.0, 0.055), "r": 0.052},
        ],
        "foot.%s" % side: [
            {"type": "box", "center": (x_leg, -0.05, 0.055),
             "half": (0.085, 0.135, 0.055)},
            # chibi counterweight: heavy battery pack in the boot sole
            # (no collision, mass/inertia only) -- keeps the COM low so the
            # top-heavy helmet does not tip the robot over.
            {"type": "sphere", "center": (x_leg, -0.045, 0.028), "r": 0.05,
             "ballast": True},
        ],
    }


COLLIDERS = {
    "root": [
        {"type": "capsule", "from": (0.0, 0.0, 0.21), "to": (0.0, 0.0, 0.33),
         "r": 0.10},
    ],
    "spine": [
        {"type": "capsule", "from": (0.0, 0.0, 0.30), "to": (0.0, 0.0, 0.42),
         "r": 0.09},
    ],
    "chest": [
        {"type": "capsule", "from": (0.0, 0.0, 0.44), "to": (0.0, 0.0, 0.58),
         "r": 0.12},
    ],
    "neck": [
        {"type": "capsule", "from": (0.0, 0.0, 0.585), "to": (0.0, 0.0, 0.66),
         "r": 0.05},
    ],
    # the oversized helmet is collision-modelled conservatively small: the
    # visor/helmet shell must not touch the floor before the chin does
    "head": [
        {"type": "sphere", "center": (0.0, 0.0, 0.92), "r": 0.19},
    ],
}
COLLIDERS.update(_limb_colliders(-1))
COLLIDERS.update(_limb_colliders(1))

# Body pairs whose simplified colliders interpenetrate at rest by
# construction (adjacent-segment overlaps that the render mesh would resolve
# visually). Excluded from contact generation like the standard MuJoCo
# humanoid does for its torso stacks.
CONTACT_EXCLUDES = (
    ("root", "chest"),        # pelvis capsule vs chest capsule
    ("root", "shin.L"),       # pelvis capsule vs shin tops
    ("root", "shin.R"),
    ("root", "thigh.L"),      # pelvis vs thigh (hip socket region)
    ("root", "thigh.R"),
    ("spine", "thigh.L"),     # abdomen vs thigh
    ("spine", "thigh.R"),
    ("thigh.L", "foot.L"),    # knee region vs boot top
    ("thigh.R", "foot.R"),
    ("forearm.L", "thigh.L"), # arm swing passes beside the thigh
    ("forearm.R", "thigh.R"),
    ("hand.L", "thigh.L"),    # hanging hands graze the thighs at rest
    ("hand.R", "thigh.R"),
)

# ---------------------------------------------------------------------------
# T2: explicit chibi mass budget (kg). The oversized helmet makes the model
# head-heavy; the boot ballast lowers the whole-body COM to ~0.30 m, safely
# inside the foot support polygon (static standing is passively stable).
# ---------------------------------------------------------------------------
BODY_MASS = {
    "root": 2.2,      # pelvis + waist belt block
    "spine": 0.7,     # abdomen armor
    "chest": 1.6,     # torso armor + chest emblem
    "neck": 0.15,
    "head": 3.0,      # oversized helmet + visor + ear pods
    "shoulder.L": 0.12, "shoulder.R": 0.12,
    "upper_arm.L": 0.35, "upper_arm.R": 0.35,
    "forearm.L": 0.25, "forearm.R": 0.25,
    "hand.L": 0.12, "hand.R": 0.12,
    "thigh.L": 0.55, "thigh.R": 0.55,
    "shin.L": 0.45, "shin.R": 0.45,
    "foot.L": 1.0, "foot.R": 1.0,
}
BALLAST_MASS = 3.0  # kg per boot (battery pack, collision-disabled)

TOTAL_MASS = sum(BODY_MASS.values()) + 2.0 * BALLAST_MASS

# ---------------------------------------------------------------------------
# Joint tuning per bone: (range-, range+) for the three local axes X/Y/Z,
# joint damping, frictionloss (servo-like static friction), position-actuator
# kp and torque limit. Ranges cover every contract motion (walk/run included)
# with margin; frictionloss + damping let the pose hold without oscillation.
# ---------------------------------------------------------------------------
JOINT_TUNING = {
    "root": None,
    "spine":      {"range": ((-0.6, 0.6), (-0.5, 0.5), (-0.35, 0.35)),
                   "damping": 1.5, "frictionloss": 0.5, "kp": 250.0, "tau": 80.0},
    "chest":      {"range": ((-0.6, 0.6), (-0.5, 0.5), (-0.35, 0.35)),
                   "damping": 1.5, "frictionloss": 0.5, "kp": 250.0, "tau": 80.0},
    # neck/head: the 3 kg helmet has ~0.25 kg m^2 inertia about the neck;
    # damping is sized for near-critical servo response (zeta ~ 0.8-1) so
    # nod/look do not ring the head and knock the chibi over
    "neck":       {"range": ((-0.5, 0.5), (-0.6, 0.6), (-0.35, 0.35)),
                   "damping": 1.2, "frictionloss": 0.05, "kp": 50.0, "tau": 20.0},
    "head":       {"range": ((-0.7, 0.7), (-0.9, 0.9), (-0.45, 0.45)),
                   "damping": 3.0, "frictionloss": 0.05, "kp": 60.0, "tau": 20.0},
    "shoulder.L": {"range": ((-0.7, 0.7), (-0.5, 0.5), (-0.6, 0.6)),
                   "damping": 0.6, "frictionloss": 0.25, "kp": 60.0, "tau": 25.0},
    "shoulder.R": {"range": ((-0.7, 0.7), (-0.5, 0.5), (-0.6, 0.6)),
                   "damping": 0.6, "frictionloss": 0.25, "kp": 60.0, "tau": 25.0},
    "upper_arm.L": {"range": ((-1.0, 1.4), (-0.45, 0.45), (-1.65, 1.65)),
                    "damping": 0.8, "frictionloss": 0.35, "kp": 100.0, "tau": 40.0},
    "upper_arm.R": {"range": ((-1.0, 1.4), (-0.45, 0.45), (-1.65, 1.65)),
                    "damping": 0.8, "frictionloss": 0.35, "kp": 100.0, "tau": 40.0},
    "forearm.L":  {"range": ((-0.3, 1.5), (-0.35, 0.35), (-1.65, 1.65)),
                   "damping": 0.6, "frictionloss": 0.3, "kp": 70.0, "tau": 30.0},
    "forearm.R":  {"range": ((-0.3, 1.5), (-0.35, 0.35), (-1.65, 1.65)),
                   "damping": 0.6, "frictionloss": 0.3, "kp": 70.0, "tau": 30.0},
    "hand.L":     {"range": ((-0.6, 0.6), (-0.6, 0.6), (-0.6, 0.6)),
                   "damping": 0.2, "frictionloss": 0.1, "kp": 20.0, "tau": 8.0},
    "hand.R":     {"range": ((-0.6, 0.6), (-0.6, 0.6), (-0.6, 0.6)),
                   "damping": 0.2, "frictionloss": 0.1, "kp": 20.0, "tau": 8.0},
    "thigh.L":    {"range": ((-1.2, 1.3), (-0.35, 0.35), (-0.45, 0.45)),
                   "damping": 1.5, "frictionloss": 0.6, "kp": 250.0, "tau": 90.0},
    "thigh.R":    {"range": ((-1.2, 1.3), (-0.35, 0.35), (-0.45, 0.45)),
                   "damping": 1.5, "frictionloss": 0.6, "kp": 250.0, "tau": 90.0},
    "shin.L":     {"range": ((-1.6, 0.08), (-0.25, 0.25), (-0.3, 0.3)),
                   "damping": 1.2, "frictionloss": 0.5, "kp": 200.0, "tau": 80.0},
    "shin.R":     {"range": ((-1.6, 0.08), (-0.25, 0.25), (-0.3, 0.3)),
                   "damping": 1.2, "frictionloss": 0.5, "kp": 200.0, "tau": 80.0},
    "foot.L":     {"range": ((-0.6, 0.8), (-0.3, 0.3), (-0.35, 0.35)),
                   "damping": 0.5, "frictionloss": 0.25, "kp": 60.0, "tau": 25.0},
    "foot.R":     {"range": ((-0.6, 0.8), (-0.3, 0.3), (-0.35, 0.35)),
                   "damping": 0.5, "frictionloss": 0.25, "kp": 60.0, "tau": 25.0},
}

# MuJoCo joint order inside each body that reproduces Blender XYZ euler
# (R_local = Rz @ Ry @ Rx): outermost hinge first.
JOINT_AXIS_ORDER = (("z", (0.0, 0.0, 1.0)), ("y", (0.0, 1.0, 0.0)),
                    ("x", (1.0, 0.0, 0.0)))


def joint_names(bone):
    """The three MuJoCo hinge names of a bone, in XML/qpos order (rz, ry, rx)."""
    return tuple("%s.%s" % (bone, a) for a, _ in JOINT_AXIS_ORDER)


# ---------------------------------------------------------------------------
# Pure FK identical to Blender's pose-bone chain (see module docstring).
# ---------------------------------------------------------------------------
def euler_local_matrix(euler):
    """Blender XYZ rotation_euler -> bone-local rotation matrix."""
    x, y, z = euler
    return mat_mul(rot_z(z), mat_mul(rot_y(y), rot_x(x)))


def rest_world_transforms():
    """Rest transform of every bone in world space: (frame, head)."""
    out = {}
    for name in BONE_ORDER:
        out[name] = (BONES[name]["frame"], BONES[name]["head"])
    return out


def forward_kinematics(pose_eulers, root_loc=(0.0, 0.0, 0.0)):
    """World-space bone heads for given bone-local euler angles.

    pose_eulers: {bone_name: (rx, ry, rz)} (missing bones are identity).
    root_loc: pose-bone location of 'root' in root-local frame (walk/run bob).
    Returns {bone: (x, y, z)} of every bone's posed head in world space.
    """
    placed = {}
    for name in BONE_ORDER:
        bone = BONES[name]
        parent = bone["parent"]
        frame, head = bone["frame"], bone["head"]
        euler = pose_eulers.get(name, (0.0, 0.0, 0.0))
        r_pose = euler_local_matrix(euler)
        if parent is None:
            loc = mat_apply(frame, root_loc)
            world_frame = mat_mul(frame, r_pose)
            placed[name] = (world_frame, vadd(head, loc))
        else:
            p_frame, p_head = placed[parent]
            p_rest_frame = BONES[parent]["frame"]
            p_rest_head = BONES[parent]["head"]
            # rest offset of this bone inside the parent's rest frame
            rel = mat_apply(mat_transpose(p_rest_frame), vsub(head, p_rest_head))
            base_pos = vadd(p_head, mat_apply(p_frame, rel))
            # the child's rest orientation relative to its parent is NOT
            # identity in general (vertical chain -> limbs -> shoulders all
            # differ); pose rotation applies on top of that rest offset
            r_rel = mat_mul(mat_transpose(p_rest_frame), frame)
            world_frame = mat_mul(p_frame, mat_mul(r_rel, r_pose))
            placed[name] = (world_frame, base_pos)
    return {name: placed[name][1] for name in BONE_ORDER}


def geom_volume(geom):
    """Volume of a collision primitive (sphere / capsule / box)."""
    if geom["type"] == "sphere":
        return 4.0 / 3.0 * math.pi * geom["r"] ** 3
    if geom["type"] == "capsule":
        f, t, r = geom["from"], geom["to"], geom["r"]
        length = math.sqrt(sum((a - b) ** 2 for a, b in zip(f, t)))
        return math.pi * r * r * length + 4.0 / 3.0 * math.pi * r ** 3
    if geom["type"] == "box":
        hx, hy, hz = geom["half"]
        return 8.0 * hx * hy * hz
    raise ValueError("unknown collider type: %r" % geom["type"])


def body_center_of_mass():
    """Whole-body COM (world, rest pose) from collider centroids + masses.

    Each body's mass is spread over its colliders by volume; the ballast
    sphere carries BALLAST_MASS. Used by tests as the static-stability audit.
    """
    moment = [0.0, 0.0, 0.0]
    total = 0.0
    for name in BONE_ORDER:
        geoms = COLLIDERS[name]
        vols = [geom_volume(g) for g in geoms]
        main_vol = sum(v for v, g in zip(vols, geoms) if not g.get("ballast"))
        mass_main = BODY_MASS[name]
        for g, v in zip(geoms, vols):
            if g.get("ballast"):
                m, c = BALLAST_MASS, g["center"]
            else:
                m = mass_main * (v / main_vol)
                c = _geom_centroid(g)
            total += m
            moment = [moment[i] + m * c[i] for i in range(3)]
    return tuple(moment[i] / total for i in range(3)), total


def _geom_centroid(g):
    if g["type"] == "sphere":
        return g["center"]
    if g["type"] == "box":
        return g["center"]
    f, t = g["from"], g["to"]
    return tuple((a + b) / 2.0 for a, b in zip(f, t))


# ---------------------------------------------------------------------------
# Axis-convention assertions (anti-mirroring firewall).
# ---------------------------------------------------------------------------
def assert_axis_conventions(tol=1e-3):
    """Return a list of (check_name, ok, detail); every entry must pass.

    Validates the spec itself (pure math), independent of Blender/MuJoCo:
      A1 '.L' bones at world -X / '.R' at +X, vertical chain on x=0
      A2 head (and whole vertical chain) local Y = world +Z (vertical)
      A3 facing world -Y: head local Z = world -Y; face landmark in +Z;
         anatomical-right ear pod in head-local -X
      A4 limb local Y = head->tail direction
      A5 exact left/right mirroring of head/tail/frames
      A6 FK invariants: rest pose matches bone heads; tpose hands at +-X;
         wave forearm vertical; walk root bob toward world -Y (front)
    """
    checks = []

    def close(a, b, t=tol):
        return abs(a - b) <= t

    def vclose(u, v, t=tol):
        return all(close(a, b, t) for a, b in zip(u, v))

    # A1 side placement
    for name in BONE_ORDER:
        b = BONES[name]
        if name.endswith(".L"):
            ok = b["head"][0] < 0.0
            checks.append(("A1 .L at world -X: %s" % name, ok,
                           "head=%s" % (b["head"],)))
        elif name.endswith(".R"):
            ok = b["head"][0] > 0.0
            checks.append(("A1 .R at world +X: %s" % name, ok,
                           "head=%s" % (b["head"],)))
        elif name in ("root", "spine", "chest", "neck", "head"):
            ok = close(b["head"][0], 0.0)
            checks.append(("A1 vertical chain on x=0: %s" % name, ok,
                           "head=%s" % (b["head"],)))

    # A2 vertical bones point up
    for name in ("root", "spine", "chest", "neck", "head"):
        y_axis = BONES[name]["frame"][1]
        ok = vclose(y_axis, (0.0, 0.0, 1.0))
        checks.append(("A2 local Y = world +Z (vertical): %s" % name, ok,
                       "y=%s" % (y_axis,)))

    # A3 facing -Y
    head_frame = BONES["head"]["frame"]
    checks.append(("A3 head local Z = world -Y (forward)",
                   vclose(head_frame[2], (0.0, -1.0, 0.0)),
                   "z=%s" % (head_frame[2],)))
    checks.append(("A3 face landmark in head-local +Z (visor front)",
                   FACE_VISOR_LOCAL[2] > 0.25,
                   "visor_local=%s" % (FACE_VISOR_LOCAL,)))
    checks.append(("A3 anatomical-right ear in head-local -X",
                   EAR_ANATOMICAL_RIGHT_LOCAL[0] < 0.0,
                   "ear_local=%s" % (EAR_ANATOMICAL_RIGHT_LOCAL,)))

    # A4 limb Y along head->tail
    for name in BONE_ORDER:
        b = BONES[name]
        if b["parent"] is None or name.startswith("shoulder"):
            continue
        axis = vsub(b["tail"], b["head"])
        norm = math.sqrt(sum(v * v for v in axis))
        if norm < 1e-9:
            continue
        axis = tuple(v / norm for v in axis)
        # limb bones point DOWN along local Y, foot points down-forward
        ok = vclose(BONES[name]["frame"][1], axis)
        checks.append(("A4 local Y = head->tail: %s" % name, ok,
                       "frame_y=%s axis=%s" % (BONES[name]["frame"][1], axis)))

    # A5 mirror symmetry: head/tail mirror across x=0, canonical frames as
    # documented, and FK pose mirroring (wave '.L' vs mirrored euler angles
    # on '.R' must produce exactly mirrored world positions).
    for stem in ("shoulder", "upper_arm", "forearm", "hand", "thigh",
                 "shin", "foot"):
        l, r = BONES[stem + ".L"], BONES[stem + ".R"]
        ok = (close(l["head"][0], -r["head"][0])
              and close(l["head"][1], r["head"][1])
              and close(l["head"][2], r["head"][2])
              and close(l["tail"][0], -r["tail"][0])
              and close(l["tail"][1], r["tail"][1])
              and close(l["tail"][2], r["tail"][2]))
        detail = "L head=%s R head=%s" % (l["head"], r["head"])
        checks.append(("A5 mirror head/tail: %s" % stem, ok, detail))

    for name, canon in (("upper_arm.L", _LIMB), ("upper_arm.R", _LIMB),
                        ("thigh.L", _LIMB), ("thigh.R", _LIMB),
                        ("shin.L", _LIMB), ("shin.R", _LIMB),
                        ("foot.L", _FOOT), ("foot.R", _FOOT),
                        ("shoulder.L", _SHOULDER_L),
                        ("shoulder.R", _SHOULDER_R),
                        ("head", _VERT), ("root", _VERT)):
        ok = all(vclose(BONES[name]["frame"][i], canon[i]) for i in range(3))
        checks.append(("A5 canonical frame: %s" % name, ok,
                       "frame=%s" % (BONES[name]["frame"],)))

    # FK pose mirroring: euler (x, y, z) mirrors to (x, -y, -z) because an
    # axial vector reflects as (ax, -ay, -az) across the x=0 plane and both
    # limb chains share the same axis-aligned frames.
    wave_l = {"upper_arm.L": (-0.15, 0.0, math.pi / 2.0),
              "forearm.L": (0.0, 0.0, math.pi / 2.0),
              "hand.L": (0.0, 0.0, 0.45),
              "upper_arm.R": (0.14, 0.0, -0.105),
              "forearm.R": (0.21, 0.0, 0.0)}
    wave_r = {}
    for bn, e in wave_l.items():
        mirror_bn = (bn[:-2] + ".R") if bn.endswith(".L") else (bn[:-2] + ".L")
        wave_r[mirror_bn] = (e[0], -e[1], -e[2])
    fk_l = forward_kinematics(wave_l)
    fk_r = forward_kinematics(wave_r)
    for bn in BONE_ORDER:
        mirror_bn = (bn[:-2] + ".R") if bn.endswith(".L") else \
            (bn[:-2] + ".L") if bn.endswith(".R") else bn
        pl, pr = fk_l[bn], fk_r[mirror_bn]
        ok = vclose((pl[0], pl[1], pl[2]), (-pr[0], pr[1], pr[2]), 1e-6)
        checks.append(("A5 FK pose mirror: %s" % bn, ok,
                       "L=%s mirrored R=%s" % (pl, pr)))

    # A6 FK invariants (the real anti-mirroring firewall)
    rest = forward_kinematics({})
    for name in BONE_ORDER:
        ok = vclose(rest[name], BONES[name]["head"], 1e-6)
        checks.append(("A6 rest FK matches bone heads: %s" % name, ok,
                       "fk=%s head=%s" % (rest[name], BONES[name]["head"])))

    tp = forward_kinematics({"upper_arm.L": (0, 0, math.pi / 2.0),
                             "upper_arm.R": (0, 0, -math.pi / 2.0)})
    hand_l, hand_r = tp["hand.L"], tp["hand.R"]
    checks.append(("A6 tpose: anatomical RIGHT hand (.L) toward world -X",
                   hand_l[0] < -0.40 and abs(hand_l[2] - 0.55) < 0.01,
                   "hand.L=%s" % (hand_l,)))
    checks.append(("A6 tpose: anatomical LEFT hand (.R) toward world +X",
                   hand_r[0] > 0.40 and abs(hand_r[2] - 0.55) < 0.01,
                   "hand.R=%s" % (hand_r,)))

    wave = forward_kinematics({"upper_arm.L": (0, 0, math.pi / 2.0),
                               "forearm.L": (0, 0, math.pi / 2.0)})
    fa = wave["forearm.L"]
    hd = wave["hand.L"]
    checks.append(("A6 wave: '.L' forearm folds straight UP",
                   abs(hd[0] - fa[0]) < 0.01 and hd[2] > fa[2] + 0.10,
                   "forearm=%s hand=%s" % (fa, hd)))

    walk = forward_kinematics({}, root_loc=(0.0, 0.0, 0.035))
    checks.append(("A6 walk root bob lands toward world -Y (facing dir)",
                   close(walk["root"][1], -0.035) and close(walk["root"][0], 0.0),
                   "root=%s" % (walk["root"],)))

    return checks


if __name__ == "__main__":
    bad = 0
    for name, ok, detail in assert_axis_conventions():
        if not ok:
            bad += 1
            print("FAIL %s | %s" % (name, detail))
    com, mass = body_center_of_mass()
    print("total_mass=%.3f kg  COM=(%.4f, %.4f, %.4f)" % ((mass,) + com))
    if bad:
        raise SystemExit("%d axis assertions failed" % bad)
    print("PASS: %d axis-convention checks" % len(assert_axis_conventions()))
