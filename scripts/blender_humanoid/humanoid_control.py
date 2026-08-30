"""Reusable control module for driving the aBot humanoid digital twin.

The aBot humanoid (assets/humanoid/humanoid.blend) has an FK armature
("HumanoidRig") whose bones were probed:

   - Vertical bones (spine/chest/neck/head/upper_arm/forearm/hand/thigh/shin):
       local +Y = bone axis, local +X = world +X (right), local +Z = world +Y
       (forward) for limbs; for spine/head local +Z = world -Y (backward).
   - rotation_euler is in the bone's local frame where +Y always points along
       head->tail, so poses are expressed as local Euler rotations.
   - SIDE CONVENTION (probed from build_humanoid.py, 2026-08-30): the rig is
       built with '.L'-suffixed bones at world -X and '.R'-suffixed bones at
       world +X, while the robot faces world -Y (visor toward the front
       camera). Anatomically the robot's RIGHT side is world -X and its LEFT
       side is world +X, so: anatomical RIGHT arm/leg = the '.L' bones
       (screen LEFT in the front camera), anatomical LEFT = the '.R' bones
       (screen right). apply_wave raises the anatomical RIGHT arm, i.e. it
       drives the '.L' bone chain; apply_wave_left mirrors with '.R'.
   - HEAD AXES: head local X = world X (pitch/nod), local Y = bone axis =
       world Z (horizontal yaw), local Z = world -Y (in-plane roll). A
       side-looking "look" is a yaw about local Y, not a roll about local Z.

This module provides:
  - load_humanoid(): load the .blend and return the armature.
  - reset_pose() / set_bone(): low-level FK helpers.
  - pose_*(): static poses (A/T/relax).
  - apply_*(): time-parameterized motions (idle/wave/walk/nod/look/run).
  - MotionTimeline: sequence motions over time and drive them on every frame
    change (frame_change_pre handler), so the twin is "drivable" and can be
    rendered as an animation.

The same API is what a future "brain" (LLM/agent) will call to make the bot
move — P1 delivers the drivable core.

Baked keyframe Actions (2026-08-30): every key action below is ALSO baked
into humanoid.blend as a named Action (ACTION_SPECS: ActionRelax, ActionTPose,
ActionAPose, ActionIdle, ActionWave, ActionWalk, ActionNod, ActionLook,
ActionRun) by build_humanoid.py, as pure pose-bone rotation keyframes (walk/
run additionally key the FK root-bob location) — no vertex/shape animation.
The Blender GUI can therefore select an Action and just press play, with no
Python driver involved. MOTION_DRIVERS maps motion names to the driver
functions used for baking, so the baked clips and this runtime FK chain share
one contract. Because an attached Action would overwrite Python-driven joint
rotations during depsgraph evaluation, load_humanoid() detaches the active
Action for runtime driving; the baked Actions remain in bpy.data.actions for
GUI selection.
"""

import math

import bpy
from mathutils import Euler

FPS = 30


# ---------------------------------------------------------------------------
# Loading / low-level
# ---------------------------------------------------------------------------
def load_humanoid(blend_path, arm_name="HumanoidRig"):
    """Open the blend and return the armature object.

    The blend ships with baked keyframe Actions (see module docstring). An
    attached Action would overwrite Python-driven joint rotations during
    depsgraph evaluation, so it is detached here: runtime FK driving via this
    module stays authoritative while the baked Actions remain in
    bpy.data.actions for GUI playback.
    """
    bpy.ops.wm.open_mainfile(filepath=blend_path)
    arm = bpy.data.objects[arm_name]
    if arm.animation_data is not None and arm.animation_data.action is not None:
        arm.animation_data.action = None
    return arm


def reset_pose(arm):
    """Reset every pose bone to its rest transform."""
    for p in arm.pose.bones:
        p.rotation_mode = "XYZ"
        p.rotation_euler = (0.0, 0.0, 0.0)
        p.location = (0.0, 0.0, 0.0)


def set_bone(arm, name, euler=(0.0, 0.0, 0.0), loc=None):
    """Set a pose bone's local Euler rotation (degrees -> radians optional)."""
    p = arm.pose.bones[name]
    p.rotation_mode = "XYZ"
    p.rotation_euler = Euler(euler)
    if loc is not None:
        p.location = loc


def R(deg):
    return math.radians(deg)


# ---------------------------------------------------------------------------
# Static poses
# ---------------------------------------------------------------------------
def pose_rest(arm):
    reset_pose(arm)


def pose_relax(arm):
    """Natural standing: arms slightly bent, straight legs."""
    reset_pose(arm)
    set_bone(arm, "upper_arm.L", (R(8), 0, R(6)))
    set_bone(arm, "upper_arm.R", (R(8), 0, R(-6)))
    set_bone(arm, "forearm.L", (R(12), 0, 0))
    set_bone(arm, "forearm.R", (R(12), 0, 0))


def pose_tpose(arm):
    """T-pose: arms straight out to the sides."""
    reset_pose(arm)
    set_bone(arm, "upper_arm.L", (0, 0, R(90)))
    set_bone(arm, "upper_arm.R", (0, 0, R(-90)))


def pose_apose(arm):
    """A-pose: arms angled ~45 deg down from horizontal."""
    reset_pose(arm)
    set_bone(arm, "upper_arm.L", (0, 0, R(45)))
    set_bone(arm, "upper_arm.R", (0, 0, R(-45)))


# ---------------------------------------------------------------------------
# Time-parameterized motions  (t = time in seconds)
# ---------------------------------------------------------------------------
def apply_idle(arm, t):
    """Gentle breathing + subtle sway."""
    reset_pose(arm)
    set_bone(arm, "chest", (0.03 * math.sin(t * 1.2), 0, 0))
    set_bone(arm, "spine", (0.02 * math.sin(t * 1.2 + 0.5), 0, 0))
    set_bone(arm, "upper_arm.L", (0.04 * math.sin(t * 0.8), 0, R(6)))
    set_bone(arm, "upper_arm.R", (0.04 * math.sin(t * 0.8 + 0.4), 0, R(-6)))
    set_bone(arm, "forearm.L", (R(10), 0, 0))
    set_bone(arm, "forearm.R", (R(10), 0, 0))


def apply_nod(arm, t):
    """Nod head up/down.

    Pitch about head local X (= world X). Amplitude deepened from 0.22 to
    0.45 rad (2026-08-30) so the bow-down reads clearly in a single still.
    """
    reset_pose(arm)
    set_bone(arm, "chest", (0.02 * math.sin(t * 1.5), 0, 0))
    set_bone(arm, "head", (0.45 * math.sin(t * 2.5), 0, 0))
    set_bone(arm, "upper_arm.L", (0.04, 0, R(6)))
    set_bone(arm, "upper_arm.R", (0.04, 0, R(-6)))
    set_bone(arm, "forearm.L", (R(10), 0, 0))
    set_bone(arm, "forearm.R", (R(10), 0, 0))


def apply_look(arm, t):
    """Turn head left/right: horizontal YAW about the world vertical axis.

    Bug fixed 2026-08-30: the previous implementation rotated the head about
    its local Z (= world -Y, backward), which is an in-plane ROLL (the visor
    kept facing the camera and the head only tilted sideways). A side look is
    a yaw about the head's local Y (= bone axis = world +Z). Positive yaw
    swings the visor toward world +X, hiding the far (-X) ear pod.
    """
    reset_pose(arm)
    set_bone(arm, "head", (0, 0.55 * math.sin(t * 1.5), 0))
    set_bone(arm, "upper_arm.L", (0.04, 0, R(6)))
    set_bone(arm, "upper_arm.R", (0.04, 0, R(-6)))
    set_bone(arm, "forearm.L", (R(10), 0, 0))
    set_bone(arm, "forearm.R", (R(10), 0, 0))


def apply_wave(arm, t):
    """Anatomical RIGHT arm raised, forearm up, waving hand.

    Side fix 2026-08-30 (visual-analyst review): the acceptance criterion is
    "right arm raised", i.e. the anatomical right arm. In this rig the
    anatomical right side is world -X = the '.L' bone chain (screen LEFT in
    the front camera); the previous implementation drove '.R' (world +X),
    which is anatomically the LEFT arm. See SIDE CONVENTION in the module
    docstring. Form: upper arm abducted ~90 deg, elbow bent 90 deg with the
    forearm vertical, hand beside the head with fingers spread, the other
    arm relaxed at the side.

    Calibrated 2026-08-30 (probe of the FK axes with the arm abducted): with
    the upper arm abducted to horizontal the forearm points straight up at
    local Z +90 on the '.L' chain (mirror of -90 on '.R'), and the hand rocks
    side-to-side around its local Z (world forward axis) = the wave.
    """
    reset_pose(arm)
    # anatomical right ('.L' bones) upper arm out to the side + slight swing
    set_bone(arm, "upper_arm.L", (-0.15 * math.sin(t * 5.0), 0, R(90)))
    # forearm folded straight up (elbow ~90 deg)
    set_bone(arm, "forearm.L", (0, 0, R(90)))
    # hand rocking side-to-side (fingers spread by the hand mesh)
    set_bone(arm, "hand.L", (0, 0, 0.45 * math.sin(t * 6.0)))
    # anatomical left ('.R' bones) arm relaxed, hanging down
    set_bone(arm, "upper_arm.R", (R(8), 0, R(-6)))
    set_bone(arm, "forearm.R", (R(12), 0, 0))


def apply_wave_left(arm, t):
    """Mirror wave with the anatomical LEFT arm ('.R' bones, world +X)."""
    reset_pose(arm)
    set_bone(arm, "upper_arm.R", (-0.15 * math.sin(t * 5.0), 0, R(-90)))
    set_bone(arm, "forearm.R", (0, 0, R(-90)))
    set_bone(arm, "hand.R", (0, 0, 0.45 * math.sin(t * 6.0)))
    set_bone(arm, "upper_arm.L", (R(8), 0, R(6)))
    set_bone(arm, "forearm.L", (R(12), 0, 0))


def apply_walk(arm, t):
    """Walk-in-place gait: legs swing opposite, arms swing opposite, bob."""
    f = t * 2.0  # step frequency
    s = math.sin(f)
    c = math.cos(f)
    reset_pose(arm)
    # legs (forward swing = +local X)
    set_bone(arm, "thigh.L", (0.5 * s, 0, 0))
    set_bone(arm, "thigh.R", (-0.5 * s, 0, 0))
    # knees bend backward (heel up) as the leg swings back
    set_bone(arm, "shin.L", (-0.55 + 0.35 * s, 0, 0))
    set_bone(arm, "shin.R", (-0.55 - 0.35 * s, 0, 0))
    # feet (slight toe point on swing)
    set_bone(arm, "foot.L", (0.12 * (1.0 - c) / 2.0, 0, 0))
    set_bone(arm, "foot.R", (0.12 * (1.0 + c) / 2.0, 0, 0))
    # arms swing opposite to the same-side leg
    set_bone(arm, "upper_arm.L", (0.35 * math.sin(f + math.pi), 0, 0))
    set_bone(arm, "upper_arm.R", (0.35 * math.sin(f), 0, 0))
    set_bone(arm, "forearm.L", (R(20), 0, 0))
    set_bone(arm, "forearm.R", (R(20), 0, 0))
    # torso counter-sway + forward lean
    set_bone(arm, "spine", (R(6) + 0.05 * math.sin(f), 0, 0))
    set_bone(arm, "chest", (0.06 * math.sin(f), 0, 0))
    # vertical bob
    set_bone(arm, "root", (0, 0, 0), loc=(0, 0, 0.035 * abs(s)))


def apply_run(arm, t):
    """Run-in-place: faster cadence, bigger swing and forward lean than walk."""
    f = t * 3.0  # stride frequency (walk uses 2.0)
    s = math.sin(f)
    c = math.cos(f)
    reset_pose(arm)
    # legs: larger swing + deeper knee bend
    set_bone(arm, "thigh.L", (0.7 * s, 0, 0))
    set_bone(arm, "thigh.R", (-0.7 * s, 0, 0))
    set_bone(arm, "shin.L", (-0.9 + 0.45 * s, 0, 0))
    set_bone(arm, "shin.R", (-0.9 - 0.45 * s, 0, 0))
    set_bone(arm, "foot.L", (0.2 * (1.0 - c) / 2.0, 0, 0))
    set_bone(arm, "foot.R", (0.2 * (1.0 + c) / 2.0, 0, 0))
    # arms: stronger opposite swing, elbows bent ~45 deg
    set_bone(arm, "upper_arm.L", (0.55 * math.sin(f + math.pi), 0, R(10)))
    set_bone(arm, "upper_arm.R", (0.55 * math.sin(f), 0, R(-10)))
    set_bone(arm, "forearm.L", (R(45), 0, 0))
    set_bone(arm, "forearm.R", (R(45), 0, 0))
    # pronounced forward lean + counter-sway
    set_bone(arm, "spine", (R(12) + 0.06 * math.sin(f), 0, 0))
    set_bone(arm, "chest", (0.08 * math.sin(f), 0, 0))
    # stronger vertical bob
    set_bone(arm, "root", (0, 0, 0), loc=(0, 0, 0.05 * abs(s)))


# ---------------------------------------------------------------------------
# Baked-Action contract (shared with the build_humanoid.py bake step)
# ---------------------------------------------------------------------------
# Every key action is baked into humanoid.blend as a named Action of
# pose-bone rotation keyframes (walk/run additionally key the FK root-bob
# location) so the Blender GUI plays the motion from the timeline without any
# Python driver. Durations are (near-)multiples of each motion's component
# periods so looped playback is seamless.
ACTION_SPECS = (
    # (motion name, baked Action name, duration in seconds; None = static hold)
    ("relax", "ActionRelax", None),
    ("tpose", "ActionTPose", None),
    ("apose", "ActionAPose", None),
    ("idle", "ActionIdle", 2 * math.pi / 1.2),     # one breathing period
    ("wave", "ActionWave", 2 * math.pi),           # common period of 5 & 6 rad/s
    ("walk", "ActionWalk", math.pi),               # one full gait cycle
    ("nod", "ActionNod", 2 * (2 * math.pi / 2.5)),  # two head-nod periods
    ("look", "ActionLook", 2 * math.pi / 1.5),     # one yaw period
    ("run", "ActionRun", 2 * math.pi / 3.0),       # one stride cycle
)

# motion name -> driver function with uniform (arm, t) signature (the single
# source of truth for baking; static poses ignore t)
MOTION_DRIVERS = {
    "relax": lambda arm, t: pose_relax(arm),
    "tpose": lambda arm, t: pose_tpose(arm),
    "apose": lambda arm, t: pose_apose(arm),
    "idle": apply_idle,
    "wave": apply_wave,
    "walk": apply_walk,
    "nod": apply_nod,
    "look": apply_look,
    "run": apply_run,
}

STATIC_ACTIONS = frozenset(spec[1] for spec in ACTION_SPECS if spec[2] is None)


# ---------------------------------------------------------------------------
# Motion timeline (drives the twin on every frame)
# ---------------------------------------------------------------------------
class MotionTimeline:
    """Sequences (t0, t1, fn) motions and drives them via a frame handler."""

    def __init__(self, arm, fps=FPS):
        self.arm = arm
        self.fps = fps
        self.segments = []

    def add(self, t0, t1, fn):
        self.segments.append((t0, t1, fn))

    def drive(self, t):
        reset_pose(self.arm)
        for t0, t1, fn in self.segments:
            if t0 <= t <= t1:
                fn(self.arm, t)
                break

    def register(self, scene=None):
        """Register a frame_change_pre handler that drives the pose."""

        def handler(scene):
            t = scene.frame_current / self.fps
            self.drive(t)

        bpy.app.handlers.frame_change_pre.append(handler)
        return handler
