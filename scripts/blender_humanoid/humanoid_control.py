"""Reusable control module for driving the aBot humanoid digital twin.

The aBot humanoid (assets/humanoid/humanoid.blend) has an FK armature
("HumanoidRig") whose bones were probed:

  - Vertical bones (spine/chest/neck/head/upper_arm/forearm/hand/thigh/shin):
      local +Y = bone axis, local +X = world +X (right), local +Z = world +Y
      (forward) for limbs; for spine/head local +Z = world -Y (backward).
  - rotation_euler is in the bone's local frame where +Y always points along
      head->tail, so poses are expressed as local Euler rotations.

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
"""

import math

import bpy
from mathutils import Euler

FPS = 30


# ---------------------------------------------------------------------------
# Loading / low-level
# ---------------------------------------------------------------------------
def load_humanoid(blend_path, arm_name="HumanoidRig"):
    """Open the blend and return the armature object."""
    bpy.ops.wm.open_mainfile(filepath=blend_path)
    return bpy.data.objects[arm_name]


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
    """Nod head up/down."""
    reset_pose(arm)
    set_bone(arm, "chest", (0.02 * math.sin(t * 1.5), 0, 0))
    set_bone(arm, "head", (0.22 * math.sin(t * 2.5), 0, 0))
    set_bone(arm, "upper_arm.L", (0.04, 0, R(6)))
    set_bone(arm, "upper_arm.R", (0.04, 0, R(-6)))
    set_bone(arm, "forearm.L", (R(10), 0, 0))
    set_bone(arm, "forearm.R", (R(10), 0, 0))


def apply_look(arm, t):
    """Turn head left/right."""
    reset_pose(arm)
    set_bone(arm, "head", (0, 0, 0.5 * math.sin(t * 1.5)))
    set_bone(arm, "upper_arm.L", (0.04, 0, R(6)))
    set_bone(arm, "upper_arm.R", (0.04, 0, R(-6)))
    set_bone(arm, "forearm.L", (R(10), 0, 0))
    set_bone(arm, "forearm.R", (R(10), 0, 0))


def apply_wave(arm, t):
    """Right arm raised, forearm up, waving hand.

    Calibrated 2026-08-30 (probe of the FK axes with the arm abducted):
    with upper_arm.R at local Z -90 the forearm points straight up at local
    Z -90 (the old Z 180 folded it back into the torso), and the hand rocks
    side-to-side around its local Z (world forward axis) = the wave.
    """
    reset_pose(arm)
    # right upper arm out to the side, slight forward swing for the wave
    set_bone(arm, "upper_arm.R", (-0.15 * math.sin(t * 5.0), 0, R(-90)))
    # forearm folded straight up
    set_bone(arm, "forearm.R", (0, 0, R(-90)))
    # hand rocking side-to-side
    set_bone(arm, "hand.R", (0, 0, 0.45 * math.sin(t * 6.0)))
    # left arm relaxed
    set_bone(arm, "upper_arm.L", (R(8), 0, R(6)))
    set_bone(arm, "forearm.L", (R(12), 0, 0))


def apply_wave_left(arm, t):
    """Mirror wave with the left arm (mirrored calibrated angles)."""
    reset_pose(arm)
    set_bone(arm, "upper_arm.L", (-0.15 * math.sin(t * 5.0), 0, R(90)))
    set_bone(arm, "forearm.L", (0, 0, R(90)))
    set_bone(arm, "hand.L", (0, 0, 0.45 * math.sin(t * 6.0)))
    set_bone(arm, "upper_arm.R", (R(8), 0, R(-6)))
    set_bone(arm, "forearm.R", (R(12), 0, 0))


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
