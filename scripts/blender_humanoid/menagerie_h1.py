#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""menagerie_h1.py -- Unitree H1 (MuJoCo Menagerie) backend for the physics twin.

P2 extension (2026-09-02): besides our chibi aBot (assets/humanoid/
humanoid.mjcf), the physics twin can also load a COMPLETE, REAL humanoid --
the Unitree H1 from DeepMind's MuJoCo Menagerie (assets/menagerie/
unitree_h1/, fetched by scripts/fetch_menagerie.py; model content licensed
BSD-3-Clause by Unitree Robotics, see assets/menagerie/unitree_h1/LICENSE).

H1Adapter implements the SAME command surface as
physics_adapter.PhysicsAdapter, so physics_twin_server.py can serve it with
zero contract changes:

    python scripts/blender_humanoid/physics_twin_server.py --model unitree_h1

Design (physically honest, joint-level only -- no hidden forces)
----------------------------------------------------------------
* The Menagerie H1 ships 19 torque-controlled `motor` actuators. At load
  time they are converted IN MEMORY (XML untouched) into native MuJoCo
  `position` actuators: force = kp*(ctrl - q) - kd*dq, computed by MuJoCo at
  every integration substep, with the original motor ctrlrange kept as the
  torque forcerange (realistic effort limits). This is the same servo-rate
  position-feedback architecture as the chibi MJCF -- and it is REQUIRED:
  a 30 Hz external torque loop with these gains injects energy and falls
  within ~1 s (measured repeatedly during bring-up).
* The twin-control 30 Hz layer modulates position TARGETS only:
    - pose / motion / raw-FK targets (see MOTIONS below)
    - H1StandController: a small closed-loop stand holder superimposed on
      the leg targets every tick -- pelvis-tilt proportional feedback
      (ankle + hip pitch + hip roll channels) plus whole-body-COM velocity
      braking and COM-over-CoP ankle regulation (drift-free standing).
      Signs were measured in-sim (constant-target probes), not guessed.
* /stop and auto-reseat snap back onto the Menagerie `home` keyframe
  (slightly crouched, feet under the hips), exactly like the chibi twin.

Measured capability envelope (external view, headless; see
tests/acceptance/test_h1_model.py):
    * standing (idle): stable indefinitely (60 s tested), up_z ~= 0.999,
      drift < 2 cm; survives the 3 cm keyframe drop start and upper-body
      motion while standing
    * wave (right arm) and look (torso yaw): playable while standing
    * push recovery: fore +150 N*0.1 s recovered, lateral +/-250 N*0.1 s
      recovered; BACKWARD pushes are NOT recoverable (the Menagerie foot
      has a ~3.5 cm heel only) -- the robot falls and is re-seated, honest
      physics
    * walk / run / nod: NOT supported (H1 has no neck joint; dynamic gaits
      need a trained whole-body policy -- future work)

Joint names (the raw-FK names of this backend, also in GET /state):
    left_hip_yaw   left_hip_roll  left_hip_pitch  left_knee   left_ankle
    right_hip_yaw  right_hip_roll right_hip_pitch right_knee  right_ankle
    torso
    left_shoulder_pitch  left_shoulder_roll  left_shoulder_yaw  left_elbow
    right_shoulder_pitch right_shoulder_roll right_shoulder_yaw right_elbow
"""

import math
import os

import numpy as np
import mujoco

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(_HERE))
H1_DIR = os.path.join(ROOT, "assets", "menagerie", "unitree_h1")
H1_SCENE = os.path.join(H1_DIR, "scene.xml")

TICK = 1.0 / 30.0          # twin-control 30 fps drive loop (contract tempo)
FALL_UP_Z = 0.6            # below this the robot is considered down

MODE_OFF = "off"
MODE_STAND = "stand"
MODE_GAIT = "gait"         # accepted for contract parity; H1 treats as stand

# position-servo gains for the converted actuators (tuned in-sim 2026-09-02;
# see probe history in the robot-modeler LTM). Strong legs to carry the 47 kg
# frame with small sag; torque limits (forcerange) stay the Menagerie ones.
KP = {"hip": 400.0, "knee": 600.0, "ankle": 200.0, "torso": 200.0,
      "shoulder": 100.0, "elbow": 50.0}
KD = {"hip": 8.0, "knee": 8.0, "ankle": 4.0, "torso": 4.0,
      "shoulder": 2.0, "elbow": 2.0}

# stand-holder gains (measured signs; see module docstring). Channels:
#   ankle  += kt*e_pitch + kv*com_vx + k1*(com_x - cop_x)
#   hip_p  -= kh*e_pitch          hip_r += kr*e_roll
# e_pitch = pelvis up-vector X component (lean toward facing direction +X),
# e_roll  = pelvis up-vector Y component (lean toward the robot's left).
STAND_GAINS = {"kt": 2.0, "kv": 1.5, "k1": 1.0, "kh": 0.6, "kr": 0.8}

# contract poses/motions supported by this backend (honest capability list)
POSES = ("relax", "tpose", "apose")
MOTIONS_SUPPORTED = ("idle", "wave", "look")
MOTIONS_UNSUPPORTED = ("walk", "run", "nod")  # need a trained policy / no neck

# arm abduction targets for the contract poses (radians of shoulder_roll;
# left arm abducts positive, right arm negative per the joint ranges)
POSE_ARMS = {
    "relax": {"left_shoulder_roll": 0.0, "right_shoulder_roll": 0.0},
    "apose": {"left_shoulder_roll": 0.5, "right_shoulder_roll": -0.5,
              "left_shoulder_pitch": -0.1, "right_shoulder_pitch": -0.1},
    "tpose": {"left_shoulder_roll": 1.3, "right_shoulder_roll": -1.3,
              "left_shoulder_pitch": -0.15, "right_shoulder_pitch": -0.15},
}

# /perturb body-name aliases so the chibi-shaped contract default body still
# works on the H1 ("chest" does not exist in the Menagerie model)
BODY_ALIASES = {"chest": "torso_link", "root": "pelvis", "head": "torso_link",
                "spine": "torso_link"}


def _group(name):
    """KP/KD group for a joint name."""
    if "hip" in name:
        return "hip"
    if "knee" in name:
        return "knee"
    if "ankle" in name:
        return "ankle"
    if name == "torso":
        return "torso"
    if "shoulder" in name:
        return "shoulder"
    return "elbow"


class H1StandController(object):
    """Closed-loop stand holder for the H1 (see module docstring).

    Public surface mirrors balance_controller.BalanceController where the
    physics_twin_server touches it: telemetry()/config()/configure()/
    set_mode()/reset_state()/request_push()/auto_reseat, so the server is
    model-agnostic. The controller modulates position TARGETS (slow 30 Hz
    loop); the fast servo is MuJoCo's position actuator.
    """

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.mode = MODE_STAND
        self.enabled = True
        self.auto_reseat = True
        self.gains = dict(STAND_GAINS)
        self._pelvis = model.body("pelvis").id
        self._torso = model.body("torso_link").id
        self._floor = model.geom("floor").id
        self._ank_bodies = {model.body("left_ankle_link").id,
                            model.body("right_ankle_link").id}
        self._ank_side = {}
        for b, name in ((model.body("left_ankle_link").id, "L"),
                        (model.body("right_ankle_link").id, "R")):
            self._ank_side[b] = name
        self.push = None
        self.telemetry_store = {}
        self._com_filt = 0.0      # low-passed COM velocity (finite difference)
        self.max_delta = 0.0

    # -- configuration (contract-compatible surface) -------------------------
    def set_mode(self, mode):
        if mode not in (MODE_OFF, MODE_STAND, MODE_GAIT):
            return False
        # H1 has no gait controller: GAIT degrades to STAND (honest)
        self.mode = MODE_STAND if mode == MODE_GAIT else mode
        return True

    def configure(self, patch):
        if "mode" in patch:
            self.set_mode(patch["mode"])
        if "enabled" in patch:
            self.enabled = bool(patch["enabled"])
        if "auto_reseat" in patch:
            self.auto_reseat = bool(patch["auto_reseat"])
        if "assist_gain" in patch:
            # explicitly unsupported on H1 (joint-only balance, no hidden
            # forces); accepted-and-ignored keeps the contract patch shape
            pass
        return self.config()

    def config(self):
        return {"mode": self.mode, "enabled": self.enabled,
                "gait_scale": 1.0, "gait_tempo": 1.0, "assist_gain": 0.0,
                "auto_reseat": self.auto_reseat, "model": "unitree_h1",
                "gains": dict(self.gains)}

    def reset_state(self):
        if self.push is not None:
            self.data.xfrc_applied[self.push["body_id"]][:3] = 0.0
        self.push = None
        self._com_filt = 0.0
        self.max_delta = 0.0

    # -- perturbations (real external forces, honest physics) ----------------
    def request_push(self, force, duration, body_name="torso_link"):
        name = BODY_ALIASES.get(body_name, body_name)
        try:
            body_id = self.model.body(name).id
        except KeyError:
            return False
        force = [float(force[0]), float(force[1]),
                 float(force[2]) if len(force) > 2 else 0.0]
        self.push = {"body_id": body_id, "force": tuple(force),
                     "until": float(self.data.time) + max(0.0, float(duration))}
        return True

    # -- sensing ---------------------------------------------------------------
    def tilt(self):
        """(e_pitch, e_roll): pelvis up-vector components. e_pitch>0 ==
        leaning toward the facing direction (+X); e_roll>0 == leaning toward
        the robot's left (+Y)."""
        xmat = self.data.xmat[self._pelvis].reshape(3, 3)
        return float(xmat[0][2]), float(xmat[1][2])

    def up_z(self):
        return float(self.data.xmat[self._pelvis].reshape(3, 3)[2][2])

    def cop_x(self):
        """Force-weighted X centroid of floor-foot contacts (world)."""
        fx = fsum = 0.0
        out = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.geom1 != self._floor and c.geom2 != self._floor:
                continue
            b1 = int(self.model.geom_bodyid[c.geom1])
            b2 = int(self.model.geom_bodyid[c.geom2])
            if b1 not in self._ank_bodies and b2 not in self._ank_bodies:
                continue
            mujoco.mj_contactForce(self.model, self.data, i, out)
            f = abs(float(out[0]))
            fx += f * float(c.pos[0])
            fsum += f
        return fx / fsum if fsum > 5.0 else None

    def foot_loads(self):
        """(N_L, N_R) summed floor-contact normal forces per foot."""
        loads = {"L": 0.0, "R": 0.0}
        out = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.geom1 != self._floor and c.geom2 != self._floor:
                continue
            for b in (int(self.model.geom_bodyid[c.geom1]),
                      int(self.model.geom_bodyid[c.geom2])):
                if b in self._ank_bodies:
                    mujoco.mj_contactForce(self.model, self.data, i, out)
                    loads[self._ank_side[b]] += abs(float(out[0]))
                    break
        return loads["L"], loads["R"]

    # -- control ----------------------------------------------------------------
    def target_deltas(self):
        """Balance target-deltas per joint name for this tick (radians).

        Must be called AFTER the motion/base targets were composed. Returns
        {} when disabled, off, or already down (no flailing).
        """
        self.max_delta = 0.0
        data = self.data

        # external perturbation forces (real physics: xfrc_applied)
        if self.push is not None:
            if float(data.time) < self.push["until"]:
                data.xfrc_applied[self.push["body_id"]][:3] = self.push["force"]
            else:
                data.xfrc_applied[self.push["body_id"]][:3] = 0.0
                self.push = None

        up_z = self.up_z()
        e_p, e_r = self.tilt()
        if not self.enabled or self.mode == MODE_OFF or up_z < FALL_UP_Z:
            return {}

        g = self.gains
        com_x = float(data.subtree_com[0][0])
        # COM velocity by finite difference (data.subtree_linvel is not
        # populated by mj_forward in MuJoCo 3.12 unless explicitly requested)
        com_v = ((com_x - self._com_prev) / TICK) \
            if hasattr(self, "_com_prev") else 0.0
        self._com_prev = com_x
        self._com_filt = 0.7 * self._com_filt + 0.3 * com_v

        d_ank = g["kt"] * e_p + g["kv"] * self._com_filt
        cop = self.cop_x()
        if cop is not None:
            d_ank += g["k1"] * (com_x - cop)
        d_hip = g["kh"] * e_p
        d_roll = g["kr"] * e_r

        deltas = {"left_ankle": d_ank, "right_ankle": d_ank,
                  "left_hip_pitch": -d_hip, "right_hip_pitch": -d_hip,
                  "left_hip_roll": d_roll, "right_hip_roll": d_roll}
        self.max_delta = max(abs(v) for v in deltas.values())
        # clamp: balance may not command more than ~0.35 rad of the servos
        for k in deltas:
            deltas[k] = max(-0.35, min(0.35, deltas[k]))
        self._last_err = (e_p, e_r)
        return deltas

    # -- telemetry ----------------------------------------------------------------
    def telemetry(self):
        data = self.data
        try:
            up_z = self.up_z()
            e_p, e_r = getattr(self, "_last_err", self.tilt())
            cvel = data.cvel[self._pelvis]
            n_l, n_r = self.foot_loads()
        except Exception:  # noqa: BLE001 - telemetry must never break serving
            return {"mode": self.mode, "enabled": self.enabled, "fell": False}
        return {
            "mode": self.mode,
            "enabled": self.enabled,
            "fell": up_z < FALL_UP_Z,
            "tilt_pitch": round(e_p, 4),
            "tilt_roll": round(e_r, 4),
            "com_pitch_err": 0.0,
            "com_roll_err": 0.0,
            "omega_pitch": round(float(cvel[1]), 3),
            "omega_roll": round(float(cvel[0]), 3),
            "foot_load_L_N": round(n_l, 1),
            "foot_load_R_N": round(n_r, 1),
            "assist_N": [0.0, 0.0],   # H1 balance is joint-only (no assist)
            "max_delta_rad": round(self.max_delta, 4),
            "gait_scale": 1.0,
            "gait_tempo": 1.0,
            "push_remaining_s": round(
                self.push["until"] - float(data.time), 3) if self.push else 0.0,
        }


class H1Adapter(object):
    """Unitree H1 backend implementing the twin-control command surface
    (drop-in alternative to physics_adapter.PhysicsAdapter)."""

    def __init__(self, scene_path=None):
        self.mjcf_path = scene_path or H1_SCENE
        if not os.path.isfile(self.mjcf_path):
            raise FileNotFoundError(
                "%s not found -- fetch it with: "
                "python scripts/fetch_menagerie.py --model unitree_h1"
                % self.mjcf_path)
        self.model = mujoco.MjModel.from_xml_path(self.mjcf_path)
        self._to_position_actuators()
        self.data = mujoco.MjData(self.model)

        # joint bookkeeping
        self.joint_names = []
        self._jnt_qpos = {}
        for a in range(self.model.nu):
            jntid = int(self.model.actuator_trnid[a][0])
            jnt = self.model.joint(jntid)
            self.joint_names.append(jnt.name)
            self._jnt_qpos[jnt.name] = int(jnt.qposadr[0])
        self._act_of_joint = {n: a for a, n in enumerate(self.joint_names)}
        self._jnt_range = {}
        for n in self.joint_names:
            jntid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            self._jnt_range[n] = (float(self.model.jnt_range[jntid][0]),
                                  float(self.model.jnt_range[jntid][1]))

        # home keyframe -> standing joint targets
        self._home_key = mujoco.mj_name2id(self.model,
                                           mujoco.mjtObj.mjOBJ_KEY, "home")
        self.stand_targets = {n: 0.0 for n in self.joint_names}
        for n in self.joint_names:
            self.stand_targets[n] = float(
                self.model.key_qpos[self._home_key][self._jnt_qpos[n]])

        self.motion_state = {"name": None, "start": 0.0, "duration": 0.0}
        self.balance = H1StandController(self.model, self.data)
        self.reset(settle_seconds=0.8)

    # -- actuator conversion (runtime model patch; XML stays untouched) -------
    def _to_position_actuators(self):
        """Convert Menagerie `motor` actuators to native position servos.

        force = kp*(ctrl - q) - kd*dq, evaluated by MuJoCo at every
        integration substep (servo-rate feedback -- essential for stability).
        The original motor ctrlrange is preserved as the torque forcerange;
        ctrlrange becomes the joint range (position-target clamp).
        """
        m = self.model
        for a in range(m.nu):
            name = m.actuator(a).name
            torque_lo, torque_hi = (float(m.actuator_ctrlrange[a][0]),
                                    float(m.actuator_ctrlrange[a][1]))
            jntid = int(m.actuator_trnid[a][0])
            lo, hi = (float(m.jnt_range[jntid][0]),
                      float(m.jnt_range[jntid][1]))
            g = _group(name)
            kp, kd = KP[g], KD[g]
            m.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED
            m.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_AFFINE
            m.actuator_gainprm[a] = [kp, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            m.actuator_biasprm[a] = [0, -kp, -kd, 0, 0, 0, 0, 0, 0, 0]
            m.actuator_ctrlrange[a] = [lo, hi]
            m.actuator_ctrllimited[a] = 1
            m.actuator_forcerange[a] = [torque_lo, torque_hi]
            m.actuator_forcelimited[a] = 1

    # -- command surface (contract semantics identical to the chibi adapter) ---
    def apply_pose(self, name):
        """Static standing poses: relax/apose/tpose differ by arm abduction."""
        if name not in POSES:
            return False
        self.motion_state["name"] = None
        self._pose_targets = dict(POSE_ARMS[name])
        return True

    def start_motion(self, name, duration=3.0):
        """Timed motions: idle (stand), wave (right arm), look (torso yaw).
        walk/run/nod are NOT supported on H1 (returns False; honest)."""
        if name not in MOTIONS_SUPPORTED:
            return False
        self.motion_state.update(name=name, start=self.data.time,
                                 duration=max(0.0, float(duration)))
        return True

    def drive_bones(self, bones):
        """Raw FK drive by H1 joint names: {joint: angle_rad} (a contract
        3-vector is tolerated, its first component is used)."""
        for n in bones:
            if n not in self._act_of_joint:
                return False
        self.motion_state["name"] = None
        self._pose_targets = {}
        for n, v in bones.items():
            angle = float(v[0]) if isinstance(v, (list, tuple)) else float(v)
            self._pose_targets[n] = angle
        return True

    def stop(self):
        """Stop any motion; targets relax to the standing pose."""
        self.motion_state["name"] = None
        self._pose_targets = {}
        self.balance.reset_state()

    def perturb(self, force, duration=0.1, body="torso_link"):
        """Apply a real external force pulse (world frame, xfrc_applied)."""
        return self.balance.request_push(force, duration, body)

    # -- drive -------------------------------------------------------------------
    def reset(self, settle_seconds=0.8):
        """Re-seat onto the Menagerie `home` keyframe (kinematic snap)."""
        self.stop()
        self._pose_targets = {}
        self.data.qpos[:] = self.model.key_qpos[self._home_key]
        self.data.qvel[:] = 0.0
        self.data.xfrc_applied[:] = 0.0
        self.data.ctrl[:] = [self.stand_targets[n] for n in self.joint_names]
        mujoco.mj_forward(self.model, self.data)
        self.balance._com_prev = float(self.data.subtree_com[0][0])
        self.settle(seconds=settle_seconds)

    def settle(self, seconds=0.5):
        end = self.data.time + seconds
        while self.data.time < end:
            self.drive_once()

    def drive_once(self):
        """One 30 fps tick: compose targets, overlay balance, step physics."""
        targets = self._compose_targets()
        if self.balance.enabled and self.balance.mode != MODE_OFF:
            for n, d in self.balance.target_deltas().items():
                targets[n] = targets.get(n, 0.0) + d
        for n in self.joint_names:
            lo, hi = self._jnt_range[n]
            self.data.ctrl[self._act_of_joint[n]] = max(lo, min(hi, targets[n]))
        t_end = self.data.time + TICK
        while self.data.time < t_end - 1e-9:
            mujoco.mj_step(self.model, self.data)

    def _compose_targets(self):
        """Base targets: standing pose + active pose offsets + motion pattern."""
        targets = dict(self.stand_targets)
        for n, v in getattr(self, "_pose_targets", {}).items():
            if n in targets:
                targets[n] = v
        name = self.motion_state["name"]
        if name is not None:
            t = self.data.time - self.motion_state["start"]
            dur = self.motion_state["duration"]
            if dur > 0 and t > dur:
                self.motion_state["name"] = None
                name = None
        if name == "idle":
            # subtle signs of life (small enough for the stand holder)
            targets["torso"] += 0.05 * math.sin(2.0 * math.pi * 0.4 *
                                                self.data.time)
            targets["left_shoulder_pitch"] += 0.04 * math.sin(
                2.0 * math.pi * 0.3 * self.data.time)
            targets["right_shoulder_pitch"] += 0.04 * math.sin(
                2.0 * math.pi * 0.3 * self.data.time)
        elif name == "wave":
            # raise the anatomical right arm and swing the forearm
            targets["right_shoulder_roll"] = -2.2
            targets["right_shoulder_pitch"] = -0.15
            targets["right_elbow"] = 1.4 + 0.7 * math.sin(
                2.0 * math.pi * 1.2 * self.data.time)
        elif name == "look":
            # H1 has no neck: horizontal look = torso yaw sweep
            targets["torso"] = 0.5 * math.sin(2.0 * math.pi * 0.3 *
                                              self.data.time)
        return targets

    # -- uplink --------------------------------------------------------------------
    def state(self):
        """Contract-shaped state + physics extras (root = pelvis body,
        up axis = pelvis local +Z, facing +X)."""
        bones = {}
        for n in self.joint_names:
            bones[n] = [round(float(self.data.qpos[self._jnt_qpos[n]]), 4),
                        0.0, 0.0]
        xmat = self.data.xmat[self.balance._pelvis].reshape(3, 3)
        return {
            "bones": bones,
            "motion": self.motion_state["name"],
            "sim_time": round(float(self.data.time), 4),
            "root": {
                "pos": [round(float(v), 4) for v in self.data.qpos[0:3]],
                "height": round(float(self.data.xpos[self.balance._pelvis][2]),
                                4),
                "up_z": round(float(xmat[2][2]), 4),
            },
            "root_loc_target": [0.0, 0.0, 0.0],
            "max_qvel": round(float(max(abs(v) for v in self.data.qvel)), 4),
            "contacts": [],
            "balance": self.balance.telemetry(),
        }

    def health(self):
        return {"status": "ok", "backend": "sim", "model": self.mjcf_path,
                "model_name": "unitree_h1",
                "bones": len(self.joint_names),
                "poses": list(POSES),
                "motions_supported": list(MOTIONS_SUPPORTED),
                "motions_unsupported": list(MOTIONS_UNSUPPORTED),
                "note": "walk/run/nod need a trained whole-body policy "
                        "(future work); backward pushes exceed the heel "
                        "margin and topple the robot (honest physics)"}


# ---------------------------------------------------------------------------
# CLI smoke: standing + supported motions under physics (headless)
# ---------------------------------------------------------------------------
def smoke(duration=4.0):
    import json
    print("== menagerie_h1 smoke (MuJoCo backend, Unitree H1) ==")
    ap = H1Adapter()
    print("health:", ap.health())
    st = ap.state()
    standing_ok = st["root"]["up_z"] > 0.97 and st["root"]["height"] > 0.6
    print("STANDING after reset+settle: up_z=%.4f h=%.4f -> %s"
          % (st["root"]["up_z"], st["root"]["height"],
             "OK" if standing_ok else "FAIL"))
    facts = {"standing": standing_ok, "motions": {}}
    for name in MOTIONS_SUPPORTED:
        ap.reset()
        ok = ap.start_motion(name, duration)
        n_ticks = int(round(duration / TICK))
        min_up = 1.0
        crashed = False
        for _ in range(n_ticks):
            ap.drive_once()
            bad = any(math.isnan(v) or math.isinf(v) for v in ap.data.qpos)
            if bad:
                crashed = True
                break
            min_up = min(min_up, ap.state()["root"]["up_z"])
        s = ap.state()
        facts["motions"][name] = {"accepted": ok, "crashed": crashed,
                                  "min_up": round(min_up, 4),
                                  "up_z": s["root"]["up_z"]}
        print("MOTION %-6s accepted=%s crashed=%s min_up=%.3f up_z=%.3f"
              % (name, ok, crashed, min_up, s["root"]["up_z"]))
    all_ok = standing_ok and all(
        m["accepted"] and not m["crashed"] and m["min_up"] > 0.9
        for m in facts["motions"].values())
    print("SMOKE_JSON " + json.dumps(facts))
    print("SMOKE %s" % ("PASS" if all_ok else "FAIL"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    import sys
    raise SystemExit(smoke())
