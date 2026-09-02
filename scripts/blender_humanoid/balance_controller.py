#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""balance_controller.py -- closed-loop balance overlay for the MuJoCo twin.

P2 milestone T4 (2026-09-02): the physics twin stops being a purely open-loop
playback. This module is a low-level feedback layer that sits UNDER the
twin-control contract: motions/poses/FK keep arriving exactly as before, the
balance controller superimposes joint-level corrections every 30 fps tick so
the robot can stand up to perturbations and (with gait modulation) walk
without immediately tipping over. The contract surface is unchanged; balance
is observable via state()["balance"] telemetry.

Strategy (physically honest scope)
----------------------------------
* JOINT-LEVEL feedback only by default -- no hidden forces:
  - ANKLE STRATEGY: PD on the foot joints, driven by trunk tilt error and
    trunk angular velocity (the classical inverted-pendulum ankle strategy).
  - HIP/TRUNK STRATEGY: PD on thigh + spine/chest joints moves the upper
    body mass against the tilt (fast, strong for this low-COM chibi).
  - Corrections are DISTRIBUTED over candidate joints by projecting each
    joint's current world-space axis onto the world pitch axis (world X) and
    roll axis (world Y) -- no hard-coded axis guesses; the mapping adapts as
    the robot moves.
  - ROLL is stance-aware: each foot's ankle-roll correction is weighted by
    that foot's measured floor normal force (a swing foot cannot push
    against the ground), which matters during single-support phases of gait.
* GAIT MODE (walk/run) adds a documented gait-envelope governor:
  gait_scale attenuates the open-loop leg swing amplitude and gait_tempo
  time-warps the gait clock, keeping the commanded pattern inside the
  robot's balance envelope. This is honest physics: the full-amplitude
  Blender gait exceeds this robot's actuation/balance envelope, so the
  physics twin plays a scaled variant and reports the scale in telemetry.
* OPTIONAL VIRTUAL BALANCE ASSIST (default OFF, explicitly labeled): a
  world-space force on the floating root proportional to trunk tilt, via
  qfrc_applied. It emulates an invisible balancing hand / base reaction
  force. When enabled it is reported in telemetry (assist_N) so nobody can
  mistake assisted balance for joint-only balance.
* PERTURBATIONS are real external forces (data.xfrc_applied on a body), so
  push-recovery experiments measure genuine closed-loop dynamics.

Known capability boundaries (measured, see balance_tune.py):
  - Open loop the robot passively survives small pushes (low COM 0.30 m,
    wide feet) but falls for lateral impulses above ~15-20 N*s and backward
    pushes above ~10 N*s (short heels). The closed loop extends that
    envelope; the achieved numbers are reported by balance_tune.py and the
    acceptance test.
  - Self-righting after a fall is NOT dynamic: the server auto-reseats the
    robot onto its home keyframe (kinematic snap, same semantics as /stop).
    Dynamic get-up from lying is future work.

Usage: owned by PhysicsAdapter; called once per tick AFTER the motion targets
are written to data.ctrl:

    self.balance.apply()   # adjusts data.ctrl (+ xfrc/qfrc when enabled)
"""

import copy
import math

import numpy as np

import mujoco

MODE_OFF = "off"
MODE_STAND = "stand"
MODE_GAIT = "gait"

# joints that never receive balance corrections (floating root + head/neck/
# arms: keeping them faithful to the motion targets preserves the contract
# motions' readability and avoids fighting the baked choreography)
_ANKLE_BONES = ("foot.L", "foot.R")
_HIP_BONES = ("thigh.L", "thigh.R")
_TRUNK_BONES = ("spine", "chest")
# only pitch (x) and roll-ish (y/z) axes are used; .y of the vertical chain
# is yaw and gets ~zero projection anyway, but skip it explicitly
_BALANCE_AXES = ("x", "y", "z")

# PD gains per (group, channel). SIGNS ARE GROUND TRUTH, measured directly in
# MuJoCo as each actuator's effect on the root (pelvis) tilt -- the balance
# variable (probe: constant +-0.1 rad ctrl delta, observe d(e_pitch)/d(e_roll)):
#   foot.x  +d -> e_pitch -0.102   (recovers forward lean)  => sign +1
#   thigh.x +d -> e_pitch -0.130   (recovers forward lean)  => sign +1
#   spine.x +d -> e_pitch +0.018   (worsens forward lean)   => sign -1
#   foot.y  +d -> e_roll  -0.019   (recovers +X lean)       => sign -1
#     (foot.y world axis ~=-Y, so the axis projection carries a -1; the net
#      sign that recovers +X lean through the projection is -1)
#   thigh.z +d -> e_roll  -0.024   (recovers +X lean)       => sign +1
#   spine.z +d -> e_roll  ~0       (weak; shifts COM only)  => sign -1
# DERIVATIVE TERM: kd is intentionally ZERO in v1. Velocity feedback through
# the compliant foot contact injects energy: ankle/hip damping flipped
# backward-push recovery into a fall and trunk damping destabilized quiet
# standing (probed both signs). Filtered/COM-velocity damping is future work.
# TUNED (config B, 2026-09-02): low proportional gains are essential -- high
# gains fight upper-body motions and pump energy; low gains keep quiet/wave/
# idle/look standing AND recover moderate pushes. Root(pelvis)-sensed tilt
# signs: ankle/hip pitch +1, ankle roll -1, hip roll +1, trunk pitch +1,
# trunk roll -1 (validated behaviorally, see probe notes above + session mem).
STAND_GAINS = {
    "ankle_pitch": {"kp": 1.0, "kd": 0.0, "sign": +1.0, "max_delta": 0.40},
    "ankle_roll":  {"kp": 1.0, "kd": 0.0, "sign": -1.0, "max_delta": 0.35},
    "hip_pitch":   {"kp": 0.5, "kd": 0.0, "sign": +1.0, "max_delta": 0.30},
    "hip_roll":    {"kp": 0.5, "kd": 0.0, "sign": +1.0, "max_delta": 0.25},
    "trunk_pitch": {"kp": 0.8, "kd": 0.0, "sign": +1.0, "max_delta": 0.35},
    "trunk_roll":  {"kp": 0.6, "kd": 0.0, "sign": -1.0, "max_delta": 0.30},
}
# gait mode: stronger roll authority (single-support phases); pitch unchanged
GAIT_GAINS = dict(STAND_GAINS)
GAIT_GAINS.update({
    "ankle_roll": {"kp": 1.4, "kd": 0.0, "sign": -1.0, "max_delta": 0.40},
    "hip_roll":   {"kp": 0.7, "kd": 0.0, "sign": +1.0, "max_delta": 0.30},
    "trunk_roll": {"kp": 0.9, "kd": 0.0, "sign": -1.0, "max_delta": 0.35},
})

FALL_UP_Z = 0.6          # below this the robot is considered already down
ASSIST_FRACTION_CAP = 0.5  # virtual assist capped at 50% of body weight

# COM-position compensation gain (ankle channel): shifts the balance
# equilibrium so the whole-body COM stays over the load-weighted ankle
# (pivot of the ankle-strategy inverted pendulum) instead of forcing the
# pelvis perfectly vertical. Without it, any asymmetric pose (e.g. wave
# raises one arm) drifts the COM out of the support polygon even while the
# pelvis reads "upright". Kept moderate: too-high gain saturates the ankle
# servos and topples the robot (probed 2026-09-02).
COM_COMP_GAIN = 2.0      # rad of ankle correction per meter of COM offset
# fore-aft COM regulation at the HIP (routed separately; see apply). The hip
# is a strong pitch actuator (thigh.x) that arrests slow backward COM drift
# (e.g. induced by nod) which the ankle alone cannot stop before saturating.
COM_HIP_GAIN = 0.0       # rad of hip correction per meter of COM offset

# In GAIT mode the roll-channel proportional gain is boosted by this factor
# (single-support phases need more lateral authority). Applied as a
# multiplier in apply() so tuned per-instance gains are never clobbered.
GAIT_ROLL_BOOST = 1.4

# Tuned gait-envelope defaults for WALK (balance_tune.py, 2026-09-02): the
# full-amplitude Blender gait (scale=tempo=1) tips this robot within ~0.6 s
# even with balance feedback. gait_scale attenuates the leg-swing amplitude
# and gait_tempo slows the cadence, keeping the commanded pattern inside the
# robot's balance envelope; at (0.3, 0.5) the physics twin walks in place for
# 12+ s without falling (open loop: 0.6 s). This is a documented, telemetry-
# visible deviation -- honest physics, not magic. RUN needs even more care
# (see physics_adapter / balance_tune).
GAIT_SCALE_DEFAULT = 0.3
GAIT_TEMPO_DEFAULT = 0.5


class BalanceController(object):
    """Joint-level PD balance overlay (see module docstring for scope)."""

    def __init__(self, model, data, mode=MODE_STAND):
        self.model = model
        self.data = data
        self.mode = mode if mode in (MODE_OFF, MODE_STAND, MODE_GAIT) else MODE_STAND
        self.enabled = True                     # master switch
        # deep-copy the gain table so each instance owns private, mutable
        # gains (the module constants stay pristine and deterministic).
        # Gait mode boosts the roll-channel kp via GAIT_ROLL_BOOST in apply().
        self.gains = copy.deepcopy(STAND_GAINS)
        self.gait_scale = GAIT_SCALE_DEFAULT     # leg-swing amplitude scale
        self.gait_tempo = GAIT_TEMPO_DEFAULT     # gait clock time warp
        self.assist_gain = 0.0                  # 0 = joint-only (honest)
        self.auto_reseat = True                 # server-level fallen recovery

        # -- joint / actuator bookkeeping ------------------------------------
        self._jnt = {}          # joint name -> dict(body_id, axis, dof, ctrl)
        self._ctrlrange = {}
        for j in range(model.njnt):
            jnt = model.joint(j)
            self._jnt[jnt.name] = {
                "body_id": int(model.jnt_bodyid[j]),
                "axis": tuple(float(v) for v in model.jnt_axis[j]),
                "dof": int(model.jnt_dofadr[j]),
                "ctrl": None,
            }
        for a in range(model.nu):
            aname = model.actuator(a).name
            if not aname.startswith("servo_"):
                continue
            jname = aname[len("servo_"):]
            if jname in self._jnt:
                self._jnt[jname]["ctrl"] = a
                self._ctrlrange[jname] = (float(model.actuator_ctrlrange[a][0]),
                                          float(model.actuator_ctrlrange[a][1]))
        self._free_dof = int(model.jnt_dofadr[model.joint("root.free").id])
        self._chest_id = model.body("chest").id
        self._root_id = model.body("root").id
        self._foot_body = {"L": model.body("foot.L").id,
                           "R": model.body("foot.R").id}
        self._floor_geom = model.geom("floor").id
        self.com_gain = COM_COMP_GAIN
        # fore-aft COM regulation also routed to the hip (strong pitch
        # actuator) -- needed to arrest slow backward COM drift (e.g. nod);
        # the ankle alone saturates. See apply().
        self.com_hip_gain = COM_HIP_GAIN
        # low-pass state for the COM-over-CoP error (first-order IIR). The raw
        # force-weighted contact centroid jitters a few cm as corner contact
        # forces shift; unfiltered, that jitter closes a positive feedback loop
        # that saturates the ankle servos and topples the robot. Filtering
        # keeps the slow COM-drift correction while rejecting contact noise.
        self._com_filt = None          # (pitch_err, roll_err) smoothed
        self.com_filt_alpha = 0.12     # per-tick smoothing (small = smoother)
        # low-pass state for the tilt error. Fast upper-body choreography
        # (nod rocks the trunk at ~2.5 rad/s, wave at ~5 rad/s) is NOT a fall;
        # correcting it injects energy. Filtering the tilt below the balance
        # mode (~1-2 rad/s) keeps genuine fall recovery while ignoring motion-
        # reaction rocking. Derivative (velocity) feedback is NOT used -- it
        # destabilizes through the compliant foot contact (probed repeatedly).
        self._tilt_filt = None         # (e_pitch, e_roll) smoothed
        # NOTE: filtering the tilt HURTS nod (its 2.5 rad/s rocking sits near
        # the balance band; filter phase lag turns the correction into pumping,
        # probed 2026-09-02). Default alpha=1.0 disables the filter. Kept as a
        # tuning hook for future work.
        self.tilt_filt_alpha = 1.0     # 1.0 = no filtering
        self._foot_geoms = {side: set() for side in ("L", "R")}
        for g in range(model.ngeom):
            gname = model.geom(g).name
            for side in ("L", "R"):
                if gname.startswith("foot.%s_c" % side):
                    self._foot_geoms[side].add(g)
        self._mass = float(sum(model.body_mass))

        # -- runtime state -----------------------------------------------------
        self.push = None          # {"body_id":, "force":(x,y,z), "until": t}
        self.telemetry_store = {}

    # -- configuration ---------------------------------------------------------
    def set_mode(self, mode):
        """Switch balance mode WITHOUT resetting gains (gains are tuned per
        instance; the gait-mode roll boost is applied as a multiplier in
        apply(), not by swapping gain tables)."""
        if mode not in (MODE_OFF, MODE_STAND, MODE_GAIT):
            return False
        self.mode = mode
        return True

    def configure(self, patch):
        """Apply a dict patch: mode/gait_scale/gait_tempo/assist_gain/
        auto_reseat/enabled. Returns the resulting config (external view)."""
        if "mode" in patch:
            self.set_mode(patch["mode"])
        for key in ("gait_scale", "gait_tempo", "assist_gain"):
            if key in patch:
                try:
                    setattr(self, key, max(0.0, float(patch[key])))
                except (TypeError, ValueError):
                    pass
        if "auto_reseat" in patch:
            self.auto_reseat = bool(patch["auto_reseat"])
        if "enabled" in patch:
            self.enabled = bool(patch["enabled"])
        return self.config()

    def config(self):
        return {"mode": self.mode, "enabled": self.enabled,
                "gait_scale": round(self.gait_scale, 3),
                "gait_tempo": round(self.gait_tempo, 3),
                "assist_gain": round(self.assist_gain, 3),
                "auto_reseat": self.auto_reseat}

    def reset_state(self):
        """Clear transient state (called on adapter reset/stop)."""
        if self.push is not None:
            self.data.xfrc_applied[self.push["body_id"]][:3] = 0.0
        self.push = None
        self.data.qfrc_applied[self._free_dof:self._free_dof + 3] = 0.0
        self._com_filt = None
        self._tilt_filt = None

    # -- perturbations (real external forces) -----------------------------------
    def request_push(self, force, duration, body_name="chest"):
        try:
            body_id = self.model.body(body_name).id
        except KeyError:
            return False
        force = [float(force[0]), float(force[1]),
                 float(force[2]) if len(force) > 2 else 0.0]
        self.push = {"body_id": body_id, "force": tuple(force),
                     "until": float(self.data.time) + max(0.0, float(duration))}
        return True

    # -- sensing ------------------------------------------------------------------
    def trunk_tilt(self):
        """(e_pitch, e_roll, omega_pitch, omega_roll) in world frame.

        e_pitch > 0 == leaning FORWARD (toward -Y, the facing direction);
        e_roll  > 0 == leaning toward world +X (the '.R' bone side).

        Sensed on the ROOT (pelvis), not the chest: the pelvis is the body of
        the inverted pendulum that actually falls. Chest/head/arm choreography
        (breathing sway, nodding, waving) intentionally tilts upper bodies;
        sensing those would make the controller fight the motion itself and
        pump energy near the whole-body sway resonance. Root tilt is the
        genuine fall signal (motions only move the root through real reaction
        dynamics).
        """
        xmat = self.data.xmat[self._root_id].reshape(3, 3)
        up = (float(xmat[0][1]), float(xmat[1][1]), float(xmat[2][1]))
        cvel = self.data.cvel[self._root_id]
        return (-up[1], up[0], float(cvel[0]), float(cvel[1]))

    def root_up_z(self):
        xmat = self.data.xmat[self._root_id].reshape(3, 3)
        return float(xmat[2][1])

    def support_info(self):
        """One contact pass -> (N_L, N_R, cop_x, cop_y).

        N_L/N_R are the summed floor-contact normal forces per foot; cop is
        the force-weighted centroid of the foot-floor contact points (world
        XY) -- the true center of pressure. Regulating the COM over the CoP
        is bias-free in static equilibrium (CoP sits exactly under the COM).
        NOTE: using the ankle joint position instead of the CoP is a steady
        bias (~4 cm forward foot offset) that pushes the robot over -- the
        CoP is the only correct reference. Falls back to the ankle midpoint
        when there is no foot contact (airborne).
        """
        loads = {"L": 0.0, "R": 0.0}
        csum = {"L": [0.0, 0.0], "R": [0.0, 0.0]}
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            # only floor-foot contacts define the support CoP; ignore robot
            # self-contacts (e.g. the pelvis capsule resting on the boot),
            # which would otherwise bias the centroid toward the body center
            if c.geom1 != self._floor_geom and c.geom2 != self._floor_geom:
                continue
            for side in ("L", "R"):
                if c.geom1 in self._foot_geoms[side] or \
                        c.geom2 in self._foot_geoms[side]:
                    out = np.zeros(6)
                    mujoco.mj_contactForce(self.model, self.data, i, out)
                    f = abs(float(out[0]))
                    p = c.pos
                    loads[side] += f
                    csum[side][0] += f * float(p[0])
                    csum[side][1] += f * float(p[1])
                    break
        total = loads["L"] + loads["R"]
        if total > 1.0:
            cop_x = (csum["L"][0] + csum["R"][0]) / total
            cop_y = (csum["L"][1] + csum["R"][1]) / total
        else:  # airborne: fall back to ankle midpoint
            fl = self.data.xpos[self._foot_body["L"]]
            fr = self.data.xpos[self._foot_body["R"]]
            cop_x = 0.5 * (float(fl[0]) + float(fr[0]))
            cop_y = 0.5 * (float(fl[1]) + float(fr[1]))
        return loads["L"], loads["R"], cop_x, cop_y

    def foot_loads(self):
        """(N_L, N_R) summed floor-contact normal forces per foot."""
        n_l, n_r, _, _ = self.support_info()
        return n_l, n_r

    # -- control -------------------------------------------------------------
    def apply(self):
        """One 30 fps control tick: perturbations, PD overlay, optional assist.

        Must be called AFTER the motion targets were written to data.ctrl and
        BEFORE mj_step. Returns the telemetry dict.
        """
        data = self.data
        sim_time = float(data.time)

        # 1) external perturbation forces (honest physics: xfrc_applied)
        if self.push is not None:
            if sim_time < self.push["until"]:
                data.xfrc_applied[self.push["body_id"]][:3] = self.push["force"]
            else:
                data.xfrc_applied[self.push["body_id"]][:3] = 0.0
                self.push = None

        if not self.enabled or self.mode == MODE_OFF:
            data.qfrc_applied[self._free_dof:self._free_dof + 3] = 0.0
            self.telemetry_store = self._telemetry(0.0, 0.0, 0.0, 0.0,
                                                   (0.0, 0.0), (0.0, 0.0))
            return self.telemetry_store

        e_p, e_r, w_p, w_r = self.trunk_tilt()
        up_z = self.root_up_z()

        # already down: do not flail; keep targets as commanded
        if up_z < FALL_UP_Z:
            data.qfrc_applied[self._free_dof:self._free_dof + 3] = 0.0
            self.telemetry_store = self._telemetry(e_p, e_r, w_p, w_r,
                                                   (0.0, 0.0), (0.0, 0.0),
                                                   fell=True)
            return self.telemetry_store

        # 2) low-pass filter the tilt error (see __init__). Fast upper-body
        #    choreography (nod/wave) is reaction rocking, not a fall; feeding
        #    it to the proportional correction injects energy. The filtered
        #    tilt keeps the slow balance-mode correction.
        a_t = self.tilt_filt_alpha
        if self._tilt_filt is None:
            self._tilt_filt = (e_p, e_r)
        else:
            self._tilt_filt = (self._tilt_filt[0] + a_t * (e_p - self._tilt_filt[0]),
                               self._tilt_filt[1] + a_t * (e_r - self._tilt_filt[1]))
        ep_f, er_f = self._tilt_filt

        # 3) channel error signals. Pelvis tilt stabilizes the inverted
        #    pendulum; the ANKLE channel additionally regulates the whole-body
        #    COM over the load-weighted geometric support center (foot-box
        #    centers -- constant, noise-free; see __init__ note). Without the
        #    COM term a forced-vertical pelvis equilibrium fights asymmetric
        #    poses (raised arm, turned head shift the COM) and topples the
        #    robot; with it the equilibrium moves so the COM stays supported.
        n_l, n_r, cop_x, cop_y = self.support_info()
        total = n_l + n_r
        w_l = (n_l / total) if total > 1.0 else 0.5
        w_r = (n_r / total) if total > 1.0 else 0.5
        # COM reference = the clean center of pressure (force-weighted centroid
        # of FLOOR-FOOT contacts only; robot self-contacts are excluded). In
        # static equilibrium the CoP sits exactly under the COM -> zero bias;
        # when the COM drifts toward/over the support edge the CoP saturates at
        # the edge and the error grows, driving a recovery lean. This is the
        # correct balance target (unlike the ankle or foot-box center, which
        # are not the robot's natural equilibrium and inject a steady bias).
        com = data.subtree_com[0]
        com_pitch_err = float(cop_y - com[1])   # >0 == COM forward of CoP
        com_roll_err = float(com[0] - cop_x)    # >0 == COM toward +X of CoP
        # low-pass filter to reject contact-centroid jitter (see __init__)
        a = self.com_filt_alpha
        if self._com_filt is None:
            self._com_filt = (com_pitch_err, com_roll_err)
        else:
            self._com_filt = (self._com_filt[0] + a * (com_pitch_err - self._com_filt[0]),
                              self._com_filt[1] + a * (com_roll_err - self._com_filt[1]))
        fp, fr = self._com_filt
        # effective errors per group. Ankle regulates tilt + COM (both axes);
        # hip regulates tilt + fore-aft COM (the hip is a strong pitch actuator
        # needed to arrest the slow backward COM drift that nod induces -- the
        # ankle alone saturates before it can stop it); trunk stays tilt-only.
        ankle_c = {"pitch": ep_f + self.com_gain * fp,
                   "roll": er_f + self.com_gain * fr}
        hip_c = {"pitch": ep_f + self.com_hip_gain * fp,
                 "roll": er_f}
        plain_c = {"pitch": ep_f, "roll": er_f}

        # stance factor: loaded foot keeps full authority, swing foot keeps a
        # small floor (both feet share double-support phases of gait)
        f_l = min(1.0, max(0.15, 2.0 * w_l))
        f_r = min(1.0, max(0.15, 2.0 * w_r))

        max_applied = 0.0
        for jname, info in self._jnt.items():
            ctrl = info["ctrl"]
            if ctrl is None:
                continue
            bone = jname.rsplit(".", 1)[0]
            if bone in _ANKLE_BONES:
                group = "ankle"
                stance_f = f_l if bone.endswith(".L") else f_r
            elif bone in _HIP_BONES:
                group = "hip"
                stance_f = 1.0
            elif bone in _TRUNK_BONES:
                group = "trunk"
                stance_f = 1.0
            else:
                continue
            # world-space joint axis (adapts as the robot moves)
            xmat = data.xmat[info["body_id"]].reshape(3, 3)
            ax_l = info["axis"]
            ax = (xmat[0][0] * ax_l[0] + xmat[0][1] * ax_l[1] + xmat[0][2] * ax_l[2],
                  xmat[1][0] * ax_l[0] + xmat[1][1] * ax_l[1] + xmat[1][2] * ax_l[2],
                  xmat[2][0] * ax_l[0] + xmat[2][1] * ax_l[1] + xmat[2][2] * ax_l[2])
            delta = 0.0
            if group == "ankle":
                c_sel = ankle_c
            elif group == "hip":
                c_sel = hip_c
            else:
                c_sel = plain_c
            for channel, comp in (("pitch", ax[0]), ("roll", ax[1])):
                if abs(comp) < 0.05:
                    continue
                g = self.gains["%s_%s" % (group, channel)]
                c = c_sel[channel]
                kp = g["kp"]
                if self.mode == MODE_GAIT and channel == "roll":
                    kp *= GAIT_ROLL_BOOST
                d = g["sign"] * kp * c * comp
                # damping term follows the same projection (kd=0 by default;
                # see STAND_GAINS note on why derivative feedback is off)
                d_damp = g["sign"] * g["kd"] * (w_p if channel == "pitch"
                                                 else w_r) * comp
                d_total = d + d_damp
                if group == "ankle" and channel == "roll":
                    d_total *= stance_f
                delta += d_total
            if abs(delta) < 1e-6:
                continue
            g_p = self.gains["%s_pitch" % group]["max_delta"]
            g_r = self.gains["%s_roll" % group]["max_delta"]
            lim = max(g_p, g_r)
            delta = max(-lim, min(lim, delta))
            lo, hi = self._ctrlrange[jname]
            new = data.ctrl[ctrl] + delta
            data.ctrl[ctrl] = max(lo, min(hi, new))
            max_applied = max(max_applied, abs(delta))

        # 4) optional explicitly-labeled virtual balance assist (default off)
        if self.assist_gain > 0.0:
            cap = ASSIST_FRACTION_CAP * self._mass * 9.81
            fx = -self.assist_gain * e_r * self._mass * 9.81
            fy = self.assist_gain * e_p * self._mass * 9.81
            fx = max(-cap, min(cap, fx))
            fy = max(-cap, min(cap, fy))
            data.qfrc_applied[self._free_dof:self._free_dof + 3] = [fx, fy, 0.0]
            assist = (fx, fy)
        else:
            data.qfrc_applied[self._free_dof:self._free_dof + 3] = 0.0
            assist = (0.0, 0.0)

        self.telemetry_store = self._telemetry(e_p, e_r, w_p, w_r,
                                               (n_l, n_r), assist,
                                               max_applied=max_applied,
                                               com=(com_pitch_err,
                                                    com_roll_err))
        return self.telemetry_store

    def _telemetry(self, e_p, e_r, w_p, w_r, loads, assist, fell=False,
                   max_applied=0.0, com=(0.0, 0.0)):
        return {
            "mode": self.mode,
            "enabled": self.enabled,
            "fell": fell,
            "tilt_pitch": round(e_p, 4),
            "tilt_roll": round(e_r, 4),
            "com_pitch_err": round(com[0], 4),
            "com_roll_err": round(com[1], 4),
            "omega_pitch": round(w_p, 3),
            "omega_roll": round(w_r, 3),
            "foot_load_L_N": round(loads[0], 1),
            "foot_load_R_N": round(loads[1], 1),
            "assist_N": [round(assist[0], 1), round(assist[1], 1)],
            "max_delta_rad": round(max_applied, 4),
            "gait_scale": round(self.gait_scale, 3),
            "gait_tempo": round(self.gait_tempo, 3),
            "push_remaining_s": round(self.push["until"] - float(self.data.time), 3)
            if self.push else 0.0,
        }

    def telemetry(self):
        return dict(self.telemetry_store) or self._telemetry(0, 0, 0, 0,
                                                             (0.0, 0.0),
                                                             (0.0, 0.0))
