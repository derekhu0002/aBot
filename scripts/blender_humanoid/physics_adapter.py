"""physics_adapter.py -- MuJoCo physics backend behind the twin-control contract.

P2 milestone T3 (2026-09-02): the digital twin gets real physics. The
twin-control contract (pose/motion/FK/state/stop/health) stays UNCHANGED;
this adapter is the third backend (backend=sim):

    downlink:  pose / motion / raw-FK eulers  ->  MuJoCo qpos targets via
               per-joint position servos (ctrl)
    uplink:    MuJoCo state (qpos/qvel/contact forces)  ->  contract-shaped
               state dict, optionally pushed back into a running Blender
               twin_server via POST /bones (reuses the 30 fps FK drive loop)

Design notes
------------
* The contract motion drivers (humanoid_control.MOTION_DRIVERS) are sampled
  on a lightweight FakeArm -- they are pure math (no bpy), so the adapter
  runs in plain Python where the MuJoCo wheel lives.
* Joint mapping is 1:1: every FK-contract bone has three MuJoCo hinges
  (.x/.y/.z) whose composition reproduces Blender XYZ rotation_euler exactly
  (R = Rz@Ry@Rx), so a pose-bone euler (rx, ry, rz) becomes ctrl values
  (bone.x=rx, bone.y=ry, bone.z=rz). The floating root (walk/run bob) is NOT
  actuated -- a free base cannot be translated by servos; the bob emerges
  from dynamics instead (documented contract deviation, /state observable
  stays bone-euler-shaped either way).
* Timing uses SIM virtual time (deterministic). In headless contexts timers
  do not fire, so the host must call drive_once() every tick -- exactly the
  twin_server headless pattern.

Usage:
    python scripts/blender_humanoid/physics_adapter.py --smoke
    python scripts/blender_humanoid/physics_adapter.py --motion walk --duration 3
"""

import json
import math
import os
import sys
import urllib.request

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import humanoid_spec as hs  # noqa: E402
import humanoid_control as hc  # noqa: E402
import mjcf_generator  # noqa: E402  (default MJCF path)

import mujoco  # noqa: E402

TICK = 1.0 / 30.0  # twin-control 30 fps drive loop


# ---------------------------------------------------------------------------
# FakeArm: minimal arm object good enough for the contract drivers
# ---------------------------------------------------------------------------
class FakePoseBone(object):
    def __init__(self, name):
        self.name = name
        self.rotation_mode = "XYZ"
        self.rotation_euler = (0.0, 0.0, 0.0)
        self.location = (0.0, 0.0, 0.0)


class FakeBoneCollection(object):
    """dict-like, but iterating yields the bones (Blender RNA semantics,
    which reset_pose relies on)."""

    def __init__(self, names):
        self._d = {n: FakePoseBone(n) for n in names}

    def __getitem__(self, key):
        return self._d[key]

    def __iter__(self):
        return iter(self._d.values())

    def __contains__(self, key):
        return key in self._d

    def get(self, key, default=None):
        return self._d.get(key, default)

    def __len__(self):
        return len(self._d)


class FakeArm(object):
    def __init__(self, names):
        self.pose = type("Pose", (), {})()
        self.pose.bones = FakeBoneCollection(names)


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------
class PhysicsAdapter(object):
    """Thin MuJoCo backend implementing the twin-control command surface."""

    def __init__(self, mjcf_path=None):
        self.mjcf_path = mjcf_path or mjcf_generator.MJCF_DEFAULT_PATH
        self.model = mujoco.MjModel.from_xml_path(self.mjcf_path)
        self.data = mujoco.MjData(self.model)
        self.fake_arm = FakeArm(hs.BONE_ORDER)

        # joint name -> qpos address
        self._jnt_qpos = {}
        for j in range(self.model.njnt):
            self._jnt_qpos[self.model.joint(j).name] = int(
                self.model.joint(j).qposadr[0])
        # bone -> {axis: ctrl index}
        self._ctrl = {}
        for a in range(self.model.nu):
            aname = self.model.actuator(a).name
            if not aname.startswith("servo_"):
                continue
            bone, axis = aname[len("servo_"):].rsplit(".", 1)
            self._ctrl.setdefault(bone, {})[axis] = a

        self.motion_state = {"name": None, "start": 0.0, "duration": 0.0}
        self._targets = {b: (0.0, 0.0, 0.0) for b in hs.BONE_ORDER}
        self._root_loc_target = (0.0, 0.0, 0.0)
        self.settle(seconds=0.5)  # drop-settle onto the floor at rest pose

    # -- command surface (contract semantics identical to twin_server) ------
    def apply_pose(self, name):
        """Static pose (relax/tpose/apose). Returns False on unknown pose."""
        fn = getattr(hc, "pose_" + name, None)
        if fn is None:
            return False
        self.motion_state["name"] = None
        fn(self.fake_arm)
        self._capture_targets()
        return True

    def start_motion(self, name, duration=3.0):
        """Time-based motion (idle/wave/walk/nod/look/run)."""
        if name not in hc.MOTION_DRIVERS:
            return False
        self.motion_state.update(name=name, start=self.data.time,
                                 duration=max(0.0, float(duration)))
        return True

    def drive_bones(self, bones):
        """Raw FK drive: {bone_name: (rx, ry, rz)} in radians (contract)."""
        for name in bones:
            if name not in self._targets:
                return False
        self.motion_state["name"] = None
        self._targets = {b: (0.0, 0.0, 0.0) for b in hs.BONE_ORDER}
        for name, euler in bones.items():
            self._targets[name] = tuple(float(v) for v in euler)
        return True

    def stop(self):
        """Stop any motion and relax all targets to the rest pose."""
        self.motion_state["name"] = None
        self._targets = {b: (0.0, 0.0, 0.0) for b in hs.BONE_ORDER}
        self._root_loc_target = (0.0, 0.0, 0.0)

    # -- drive loop ----------------------------------------------------------
    def drive_once(self):
        """Advance the simulation by one 30 fps tick (call manually in
        headless contexts, where timers do not fire)."""
        self._apply_motion_targets()
        self._write_ctrl()
        t_end = self.data.time + TICK
        while self.data.time < t_end - 1e-9:
            mujoco.mj_step(self.model, self.data)

    def settle(self, seconds=0.5):
        """Run the sim with rest-pose targets (used at init to drop-settle)."""
        end = self.data.time + seconds
        while self.data.time < end:
            self._write_ctrl()
            mujoco.mj_step(self.model, self.data)

    def reset(self, settle_seconds=0.8):
        """Return to the upright 'home' keyframe (fresh standing start).

        MuJoCo time keeps advancing (deterministic motion timing survives a
        reset inside a longer session); only the dynamic state is re-seated.
        """
        self.stop()
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY,
                                   "home")
        if key_id >= 0:
            self.data.qpos[:] = self.model.key_qpos[key_id]
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.settle(seconds=settle_seconds)

    def _apply_motion_targets(self):
        """Sample the active contract motion into joint targets."""
        name = self.motion_state["name"]
        if name is None:
            return
        t = self.data.time - self.motion_state["start"]
        dur = self.motion_state["duration"]
        if dur > 0 and t > dur:
            self.stop()
            return
        hc.MOTION_DRIVERS[name](self.fake_arm, t)
        self._capture_targets()

    def _capture_targets(self):
        for b in hs.BONE_ORDER:
            pb = self.fake_arm.pose.bones[b]
            self._targets[b] = tuple(float(v) for v in pb.rotation_euler)
        root_loc = tuple(float(v)
                         for v in self.fake_arm.pose.bones["root"].location)
        # the free base cannot be position-commanded; keep the contract bob
        # observable but un-actuated (see module docstring)
        self._root_loc_target = root_loc

    def _write_ctrl(self):
        for bone, euler in self._targets.items():
            axes = self._ctrl.get(bone)
            if axes is None:
                continue  # floating root
            self.data.ctrl[axes["x"]] = euler[0]
            self.data.ctrl[axes["y"]] = euler[1]
            self.data.ctrl[axes["z"]] = euler[2]

    # -- uplink --------------------------------------------------------------
    def state(self):
        """Contract-shaped state + physics extras (qvel, contacts, root)."""
        bones = {}
        for b in hs.BONE_ORDER:
            axes = self._ctrl.get(b)
            if axes is None:
                bones[b] = [0.0, 0.0, 0.0]
                continue
            bones[b] = [round(float(self.data.qpos[self._jnt_qpos[b + ".x"]]), 4),
                        round(float(self.data.qpos[self._jnt_qpos[b + ".y"]]), 4),
                        round(float(self.data.qpos[self._jnt_qpos[b + ".z"]]), 4)]
        root_id = self.model.body("root").id
        xmat = self.data.xmat[root_id].reshape(3, 3)
        contacts = self.top_contacts(4)
        return {
            "bones": bones,
            "motion": self.motion_state["name"],
            "sim_time": round(float(self.data.time), 4),
            "root": {
                "pos": [round(float(v), 4) for v in self.data.qpos[0:3]],
                "height": round(float(self.data.xpos[root_id][2]), 4),
                # local +Y is the body 'up' axis for the vertical chain
                "up_z": round(float(xmat[2][1]), 4),
            },
            "root_loc_target": [round(v, 4) for v in self._root_loc_target],
            "max_qvel": round(float(max(abs(v) for v in self.data.qvel)), 4),
            "contacts": contacts,
        }

    def top_contacts(self, n=4):
        """Strongest current contacts (name pair + normal force magnitude)."""
        import numpy as np
        items = []
        for i in range(len(self.data.contact)):
            c = self.data.contact[i]
            g1 = self.model.geom(c.geom1).name
            g2 = self.model.geom(c.geom2).name
            out = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, out)
            items.append((float(abs(out[0])), g1, g2,
                          [round(float(v), 3) for v in out]))
        items.sort(reverse=True)
        return [{"geoms": [g1, g2], "normal_N": round(f, 2),
                 "force": fv} for f, g1, g2, fv in items[:n]]

    def health(self):
        return {"status": "ok", "backend": "sim", "model": self.mjcf_path,
                "bones": len(hs.BONE_ORDER)}

    # -- Blender feedback (optional) ------------------------------------------
    def push_state_to_blender(self, base_url="http://127.0.0.1:8123",
                              timeout=1.0):
        """Uplink: send current sim bone eulers to a running twin_server via
        POST /bones (reuses the Blender 30 fps FK drive loop). Returns True
        on success; False (never raises) when no twin_server is reachable."""
        payload = json.dumps({"bones": self.state()["bones"]}).encode("utf-8")
        req = urllib.request.Request(base_url.rstrip("/") + "/bones",
                                     data=payload,
                                     headers={"Content-Type":
                                              "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return resp.status == 200
        except Exception:
            return False


# ---------------------------------------------------------------------------
# CLI smoke: static standing + all nine contract motions under physics
# ---------------------------------------------------------------------------
def smoke(duration=2.0):
    print("== physics_adapter smoke (MuJoCo backend) ==")
    ap = PhysicsAdapter()
    print("health:", ap.health())

    ap.settle(seconds=3.0)
    st = ap.state()
    standing_ok = (st["root"]["up_z"] > 0.99 and st["root"]["height"] > 0.01
                   and st["max_qvel"] < 0.1)
    print("STANDING after 3.5s settle: up_z=%.4f root_h=%.4f max_qvel=%.4f -> %s"
          % (st["root"]["up_z"], st["root"]["height"], st["max_qvel"],
             "OK" if standing_ok else "FAIL"))

    facts = {"standing": standing_ok, "motions": {}}
    for name, _action, dur in hc.ACTION_SPECS:
        ap.reset()  # every motion starts from a fresh upright standing pose
        ok = (ap.apply_pose(name) if dur is None
              else ap.start_motion(name, min(duration, dur)))
        n_ticks = int(round(duration / TICK))
        crashed = False
        for _ in range(n_ticks):
            ap.drive_once()
            s = ap.state()
            bad = any(math.isnan(v) or math.isinf(v)
                      for v in ap.data.qpos) or \
                any(math.isnan(v) or math.isinf(v) for v in ap.data.qvel)
            if bad:
                crashed = True
                break
        s = ap.state()
        facts["motions"][name] = {
            "accepted": ok,
            "crashed": crashed,
            "sim_time": s["sim_time"],
            "root_height": s["root"]["height"],
            "root_up_z": s["root"]["up_z"],
            "max_qvel": s["max_qvel"],
        }
        print("MOTION %-6s accepted=%s crashed=%s root_h=%.3f up_z=%.3f"
              % (name, ok, crashed, s["root"]["height"], s["root"]["up_z"]))
    all_ok = standing_ok and all(
        m["accepted"] and not m["crashed"] for m in facts["motions"].values())
    print("SMOKE_JSON " + json.dumps(facts))
    print("SMOKE %s" % ("PASS" if all_ok else "FAIL"))
    return 0 if all_ok else 1


def main(argv):
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--smoke", action="store_true",
                   help="static standing + 9-motion MuJoCo smoke")
    p.add_argument("--motion", default=None,
                   help="play one motion: idle|wave|walk|nod|look|run")
    p.add_argument("--pose", default=None,
                   help="apply one pose: relax|tpose|apose")
    p.add_argument("--duration", type=float, default=3.0)
    p.add_argument("--push-url", default=None,
                   help="push sim state to a running twin_server (POST /bones)")
    args = p.parse_args(argv)

    if args.smoke:
        return smoke(args.duration)

    ap = PhysicsAdapter()
    if args.pose:
        if not ap.apply_pose(args.pose):
            print("unknown pose: %s" % args.pose)
            return 1
    elif args.motion:
        if not ap.start_motion(args.motion, args.duration):
            print("unknown motion: %s" % args.motion)
            return 1
    for _ in range(int(round(args.duration / TICK))):
        ap.drive_once()
        if args.push_url:
            ap.push_state_to_blender(args.push_url)
    print(json.dumps(ap.state(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
