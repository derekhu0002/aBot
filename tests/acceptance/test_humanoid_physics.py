#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: P2 physics -- MJCF dual outlet, physics assets, adapter.

GIVEN the single source of truth (scripts/blender_humanoid/humanoid_spec.py)
  with its MJCF outlet (mjcf_generator.py) and the MuJoCo backend
  (physics_adapter.py), and the committed asset assets/humanoid/humanoid.mjcf
WHEN the MJCF is regenerated, inspected and simulated with MuJoCo
THEN
  T1 (dual outlet): the committed humanoid.mjcf exists and is byte-identical
    to a fresh regeneration from the same procedural parameters (never
    reverse-engineered from the .blend); it contains the 19 FK-contract bones
    by the same names; the axis conventions hold (anti-mirroring): '.L' bones
    at world -X (= anatomical right), head local Y vertical, robot facing
    world -Y -- asserted at spec level AND inside the loaded MuJoCo model;
  T2 (physics assets): collision primitives only (head sphere / torso-limb
    capsules / foot boxes, never render meshes), explicit chibi mass budget
    with boot ballast, rest pose penetration-free, whole-body COM inside the
    foot support polygon, and MuJoCo static standing stays upright & finite
    (does not tip over or diverge);
  T3 (physics_adapter): the twin-control contract surface (pose/motion/FK/
    state/stop/health) drives MuJoCo; all 9 contract motions are accepted and
    simulate without crash/NaN; idle stands upright; raw-FK tpose moves the
    '.L' hand toward world -X in sim (mirror firewall).

Requires: pip install mujoco   (runs in plain Python, no Blender needed).
"""

import hashlib
import http.server
import json
import math
import os
import sys
import tempfile
import threading

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(REPO_ROOT, "scripts", "blender_humanoid")
COMMITTED_MJCF = os.path.join(REPO_ROOT, "assets", "humanoid", "humanoid.mjcf")

sys.path.insert(0, SCRIPTS)

EXPECTED_BONES = ["root", "spine", "chest", "neck", "head"]
for _b in ("shoulder", "upper_arm", "forearm", "hand", "thigh", "shin", "foot"):
    for _s in ("L", "R"):
        EXPECTED_BONES.append("%s.%s" % (_b, _s))


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    failures = []

    try:
        import mujoco
    except ImportError:
        print("FAIL: mujoco not installed -- run: pip install mujoco")
        return 1

    import humanoid_spec as hs
    import mjcf_generator
    from physics_adapter import PhysicsAdapter

    # ------------------------------------------------------------------ T1
    # committed asset exists
    if not os.path.exists(COMMITTED_MJCF):
        failures.append("committed asset assets/humanoid/humanoid.mjcf missing")
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    # byte-deterministic regeneration from the same procedural parameters
    with tempfile.TemporaryDirectory(prefix="abot_mjcf_") as tmp:
        regen = os.path.join(tmp, "humanoid.mjcf")
        mjcf_generator.write_mjcf(regen)
        if sha256(regen) != sha256(COMMITTED_MJCF):
            failures.append("committed humanoid.mjcf differs from a fresh "
                            "regeneration (single-source determinism broken, "
                            "or artifact hand-edited)")

    # spec-level axis-convention assertions (anti-mirroring firewall)
    checks = hs.assert_axis_conventions()
    bad = [c for c in checks if not c[1]]
    if bad:
        for name, _ok, detail in bad[:5]:
            failures.append("axis assertion failed: %s | %s" % (name, detail))

    # MJCF structure: 19 FK-contract bones, same names, joints, actuators
    import xml.etree.ElementTree as ET
    tree = ET.parse(COMMITTED_MJCF)
    root = tree.getroot()
    bodies = root.findall(".//body")
    body_names = [b.get("name") for b in bodies]
    if sorted(body_names) != sorted(EXPECTED_BONES):
        failures.append("MJCF bodies %s != 19 FK-contract bones %s"
                        % (sorted(body_names), sorted(EXPECTED_BONES)))
    joints = root.findall(".//joint")
    freejoints = root.findall(".//freejoint")
    hinge_names = [j.get("name") for j in joints if j.get("type", "hinge") == "hinge"]
    for bone in EXPECTED_BONES:
        if bone == "root":
            if not any(j.get("name") == "root.free" for j in freejoints):
                failures.append("root body missing its free joint")
            continue
        for axis in ("x", "y", "z"):
            jn = "%s.%s" % (bone, axis)
            if jn not in hinge_names:
                failures.append("MJCF missing hinge joint %s" % jn)
    actuators = root.findall(".//actuator/position")
    if len(actuators) != 3 * (len(EXPECTED_BONES) - 1):
        failures.append("MJCF actuator count %d != %d position servos"
                        % (len(actuators), 3 * (len(EXPECTED_BONES) - 1)))

    # ------------------------------------------------------------------ T2
    model = mujoco.MjModel.from_xml_path(COMMITTED_MJCF)
    data = mujoco.MjData(model)

    # mass budget: MuJoCo-integrated mass equals the explicit chibi budget
    total = float(sum(model.body_mass))
    if abs(total - hs.TOTAL_MASS) > 0.01:
        failures.append("MuJoCo total mass %.3f != spec budget %.3f "
                        "(ballast/mass bookkeeping broken)" % (total, hs.TOTAL_MASS))

    # collision primitive types only (no mesh colliders)
    allowed = {"plane", "sphere", "capsule", "box"}
    for g in range(model.ngeom):
        kind = mujoco.mjtGeom(int(model.geom_type[g])).name.lower()
        kind = kind.replace("mjgeom_", "")
        if kind not in allowed:
            failures.append("non-primitive collider geom %s (%s) -- render "
                            "meshes must never collide"
                            % (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g),
                               kind))

    # rest pose is penetration-free (drop-settle clearance aside)
    mujoco.mj_forward(model, data)
    deep = [(model.geom(c.geom1).name, model.geom(c.geom2).name, round(float(c.dist), 4))
            for c in data.contact if c.dist < -1e-3]
    if deep:
        failures.append("rest pose has penetrating collider pairs: %s" % deep)

    # whole-body COM inside the foot support polygon (static stability)
    com, _mass = hs.body_center_of_mass()
    foot_boxes = [g for bn in ("foot.L", "foot.R") for g in hs.COLLIDERS[bn]
                  if g["type"] == "box"]
    xs, ys = [], []
    for fb in foot_boxes:
        for sx in (-1, 1):
            for sy in (-1, 1):
                xs.append(fb["center"][0] + sx * fb["half"][0])
                ys.append(fb["center"][1] + sy * fb["half"][1])
    if not (min(xs) < com[0] < max(xs) and min(ys) < com[1] < max(ys)):
        failures.append("COM %s outside foot support polygon x%s y%s"
                        % (com, (min(xs), max(xs)), (min(ys), max(ys))))
    if com[2] > 0.35:
        failures.append("COM too high (%.3f m) -- chibi ballast ineffective" % com[2])

    # static standing: upright and finite for 4 s, contact force ~= m*g
    head_id = model.body("head").id
    root_id = model.body("root").id

    def root_up(d):
        return float(d.xmat[root_id].reshape(3, 3)[2][1])  # local +Y = up

    for _ in range(2000):
        mujoco.mj_step(model, data)
    finite = all(not (math.isnan(v) or math.isinf(v)) for v in data.qpos)
    if not finite:
        failures.append("static standing diverged (NaN/Inf in qpos)")
    if root_up(data) < 0.99:
        failures.append("static standing tipped over (root up=%.3f)" % root_up(data))
    if float(data.xpos[head_id][2]) < 0.60:
        failures.append("static standing collapsed (head z=%.3f)"
                        % float(data.xpos[head_id][2]))
    ncon = len(data.contact)
    if not any("floor" in model.geom(c.geom1).name or "floor" in model.geom(c.geom2).name
               for c in data.contact):
        failures.append("static standing has no floor contact")
    # summed contact normal force bears the weight (soft-contact settle can
    # overshoot ~2x briefly, so this is an order-of-magnitude sanity bound;
    # the real stability evidence is up_z/head-height/finite above)
    import numpy as np
    normal = 0.0
    for i in range(ncon):
        out = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, out)
        normal += abs(float(out[0]))
    if not (0.5 * total * 9.81 < normal < 2.6 * total * 9.81):
        failures.append("floor contact force %.1f N not bearing weight %.1f N"
                        % (normal, total * 9.81))

    # MuJoCo-level axis checks on the loaded model (not just the spec math)
    hand_l = float(data.xpos[model.body("hand.L").id][0])
    hand_r = float(data.xpos[model.body("hand.R").id][0])
    if not (hand_l < -0.15 < 0.0 < 0.15 < hand_r):
        failures.append("sim mirroring: hand.L x=%.3f / hand.R x=%.3f "
                        "(anatomical right must sit at world -X)" % (hand_l, hand_r))
    head_xmat = data.xmat[head_id].reshape(3, 3)
    head_up = [float(head_xmat[0][1]), float(head_xmat[1][1]), float(head_xmat[2][1])]
    head_fwd = [float(head_xmat[0][2]), float(head_xmat[1][2]), float(head_xmat[2][2])]
    if head_up[2] < 0.99:
        failures.append("head local Y not vertical in sim: %s" % head_up)
    if head_fwd[1] > -0.99:
        failures.append("robot not facing world -Y in sim (head fwd=%s)" % head_fwd)

    # ------------------------------------------------------------------ T3
    ap = PhysicsAdapter(COMMITTED_MJCF)
    health = ap.health()
    if health.get("backend") != "sim" or health.get("bones") != 19:
        failures.append("adapter health unexpected: %s" % health)

    import humanoid_control as hc
    motion_report = {}
    for name, _action, dur in hc.ACTION_SPECS:
        ap.reset(settle_seconds=0.5)
        accepted = (ap.apply_pose(name) if dur is None
                    else ap.start_motion(name, 1.5))
        crashed = False
        for _ in range(int(round(1.5 / (1.0 / 30.0)))):
            ap.drive_once()
            q = ap.data.qpos
            if any(math.isnan(v) or math.isinf(v) for v in q):
                crashed = True
                break
        st = ap.state()
        motion_report[name] = st["root"]["up_z"]
        if not accepted:
            failures.append("adapter rejected contract motion %s" % name)
        if crashed:
            failures.append("motion %s crashed/diverged under MuJoCo" % name)
        if len(st["bones"]) != 19:
            failures.append("state()['bones'] has %d entries, want 19"
                            % len(st["bones"]))
    if motion_report.get("idle", 0.0) < 0.99:
        failures.append("idle does not stand upright in physics "
                        "(root up=%.3f)" % motion_report.get("idle"))

    # raw-FK contract path (drive_bones == POST /bones semantics): tpose must
    # swing the '.L' hand toward world -X inside the simulation too
    ap.reset(settle_seconds=0.5)
    if not ap.drive_bones({"upper_arm.L": (0, 0, math.pi / 2.0),
                           "upper_arm.R": (0, 0, -math.pi / 2.0)}):
        failures.append("adapter.drive_bones rejected contract FK payload")
    for _ in range(600):
        ap.drive_once()
    hl = float(ap.data.xpos[ap.model.body("hand.L").id][0])
    hr = float(ap.data.xpos[ap.model.body("hand.R").id][0])
    if not (hl < -0.35 and hr > 0.35):
        failures.append("sim FK tpose wrong direction: hand.L x=%.3f, "
                        "hand.R x=%.3f (mirrored axes?)" % (hl, hr))

    # stop() relaxes targets; state reports it
    ap.stop()
    if ap.state()["motion"] is not None:
        failures.append("stop() did not clear the active motion")

    # uplink: push_state_to_blender posts the contract FK payload; verified
    # against a local stub twin_server (no real Blender required)
    received = {}

    class StubTwin(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
            received["path"] = self.path
            received["payload"] = json.loads(body)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"queued": true}')

        def log_message(self, *args):
            pass

    stub = http.server.ThreadingHTTPServer(("127.0.0.1", 0), StubTwin)
    port = stub.server_address[1]
    t = threading.Thread(target=stub.serve_forever, daemon=True)
    t.start()
    try:
        pushed = ap.push_state_to_blender("http://127.0.0.1:%d" % port)
    finally:
        stub.shutdown()
    if not pushed:
        failures.append("push_state_to_blender failed against a live stub")
    elif received.get("path") != "/bones" or \
            len(received.get("payload", {}).get("bones", {})) != 19:
        failures.append("push_state_to_blender payload malformed: %s"
                        % received.get("path"))
    # unreachable twin_server must degrade gracefully (False, no raise)
    if ap.push_state_to_blender("http://127.0.0.1:1", timeout=0.3):
        failures.append("push_state_to_blender claimed success without a server")

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: P2 physics -- MJCF dual outlet (byte-deterministic, 19 bones, "
          "axis conventions), collision primitives + chibi ballast stand "
          "stable in MuJoCo, physics_adapter drives all 9 contract motions "
          "(idle upright) and uplinks state for Blender feedback")
    return 0


if __name__ == "__main__":
    sys.exit(main())
