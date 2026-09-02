#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: Unitree H1 (MuJoCo Menagerie) physics-twin model.

GIVEN the repo provides the MuJoCo Menagerie Unitree H1 integration --
  assets/menagerie/unitree_h1/ (scene.xml + h1.xml + LICENSE; the heavy
  STL/PNG binaries are git-ignored and reproducible via
  scripts/fetch_menagerie.py, which this test runs once when the assets are
  missing) and scripts/blender_humanoid/menagerie_h1.py (H1Adapter behind
  the twin-control contract, selected with --model unitree_h1)
WHEN loading the scene headless in MuJoCo, driving the H1 adapter (standing
  hold, raw joint FK, motions), serving it over the contract HTTP surface on
  an ephemeral loopback port, and running the CLI bounded smoke for BOTH
  the H1 and the default (chibi) model
THEN
  - the model loads and is a complete humanoid: bodies/joints/coordinates
    exceed 20 (nbody>=21, njnt>=20, nq>=26) with exactly 19 actuated joints
    and a 'home' keyframe; the shipped LICENSE names Unitree Robotics
    (BSD-3-Clause model content);
  - the standing hold is quantitatively stable: after reset+settle
    up_z > 0.98, and over 6 s of quiet standing the minimum up_z stays
    > 0.95, all qpos/qvel finite, and horizontal drift < 0.1 m (no
    divergence, no fall);
  - the robot is drivable: raw FK by H1 joint names tracks a servo target
    (unknown joints rejected), 'wave' plays observably (right shoulder
    raises), expires back to motion=None and the robot is still standing;
    walk/run/nod are honestly rejected (no trained policy -- the honest
    capability boundary);
  - the contract/GUI surface does not regress: /health reports the H1 model
    with 19 bones, /pose /motion /stop reply twin_server-shaped acks,
    unknown pose names are tolerated, the CLI smoke --model unitree_h1
    --headless --frames 120 exits 0 with finite standing SMOKE facts, and
    the DEFAULT model stays the chibi humanoid.mjcf (its CLI smoke still
    passes) -- all external-view, no GUI, loopback only.
"""

import importlib
import json
import math
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS_DIR = os.path.join(ROOT, "scripts", "blender_humanoid")
H1_DIR = os.path.join(ROOT, "assets", "menagerie", "unitree_h1")
H1_SCENE = os.path.join(H1_DIR, "scene.xml")
FETCH = os.path.join(ROOT, "scripts", "fetch_menagerie.py")
SERVER = os.path.join(SCRIPTS_DIR, "physics_twin_server.py")


def wait_for(predicate, timeout=8.0, interval=0.1):
    t_end = time.time() + timeout
    while time.time() < t_end:
        if predicate():
            return True
        time.sleep(interval)
    return False


def ensure_assets(failures):
    """Assets present? If missing, fetch reproducibly via the committed
    script (needs network once)."""
    if os.path.isfile(H1_SCENE) and \
            os.path.isfile(os.path.join(H1_DIR, "h1.xml")):
        return True
    print("H1 assets missing -- fetching via scripts/fetch_menagerie.py ...",
          flush=True)
    proc = subprocess.run([sys.executable, FETCH, "--model", "unitree_h1"],
                          capture_output=True, text=True, timeout=600)
    if proc.returncode != 0 or not os.path.isfile(H1_SCENE):
        failures.append("unitree_h1 assets missing and fetch failed: %s"
                        % (proc.stderr or proc.stdout or "")[:400])
        return False
    return True


def main():
    failures = []

    for path in (SERVER, FETCH):
        if not os.path.exists(path):
            print("FAIL: %s not found" % path)
            return 1

    if not ensure_assets(failures):
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    if SCRIPTS_DIR not in sys.path:
        sys.path.insert(0, SCRIPTS_DIR)
    import mujoco
    mh1 = importlib.import_module("menagerie_h1")
    pts = importlib.import_module("physics_twin_server")
    from twin_client import TwinClient  # the UNMODIFIED stdlib client

    # ------------------------------------------------------------------
    # THEN part 1: the scene loads and is a complete humanoid
    # ------------------------------------------------------------------
    m = mujoco.MjModel.from_xml_path(H1_SCENE)
    if m.nbody < 21 or m.njnt < 20 or m.nq < 26:
        failures.append("H1 too small to be a complete humanoid: "
                        "nbody=%d njnt=%d nq=%d" % (m.nbody, m.njnt, m.nq))
    if m.nu != 19:
        failures.append("H1 must expose 19 actuated joints, got nu=%d" % m.nu)
    if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home") < 0:
        failures.append("H1 scene lacks the 'home' keyframe")
    with open(os.path.join(H1_DIR, "LICENSE"), encoding="utf-8") as fh:
        lic = fh.read()
    if "Unitree" not in lic:
        failures.append("H1 LICENSE must name Unitree Robotics (BSD-3-Clause "
                        "model content from the Menagerie digest)")

    # ------------------------------------------------------------------
    # THEN part 2: standing hold is quantitatively stable (headless)
    # ------------------------------------------------------------------
    ap = mh1.H1Adapter()
    st0 = ap.state()
    if st0["root"]["up_z"] <= 0.98:
        failures.append("H1 not upright after reset+settle: up_z=%.4f"
                        % st0["root"]["up_z"])
    min_up = 1.0
    diverged = False
    for _ in range(180):  # 6 s quiet standing
        ap.drive_once()
        if any(math.isnan(v) or math.isinf(v) for v in ap.data.qpos) or \
                any(math.isnan(v) or math.isinf(v) for v in ap.data.qvel):
            diverged = True
            break
        min_up = min(min_up, ap.state()["root"]["up_z"])
    if diverged:
        failures.append("H1 standing diverged (non-finite state)")
    if min_up <= 0.95:
        failures.append("H1 standing hold not stable: min up_z=%.4f over 6s"
                        % min_up)
    drift = abs(float(ap.data.qpos[0]))
    if drift >= 0.1:
        failures.append("H1 standing drifted %.3f m horizontally" % drift)

    # ------------------------------------------------------------------
    # THEN part 3: drivable -- raw FK by joint name + motions
    # ------------------------------------------------------------------
    ap.reset()
    if not ap.drive_bones({"left_shoulder_pitch": -0.5}):
        failures.append("drive_bones by H1 joint name rejected")
    for _ in range(60):  # 2 s for the servo to track
        ap.drive_once()
    got = float(ap.data.qpos[ap._jnt_qpos["left_shoulder_pitch"]])
    if abs(got - (-0.5)) > 0.15:
        failures.append("left_shoulder_pitch servo did not track -0.5 rad: "
                        "%.3f" % got)
    if ap.drive_bones({"not_a_joint": 0.1}) is not False:
        failures.append("unknown joint name must be rejected")

    ap.reset()
    if not ap.start_motion("wave", 2.0):
        failures.append("motion 'wave' rejected on H1")
    stood = True
    s_shoulder = None
    for _ in range(60):
        ap.drive_once()
        s_shoulder = float(ap.data.qpos[ap._jnt_qpos["right_shoulder_roll"]])
        if ap.state()["root"]["up_z"] < 0.9:
            stood = False
    if ap.motion_state["name"] is not None:
        failures.append("wave did not expire back to motion=None")
    # wave raises the right arm: shoulder_roll must leave the standing value
    stand_roll = ap.stand_targets["right_shoulder_roll"]
    if s_shoulder is None or abs(s_shoulder - stand_roll) < 0.5:
        failures.append("wave did not raise the right shoulder "
                        "(%.3f vs stand %.3f)" % (s_shoulder, stand_roll))
    if not stood:
        failures.append("H1 fell over while waving")

    ap.reset()
    for name in ("walk", "run", "nod"):
        if ap.start_motion(name, 1.0) is not False:
            failures.append("'%s' must be honestly rejected on H1 "
                            "(no trained policy)" % name)

    # ------------------------------------------------------------------
    # THEN part 4: contract surface (HTTP, ephemeral port, unmodified client)
    # ------------------------------------------------------------------
    twin = pts.PhysicsTwin(model="unitree_h1")
    srv = pts.build_http_server(twin, "127.0.0.1", 0)
    port = srv.server_address[1]
    stop_event = threading.Event()
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    drive_thread = threading.Thread(
        target=pts.drive_loop,
        kwargs={"twin": twin, "frames": None, "realtime": True,
                "stop_event": stop_event},
        daemon=True)
    drive_thread.start()
    try:
        client = TwinClient(port=port)
        health = client.health()
        if health.get("status") != "ok" or health.get("bones") != 19 or \
                "unitree_h1" not in health.get("model", ""):
            failures.append("H1 /health unexpected: %r" % (health,))
        ack = client.set_pose("relax")
        if ack.get("queued") is not True:
            failures.append("H1 set_pose ack not twin_server-shaped: %r"
                            % (ack,))
        if not wait_for(lambda: client.get_state().get("root", {})
                        .get("up_z", 0.0) > 0.98, timeout=6.0):
            failures.append("H1 not standing over HTTP: %r"
                            % client.get_state().get("root"))
        ack = client.start_motion("wave", 1.5)
        if ack.get("queued") is not True or \
                not wait_for(lambda: client.get_state().get("motion")
                             == "wave", timeout=2.0):
            failures.append("H1 wave not observable over HTTP")
        ack = client.stop()
        if ack.get("queued") is not True:
            failures.append("H1 stop ack not twin_server-shaped: %r" % (ack,))
        if not wait_for(lambda: client.get_state().get("motion") is None,
                        timeout=3.0):
            failures.append("H1 stop did not clear the motion")
        ack1 = client.set_pose("fly")
        ack2 = client.start_motion("fly", 1.0)
        if ack1.get("queued") is not True or ack2.get("queued") is not True:
            failures.append("H1 unknown pose/motion not tolerated: %r %r"
                            % (ack1, ack2))
    finally:
        stop_event.set()
        srv.shutdown()

    # ------------------------------------------------------------------
    # THEN part 5: CLI smoke --model unitree_h1 (bounded, headless)
    # ------------------------------------------------------------------
    proc = subprocess.run(
        [sys.executable, SERVER, "--model", "unitree_h1", "--headless",
         "--frames", "120"],
        capture_output=True, text=True, timeout=300)
    if proc.returncode != 0:
        failures.append("H1 CLI smoke exited %d: %s"
                        % (proc.returncode, proc.stderr.strip()[:400]))
    facts = None
    for line in proc.stdout.splitlines():
        if line.startswith("SMOKE_JSON "):
            facts = json.loads(line[len("SMOKE_JSON "):])
    if facts is None:
        failures.append("H1 CLI smoke printed no SMOKE_JSON: %r"
                        % proc.stdout[:400])
    else:
        if facts.get("frames") != 120 or not facts.get("finite"):
            failures.append("H1 CLI smoke facts unexpected: %r" % (facts,))
        if facts.get("up_z", 0.0) <= 0.97:
            failures.append("H1 CLI smoke not standing at frame end: %r"
                            % (facts,))
        if not facts.get("qpos_changed"):
            failures.append("H1 CLI smoke qpos never changed (sim frozen?)")
    if "SMOKE PASS" not in proc.stdout:
        failures.append("H1 CLI smoke did not report SMOKE PASS")

    # ------------------------------------------------------------------
    # THEN part 6: the DEFAULT model is still the chibi (no regression)
    # ------------------------------------------------------------------
    default_twin = pts.PhysicsTwin()
    dh = default_twin.adapter.health()
    if "humanoid.mjcf" not in dh.get("model", "") or dh.get("bones") != 19:
        failures.append("default model must remain the chibi humanoid.mjcf, "
                        "got %r" % (dh,))
    proc = subprocess.run(
        [sys.executable, SERVER, "--headless", "--frames", "60"],
        capture_output=True, text=True, timeout=300)
    if proc.returncode != 0 or "SMOKE PASS" not in proc.stdout:
        failures.append("default-model CLI smoke regressed: rc=%s out=%r"
                        % (proc.returncode, proc.stdout[:300]))

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: Unitree H1 (MuJoCo Menagerie) loads as a complete humanoid "
          "(nq=26, 19 actuated joints), stands stably under the closed-loop "
          "stand holder (min up_z>0.95 over 6s, drift<0.1m), is drivable "
          "(FK tracking, wave stands, walk/run/nod honestly rejected), and "
          "the twin-control contract + default chibi model are unregressed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
