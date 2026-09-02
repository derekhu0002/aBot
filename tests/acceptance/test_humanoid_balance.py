#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: P2-T4 closed-loop balance & stable gait (headless).

GIVEN the MuJoCo physics twin (physics_adapter.PhysicsAdapter +
  balance_controller.BalanceController) standing quietly on the floor, with
  joint-level proportional ankle/hip/trunk balance feedback and whole-body
  COM-over-CoP regulation running underneath the unchanged twin-control
  contract
WHEN (a) external perturbations are applied as real force pulses
  (adapter.perturb / POST /perturb), (b) upper-body motions play
  (idle/wave/nod/look), and (c) the walk/run gaits play through the
  documented balance envelope
THEN
  - standing balance: after a lateral / forward / backward push the robot
    recovers to upright (root up_z back above the standing threshold within
    the recovery window) instead of tipping, and the closed loop recovers a
    forward push that the OPEN-LOOP robot cannot (proving feedback works);
  - upper-body motions: idle/wave/nod/look all keep standing (up_z stays
    above threshold for the whole motion + recovery), where open-loop nod
    falls;
  - stable gait: walk survives several seconds without falling (quantified
    improvement over the ~0.6 s open-loop fall), and run survives its
    envelope; balance telemetry (tilt / COM error / foot loads / assist) is
    observable in state;
  - server surface: POST /perturb + GET/POST /balance are served headless
    over the loopback contract without breaking any existing endpoint;
  all external-view, deterministic, no GUI, no external network.

Requires: pip install mujoco (plain Python, no Blender).
"""

import importlib
import json
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS_DIR = os.path.join(ROOT, "scripts", "blender_humanoid")
MODULE_PATH = os.path.join(SCRIPTS_DIR, "physics_twin_server.py")

sys.path.insert(0, SCRIPTS_DIR)

TICK = 1.0 / 30.0
STAND_UP_Z = 0.95      # recovered / standing threshold
FALL_UP_Z = 0.70       # considered tipped below this
RECOVER_S = 4.0        # recovery window after a perturbation


def up_z_of(adapter):
    return float(adapter.data.xmat[adapter.model.body("root").id]
                 .reshape(3, 3)[2][1])


def fresh_adapter():
    import physics_adapter as pa
    ap = pa.PhysicsAdapter()
    ap.reset(settle_seconds=1.0)
    return ap


def drive_seconds(ap, seconds):
    umin = 1.0
    for _ in range(int(seconds / TICK)):
        ap.drive_once()
        umin = min(umin, up_z_of(ap))
    return umin


def recover_after_push(force, duration=0.1, body="chest", balance_on=True):
    ap = fresh_adapter()
    ap.balance.enabled = balance_on
    ap.perturb(force, duration, body)
    umin = 1.0
    for _ in range(int(RECOVER_S / TICK)):
        ap.drive_once()
        umin = min(umin, up_z_of(ap))
        if umin < 0.4:
            break
    return up_z_of(ap), umin, ap


def main():
    failures = []

    if not os.path.exists(MODULE_PATH):
        print("FAIL: %s not found" % MODULE_PATH)
        return 1

    import physics_adapter as pa
    import balance_controller as bc

    # ------------------------------------------------------------------ T4a
    # (1) quiet standing is stable with balance on (the controller must not
    #     itself destabilize the naturally-stable static pose)
    ap = fresh_adapter()
    umin = drive_seconds(ap, 5.0)
    if not (up_z_of(ap) > STAND_UP_Z and umin > FALL_UP_Z):
        failures.append("quiet standing not stable with balance on: "
                        "up_end=%.3f up_min=%.3f" % (up_z_of(ap), umin))

    # (2) perturbation recovery -- closed loop must bring the robot back to
    #     upright after a real force pulse (lateral / forward / backward).
    push_cases = [
        ("lateral +X", (150, 0, 0)),
        ("lateral -X", (-150, 0, 0)),
        ("forward -Y", (0, -150, 0)),
        ("backward +Y", (0, 80, 0)),
    ]
    for label, force in push_cases:
        uend, umin, _ = recover_after_push(force)
        if uend <= STAND_UP_Z:
            failures.append("push %s not recovered: up_end=%.3f up_min=%.3f"
                            % (label, uend, umin))

    # (3) proof the feedback does real work: a forward push the OPEN-LOOP
    #     robot cannot survive must be recovered by the closed loop.
    u_open, _, _ = recover_after_push((0, -150, 0), balance_on=False)
    u_closed, _, _ = recover_after_push((0, -150, 0), balance_on=True)
    if not (u_open <= STAND_UP_Z and u_closed > STAND_UP_Z):
        failures.append("closed loop does not beat open loop on forward "
                        "-150N push: open=%.3f closed=%.3f (want open<=0.95 "
                        "< closed)" % (u_open, u_closed))

    # (4) upper-body motions keep standing (open-loop nod falls, so the
    #     controller + nod envelope must hold it up).
    for name in ("idle", "wave", "nod", "look"):
        ap = fresh_adapter()
        ap.start_motion(name, 3.0)
        umin = 1.0
        for _ in range(int(4.0 / TICK)):
            ap.drive_once()
            umin = min(umin, up_z_of(ap))
            if umin < 0.4:
                break
        if not (up_z_of(ap) > STAND_UP_Z and umin > FALL_UP_Z):
            failures.append("motion %s does not stay standing: up_end=%.3f "
                            "up_min=%.3f" % (name, up_z_of(ap), umin))

    # ------------------------------------------------------------------ T4b
    # (5) stable gait: walk survives several seconds (open loop falls ~0.6 s).
    def gait_survival(motion, seconds, balance_on=True):
        ap = fresh_adapter()
        ap.balance.enabled = balance_on
        ap.start_motion(motion, seconds)
        if not balance_on:
            # faithful OPEN-LOOP baseline: full amplitude + tempo, no balance
            # (start_motion loads the tuned envelope; reset it here)
            ap.balance.gait_scale = 1.0
            ap.balance.gait_tempo = 1.0
        fall_t = None
        for i in range(int(seconds / TICK)):
            ap.drive_once()
            if up_z_of(ap) < FALL_UP_Z and fall_t is None:
                fall_t = i * TICK
        return fall_t, up_z_of(ap)

    walk_target_s = 5.0
    fall_closed, u_walk = gait_survival("walk", walk_target_s, True)
    if fall_closed is not None:
        failures.append("walk fell at %.2fs (< %.1fs target): up_end=%.3f"
                        % (fall_closed, walk_target_s, u_walk))
    fall_open, _ = gait_survival("walk", walk_target_s, False)
    # quantified improvement: open loop must fall well before the closed loop
    if fall_open is None or fall_open >= walk_target_s:
        failures.append("open-loop walk unexpectedly survived %.1fs -- the "
                        "comparison baseline is broken" % walk_target_s)
    elif fall_closed is not None and fall_closed <= fall_open:
        failures.append("closed-loop walk (%.2fs) did not outlive open loop "
                        "(%.2fs)" % (fall_closed, fall_open))

    # run survives its envelope too (scaled swing + slower cadence)
    fall_run, u_run = gait_survival("run", 4.0, True)
    if fall_run is not None:
        failures.append("run fell at %.2fs inside its envelope: up_end=%.3f"
                        % (fall_run, u_run))

    # (6) balance telemetry is observable in state (external view)
    ap = fresh_adapter()
    st = ap.state()
    bal = st.get("balance")
    if not isinstance(bal, dict):
        failures.append("state() has no 'balance' telemetry block")
    else:
        for key in ("mode", "enabled", "tilt_pitch", "tilt_roll",
                    "com_pitch_err", "com_roll_err", "foot_load_L_N",
                    "foot_load_R_N", "assist_N", "gait_scale", "gait_tempo"):
            if key not in bal:
                failures.append("balance telemetry missing key %r" % key)
        # joint-only honesty: virtual assist must be OFF (0 N) by default
        if bal.get("assist_N") != [0.0, 0.0]:
            failures.append("virtual balance assist is on by default: %r "
                            "(must be 0 unless explicitly enabled)"
                            % (bal.get("assist_N"),))

    # ------------------------------------------------------------------ HTTP
    # (7) server surface: /perturb + /balance served headless over loopback
    pts = importlib.import_module("physics_twin_server")
    from twin_client import TwinClient

    twin = pts.PhysicsTwin()
    srv = pts.build_http_server(twin, "127.0.0.1", 0)
    port = srv.server_address[1]
    stop_event = threading.Event()
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=pts.drive_loop,
                     kwargs={"twin": twin, "frames": None, "realtime": True,
                             "stop_event": stop_event}, daemon=True).start()
    try:
        client = TwinClient(port=port)
        if client.health().get("status") != "ok":
            failures.append("server health not ok")

        # balance config readable
        cfg = _get_json(port, "/balance")
        if "config" not in cfg or "telemetry" not in cfg:
            failures.append("/balance did not return config+telemetry: %r"
                            % sorted(cfg))

        # perturb the robot over HTTP and confirm it recovers
        client.set_pose("relax")
        time.sleep(0.8)
        _post_json(port, "/perturb", {"force": [150, 0, 0],
                                      "duration": 0.1, "body": "chest"})
        time.sleep(0.2)  # let the push land
        # wait up to ~5 s wall-clock for recovery (real-time drive loop)
        recovered = False
        for _ in range(50):
            st = client.get_state()
            if st.get("root", {}).get("up_z", 0) > STAND_UP_Z:
                recovered = True
                break
            time.sleep(0.1)
        if not recovered:
            failures.append("robot did not recover from HTTP /perturb: "
                            "up_z=%r" % client.get_state().get("root"))

        # balance patch over HTTP is accepted and applied
        _post_json(port, "/balance", {"auto_reseat": False})
        time.sleep(0.2)
        cfg2 = _get_json(port, "/balance")
        if cfg2.get("config", {}).get("auto_reseat") is not False:
            failures.append("POST /balance patch not applied: %r"
                            % cfg2.get("config"))

        # contract still intact after the balance work
        client.start_motion("wave", 1.0)
        time.sleep(0.4)
        if client.get_state().get("motion") != "wave":
            failures.append("motion contract broken after balance changes")
    finally:
        stop_event.set()
        srv.shutdown()

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: P2-T4 closed-loop balance -- quiet stand stable; lateral/"
          "forward/backward pushes recovered (closed loop beats open loop on "
          "a forward push); idle/wave/nod/look stay standing; walk survives "
          "%.0fs (open loop ~0.6s) and run its envelope; balance telemetry "
          "observable; /perturb + /balance served headless" % walk_target_s)
    return 0


def _get_json(port, path):
    import urllib.request
    with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path),
                                timeout=5) as r:
        return json.loads(r.read())


def _post_json(port, path, obj):
    import urllib.request
    req = urllib.request.Request(
        "http://127.0.0.1:%d%s" % (port, path),
        data=json.dumps(obj).encode("utf-8"),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


if __name__ == "__main__":
    sys.exit(main())
