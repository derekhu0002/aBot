#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: MuJoCo physics twin server contract (headless, loopback).

GIVEN the repo contains scripts/blender_humanoid/physics_twin_server.py -- a
  MuJoCo physics twin of the aBot humanoid serving the SAME HTTP contract as
  the Blender twin_server on top of physics_adapter.PhysicsAdapter (GUI
  viewer by default, --headless for automation), plus the UNMODIFIED stdlib
  client scripts/blender_humanoid/twin_client.py
WHEN starting the physics twin headless in-process (HTTP on an ephemeral
  127.0.0.1 loopback port + drive-loop thread) and driving it through the
  unmodified TwinClient, and separately running the CLI bounded headless
  frame smoke (subprocess)
THEN
  - the contract is usable: GET /health answers status=ok/backend=sim with
    19 bones; GET /state returns contract bones+motion plus physics extras
    (root.up_z); /pose, /motion, /bones, /stop reply {"queued": true, ...}
    exactly like twin_server;
  - static standing holds under physics: after set_pose('relax') and settle,
    state root.up_z ~= 1 (upright, not tipped);
  - motion plays under physics: start_motion('wave') is observable via
    /state (motion == 'wave' while playing, bone eulers change over time),
    expires back to motion=None, and the robot is still standing;
  - raw FK (/bones) drives the sim (head pitch tracked by the servo);
  - /stop resets to rest standing (motion cleared, up_z ~= 1 again);
  - unknown pose/motion names are tolerated without crash (twin_server
    parity: queued ack, silently ignored);
  - the CLI smoke `--headless --frames N` runs N frames without crash,
    reports finite state, standing up_z ~= 1 and changed qpos;
  - the AGENT control chain is registered: opencode.json carries BOTH MCP
    entries side by side -- 'twin-control' (Blender) and 'twin-physics'
    (same twin_mcp_server.py wrapper, environment TWIN_PORT=8124, enabled);
  all external-view, no GUI, no external network (loopback only).
"""

import importlib
import json
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS_DIR = os.path.join(ROOT, "scripts", "blender_humanoid")
MODULE_PATH = os.path.join(SCRIPTS_DIR, "physics_twin_server.py")
TICK = 1.0 / 30.0


def wait_for(predicate, timeout=8.0, interval=0.1):
    """Poll a predicate until true or timeout (physics settle is not instant)."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        if predicate():
            return True
        time.sleep(interval)
    return False


def main():
    failures = []

    if not os.path.exists(MODULE_PATH):
        print("FAIL: %s not found" % MODULE_PATH)
        return 1

    with open(MODULE_PATH, encoding="utf-8") as fh:
        source = fh.read()
    if 'if __name__ == "__main__":' not in source:
        failures.append("physics_twin_server.py lacks an "
                        "`if __name__ == \"__main__\":` guard")

    if SCRIPTS_DIR not in sys.path:
        sys.path.insert(0, SCRIPTS_DIR)
    # Importing must NOT start any server/viewer (guarded by __main__)
    pts = importlib.import_module("physics_twin_server")
    from twin_client import TwinClient  # the UNMODIFIED stdlib client

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    # ------------------------------------------------------------------
    # GIVEN the physics twin started headless in-process (ephemeral port)
    # ------------------------------------------------------------------
    twin = pts.PhysicsTwin()
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
        client = TwinClient(port=port)  # zero code changes vs the contract

        # ---- contract usable: /health ------------------------------------
        health = client.health()
        if health.get("status") != "ok" or health.get("backend") != "sim" \
                or health.get("bones") != 19:
            failures.append("health() unexpected: %r" % (health,))
        if "humanoid.mjcf" not in health.get("model", ""):
            failures.append("health() model is not the humanoid MJCF: %r"
                            % (health.get("model"),))

        # ---- initial standing after adapter drop-settle --------------------
        if not wait_for(lambda: client.get_state().get("root", {})
                        .get("up_z", 0.0) > 0.99):
            failures.append("initial standing failed: up_z=%r"
                            % client.get_state().get("root"))

        # ---- /pose relax -> standing up_z ~= 1 -----------------------------
        ack = client.set_pose("relax")
        if ack.get("queued") is not True or ack.get("name") != "relax":
            failures.append("set_pose ack not twin_server-shaped: %r" % (ack,))
        time.sleep(1.2)  # let the servos reach the pose and settle
        st = client.get_state()
        if st.get("motion") is not None:
            failures.append("pose left motion=%r, want None" % st.get("motion"))
        if len(st.get("bones", {})) != 19:
            failures.append("state bones count %d != 19"
                            % len(st.get("bones", {})))
        up_z = st.get("root", {}).get("up_z", 0.0)
        if up_z <= 0.99:
            failures.append("relax does not stand upright in physics: "
                            "up_z=%.4f" % up_z)

        # ---- /motion wave: observable, physics-driven, self-expires --------
        ack = client.start_motion("wave", 2.0)
        if ack.get("queued") is not True or ack.get("name") != "wave":
            failures.append("start_motion ack not twin_server-shaped: %r"
                            % (ack,))
        if not wait_for(lambda: client.get_state().get("motion") == "wave",
                        timeout=2.0):
            failures.append("motion 'wave' never became observable in /state")
        s1 = client.get_state()
        time.sleep(0.6)
        s2 = client.get_state()
        if s1.get("bones") == s2.get("bones"):
            failures.append("wave did not change bone eulers over 0.6s "
                            "(qpos frozen under physics?)")
        if s2.get("sim_time", 0.0) <= s1.get("sim_time", 0.0):
            failures.append("sim_time did not advance during motion")
        if not wait_for(lambda: client.get_state().get("motion") is None,
                        timeout=3.0):
            failures.append("wave did not expire back to motion=None")
        if client.get_state().get("root", {}).get("up_z", 0.0) <= 0.99:
            failures.append("robot not standing after wave expired: %r"
                            % client.get_state().get("root"))

        # ---- /bones raw FK: head pitch servo tracks the target -------------
        ack = client.drive_bones({"head": [0.3, 0.0, 0.0]})
        if ack.get("queued") is not True or ack.get("bones") != 1:
            failures.append("drive_bones ack not twin_server-shaped: %r"
                            % (ack,))
        if not wait_for(lambda: abs(client.get_state()["bones"]["head"][0]
                                    - 0.3) < 0.1, timeout=3.0):
            failures.append("head pitch servo did not track 0.3 rad: %r"
                            % client.get_state()["bones"]["head"])

        # ---- /stop: motion cleared + rest standing -------------------------
        client.start_motion("wave", 5.0)
        wait_for(lambda: client.get_state().get("motion") == "wave",
                 timeout=2.0)
        ack = client.stop()
        if ack.get("queued") is not True:
            failures.append("stop ack not twin_server-shaped: %r" % (ack,))
        if not wait_for(lambda: client.get_state().get("motion") is None,
                        timeout=3.0):
            failures.append("stop did not clear the active motion")
        # reset() re-seats home + settles ~0.8s of sim time
        if not wait_for(lambda: client.get_state().get("root", {})
                        .get("up_z", 0.0) > 0.99, timeout=4.0):
            failures.append("stop did not restore rest standing: %r"
                            % client.get_state().get("root"))

        # ---- unknown names tolerated (twin_server parity, no crash) --------
        ack1 = client.set_pose("fly")
        ack2 = client.start_motion("fly", 1.0)
        if ack1.get("queued") is not True or ack2.get("queued") is not True:
            failures.append("unknown pose/motion not acked like twin_server: "
                            "%r %r" % (ack1, ack2))
        time.sleep(0.4)
        if client.get_state().get("motion") is not None:
            failures.append("unknown motion name must be ignored")
        if client.get_state().get("root", {}).get("up_z", 0.0) <= 0.99:
            failures.append("robot fell over after unknown-name commands")
    finally:
        stop_event.set()
        srv.shutdown()

    # ------------------------------------------------------------------
    # AGENT control chain: opencode.json registers 'twin-physics' (same
    # twin_mcp_server.py wrapper, TWIN_PORT=8124) next to 'twin-control'
    # ------------------------------------------------------------------
    with open(os.path.join(ROOT, "opencode.json"), encoding="utf-8") as fh:
        cfg = json.load(fh)
    mcp_cfg = cfg.get("mcp", {})
    if "twin-control" not in mcp_cfg or "twin-physics" not in mcp_cfg:
        failures.append("opencode.json must register BOTH twin-control and "
                        "twin-physics MCPs side by side, got %s"
                        % sorted(mcp_cfg))
    else:
        tp = mcp_cfg["twin-physics"]
        cmd = " ".join(tp.get("command", []))
        env_port = str(tp.get("environment", {}).get("TWIN_PORT", ""))
        if "twin_mcp_server.py" not in cmd:
            failures.append("twin-physics must reuse twin_mcp_server.py, "
                            "got command %r" % cmd)
        if env_port != "8124":
            failures.append("twin-physics must set TWIN_PORT=8124, got %r"
                            % env_port)
        if tp.get("enabled") is not True:
            failures.append("twin-physics MCP entry must be enabled")

    # ------------------------------------------------------------------
    # CLI bounded headless smoke: N frames, finite, standing, qpos changed
    # ------------------------------------------------------------------
    proc = subprocess.run(
        [sys.executable, MODULE_PATH, "--headless", "--frames", "120",
         "--pose", "relax"],
        capture_output=True, text=True, timeout=180)
    if proc.returncode != 0:
        failures.append("CLI headless frame smoke exited %d: %s"
                        % (proc.returncode, proc.stderr.strip()[:400]))
    facts = None
    for line in proc.stdout.splitlines():
        if line.startswith("SMOKE_JSON "):
            facts = json.loads(line[len("SMOKE_JSON "):])
    if facts is None:
        failures.append("CLI smoke printed no SMOKE_JSON line; stdout=%r"
                        % proc.stdout[:400])
    else:
        if facts.get("frames") != 120 or not facts.get("finite"):
            failures.append("CLI smoke facts unexpected: %r" % (facts,))
        if facts.get("up_z", 0.0) <= 0.99:
            failures.append("CLI smoke not standing at frame end: %r"
                            % (facts,))
        if not facts.get("qpos_changed"):
            failures.append("CLI smoke qpos never changed (sim frozen?)")
    if "SMOKE PASS" not in proc.stdout:
        failures.append("CLI smoke did not report SMOKE PASS")

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: MuJoCo physics twin server serves the twin-control contract "
          "headless (health/state/pose/motion/bones/stop via unmodified "
          "TwinClient; standing up_z~=1, wave plays & expires, stop resets, "
          "CLI frame smoke finite & standing)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
