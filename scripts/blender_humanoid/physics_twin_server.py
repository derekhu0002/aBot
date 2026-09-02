#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""physics_twin_server.py -- MuJoCo physics twin with a visible window.

P2 physics (2026-09-02): the digital twin gets REAL physics you can watch.
Loads assets/humanoid/humanoid.mjcf into MuJoCo and serves the SAME HTTP
contract as the Blender twin_server (twin_client.py connects with ZERO code
changes), but every pose/motion goes through PhysicsAdapter -- qpos/ctrl
position-servo targets integrated by MuJoCo dynamics -- while a MuJoCo
viewer window (GLFW) shows the robot live: drag to orbit, scroll to zoom.

Run (GUI by default -- a window opens, watch/orbit/zoom the robot):
    python scripts/blender_humanoid/physics_twin_server.py
Run (headless, for tests / automation / no display):
    python scripts/blender_humanoid/physics_twin_server.py --headless
Run (bounded smoke: N frames then exit, prints SMOKE_JSON):
    python scripts/blender_humanoid/physics_twin_server.py --headless --frames 300
    python scripts/blender_humanoid/physics_twin_server.py --frames 90  # GUI probe

Closed-loop balance (P2-T4, 2026-09-02): a BalanceController runs underneath
the contract on every tick -- joint-level proportional ankle/hip/trunk
feedback on pelvis tilt plus whole-body-COM-over-CoP regulation. It recovers
moderate external pushes, keeps idle/wave/nod/look standing, and stabilizes
walk/run via a documented gait envelope (scaled swing + slower cadence) and
nod via a scaled head amplitude. This is JOINT-LEVEL control with NO hidden
forces by default (an optional, explicitly-labeled virtual balance assist is
off). Balance is observable in GET /state ("balance" block) and GET /balance;
POST /perturb applies a real external force pulse; POST /balance patches the
controller. /stop (and auto-reseat after a fall) re-seats the robot onto its
upright 'home' keyframe (kinematic snap -- same contract semantics as the
Blender twin_server's stop -> rest pose). Dynamic self-righting from lying is
future work.

API (JSON -- identical contract to twin_server.py, default port 8124):
    GET  /health  -> {"status": "ok", "backend": "sim", "model": ..., "bones": 19}
    GET  /state   -> contract state + physics extras (root.pos/height/up_z,
                     sim_time, max_qvel, contacts) + "balance" telemetry
    POST /pose    {"name": "relax"|"tpose"|"apose"}            static pose
    POST /motion  {"name": "idle"|"wave"|"walk"|"nod"|"look"|"run",
                   "duration": seconds}                         timed motion
    POST /bones   {"bones": {"upper_arm.R": [0,0,1.57], ...}}  raw FK (rad)
    POST /stop    {}                                            stop + rest
    -- P2-T4 balance extensions (do not change the contract above) --
    POST /perturb {"force": [fx,fy,fz], "duration": s, "body": "chest"}
    GET  /balance -> {"config": ..., "telemetry": ...}
    POST /balance {"mode","enabled","gait_scale","gait_tempo","assist_gain",
                   "auto_reseat"}  (any subset)

Endpoint: 127.0.0.1:8124 by default (the Blender twin_server owns 8123);
override with PHYSICS_TWIN_HOST / PHYSICS_TWIN_PORT env vars or --host/--port.
Agent control: opencode.json registers a second local MCP 'twin-physics'
(twin_mcp_server.py with TWIN_PORT=8124) so Agents drive this server through
the same pose/motion/fk/stop/state/health tool set.
"""

import argparse
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import physics_adapter as pa  # noqa: E402

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8124  # Blender twin_server owns 8123; keep them separable
TICK = pa.TICK       # 30 fps drive loop (contract tempo)
# P2-T4: auto-reseat a fallen robot after this many seconds down (honest
# kinematic recovery, not dynamic self-righting; see BalanceController).
RESEAT_AFTER_S = 2.0


# ---------------------------------------------------------------------------
# Core service: PhysicsAdapter + command queue + cached state
# ---------------------------------------------------------------------------
class PhysicsTwin(object):
    """MuJoCo physics twin behind the twin-control HTTP contract.

    Commands arrive from HTTP threads via cmd_queue and are consumed on the
    single drive thread (same pattern as twin_server.py, where bpy calls must
    stay on the main thread); here it keeps mj_step and the servo targets on
    one thread so the simulation stays deterministic per tick.
    """

    def __init__(self, adapter=None, mjcf_path=None):
        self.adapter = adapter or pa.PhysicsAdapter(mjcf_path)
        self.cmd_queue = queue.Queue()
        self.state_cache = self.adapter.state()
        self._fallen_ticks = 0  # P2-T4 auto-reseat counter

    # -- command surface (queued; consumed by the drive thread) -------------
    def queue_pose(self, name):
        self.cmd_queue.put(("pose", {"name": name}))

    def queue_motion(self, name, duration=3.0):
        self.cmd_queue.put(("motion", {"name": name, "duration": duration}))

    def queue_bones(self, bones):
        self.cmd_queue.put(("bones", {"bones": bones}))

    def queue_stop(self):
        self.cmd_queue.put(("stop", {}))

    def queue_perturb(self, force, duration=0.1, body="chest"):
        """P2-T4: queue a real external force pulse (push the robot)."""
        self.cmd_queue.put(("perturb", {"force": force, "duration": duration,
                                        "body": body}))

    def queue_balance(self, patch):
        """P2-T4: queue a balance-controller configuration patch."""
        self.cmd_queue.put(("balance", {"patch": patch}))

    def consume_commands(self):
        while not self.cmd_queue.empty():
            typ, payload = self.cmd_queue.get()
            try:
                if typ == "pose":
                    # unknown pose names are ignored (twin_server parity)
                    self.adapter.apply_pose(payload.get("name", ""))
                elif typ == "motion":
                    self.adapter.start_motion(payload.get("name", ""),
                                              payload.get("duration", 3.0))
                elif typ == "bones":
                    self.adapter.drive_bones(payload.get("bones", {}))
                elif typ == "stop":
                    # contract semantics "stop -> rest pose": re-seat onto the
                    # upright 'home' keyframe (kinematic snap), then settle
                    self.adapter.reset()
                elif typ == "perturb":
                    self.adapter.perturb(payload.get("force", [0, 0, 0]),
                                         payload.get("duration", 0.1),
                                         payload.get("body", "chest"))
                elif typ == "balance":
                    self.adapter.balance.configure(payload.get("patch", {}))
            except Exception as exc:  # noqa: BLE001 - serve must survive
                print("physics_twin_server: %s command failed: %s"
                      % (typ, exc), flush=True)

    # -- drive ----------------------------------------------------------------
    def drive_once(self):
        """One 30 fps tick: consume commands, step physics, refresh cache.

        P2-T4 auto-reseat: if the robot has been down (balance 'fell') for
        RESEAT_AFTER_S of sim time and auto_reseat is on, kinematically re-seat
        it onto the upright home keyframe (same semantics as /stop). This is a
        recovery convenience, NOT dynamic self-righting (future work).
        """
        self.consume_commands()
        self.adapter.drive_once()
        tel = self.adapter.balance.telemetry()
        if tel.get("fell") and self.adapter.balance.auto_reseat:
            self._fallen_ticks = getattr(self, "_fallen_ticks", 0) + 1
            if self._fallen_ticks >= int(RESEAT_AFTER_S / TICK):
                self.adapter.reset()
                self._fallen_ticks = 0
        else:
            self._fallen_ticks = 0
        self.state_cache = self.adapter.state()


def drive_loop(twin, frames=None, realtime=True, stop_event=None,
               on_tick=None):
    """Headless drive loop. frames=None -> run until stop_event/KeyboardInterrupt.
    realtime=True paces to wall-clock 30 fps (interactive serving); False runs
    as fast as possible (bounded --frames automation smoke)."""
    n = 0
    while stop_event is None or not stop_event.is_set():
        if frames is not None and n >= frames:
            break
        twin.drive_once()
        if on_tick is not None:
            on_tick(twin, n)
        n += 1
        if frames is None and realtime:
            time.sleep(TICK)
        elif frames is not None and realtime:
            time.sleep(TICK)
    return n


# ---------------------------------------------------------------------------
# HTTP layer -- response shapes identical to twin_server.py
# ---------------------------------------------------------------------------
def make_handler(twin):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code, obj):
            data = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                return json.loads(raw)
            except json.JSONDecodeError:
                return {}

        def do_GET(self):
            if self.path.startswith("/health"):
                self._reply(200, twin.adapter.health())
            elif self.path.startswith("/state"):
                self._reply(200, twin.state_cache)
            elif self.path.startswith("/balance"):
                # P2-T4: balance config + telemetry (external observability)
                bal = twin.adapter.balance
                self._reply(200, {"config": bal.config(),
                                  "telemetry": bal.telemetry()})
            else:
                self._reply(404, {"error": "not found"})

        def do_POST(self):
            body = self._body()
            if self.path.startswith("/pose"):
                twin.queue_pose(body.get("name", ""))
                self._reply(200, {"queued": True, "name": body.get("name")})
            elif self.path.startswith("/motion"):
                twin.queue_motion(body.get("name", ""),
                                  body.get("duration", 3.0))
                self._reply(200, {"queued": True, "name": body.get("name"),
                                  "duration": body.get("duration", 3.0)})
            elif self.path.startswith("/bones"):
                twin.queue_bones(body.get("bones", {}))
                self._reply(200, {"queued": True,
                                  "bones": len(body.get("bones", {}))})
            elif self.path.startswith("/stop"):
                twin.queue_stop()
                self._reply(200, {"queued": True})
            elif self.path.startswith("/perturb"):
                # P2-T4: apply a real external force pulse (push the robot)
                force = body.get("force", [0, 0, 0])
                twin.queue_perturb(force, body.get("duration", 0.1),
                                   body.get("body", "chest"))
                self._reply(200, {"queued": True, "force": force,
                                  "duration": body.get("duration", 0.1),
                                  "body": body.get("body", "chest")})
            elif self.path.startswith("/balance"):
                # P2-T4: patch balance-controller configuration
                twin.queue_balance(body)
                self._reply(200, {"queued": True, "patch": body})
            else:
                self._reply(404, {"error": "not found"})

        def log_message(self, *args):
            pass  # silence request logging

    return Handler


def build_http_server(twin, host=DEFAULT_HOST, port=DEFAULT_PORT):
    """ThreadingHTTPServer bound to (host, port); port=0 -> ephemeral (tests)."""
    return ThreadingHTTPServer((host, port), make_handler(twin))


def start_http_server(twin, host=DEFAULT_HOST, port=DEFAULT_PORT):
    """Build + start the HTTP server on a daemon thread; returns the server."""
    srv = build_http_server(twin, host, port)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# ---------------------------------------------------------------------------
# GUI: MuJoCo passive viewer (GLFW) + real-time paced physics
# ---------------------------------------------------------------------------
def run_gui(twin, frames=None):
    """Show the robot in a MuJoCo window; physics steps at 30 fps wall-clock.

    frames=None -> until the window closes or Ctrl+C. frames=N -> N ticks
    (~N/30 s) then exit cleanly (used as a GUI smoke probe).
    """
    import mujoco.viewer  # lazy: headless hosts never import the GUI stack
    try:
        viewer_ctx = mujoco.viewer.launch_passive(twin.adapter.model,
                                                  twin.adapter.data)
    except Exception as exc:  # noqa: BLE001 - friendly degradation
        print("physics_twin_server: could not open the MuJoCo viewer (%s). "
              "No display? Use --headless." % exc, file=sys.stderr, flush=True)
        return 1
    with viewer_ctx as viewer:
        next_tick = time.perf_counter()
        n = 0
        try:
            while viewer.is_running():
                if frames is not None and n >= frames:
                    break
                twin.drive_once()
                viewer.sync()
                n += 1
                next_tick += TICK
                lag = next_tick - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)
                else:
                    next_tick = time.perf_counter()  # drop ticks, never spiral
        except KeyboardInterrupt:
            pass
    return 0


def smoke_facts(twin, frames, qpos0, mode):
    """Facts for the bounded --frames smoke (external observables)."""
    import math
    st = twin.adapter.state()
    qpos1 = [float(v) for v in twin.adapter.data.qpos]
    finite = all(not (math.isnan(v) or math.isinf(v))
                 for v in qpos1) and \
        all(not (math.isnan(v) or math.isinf(v))
            for v in twin.adapter.data.qvel)
    return {
        "mode": mode,
        "frames": frames,
        "sim_time": st["sim_time"],
        "root_height": st["root"]["height"],
        "up_z": st["root"]["up_z"],
        "max_qvel": st["max_qvel"],
        "finite": finite,
        "qpos_changed": max(abs(a - b) for a, b in
                            zip(qpos1, [float(v) for v in qpos0])) > 1e-9,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv=None):
    env_host = os.environ.get("PHYSICS_TWIN_HOST", DEFAULT_HOST)
    env_port = int(os.environ.get("PHYSICS_TWIN_PORT", str(DEFAULT_PORT)))

    p = argparse.ArgumentParser(
        description="MuJoCo physics twin of the aBot humanoid "
                    "(twin-control HTTP contract, visible physics window).")
    p.add_argument("--headless", action="store_true",
                   help="no viewer window (tests/automation/no display)")
    p.add_argument("--frames", type=int, default=None,
                   help="run only N 30fps ticks then exit (smoke/automation)")
    p.add_argument("--host", default=env_host,
                   help="HTTP bind host (default %s, env PHYSICS_TWIN_HOST)"
                        % env_host)
    p.add_argument("--port", type=int, default=env_port,
                   help="HTTP port (default %s, env PHYSICS_TWIN_PORT)"
                        % env_port)
    p.add_argument("--pose", default=None,
                   help="apply this static pose at start (frames smoke "
                        "default: relax)")
    p.add_argument("--motion", default=None,
                   help="play this motion at start instead of a static pose")
    p.add_argument("--duration", type=float, default=3.0,
                   help="duration for --motion")
    p.add_argument("--mjcf", default=None, help="MJCF path override")
    args = p.parse_args(argv)

    twin = PhysicsTwin(mjcf_path=args.mjcf)

    if args.frames is not None:
        # bounded smoke: seed an initial pose/motion so the state demonstrably
        # changes, run N frames (with HTTP up for optional interaction),
        # then report facts and exit
        pose = args.pose or (None if args.motion else "relax")
        if args.motion:
            twin.queue_motion(args.motion, args.duration)
        elif pose:
            twin.queue_pose(pose)
        twin.consume_commands()
        qpos0 = [float(v) for v in twin.adapter.data.qpos]
        mode = "headless" if args.headless else "gui"
        if args.headless:
            drive_loop(twin, frames=args.frames, realtime=False)
            rc = 0
        else:
            rc = run_gui(twin, frames=args.frames)
        facts = smoke_facts(twin, args.frames, qpos0, mode)
        print("SMOKE_JSON " + json.dumps(facts), flush=True)
        ok = rc == 0 and facts["finite"]
        print("SMOKE %s" % ("PASS" if ok else "FAIL"), flush=True)
        return 0 if ok else 1

    srv = start_http_server(twin, args.host, args.port)
    print("PHYSICS_TWIN_SERVER up on http://%s:%d  model=%s  mode=%s"
          % (args.host, args.port, twin.adapter.mjcf_path,
             "headless" if args.headless else "gui"), flush=True)
    try:
        if args.headless:
            stop_event = threading.Event()
            try:
                drive_loop(twin, frames=None, realtime=True,
                           stop_event=stop_event)
            except KeyboardInterrupt:
                pass
            finally:
                stop_event.set()
        else:
            print("watch the window: drag=orbit, scroll=zoom; closed-loop "
                  "balance keeps it standing against moderate pushes; "
                  "walk/run/nod play through the documented balance envelope "
                  "(P2-T4)", flush=True)
            run_gui(twin, frames=None)
    finally:
        srv.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
