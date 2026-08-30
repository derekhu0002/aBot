"""Blender-side local control server for the aBot humanoid digital twin.

Runs inside Blender, loads the humanoid model, and exposes a small HTTP API on
127.0.0.1 so that any agent process on the same machine can manipulate the
model in real time.

Run (GUI, so you can watch the model move):
    blender --python scripts/blender_humanoid/twin_server.py

Run (headless, for testing):
    blender -b -P scripts/blender_humanoid/twin_server.py

API (JSON):
    GET  /health  -> {"status": "ok", "model": "...", "bones": n}
    GET  /state   -> {"bones": {name: [x,y,z], ...}, "motion": name|None}
    POST /pose    {"name": "relax"|"tpose"|"apose"}            static pose
    POST /motion  {"name": "idle"|"wave"|"walk"|"nod"|"look"|"run",
                   "duration": seconds}                         time-based motion
    POST /bones   {"bones": {"upper_arm.R": [0,0,1.57], ...}}   raw FK drive
    POST /stop    {}                                            stop current motion

bpy API calls happen only on the main thread (bpy.app.timers); the HTTP threads
only enqueue commands / read a cached state, so it is thread-safe.
"""

import json
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import bpy

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import humanoid_control as hc

HOST = "127.0.0.1"
PORT = 8123
BLEND = r"D:\Projects\aBot\assets\humanoid\humanoid.blend"
ARM_NAME = "HumanoidRig"
TICK = 1.0 / 30.0  # 30 fps drive loop

# ---------------------------------------------------------------------------
# Shared state: commands produced by HTTP threads, consumed by the main-thread
# timer.  state_cache is refreshed on the main thread for safe reads.
# ---------------------------------------------------------------------------
cmd_queue = queue.Queue()
motion_state = {"name": None, "start": 0.0, "duration": 0.0}
state_cache = {"bones": {}, "motion": None}


def refresh_state():
    arm = bpy.data.objects.get(ARM_NAME)
    if arm is None:
        return
    d = {}
    for p in arm.pose.bones:
        d[p.name] = [round(v, 4) for v in p.rotation_euler]
    state_cache["bones"] = d
    state_cache["motion"] = motion_state["name"]


def apply_pose(name):
    fn = getattr(hc, "pose_" + name, None)
    if fn is None:
        return False
    arm = bpy.data.objects.get(ARM_NAME)
    if arm is None:
        return False
    fn(arm)
    return True


def start_motion(name, duration):
    if not hasattr(hc, "apply_" + name):
        return False
    arm = bpy.data.objects.get(ARM_NAME)
    if arm is None:
        return False
    motion_state.update(name=name, start=time.time(),
                        duration=max(0.0, duration))
    return True


def drive_bones(bones):
    arm = bpy.data.objects.get(ARM_NAME)
    if arm is None:
        return False
    hc.reset_pose(arm)
    for name, euler in bones.items():
        if name in arm.pose.bones:
            hc.set_bone(arm, name, euler)
    return True


def drive_once():
    """Main-thread driver: consume commands, then apply the active motion.

    Runs on the main thread only — registered as a timer in GUI mode, or
    called manually by the keep-alive loop in headless mode.
    """
    while not cmd_queue.empty():
        typ, payload = cmd_queue.get()
        if typ == "pose":
            if apply_pose(payload.get("name", "")):
                motion_state["name"] = None
        elif typ == "motion":
            start_motion(payload.get("name", ""),
                         payload.get("duration", 3.0))
        elif typ == "bones":
            if drive_bones(payload.get("bones", {})):
                motion_state["name"] = None
        elif typ == "stop":
            motion_state["name"] = None
            arm = bpy.data.objects.get(ARM_NAME)
            if arm is not None:
                hc.reset_pose(arm)

    arm = bpy.data.objects.get(ARM_NAME)
    name = motion_state["name"]
    if arm is not None and name is not None:
        t = time.time() - motion_state["start"]
        if motion_state["duration"] > 0 and t > motion_state["duration"]:
            motion_state["name"] = None
            hc.reset_pose(arm)
        else:
            getattr(hc, "apply_" + name)(arm, t)

    refresh_state()


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
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
            arm = bpy.data.objects.get(ARM_NAME)
            self._reply(200, {"status": "ok",
                              "model": BLEND,
                              "bones": len(arm.pose.bones) if arm else 0})
        elif self.path.startswith("/state"):
            self._reply(200, state_cache)
        else:
            self._reply(404, {"error": "not found"})

    def do_POST(self):
        body = self._body()
        if self.path.startswith("/pose"):
            cmd_queue.put(("pose", body))
            self._reply(200, {"queued": True, "name": body.get("name")})
        elif self.path.startswith("/motion"):
            cmd_queue.put(("motion", body))
            self._reply(200, {"queued": True, "name": body.get("name"),
                              "duration": body.get("duration", 3.0)})
        elif self.path.startswith("/bones"):
            cmd_queue.put(("bones", body))
            self._reply(200, {"queued": True, "bones": len(body.get("bones", {}))})
        elif self.path.startswith("/stop"):
            cmd_queue.put(("stop", body))
            self._reply(200, {"queued": True})
        else:
            self._reply(404, {"error": "not found"})

    def log_message(self, *args):
        pass  # silence request logging


def main():
    hc.load_humanoid(BLEND)
    hc.reset_pose(bpy.data.objects[ARM_NAME])

    srv = ThreadingHTTPServer((HOST, PORT), Handler)
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    print(f"TWIN_SERVER up on http://{HOST}:{PORT}  model={BLEND}", flush=True)

    if bpy.app.background:
        # Headless: timers do not fire without the main loop, so drive manually
        try:
            while True:
                drive_once()
                time.sleep(TICK)
        except KeyboardInterrupt:
            srv.shutdown()
    else:
        # GUI: Blender's main loop runs, so drive via a timer
        def _timer():
            drive_once()
            return TICK
        bpy.app.timers.register(_timer)


if __name__ == "__main__":
    main()
