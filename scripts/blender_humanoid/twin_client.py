"""Agent-side client for the aBot humanoid digital twin control server.

Any agent process on the same machine can import this and drive the Blender
model in real time:

    from twin_client import TwinClient
    twin = TwinClient()                       # 127.0.0.1:8123
    twin.set_pose("relax")
    twin.start_motion("wave", 4.0)
    twin.drive_bones({"upper_arm.R": [0.0, 0.0, -1.57]})
    state = twin.get_state()                  # read current bone rotations

Only the Python standard library is used (urllib), so no extra dependencies.
"""

import json
import urllib.request
import urllib.error


class TwinClient:
    def __init__(self, host="127.0.0.1", port=8123, timeout=3.0):
        self.base = f"http://{host}:{port}"
        self.timeout = timeout

    # -- low-level ----------------------------------------------------------
    def _request(self, method, path, payload=None):
        data = json.dumps(payload).encode("utf-8") if payload is not None else None
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def _get(self, path):
        return self._request("GET", path)

    def _post(self, path, payload=None):
        return self._request("POST", path, payload or {})

    # -- API ----------------------------------------------------------------
    def health(self):
        return self._get("/health")

    def get_state(self):
        return self._get("/state")

    def set_pose(self, name):
        """Apply a static pose: relax | tpose | apose."""
        return self._post("/pose", {"name": name})

    def start_motion(self, name, duration=3.0):
        """Play a time-based motion: idle | wave | walk | nod | look | run."""
        return self._post("/motion", {"name": name, "duration": duration})

    def drive_bones(self, bones):
        """Raw FK drive: {"bone_name": [rx, ry, rz]} in radians."""
        return self._post("/bones", {"bones": bones})

    def stop(self):
        return self._post("/stop")

    def __repr__(self):
        return f"<TwinClient {self.base}>"
