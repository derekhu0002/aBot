#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: twin-control MCP server contract (stage 2, no network).

GIVEN the repo contains scripts/blender_humanoid/twin_mcp_server.py — a FastMCP
(stdio) wrapper around twin_client with an injectable client
(set_client / create_server(client=...)) and an `if __name__ == "__main__"`
guard (importing never starts the stdio serve loop)
WHEN importing the module without Blender/GUI/network, injecting a stub
twin client, then listing tools via the FastMCP surface and calling each tool
handler directly and through mcp.call_tool
THEN the tool set is exactly {pose, motion, fk, stop, state, health} with
agent-facing descriptions, and every handler forwards to the client contract
(pose('relax') -> set_pose('relax'); motion('walk', 5) -> start_motion(
'walk', 5.0); fk('head', x, y, z) -> drive_bones({'head': [x, y, z]}) in
radians with degrees=True converted; stop() -> stop(); state()/health() read
back), invalid pose/motion/bone names and non-positive duration return clear
{"ok": false} errors WITHOUT any client call, client failures are converted to
{"ok": false} errors containing the twin_server start command
(blender --python scripts/blender_humanoid/twin_server.py) instead of raising,
and the client endpoint honors TWIN_HOST/TWIN_PORT overrides
(127.0.0.1:8123 by default) — external view only, no network, no GUI.
"""

import asyncio
import importlib
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS_DIR = os.path.join(ROOT, "scripts", "blender_humanoid")
MODULE_PATH = os.path.join(SCRIPTS_DIR, "twin_mcp_server.py")

EXPECTED_TOOLS = {"pose", "motion", "fk", "stop", "state", "health"}
START_HINT_TOKEN = "blender --python scripts/blender_humanoid/twin_server.py"


class StubTwinClient(object):
    """Records calls instead of doing any network I/O (twin_client contract)."""

    def __init__(self, raise_exc=None):
        self.calls = []
        self.raise_exc = raise_exc  # exception to raise from every call

    def _record(self, name, args):
        if self.raise_exc is not None:
            raise self.raise_exc
        self.calls.append((name, args))
        return {"queued": True}

    def health(self):
        if self.raise_exc is not None:
            raise self.raise_exc
        self.calls.append(("health", ()))
        return {"status": "ok", "model": "stub-model.blend", "bones": 19}

    def get_state(self):
        if self.raise_exc is not None:
            raise self.raise_exc
        self.calls.append(("get_state", ()))
        return {"bones": {"head": [0.0, 0.0, 0.0]}, "motion": None}

    def set_pose(self, name):
        return self._record("set_pose", (name,))

    def start_motion(self, name, duration=3.0):
        return self._record("start_motion", (name, duration))

    def drive_bones(self, bones):
        return self._record("drive_bones", (bones,))

    def stop(self):
        return self._record("stop", ())


def check_forwarding(tms, failures):
    """Handler forwarding + validation + robustness, all through a stub."""
    # pose -> set_pose
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.pose("relax")
    if not (res.get("ok") is True and stub.calls == [("set_pose", ("relax",))]):
        failures.append("pose('relax') -> res=%r calls=%r, want ok + set_pose('relax')"
                        % (res, stub.calls))

    # invalid pose name -> clear error, no client call
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.pose("wave")  # 'wave' is a MOTION, not a pose
    if res.get("ok") is not False or not res.get("error") or stub.calls:
        failures.append("pose('wave') must fail validation without client call, "
                        "got res=%r calls=%r" % (res, stub.calls))
    if "relax" not in res.get("error", ""):
        failures.append("pose validation error should list valid poses: %r" % res)

    # motion -> start_motion(name, duration)
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.motion("walk", 5)
    if not (res.get("ok") is True
            and stub.calls == [("start_motion", ("walk", 5.0))]):
        failures.append("motion('walk', 5) -> res=%r calls=%r, want ok + "
                        "start_motion('walk', 5.0)" % (res, stub.calls))

    # invalid motion name -> clear error, no client call
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.motion("fly", 3)
    if res.get("ok") is not False or not res.get("error") or stub.calls:
        failures.append("motion('fly', 3) must fail validation without client "
                        "call, got res=%r calls=%r" % (res, stub.calls))
    if "idle" not in res.get("error", ""):
        failures.append("motion validation error should list valid motions: %r" % res)

    # non-positive duration -> validation error
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.motion("wave", 0)
    if res.get("ok") is not False or stub.calls:
        failures.append("motion('wave', 0) must fail validation, got res=%r calls=%r"
                        % (res, stub.calls))

    # fk -> drive_bones({bone: [rx, ry, rz]}) in radians (twin_client contract)
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.fk("head", 0.1, 0.2, 0.3)
    if not (res.get("ok") is True
            and stub.calls == [("drive_bones", ({"head": [0.1, 0.2, 0.3]},))]):
        failures.append("fk('head', 0.1, 0.2, 0.3) -> res=%r calls=%r" % (res, stub.calls))

    # fk with degrees=True converts to radians before sending
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.fk("head", 90, 0, 0, degrees=True)
    sent = stub.calls and stub.calls[0][1][0]["head"] or None
    if not (res.get("ok") is True and sent is not None
            and abs(sent[0] - math.radians(90)) < 1e-9
            and sent[1] == 0.0 and sent[2] == 0.0):
        failures.append("fk degrees=True must convert 90deg->pi/2 rad, got calls=%r"
                        % (stub.calls,))

    # invalid bone name -> clear error listing valid bones, no client call
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.fk("wing.L", 0.0, 0.0, 0.0)
    if res.get("ok") is not False or not res.get("error") or stub.calls:
        failures.append("fk('wing.L', ...) must fail validation without client "
                        "call, got res=%r calls=%r" % (res, stub.calls))
    if "head" not in res.get("error", ""):
        failures.append("fk validation error should list valid bones: %r" % res)

    # stop -> stop()
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.stop()
    if not (res.get("ok") is True and stub.calls == [("stop", ())]):
        failures.append("stop() -> res=%r calls=%r" % (res, stub.calls))

    # state -> get_state() read-back
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.state()
    if not (res.get("ok") is True and stub.calls == [("get_state", ())]
            and res.get("result", {}).get("bones") == {"head": [0.0, 0.0, 0.0]}):
        failures.append("state() -> res=%r calls=%r" % (res, stub.calls))

    # health reachable -> model info surfaced
    stub = StubTwinClient()
    tms.set_client(stub)
    res = tms.health()
    if not (res.get("ok") is True and res.get("reachable") is True
            and res.get("result", {}).get("model") == "stub-model.blend"):
        failures.append("health() reachable -> res=%r" % (res,))

    # health unreachable -> ok=false + start-command hint (no crash)
    stub = StubTwinClient(raise_exc=ConnectionError("connection refused"))
    tms.set_client(stub)
    res = tms.health()
    if res.get("ok") is not False or res.get("reachable") is not False:
        failures.append("health() with failing client must be unreachable: %r" % (res,))
    if START_HINT_TOKEN not in res.get("hint", ""):
        failures.append("health() failure hint must contain %r, got %r"
                        % (START_HINT_TOKEN, res.get("hint")))

    # action tools must not crash either when the server is down
    stub = StubTwinClient(raise_exc=OSError("twin server down"))
    tms.set_client(stub)
    for label, res in (("pose", tms.pose("relax")),
                       ("motion", tms.motion("wave", 3)),
                       ("fk", tms.fk("head", 0.1, 0.0, 0.0)),
                       ("stop", tms.stop()),
                       ("state", tms.state())):
        if res.get("ok") is not False or START_HINT_TOKEN not in res.get("hint", ""):
            failures.append("%s() with failing client must return ok=false + "
                            "start hint, got %r" % (label, res))
    if stub.calls:
        failures.append("failing stub unexpectedly received calls: %r" % (stub.calls,))


def main():
    failures = []

    if not os.path.exists(MODULE_PATH):
        print("FAIL: %s not found" % MODULE_PATH)
        return 1

    with open(MODULE_PATH, encoding="utf-8") as fh:
        source = fh.read()
    if 'if __name__ == "__main__":' not in source:
        failures.append("twin_mcp_server.py lacks an `if __name__ == \"__main__\":` guard")

    if SCRIPTS_DIR not in sys.path:
        sys.path.insert(0, SCRIPTS_DIR)
    # Importing must NOT start the stdio serve loop (guarded by __main__).
    tms = importlib.import_module("twin_mcp_server")

    if not callable(getattr(tms, "create_server", None)):
        failures.append("twin_mcp_server.create_server missing or not callable")
    if not callable(getattr(tms, "set_client", None)):
        failures.append("twin_mcp_server.set_client missing or not callable")
    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    try:
        # ---- Tool surface: exactly the six contracted tools -----------------
        stub = StubTwinClient()
        server = tms.create_server(client=stub)
        tools = asyncio.run(server.list_tools())
        names = {t.name for t in tools}
        if names != EXPECTED_TOOLS:
            failures.append("tool set %s != expected %s"
                            % (sorted(names), sorted(EXPECTED_TOOLS)))
        for t in tools:
            if not (t.description or "").strip():
                failures.append("tool %r has no agent-facing description" % t.name)
        by_name = {t.name: t for t in tools}
        if names == EXPECTED_TOOLS:
            if "relax" not in by_name["pose"].description:
                failures.append("pose description should list valid poses")
            if "idle" not in by_name["motion"].description:
                failures.append("motion description should list valid motions")
            if "radian" not in by_name["fk"].description.lower():
                failures.append("fk description should document the radians contract")
            if "reachab" not in by_name["health"].description.lower():
                failures.append("health description should mention reachability")

        # ---- Handler forwarding / validation / robustness -------------------
        check_forwarding(tms, failures)

        # ---- MCP surface wiring: call_tool must reach the stub client -------
        stub = StubTwinClient()
        tms.set_client(stub)
        asyncio.run(server.call_tool("pose", {"name": "tpose"}))
        asyncio.run(server.call_tool("motion", {"name": "wave", "duration": 4.0}))
        if stub.calls != [("set_pose", ("tpose",)),
                          ("start_motion", ("wave", 4.0))]:
            failures.append("call_tool wiring wrong, stub calls=%r" % (stub.calls,))

        # ---- Endpoint overrides: TWIN_HOST/TWIN_PORT + default --------------
        tms.set_client(None)
        old_host, old_port = os.environ.get("TWIN_HOST"), os.environ.get("TWIN_PORT")
        try:
            os.environ["TWIN_HOST"] = "10.0.0.9"
            os.environ["TWIN_PORT"] = "9999"
            client = tms.get_client()
            if getattr(client, "base", None) != "http://10.0.0.9:9999":
                failures.append("TWIN_HOST/TWIN_PORT override ignored: base=%r"
                                % getattr(client, "base", None))
            tms.set_client(None)
            os.environ.pop("TWIN_HOST", None)
            os.environ.pop("TWIN_PORT", None)
            client = tms.get_client()
            if getattr(client, "base", None) != "http://127.0.0.1:8123":
                failures.append("default endpoint must be http://127.0.0.1:8123, "
                                "got %r" % getattr(client, "base", None))
        finally:
            if old_host is None:
                os.environ.pop("TWIN_HOST", None)
            else:
                os.environ["TWIN_HOST"] = old_host
            if old_port is None:
                os.environ.pop("TWIN_PORT", None)
            else:
                os.environ["TWIN_PORT"] = old_port
            tms.set_client(None)
    finally:
        tms.set_client(None)

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    print("PASS: twin-control MCP server contract (tools, forwarding, "
          "validation, robustness, endpoint env) holds with stub client")
    return 0


if __name__ == "__main__":
    sys.exit(main())
