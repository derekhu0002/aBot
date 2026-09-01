#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MCP server exposing the aBot humanoid digital twin to Agents (stage 2).

Stage ② of abot-dev-process-001: wraps twin_client.py (pure-stdlib HTTP client
for the Blender-side twin_server on 127.0.0.1:8123) as a FastMCP server with
stdio transport, so any opencode Agent can drive the digital twin through MCP
tool calls.

Tools (all return a JSON-able dict with an "ok" flag):
    pose(name)               static pose: relax | tpose | apose
    motion(name, duration)   timed motion: idle | wave | walk | nod | look | run
    fk(bone, x, y, z)        raw FK bone drive — local Euler angles; RADIANS by
                             default (twin_client contract), pass degrees=True
                             to give degrees (converted to radians before send)
    stop()                   stop the current motion and reset the pose
    state()                  read back all bone rotations + active motion
    health()                 twin_server reachability + model info

Robustness: when twin_server is not running, every tool returns a clear
error dict that includes the server start command — it never crashes:
    blender --python scripts/blender_humanoid/twin_server.py

Testability: handlers call through a module-level injectable client.
Tests inject a stub via set_client(stub) or create_server(client=stub) and
assert forwarding without a real twin_server / network.

Registered as the local MCP server "twin-control" in opencode.json:
    {"mcp": {"twin-control": {"type": "local",
      "command": ["python", "scripts/blender_humanoid/twin_mcp_server.py"],
      "enabled": true}}}

Run manually (stdio, blocks serving over stdin/stdout):
    python scripts/blender_humanoid/twin_mcp_server.py
"""

import math
import os

from mcp.server.fastmcp import FastMCP

# ---------------------------------------------------------------------------
# External contract — must stay in sync with twin_server.py / humanoid_control.py
# ---------------------------------------------------------------------------
VALID_POSES = ("relax", "tpose", "apose")
VALID_MOTIONS = ("idle", "wave", "walk", "nod", "look", "run")
# HumanoidRig bones (scripts/blender_humanoid/build_humanoid.py).
VALID_BONES = frozenset((
    "root", "spine", "chest", "neck", "head",
    "shoulder.L", "shoulder.R",
    "upper_arm.L", "upper_arm.R",
    "forearm.L", "forearm.R",
    "hand.L", "hand.R",
    "thigh.L", "thigh.R",
    "shin.L", "shin.R",
    "foot.L", "foot.R",
))
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8123
SERVER_START_HINT = (
    "twin_server unreachable. Start it first (GUI Blender so you can watch "
    "the model move):  blender --python scripts/blender_humanoid/twin_server.py"
)
SERVER_NAME = "twin-control"

INSTRUCTIONS = (
    "Control the aBot humanoid digital twin running in Blender via twin_server "
    "(HTTP on 127.0.0.1:8123). Prerequisite: the twin server must be running — "
    "start it with: blender --python scripts/blender_humanoid/twin_server.py. "
    "Use health() to check reachability, pose()/motion() for high-level "
    "animation, fk() for raw bone drives, state() to read back bone rotations, "
    "stop() to halt. Every tool returns a dict with ok=true on success or "
    "ok=false plus error/hint on failure."
)

# Injectable client (testability). None => lazily build the real TwinClient
# from the TWIN_HOST / TWIN_PORT environment variables on first use.
_client = None


def set_client(client):
    """Inject the twin client instance (tests swap in a stub; None resets)."""
    global _client
    _client = client


def get_client():
    """Return the injected client, else lazily build the real TwinClient."""
    global _client
    if _client is None:
        # Lazy import keeps this module importable anywhere and the client
        # construction free of import-time side effects.
        from twin_client import TwinClient
        host = os.environ.get("TWIN_HOST", DEFAULT_HOST)
        port = int(os.environ.get("TWIN_PORT", str(DEFAULT_PORT)))
        _client = TwinClient(host=host, port=port)
    return _client


def _validation_error(message):
    """Parameter-validation failure (no start hint: not a connectivity issue)."""
    return {"ok": False, "error": message}


def _connection_error(label, exc):
    return {"ok": False,
            "error": "%s failed: %s" % (label, exc),
            "hint": SERVER_START_HINT}


def _call(label, fn):
    """Run a client call, converting any failure into a clear error dict."""
    try:
        return {"ok": True, "result": fn()}
    except Exception as exc:  # noqa: BLE001 - MCP tools must never crash
        return _connection_error(label, exc)


# ---------------------------------------------------------------------------
# MCP tool handlers (plain functions so tests can call them directly)
# ---------------------------------------------------------------------------
def pose(name):
    """Apply a static pose to the digital twin humanoid.

    Args:
        name: pose name, one of: relax | tpose | apose.

    Returns:
        {"ok": true, "result": <twin_server reply>} on success;
        {"ok": false, "error": ...} for an invalid pose name;
        {"ok": false, "error": ..., "hint": <start command>} when twin_server
        is not running.
    """
    if name not in VALID_POSES:
        return _validation_error(
            "invalid pose %r; valid poses: %s" % (name, ", ".join(VALID_POSES)))
    return _call("pose(%s)" % name, lambda: get_client().set_pose(name))


def motion(name, duration=3.0):
    """Play a time-based motion on the digital twin for `duration` seconds.

    Args:
        name: motion name, one of: idle | wave | walk | nod | look | run.
        duration: how long the motion plays, in seconds (> 0).

    Returns:
        {"ok": true, "result": <twin_server reply>} on success;
        {"ok": false, "error": ...} for an invalid motion name or duration;
        {"ok": false, "error": ..., "hint": <start command>} when twin_server
        is not running.
    """
    if name not in VALID_MOTIONS:
        return _validation_error(
            "invalid motion %r; valid motions: %s"
            % (name, ", ".join(VALID_MOTIONS)))
    try:
        duration = float(duration)
    except (TypeError, ValueError):
        return _validation_error(
            "invalid duration %r; must be a number of seconds" % (duration,))
    if duration <= 0:
        return _validation_error("duration must be > 0 seconds, got %r" % duration)
    return _call("motion(%s, %ss)" % (name, duration),
                 lambda: get_client().start_motion(name, duration))


def fk(bone, x, y, z, degrees=False):
    """Raw FK drive: rotate one rig bone by a local Euler angle.

    Angles are RADIANS by default — the twin_client contract
    (drive_bones {bone: [rx, ry, rz]} in radians). Pass degrees=True to give
    degrees; they are converted to radians before being sent.
    Bone axes follow humanoid_control.py conventions (e.g. head local X =
    pitch/nod, local Y = yaw, local Z = roll; '.L' chain = anatomical right).

    Args:
        bone: rig bone name, one of root, spine, chest, neck, head,
            shoulder.L/R, upper_arm.L/R, forearm.L/R, hand.L/R, thigh.L/R,
            shin.L/R, foot.L/R.
        x, y, z: local Euler rotation components.
        degrees: when true, interpret x/y/z as degrees instead of radians.

    Returns:
        {"ok": true, "result": <twin_server reply>} on success;
        {"ok": false, "error": ...} for an invalid bone name or angle value;
        {"ok": false, "error": ..., "hint": <start command>} when twin_server
        is not running.
    """
    if bone not in VALID_BONES:
        return _validation_error(
            "invalid bone %r; valid bones: %s"
            % (bone, ", ".join(sorted(VALID_BONES))))
    try:
        rx, ry, rz = float(x), float(y), float(z)
    except (TypeError, ValueError):
        return _validation_error(
            "invalid Euler angles (%r, %r, %r); must be numbers" % (x, y, z))
    if degrees:
        rx, ry, rz = math.radians(rx), math.radians(ry), math.radians(rz)
    return _call("fk(%s)" % bone,
                 lambda: get_client().drive_bones({bone: [rx, ry, rz]}))


def stop():
    """Stop the current motion and reset the twin to the rest pose.

    Returns:
        {"ok": true, "result": <twin_server reply>} on success;
        {"ok": false, "error": ..., "hint": <start command>} when twin_server
        is not running.
    """
    return _call("stop", lambda: get_client().stop())


def state():
    """Read back the model state: all bone rotations + the active motion.

    Returns:
        {"ok": true, "result": {"bones": {name: [x, y, z], ...},
        "motion": name|null}} on success;
        {"ok": false, "error": ..., "hint": <start command>} when twin_server
        is not running.
    """
    return _call("state", lambda: get_client().get_state())


def health():
    """Health check: is twin_server reachable, and what model does it serve?

    Returns:
        {"ok": true, "reachable": true, "result": {"status": "ok",
        "model": ..., "bones": n}} when the server answers;
        {"ok": false, "reachable": false, "error": ...,
        "hint": <start command>} when it is not running.
    """
    res = _call("health", lambda: get_client().health())
    res["reachable"] = res["ok"]
    return res


# ---------------------------------------------------------------------------
# Server factory / entry point
# ---------------------------------------------------------------------------
def create_server(client=None):
    """Build the FastMCP (stdio) server; optionally inject a client (tests).

    Args:
        client: optional twin client instance (e.g. a stub) injected via
            set_client before the tools are registered. None keeps/uses the
            lazily constructed real TwinClient.
    """
    if client is not None:
        set_client(client)
    mcp = FastMCP(SERVER_NAME, instructions=INSTRUCTIONS)
    for fn in (pose, motion, fk, stop, state, health):
        mcp.tool()(fn)
    return mcp


def main():
    create_server().run("stdio")


if __name__ == "__main__":
    main()
