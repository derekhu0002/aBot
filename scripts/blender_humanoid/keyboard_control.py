#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keyboard control entry for the aBot humanoid digital twin.

Real-time keyboard -> twin_client control of the Blender digital twin:
press a key, the mapped pose/motion/stop command is sent immediately to the
twin control server (HTTP 127.0.0.1:8123).

Usage (two terminals):

    # 1) start the twin server inside Blender (keep the window open):
    blender --python scripts/blender_humanoid/twin_server.py

    # 2) start this keyboard controller:
    python scripts/blender_humanoid/keyboard_control.py

Key map (single source of truth = KEY_MAP; the help screen is generated from
it so help and mapping can never drift apart):

    SPACE  play idle            1 relax   2 tpose   3 apose
    4 wave  5 nod  6 look       7 walk    8 run     9 apose
    0 stop (回到驱动前状态)      h help    q quit

Design notes:
- key_to_command() is a PURE function (no I/O, no client) so the mapping is
  unit-testable without Blender/network; see tests/acceptance/
  test_keyboard_control.py.
- Only the Python standard library is used: msvcrt on Windows, termios/tty
  elsewhere (with a plain-readline fallback when stdin is not a TTY).
- Importing this module never starts the loop; it is guarded by
  `if __name__ == "__main__":`.
"""

import argparse
import os
import sys

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8123

# Time-based motions get a playback duration: interactive one-shots 3s,
# locomotion (walk/run) 6s so they can be watched for a while.
MOTION_DURATIONS = {
    "idle": 3.0,
    "wave": 3.0,
    "nod": 3.0,
    "look": 3.0,
    "walk": 6.0,
    "run": 6.0,
}

# Single source of truth: key -> (command_type, args_tuple)
#   ("pose",   (name,))            -> TwinClient.set_pose(name)
#   ("motion", (name, duration))   -> TwinClient.start_motion(name, duration)
#   ("stop",   ())                 -> TwinClient.stop()
#   ("help",   ()) / ("quit", ())  -> local control flow
KEY_MAP = {
    " ": ("motion", ("idle", MOTION_DURATIONS["idle"])),
    "1": ("pose", ("relax",)),
    "2": ("pose", ("tpose",)),
    "3": ("pose", ("apose",)),
    "4": ("motion", ("wave", MOTION_DURATIONS["wave"])),
    "5": ("motion", ("nod", MOTION_DURATIONS["nod"])),
    "6": ("motion", ("look", MOTION_DURATIONS["look"])),
    "7": ("motion", ("walk", MOTION_DURATIONS["walk"])),
    "8": ("motion", ("run", MOTION_DURATIONS["run"])),
    "9": ("pose", ("apose",)),
    "0": ("stop", ()),
    "h": ("help", ()),
    "q": ("quit", ()),
}

KEY_DESCRIPTIONS = {
    " ": "idle —— 呼吸待机（时间动作 3s）",
    "1": "relax —— 放松站姿",
    "2": "tpose —— T-pose 标定姿势",
    "3": "apose —— A-pose 标定姿势",
    "4": "wave —— 挥手（解剖学右臂，3s）",
    "5": "nod —— 点头（3s）",
    "6": "look —— 左右转头（3s）",
    "7": "walk —— 原地行走（6s）",
    "8": "run —— 原地跑（6s）",
    "9": "apose —— A-pose（同 3）",
    "0": "stop —— 停止动作播放",
    "h": "显示本帮助",
    "q": "退出键盘控制",
}

SERVER_HINT = (
    "请先在另一个终端启动 twin_server（保持该 Blender 窗口打开）：\n"
    "    blender --python scripts/blender_humanoid/twin_server.py\n"
    "然后再重新运行本键盘控制器。"
)


# --------------------------------------------------------------------------
# Pure mapping layer (no I/O) — externally tested by
# tests/acceptance/test_keyboard_control.py
# --------------------------------------------------------------------------
def key_to_command(key):
    """Map a single key press to a twin command.

    Returns (command_type, args_tuple) as stored in KEY_MAP, or None for
    unknown/invalid keys. Letter keys are case-insensitive. Pure function:
    no I/O, no client, no network.
    """
    if not isinstance(key, str) or len(key) != 1:
        return None
    return KEY_MAP.get(key.lower())


def key_label(key):
    """Human-readable label for a key, used by the help screen and echoes."""
    if key == " ":
        return "SPACE"
    return key.upper()


def help_text():
    """Help screen, generated from KEY_MAP so it always matches the mapping."""
    lines = [
        "=" * 62,
        "aBot 数字孪生键盘控制（twin-control）",
        "=" * 62,
    ]
    for key in KEY_MAP:
        lines.append("  %-6s %s" % (key_label(key), KEY_DESCRIPTIONS.get(key, "")))
    lines.append("-" * 62)
    lines.append("按键即时生效；未列出的按键会被忽略。")
    return "\n".join(lines)


def dispatch_command(client, command):
    """Execute a mapped command against a twin client.

    Returns 'ok' for pose/motion/stop, 'help'/'quit' for local control flow,
    'unknown' otherwise. Network errors are propagated to the caller.
    """
    ctype, args = command
    if ctype == "pose":
        client.set_pose(*args)
        return "ok"
    if ctype == "motion":
        client.start_motion(*args)
        return "ok"
    if ctype == "stop":
        client.stop()
        return "ok"
    if ctype == "help":
        return "help"
    if ctype == "quit":
        return "quit"
    return "unknown"


# --------------------------------------------------------------------------
# Keyboard reading (platform specific, standard library only)
# --------------------------------------------------------------------------
def _read_key_windows():
    import msvcrt

    ch = msvcrt.getwch()
    if ch in ("\x00", "\xe0"):
        # Special key (arrow etc.): consume the scan code and ignore it.
        msvcrt.getwch()
        return None
    return ch


def _read_key_posix():
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # one keypress -> one char, still Ctrl+C capable
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_key():
    """Blocking single-key read. Returns the key char, or None for ignored
    special keys. Falls back to line reading when stdin is not a TTY."""
    if os.name == "nt":
        return _read_key_windows()
    try:
        return _read_key_posix()
    except Exception:
        line = sys.stdin.readline()
        if not line:  # EOF -> quit instead of busy-looping
            return "q"
        return line[:1]


# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Keyboard control for the aBot digital twin (twin-control)."
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="twin_server host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="twin_server port")
    args = parser.parse_args(argv)

    # Lazy import keeps the pure mapping layer importable anywhere.
    from twin_client import TwinClient

    client = TwinClient(host=args.host, port=args.port)
    try:
        health = client.health()
    except Exception as exc:
        print("[!] 无法连接 twin_server（http://%s:%d）：%s" % (args.host, args.port, exc))
        print(SERVER_HINT)
        return 1
    print("[ok] 已连接 twin_server：%s" % health)
    print(help_text(), flush=True)
    print("等待按键 …（h=帮助，q=退出）", flush=True)

    try:
        while True:
            key = read_key()
            if key is None:
                continue
            command = key_to_command(key)
            if command is None:
                print("[?] 忽略未映射按键 %r（h 查看帮助）" % key, flush=True)
                continue
            ctype, cargs = command
            try:
                status = dispatch_command(client, command)
            except Exception as exc:
                print("[!] 指令失败：%s" % exc, flush=True)
                continue
            if status == "quit":
                print("再见。", flush=True)
                return 0
            if status == "help":
                print(help_text(), flush=True)
                continue
            if ctype == "pose":
                print("[ok] pose   -> %s" % cargs[0], flush=True)
            elif ctype == "motion":
                print("[ok] motion -> %s (%.1fs)" % (cargs[0], cargs[1]), flush=True)
            elif ctype == "stop":
                print("[ok] stop", flush=True)
    except KeyboardInterrupt:
        print("\n中断，再见。", flush=True)
        return 0


if __name__ == "__main__":
    sys.exit(main())
