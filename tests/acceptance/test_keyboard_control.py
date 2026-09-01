#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: keyboard control entry for the digital twin (pure mapping).

GIVEN the repo contains scripts/blender_humanoid/keyboard_control.py with a
pure key->twin-command mapping and an `if __name__ == "__main__"` guard (so
importing the module never starts the blocking keyboard loop)
WHEN importing the module without Blender/GUI/network and feeding each
documented key into key_to_command, then dispatching commands against a stub
client
THEN every documented key maps to the exact twin command
(space=idle, 1=relax, 2=tpose, 3=apose, 4=wave, 5=nod, 6=look, 7=walk,
8=run, 9=apose, 0=stop, h=help, q=quit), letter keys are case-insensitive,
unknown keys return None, the help screen covers exactly the mapped keys,
and dispatch_command drives the (stub) TwinClient with the correct calls
(set_pose / start_motion(name, duration) / stop) — external view only,
no network and no running twin_server required.
"""

import importlib
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS_DIR = os.path.join(ROOT, "scripts", "blender_humanoid")
MODULE_PATH = os.path.join(SCRIPTS_DIR, "keyboard_control.py")

# External contract: documented key -> (command_type, args tuple).
# pose args -> TwinClient.set_pose(name); motion args ->
# TwinClient.start_motion(name, duration); stop/help/quit take no args.
EXPECTED_COMMANDS = {
    " ": ("motion", ("idle", 3.0)),
    "1": ("pose", ("relax",)),
    "2": ("pose", ("tpose",)),
    "3": ("pose", ("apose",)),
    "4": ("motion", ("wave", 3.0)),
    "5": ("motion", ("nod", 3.0)),
    "6": ("motion", ("look", 3.0)),
    "7": ("motion", ("walk", 6.0)),
    "8": ("motion", ("run", 6.0)),
    "9": ("pose", ("apose",)),
    "0": ("stop", ()),
    "h": ("help", ()),
    "q": ("quit", ()),
}
UNKNOWN_KEYS = ["x", "z", "", "\x00", "\xe0", "?"]


class StubTwinClient(object):
    """Records calls instead of doing any network I/O."""

    def __init__(self):
        self.calls = []

    def set_pose(self, name):
        self.calls.append(("set_pose", (name,)))

    def start_motion(self, name, duration=3.0):
        self.calls.append(("start_motion", (name, duration)))

    def stop(self):
        self.calls.append(("stop", ()))


def main():
    failures = []

    if not os.path.exists(MODULE_PATH):
        print("FAIL: %s not found" % MODULE_PATH)
        return 1

    with open(MODULE_PATH, encoding="utf-8") as fh:
        source = fh.read()
    if 'if __name__ == "__main__":' not in source:
        failures.append("keyboard_control.py lacks an `if __name__ == \"__main__\":` guard")

    if SCRIPTS_DIR not in sys.path:
        sys.path.insert(0, SCRIPTS_DIR)
    # Importing must NOT start the blocking keyboard loop: if it did, this
    # test would hang instead of finishing.
    kc = importlib.import_module("keyboard_control")

    if not callable(getattr(kc, "key_to_command", None)):
        failures.append("keyboard_control.key_to_command missing or not callable")
    if not callable(getattr(kc, "dispatch_command", None)):
        failures.append("keyboard_control.dispatch_command missing or not callable")
    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    # 1) Full key->command table (external contract).
    for key, expected in sorted(EXPECTED_COMMANDS.items(), key=lambda kv: kv[0]):
        actual = kc.key_to_command(key)
        if actual != expected:
            failures.append(
                "key_to_command(%r) = %r, want %r" % (key, actual, expected)
            )

    # 2) Letter keys are case-insensitive.
    for key in ("h", "q"):
        if kc.key_to_command(key.upper()) != EXPECTED_COMMANDS[key]:
            failures.append(
                "key_to_command(%r) not case-insensitive (want %r)"
                % (key.upper(), EXPECTED_COMMANDS[key])
            )

    # 3) Unknown keys map to None (ignored by the loop).
    for key in UNKNOWN_KEYS:
        if kc.key_to_command(key) is not None:
            failures.append(
                "key_to_command(%r) = %r, want None for unknown key"
                % (key, kc.key_to_command(key))
            )

    # 4) Help screen covers exactly the mapped keys (help <-> mapping consistent).
    mapped_keys = set(EXPECTED_COMMANDS.keys())
    module_keys = set(getattr(kc, "KEY_MAP", {}).keys())
    if module_keys != mapped_keys:
        failures.append(
            "KEY_MAP keys %s != documented keys %s"
            % (sorted(module_keys), sorted(mapped_keys))
        )
    help_text = kc.help_text()
    for key in mapped_keys:
        label = kc.key_label(key) if callable(getattr(kc, "key_label", None)) else key
        if label not in help_text:
            failures.append("help screen missing key label %r" % label)

    # 5) dispatch_command drives a (stub) twin client with the right calls.
    stub = StubTwinClient()
    status = kc.dispatch_command(stub, ("pose", ("relax",)))
    if status != "ok" or stub.calls != [("set_pose", ("relax",))]:
        failures.append("dispatch pose failed: status=%r calls=%r" % (status, stub.calls))

    stub = StubTwinClient()
    status = kc.dispatch_command(stub, ("motion", ("walk", 6.0)))
    if status != "ok" or stub.calls != [("start_motion", ("walk", 6.0))]:
        failures.append("dispatch motion failed: status=%r calls=%r" % (status, stub.calls))

    stub = StubTwinClient()
    status = kc.dispatch_command(stub, ("stop", ()))
    if status != "ok" or stub.calls != [("stop", ())]:
        failures.append("dispatch stop failed: status=%r calls=%r" % (status, stub.calls))

    stub = StubTwinClient()
    if kc.dispatch_command(stub, ("quit", ())) != "quit" or stub.calls:
        failures.append("dispatch quit must return 'quit' without client calls")
    if kc.dispatch_command(stub, ("help", ())) != "help" or stub.calls:
        failures.append("dispatch help must return 'help' without client calls")

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    print("PASS: keyboard control pure mapping matches the twin command contract")
    return 0


if __name__ == "__main__":
    sys.exit(main())
