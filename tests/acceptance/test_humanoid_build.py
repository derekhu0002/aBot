#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: humanoid build process is reproducible.

GIVEN the repo contains scripts/blender_humanoid/build_humanoid.py
WHEN the build is run headless with ABOT_HUMANOID_OUT_DIR pointed at a temp dir
THEN the build exits 0 and regenerates humanoid.blend plus both 1280x1280
preview PNGs into that dir (proving the model is fully script-reproducible
without touching the committed artifacts), and the regenerated blend ships
the nine baked keyframe Actions ActionRelax/ActionTPose/ActionAPose/
ActionIdle/ActionWave/ActionWalk/ActionNod/ActionLook/ActionRun, all
joint-only (pose-bone rotation_euler / FK root location F-curves), with one
Action already attached to the armature so GUI playback works out of the box.
"""

import json
import os
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BUILD_SCRIPT = os.path.join(REPO_ROOT, "scripts", "blender_humanoid", "build_humanoid.py")

ACTION_NAMES = ["ActionRelax", "ActionTPose", "ActionAPose", "ActionIdle",
                "ActionWave", "ActionWalk", "ActionNod", "ActionLook",
                "ActionRun"]

ACTIONS_CHECKER = r"""
import bpy, json, sys

argv = sys.argv[sys.argv.index("--") + 1:]
bpy.ops.wm.open_mainfile(filepath=argv[0])
expected = json.loads(argv[1])
arm = bpy.data.objects["HumanoidRig"]
ad = arm.animation_data

def action_fcurves(act):
    # Blender 4.4+ slotted Actions expose F-curves via layers/strips/
    # channelbags; legacy actions expose them flat.
    if hasattr(act, "fcurves"):
        try:
            return list(act.fcurves)
        except (AttributeError, RuntimeError):
            pass
    curves = []
    for layer in getattr(act, "layers", ()):
        for strip in layer.strips:
            for slot in getattr(act, "slots", ()):
                bag = strip.channelbag(slot)
                if bag is not None:
                    curves.extend(bag.fcurves)
    return curves

facts = {"default": ad.action.name if ad and ad.action else None,
         "actions": {}}
for n in expected:
    act = bpy.data.actions.get(n)
    info = {"exists": act is not None}
    if act is not None:
        curves = action_fcurves(act)
        info["joint_only"] = bool(curves) and all(
            fc.data_path.startswith("pose.bones[")
            and (fc.data_path.endswith("rotation_euler")
                 or fc.data_path.endswith("location"))
            for fc in curves)
    facts["actions"][n] = info
print("BUILD_ACTIONS_JSON " + json.dumps(facts))
"""


def find_blender():
    import shutil
    return shutil.which("blender") or r"D:\Programs\Blender\blender.exe"


def png_size(path):
    with open(path, "rb") as fh:
        data = fh.read(33)
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        return None
    width = int.from_bytes(data[16:20], "big")
    height = int.from_bytes(data[20:24], "big")
    return width, height


def main():
    blender = find_blender()
    if not os.path.exists(blender):
        print("FAIL: blender executable not found")
        return 1
    if not os.path.exists(BUILD_SCRIPT):
        print("FAIL: %s not found" % BUILD_SCRIPT)
        return 1

    out_dir = tempfile.mkdtemp(prefix="abot_humanoid_build_")
    env = dict(os.environ, ABOT_HUMANOID_OUT_DIR=out_dir)
    proc = subprocess.run([blender, "-b", "-P", BUILD_SCRIPT],
                          capture_output=True, text=True, timeout=900, env=env)
    if proc.returncode != 0:
        print("FAIL: build exit %d\n%s" % (proc.returncode, proc.stderr[-2000:]))
        return 1

    failures = []
    blend = os.path.join(out_dir, "humanoid.blend")
    if not os.path.exists(blend) or os.path.getsize(blend) < 10000:
        failures.append("humanoid.blend not regenerated in %s" % out_dir)
    for name in ("preview_front.png", "preview_3quarter.png"):
        path = os.path.join(out_dir, name)
        size = png_size(path) if os.path.exists(path) else None
        if size != (1280, 1280):
            failures.append("%s regenerated size=%r, want (1280,1280)" % (name, size))
    if "SAVED:" not in proc.stdout:
        failures.append("build log missing SAVED marker")

    if not failures:
        # Verify the regenerated blend ships the baked joint-only Actions.
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
            fh.write(ACTIONS_CHECKER)
            checker_path = fh.name
        try:
            aproc = subprocess.run(
                [blender, "-b", "-P", checker_path, "--", blend,
                 json.dumps(ACTION_NAMES)],
                capture_output=True, text=True, timeout=300)
        finally:
            os.unlink(checker_path)
        if aproc.returncode != 0:
            failures.append("actions checker exit %d\n%s"
                            % (aproc.returncode, aproc.stderr[-1500:]))
        else:
            line = next((l for l in aproc.stdout.splitlines()
                         if l.startswith("BUILD_ACTIONS_JSON ")), None)
            if line is None:
                failures.append("no BUILD_ACTIONS_JSON in checker output")
            else:
                # raw_decode tolerates Blender status text glued to the JSON
                afacts, _ = json.JSONDecoder().raw_decode(
                    line[len("BUILD_ACTIONS_JSON "):])
                if not (afacts.get("default") or "").startswith("Action"):
                    failures.append("regenerated blend has no default Action "
                                    "attached to the armature (GUI play would "
                                    "show nothing): %r" % afacts.get("default"))
                for n in ACTION_NAMES:
                    info = afacts["actions"].get(n, {})
                    if not info.get("exists"):
                        failures.append("regenerated blend missing baked Action %s" % n)
                    elif not info.get("joint_only"):
                        failures.append("baked Action %s is not joint-only "
                                        "(bone rotation / FK root location only)" % n)

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: build_humanoid.py reproducibly regenerates blend + previews + "
          "9 baked joint-only Actions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
