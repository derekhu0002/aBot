#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: humanoid build process is reproducible.

GIVEN the repo contains scripts/blender_humanoid/build_humanoid.py
WHEN the build is run headless with ABOT_HUMANOID_OUT_DIR pointed at a temp dir
THEN the build exits 0 and regenerates humanoid.blend plus both 1280x1280
preview PNGs into that dir (proving the model is fully script-reproducible
without touching the committed artifacts).
"""

import os
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BUILD_SCRIPT = os.path.join(REPO_ROOT, "scripts", "blender_humanoid", "build_humanoid.py")


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
            failures.append("%s regenerated size=%r, want (1280, 1280)" % (name, size))
    if "SAVED:" not in proc.stdout:
        failures.append("build log missing SAVED marker")

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: build_humanoid.py reproducibly regenerates blend + previews")
    return 0


if __name__ == "__main__":
    sys.exit(main())
