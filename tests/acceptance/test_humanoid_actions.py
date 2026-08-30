#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: twin-control key actions drive the chibi twin correctly.

GIVEN the repo contains assets/humanoid/humanoid.blend (rigid per-part
skinned chibi robot) and scripts/blender_humanoid/humanoid_control.py
WHEN each key action (static poses relax/tpose/apose and time motions
idle/wave/walk/nod/look/run) is driven headless through the control module
and the resulting rig state is observed externally (pose-bone local Euler
rotations + world-space bone head/tail, the same observables the /state
endpoint of twin_server exposes), plus the committed evidence stills
assets/humanoid/action_<name>.png
THEN every action produces its identifiable signature:
  - relax: arms hang at the sides, slightly bent (hands low, |x| small);
  - tpose: both arms horizontal at shoulder height, stretched sideways;
  - apose: both arms angled ~45 deg down-out between relax and tpose;
  - idle: subtle breathing (chest pitch oscillation > 0) with relaxed arms;
  - wave: right upper arm abducted ~90 deg, forearm folded straight UP
    (hand above shoulder), left arm stays low, hand rocking;
  - walk: opposite-phase thigh swing >= 0.4 rad, one foot clearly lifted,
    vertical root bob > 0;
  - nod: head pitch amplitude >= 0.2 rad;
  - look: head yaw amplitude >= 0.45 rad;
  - run: thigh swing >= 0.6 rad and forward lean >= 0.2 rad (both strictly
    stronger than walk) with a high heel kick and stronger bob;
  - and each action_<name>.png exists as a 1280x1280 PNG evidence still.
"""

import json
import os
import shutil
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BLEND_PATH = os.path.join(REPO_ROOT, "assets", "humanoid", "humanoid.blend")
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts", "blender_humanoid")
EVIDENCE = [os.path.join(REPO_ROOT, "assets", "humanoid", "action_%s.png" % n)
            for n in ("relax", "tpose", "apose", "idle", "wave",
                      "walk", "nod", "look", "run")]

CHECKER = r"""
import json
import os
import sys

argv = sys.argv[sys.argv.index("--") + 1:]
blend_path, scripts_dir = argv[0], argv[1]
sys.path.insert(0, scripts_dir)
import bpy
import humanoid_control as hc

arm = hc.load_humanoid(blend_path)

CASES = [
    ("relax", lambda a, t: hc.pose_relax(a), 0.0),
    ("tpose", lambda a, t: hc.pose_tpose(a), 0.0),
    ("apose", lambda a, t: hc.pose_apose(a), 0.0),
    ("idle", hc.apply_idle, 1.3),
    ("wave", hc.apply_wave, 0.6),
    ("walk", hc.apply_walk, 0.9),
    ("nod", hc.apply_nod, 0.628),
    ("look", hc.apply_look, 1.047),
    ("run", hc.apply_run, 0.42),
]

BONES = ["root", "spine", "chest", "head", "upper_arm.L", "upper_arm.R",
         "forearm.R", "hand.L", "hand.R", "thigh.L", "thigh.R",
         "foot.L", "foot.R"]

out = {}
for name, fn, t in CASES:
    fn(arm, t)
    bpy.context.view_layer.update()
    f = {}
    for b in BONES:
        pb = arm.pose.bones[b]
        f[b] = {"rot": [round(v, 4) for v in pb.rotation_euler],
                "loc": [round(v, 4) for v in pb.location],
                "head": [round(v, 4) for v in (arm.matrix_world @ pb.head)],
                "tail": [round(v, 4) for v in (arm.matrix_world @ pb.tail)]}
    out[name] = f
print("ACTION_FACTS_JSON " + json.dumps(out))
"""


def find_blender():
    return shutil.which("blender") or r"D:\Programs\Blender\blender.exe"


def png_size(path):
    with open(path, "rb") as fh:
        data = fh.read(33)
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        return None
    return int.from_bytes(data[16:20], "big"), int.from_bytes(data[20:24], "big")


def main():
    failures = []
    blender = find_blender()
    if not os.path.exists(blender):
        print("FAIL: blender executable not found")
        return 1
    if not os.path.exists(BLEND_PATH):
        print("FAIL: %s not found" % BLEND_PATH)
        return 1

    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
        fh.write(CHECKER)
        checker_path = fh.name
    try:
        proc = subprocess.run(
            [blender, "-b", "-P", checker_path, "--", BLEND_PATH, SCRIPTS_DIR],
            capture_output=True, text=True, timeout=600)
    finally:
        os.unlink(checker_path)
    if proc.returncode != 0:
        print("FAIL: blender checker exit %d\n%s" % (proc.returncode, proc.stderr[-2000:]))
        return 1
    line = next((l for l in proc.stdout.splitlines()
                 if l.startswith("ACTION_FACTS_JSON ")), None)
    if line is None:
        print("FAIL: no ACTION_FACTS_JSON in blender output")
        return 1
    facts = json.loads(line[len("ACTION_FACTS_JSON "):])

    def rot(name, bone, i):
        return facts[name][bone]["rot"][i]

    def tail(name, bone, i):
        return facts[name][bone]["tail"][i]

    def head(name, bone, i):
        return facts[name][bone]["head"][i]

    # relax: hands hang low at the sides, elbows slightly bent
    if not (tail("relax", "hand.L", 2) < 0.35 and tail("relax", "hand.R", 2) < 0.35):
        failures.append("relax: hands not low (tail z %.3f/%.3f)"
                        % (tail("relax", "hand.L", 2), tail("relax", "hand.R", 2)))
    if abs(rot("relax", "forearm.R", 0)) < 0.1:
        failures.append("relax: forearm not slightly bent")

    # tpose: arms horizontal at shoulder height, stretched sideways
    for side in ("L", "R"):
        dz = abs(tail("tpose", "hand." + side, 2) - head("tpose", "upper_arm." + side, 2))
        if dz > 0.05:
            failures.append("tpose: %s arm not horizontal (dz=%.3f)" % (side, dz))
        if abs(tail("tpose", "hand." + side, 0)) < 0.45:
            failures.append("tpose: %s arm not stretched sideways" % side)

    # apose: arms between relax and tpose (down-out ~45 deg)
    for side in ("L", "R"):
        hz = tail("apose", "hand." + side, 2)
        if not (0.28 < hz < 0.45):
            failures.append("apose: %s hand z %.3f not in 45-deg band" % (side, hz))

    # idle: breathing chest oscillation with relaxed arms
    if not (0.01 < abs(rot("idle", "chest", 0)) <= 0.05):
        failures.append("idle: chest breathing out of range (%.4f)"
                        % rot("idle", "chest", 0))

    # wave: right arm abducted, forearm UP, left arm stays low
    if abs(rot("wave", "upper_arm.R", 2) + 1.5708) > 0.1:
        failures.append("wave: upper_arm.R not abducted ~90deg")
    if tail("wave", "hand.R", 2) < head("wave", "upper_arm.R", 2) + 0.15:
        failures.append("wave: right hand not raised above shoulder")
    if tail("wave", "forearm.R", 2) < head("wave", "forearm.R", 2) + 0.08:
        failures.append("wave: right forearm not folded upward")
    if tail("wave", "hand.L", 2) > 0.35:
        failures.append("wave: left arm should stay relaxed/low")

    # walk: opposite-phase thigh swing, one foot lifted, root bob
    tl, tr = rot("walk", "thigh.L", 0), rot("walk", "thigh.R", 0)
    if tl * tr >= 0 or min(abs(tl), abs(tr)) < 0.4:
        failures.append("walk: thighs not opposite-phase >=0.4 (%.3f/%.3f)" % (tl, tr))
    if abs(tail("walk", "foot.L", 2) - tail("walk", "foot.R", 2)) < 0.1:
        failures.append("walk: no foot lift")
    if facts["walk"]["root"]["loc"][2] <= 0.01:
        failures.append("walk: no vertical bob")

    # nod / look: head amplitude
    if abs(rot("nod", "head", 0)) < 0.2:
        failures.append("nod: head pitch %.3f < 0.2" % rot("nod", "head", 0))
    if abs(rot("look", "head", 2)) < 0.45:
        failures.append("look: head yaw %.3f < 0.45" % rot("look", "head", 2))

    # run: strictly stronger gait than walk + heel kick + stronger bob
    if max(abs(rot("run", "thigh.L", 0)), abs(rot("run", "thigh.R", 0))) < 0.6:
        failures.append("run: thigh swing < 0.6 (not stronger than walk)")
    if rot("run", "spine", 0) < 0.2:
        failures.append("run: forward lean %.3f < 0.2" % rot("run", "spine", 0))
    if max(tail("run", "foot.L", 2), tail("run", "foot.R", 2)) < 0.25:
        failures.append("run: no heel kick")
    if facts["run"]["root"]["loc"][2] <= facts["walk"]["root"]["loc"][2]:
        failures.append("run: bob not stronger than walk")

    # evidence stills
    for path in EVIDENCE:
        if not os.path.exists(path):
            failures.append("missing evidence still %s" % path)
            continue
        size = png_size(path)
        if size != (1280, 1280):
            failures.append("evidence %s size=%r, want (1280,1280)" % (path, size))

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: all 9 key actions drive the chibi twin with identifiable "
          "signatures; 9 evidence stills present at 1280x1280")
    return 0


if __name__ == "__main__":
    sys.exit(main())
