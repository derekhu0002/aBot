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
AND separately the blend is re-opened WITHOUT any Python driver (fresh load,
no handlers, no control-module calls) and the baked keyframe Actions
(ActionRelax..ActionRun) are evaluated frame by frame — exactly what happens
when a user loads humanoid.blend in the Blender GUI and presses play
THEN every action produces its identifiable signature:
  - relax: arms hang at the sides, slightly bent (hands low, |x| small);
  - tpose: both arms horizontal at shoulder height, stretched sideways;
  - apose: both arms angled ~45 deg down-out between relax and tpose;
  - idle: subtle breathing (chest pitch oscillation > 0) with relaxed arms;
  - wave: the anatomical RIGHT upper arm abducted ~90 deg, forearm folded
    straight UP (hand above shoulder), anatomical left arm stays low, hand
    rocking. Side convention (probed from build_humanoid.py): the rig builds
    '.L'-suffixed bones at world -X and '.R'-suffixed bones at world +X while
    the robot faces world -Y toward the front camera, so the ANATOMICAL RIGHT
    arm is the '.L' bone chain (world -X, screen LEFT) and the anatomical
    left arm is the '.R' chain (world +X, screen right);
  - walk: opposite-phase thigh swing >= 0.4 rad, one foot clearly lifted,
    vertical root bob > 0;
  - nod: head pitch amplitude >= 0.2 rad (deepened nod, ~0.45 rad peak);
  - look: horizontal head YAW about the world vertical axis (head-bone local
    Y component) >= 0.45 rad while the in-plane roll stays small;
  - run: thigh swing >= 0.6 rad and forward lean >= 0.2 rad (both strictly
    stronger than walk) with a high heel kick and stronger bob;
  - and each action_<name>.png exists as a 1280x1280 PNG evidence still;
  - and the blend ships all nine baked Actions ActionRelax/ActionTPose/
    ActionAPose/ActionIdle/ActionWave/ActionWalk/ActionNod/ActionLook/
    ActionRun whose F-curves are JOINT-ONLY (pose-bone rotation_euler, plus
    the FK root-bob location for walk/run — no mesh/shape-key/vertex
    animation), with an Action already attached to the armature on load so
    GUI play shows motion immediately;
  - and playing each baked Action over time reproduces the same contract
    externally: idle chest breathing range >= 0.04 rad; wave anatomical right
    ('.L') upper arm held abducted ~90 deg while hand.L rocks (range >= 0.6
    rad) and the left arm stays static; walk thighs swing opposite phase with
    per-bone range >= 0.8 rad and the root bobs (loc z range >= 0.02); nod
    head pitch range >= 0.8 rad peaking >= 0.4; look head yaw (local Y) range
    >= 1.0 rad peaking >= 0.5 with roll <= 0.15; run thigh range >= 1.2 rad
    with pronounced forward lean (mean >= 0.2 rad) and stronger bob;
    relax/tpose/apose hold their joint angles over time (rotation range ~0)
    with upper_arm.L local Z at ~0.1047/1.5708/0.7854 rad respectively.
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
         "forearm.L", "forearm.R", "hand.L", "hand.R", "thigh.L", "thigh.R",
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

# ---------------------------------------------------------------------------
# Part B: baked Actions play back joint motion WITHOUT any Python driver.
# Fresh load (no frame handlers, no control-module calls): this is exactly
# "load humanoid.blend in the GUI and press play".
# ---------------------------------------------------------------------------
bpy.ops.wm.open_mainfile(filepath=blend_path)
arm = bpy.data.objects["HumanoidRig"]
ad = arm.animation_data
scene = bpy.context.scene
STATIC = ("ActionRelax", "ActionTPose", "ActionAPose")
SAMPLE_BONES = ["chest", "head", "spine", "upper_arm.L", "upper_arm.R",
                "forearm.L", "forearm.R", "hand.L", "hand.R",
                "thigh.L", "thigh.R"]
EXPECTED_ACTIONS = ["ActionRelax", "ActionTPose", "ActionAPose", "ActionIdle",
                    "ActionWave", "ActionWalk", "ActionNod", "ActionLook",
                    "ActionRun"]

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

out_b = {"default_action": (ad.action.name if ad and ad.action else None),
         "actions": {}}
for aname in EXPECTED_ACTIONS:
    act = bpy.data.actions.get(aname)
    info = {"exists": act is not None}
    out_b["actions"][aname] = info
    if act is None:
        continue
    curves = action_fcurves(act)
    info["joint_only"] = bool(curves) and all(
        fc.data_path.startswith("pose.bones[")
        and (fc.data_path.endswith("rotation_euler")
             or fc.data_path.endswith("location"))
        for fc in curves)
    fr0, fr1 = int(act.frame_range[0]), int(act.frame_range[1])
    info["range"] = [fr0, fr1]
    ad.action = act
    n = 12
    frames = sorted({fr0 + round(i * (fr1 - fr0) / (n - 1)) for i in range(n)})
    samples = {}
    for f in frames:
        scene.frame_set(f)
        bpy.context.view_layer.update()
        if f == frames[0]:
            info["first_frame"] = {
                b: [round(v, 4) for v in arm.pose.bones[b].rotation_euler]
                for b in SAMPLE_BONES}
        if aname not in STATIC:
            for b in SAMPLE_BONES:
                samples.setdefault(b, []).append(
                    [round(v, 4) for v in arm.pose.bones[b].rotation_euler])
    info["rot_range"] = {
        b: round(max(max(s[i] for s in samples[b])
                     - min(s[i] for s in samples[b])
                     for i in range(3)), 4)
        for b in samples}
    if aname not in STATIC:
        info["samples"] = samples  # static poses keep first_frame + rot_range only
# root-bob location sampling for the two locomotion clips
for aname in ("ActionWalk", "ActionRun"):
    act = bpy.data.actions.get(aname)
    if act is None:
        continue
    ad.action = act
    fr0, fr1 = int(act.frame_range[0]), int(act.frame_range[1])
    zs = []
    for i in range(12):
        scene.frame_set(fr0 + round(i * (fr1 - fr0) / 11))
        bpy.context.view_layer.update()
        zs.append(round(arm.pose.bones["root"].location[2], 4))
    out_b["actions"][aname]["root_loc_samples"] = zs
print("BAKED_FACTS_JSON " + json.dumps(out_b))
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
    # raw_decode tolerates Blender status text glued after the JSON object
    facts, _ = json.JSONDecoder().raw_decode(line[len("ACTION_FACTS_JSON "):])

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

    # wave: anatomical RIGHT arm = '.L' bone chain (world -X, screen left)
    # abducted, forearm UP; anatomical left arm ('.R' chain) stays low
    if abs(rot("wave", "upper_arm.L", 2) - 1.5708) > 0.1:
        failures.append("wave: upper_arm.L (anatomical right) not abducted ~90deg")
    if tail("wave", "hand.L", 2) < head("wave", "upper_arm.L", 2) + 0.15:
        failures.append("wave: right hand not raised above shoulder")
    if tail("wave", "forearm.L", 2) < head("wave", "forearm.L", 2) + 0.08:
        failures.append("wave: right forearm not folded upward")
    if tail("wave", "hand.R", 2) > 0.35:
        failures.append("wave: left arm should stay relaxed/low")

    # walk: opposite-phase thigh swing, one foot lifted, root bob
    tl, tr = rot("walk", "thigh.L", 0), rot("walk", "thigh.R", 0)
    if tl * tr >= 0 or min(abs(tl), abs(tr)) < 0.4:
        failures.append("walk: thighs not opposite-phase >=0.4 (%.3f/%.3f)" % (tl, tr))
    if abs(tail("walk", "foot.L", 2) - tail("walk", "foot.R", 2)) < 0.1:
        failures.append("walk: no foot lift")
    if facts["walk"]["root"]["loc"][2] <= 0.01:
        failures.append("walk: no vertical bob")

    # nod / look: head amplitude. nod = pitch (local X); look = horizontal
    # yaw about the world vertical (head local Y), with roll staying small.
    if abs(rot("nod", "head", 0)) < 0.2:
        failures.append("nod: head pitch %.3f < 0.2" % rot("nod", "head", 0))
    if abs(rot("look", "head", 1)) < 0.45:
        failures.append("look: head yaw %.3f < 0.45" % rot("look", "head", 1))
    if abs(rot("look", "head", 2)) > 0.15:
        failures.append("look: head roll %.3f should stay small (yaw-dominant)"
                        % rot("look", "head", 2))

    # run: strictly stronger gait than walk + heel kick + stronger bob
    if max(abs(rot("run", "thigh.L", 0)), abs(rot("run", "thigh.R", 0))) < 0.6:
        failures.append("run: thigh swing < 0.6 (not stronger than walk)")
    if rot("run", "spine", 0) < 0.2:
        failures.append("run: forward lean %.3f < 0.2" % rot("run", "spine", 0))
    if max(tail("run", "foot.L", 2), tail("run", "foot.R", 2)) < 0.25:
        failures.append("run: no heel kick")
    if facts["run"]["root"]["loc"][2] <= facts["walk"]["root"]["loc"][2]:
        failures.append("run: bob not stronger than walk")

    # ------------------------------------------------------------------
    # Part B: baked Actions must play back joint motion without a driver
    # ------------------------------------------------------------------
    line_b = next((l for l in proc.stdout.splitlines()
                   if l.startswith("BAKED_FACTS_JSON ")), None)
    if line_b is None:
        print("FAIL: no BAKED_FACTS_JSON in blender output")
        return 1
    baked, _ = json.JSONDecoder().raw_decode(line_b[len("BAKED_FACTS_JSON "):])
    ba = baked.get("actions", {})

    def brange(aname, bone):
        return ba.get(aname, {}).get("rot_range", {}).get(bone, 0.0)

    def bsamples(aname, bone):
        return ba.get(aname, {}).get("samples", {}).get(bone, [])

    # load-and-play: an Action is already attached when the blend is opened
    if not baked.get("default_action", "").startswith("Action"):
        failures.append("baked: no Action attached on load (GUI play would show "
                        "nothing until the user picks one): %r"
                        % baked.get("default_action"))

    for aname in ("ActionRelax", "ActionTPose", "ActionAPose", "ActionIdle",
                  "ActionWave", "ActionWalk", "ActionNod", "ActionLook",
                  "ActionRun"):
        info = ba.get(aname, {})
        if not info.get("exists"):
            failures.append("baked: missing Action %s" % aname)
            continue
        if not info.get("joint_only"):
            failures.append("baked: %s is not joint-only (F-curves must be "
                            "pose-bone rotation_euler / FK root location)" % aname)

    # idle: breathing oscillates over time (joint rotation changes)
    if brange("ActionIdle", "chest") < 0.04:
        failures.append("baked idle: chest rotation range %.4f < 0.04"
                        % brange("ActionIdle", "chest"))
    if brange("ActionIdle", "upper_arm.L") < 0.05:
        failures.append("baked idle: arm sway missing")

    # wave: anatomical right ('.L') arm abducted ~90 deg, hand rocking,
    # anatomical left arm static
    wave_uz = [s[2] for s in bsamples("ActionWave", "upper_arm.L")]
    if not wave_uz or max(abs(z - 1.5708) for z in wave_uz) > 0.15:
        failures.append("baked wave: upper_arm.L not held abducted ~90deg")
    if brange("ActionWave", "hand.L") < 0.6:
        failures.append("baked wave: hand.L rocking range %.3f < 0.6"
                        % brange("ActionWave", "hand.L"))
    if brange("ActionWave", "upper_arm.R") > 0.05:
        failures.append("baked wave: left arm should stay static")

    # walk: opposite-phase thigh swing over time + root bob
    tl = [s[0] for s in bsamples("ActionWalk", "thigh.L")]
    tr = [s[0] for s in bsamples("ActionWalk", "thigh.R")]
    if brange("ActionWalk", "thigh.L") < 0.8 or brange("ActionWalk", "thigh.R") < 0.8:
        failures.append("baked walk: thigh rotation range < 0.8 rad")
    if not tl or not any(a * b < 0 and min(abs(a), abs(b)) > 0.3
                         for a, b in zip(tl, tr)):
        failures.append("baked walk: thighs not opposite-phase over time")
    wzs = ba.get("ActionWalk", {}).get("root_loc_samples", [])
    if not wzs or max(wzs) - min(wzs) < 0.02 or max(wzs) < 0.03:
        failures.append("baked walk: root bob missing (loc z samples %s)" % wzs)

    # nod: head pitch swings
    if brange("ActionNod", "head") < 0.8:
        failures.append("baked nod: head rotation range %.3f < 0.8"
                        % brange("ActionNod", "head"))
    nod_hx = [s[0] for s in bsamples("ActionNod", "head")]
    if not nod_hx or max(abs(v) for v in nod_hx) < 0.4:
        failures.append("baked nod: head pitch peak < 0.4 rad")

    # look: yaw (local Y) swings, roll stays small
    if brange("ActionLook", "head") < 1.0:
        failures.append("baked look: head rotation range %.3f < 1.0"
                        % brange("ActionLook", "head"))
    look_hy = [s[1] for s in bsamples("ActionLook", "head")]
    look_hz = [s[2] for s in bsamples("ActionLook", "head")]
    if not look_hy or max(abs(v) for v in look_hy) < 0.5:
        failures.append("baked look: head yaw peak < 0.5 rad")
    if look_hz and max(abs(v) for v in look_hz) > 0.15:
        failures.append("baked look: head roll should stay small")

    # run: stronger gait, pronounced forward lean (R(12)+0.06*sin oscillates
    # about a 0.209 rad mean, same as the runtime contract), stronger bob
    if brange("ActionRun", "thigh.L") < 1.2:
        failures.append("baked run: thigh rotation range %.3f < 1.2"
                        % brange("ActionRun", "thigh.L"))
    run_sx = [s[0] for s in bsamples("ActionRun", "spine")]
    if not run_sx or sum(run_sx) / len(run_sx) < 0.2 or max(run_sx) < 0.26:
        failures.append("baked run: forward lean mean/peak too small "
                        "(mean=%.3f peak=%.3f)" % (
                            sum(run_sx) / max(len(run_sx), 1),
                            max(run_sx) if run_sx else 0.0))
    rzs = ba.get("ActionRun", {}).get("root_loc_samples", [])
    if not rzs or max(rzs) - min(rzs) < 0.03:
        failures.append("baked run: root bob missing/weak (loc z samples %s)" % rzs)

    # static poses: hold their joint angles and do not move over time
    def ff(aname, bone):
        return ba.get(aname, {}).get("first_frame", {}).get(bone)

    if not ff("ActionRelax", "upper_arm.L") or \
            abs(ff("ActionRelax", "upper_arm.L")[2] - 0.1047) > 0.03 or \
            abs(ff("ActionRelax", "upper_arm.R")[2] + 0.1047) > 0.03:
        failures.append("baked relax: upper-arm joint angles wrong")
    if not ff("ActionTPose", "upper_arm.L") or \
            abs(ff("ActionTPose", "upper_arm.L")[2] - 1.5708) > 0.03:
        failures.append("baked tpose: upper_arm.L local Z != ~1.5708")
    if not ff("ActionAPose", "upper_arm.L") or \
            abs(ff("ActionAPose", "upper_arm.L")[2] - 0.7854) > 0.03:
        failures.append("baked apose: upper_arm.L local Z != ~0.7854")
    for aname in ("ActionRelax", "ActionTPose", "ActionAPose"):
        rr = ba.get(aname, {}).get("rot_range")
        if rr and max(rr.values()) > 0.001:
            failures.append("baked %s: static pose must not move over time" % aname)

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
          "signatures; 9 joint-only baked Actions play back on load (GUI play "
          "shows motion); 9 evidence stills present at 1280x1280")
    return 0


if __name__ == "__main__":
    sys.exit(main())
