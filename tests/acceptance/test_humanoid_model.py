#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: humanoid digital twin robot restyle + rig integrity.

GIVEN the repo contains the rebuilt humanoid artifacts produced by
scripts/blender_humanoid/build_humanoid.py (assets/humanoid/humanoid.blend,
preview_front.png, preview_3quarter.png)
WHEN the artifacts are inspected externally with headless Blender
THEN the model is the chibi robot restyled after assets/humanoid/reference_target.png:
  - armature 'HumanoidRig' exposes the full twin-control FK bone set;
  - mesh 'Humanoid_Body' is skinned to it (armature modifier + per-bone groups);
  - robot palette present: metallic red body, near-black glossy visor,
    emissive green eyes, warm cream/gold trim;
  - chibi proportions: vertices dominated by the 'head' bone span >= 30% of
    total body height (the old human-proportioned model scored ~13%);
  - both previews are valid 1280x1280 PNGs.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BLEND_PATH = os.path.join(REPO_ROOT, "assets", "humanoid", "humanoid.blend")
PREVIEWS = [
    os.path.join(REPO_ROOT, "assets", "humanoid", "preview_front.png"),
    os.path.join(REPO_ROOT, "assets", "humanoid", "preview_3quarter.png"),
]

EXPECTED_BONES = {"root", "spine", "chest", "neck", "head"}
for _b in ("shoulder", "upper_arm", "forearm", "hand", "thigh", "shin", "foot"):
    for _s in ("L", "R"):
        EXPECTED_BONES.add("%s.%s" % (_b, _s))

CHECKER = r"""
import bpy, json, sys

argv = sys.argv[sys.argv.index("--") + 1:]
blend_path = argv[0]
bpy.ops.wm.open_mainfile(filepath=blend_path)

arm = bpy.data.objects.get("HumanoidRig")
body = bpy.data.objects.get("Humanoid_Body")
facts = {"bones": [], "has_body": body is not None, "modifiers": [],
         "vgroups": [], "materials": [], "head_share": 0.0, "height": 0.0}
if arm is not None:
    facts["bones"] = sorted(b.name for b in arm.data.bones)
if body is not None:
    facts["modifiers"] = [m.type for m in body.modifiers]
    facts["vgroups"] = sorted(g.name for g in body.vertex_groups)
    for slot in body.data.materials:
        if slot is None or not slot.use_nodes:
            continue
        bsdf = slot.node_tree.nodes.get("Principled BSDF")
        if bsdf is None:
            continue
        base = tuple(bsdf.inputs["Base Color"].default_value)[:3]
        emis = tuple(bsdf.inputs["Emission Color"].default_value)[:3]
        facts["materials"].append({
            "name": slot.name,
            "base": [round(c, 3) for c in base],
            "metallic": round(bsdf.inputs["Metallic"].default_value, 3),
            "roughness": round(bsdf.inputs["Roughness"].default_value, 3),
            "emis_strength": round(bsdf.inputs["Emission Strength"].default_value, 3),
            "emis": [round(c, 3) for c in emis],
        })
    zs = [v.co.z for v in body.data.vertices]
    zmin, zmax = min(zs), max(zs)
    facts["height"] = round(zmax - zmin, 4)
    gi = {g.index: g.name for g in body.vertex_groups}
    hz = []
    for v in body.data.vertices:
        best_w, best_n = 0.0, None
        for g in v.groups:
            if g.weight > best_w:
                best_w, best_n = g.weight, gi.get(g.group)
        if best_n == "head":
            hz.append(v.co.z)
    if hz and zmax > zmin:
        facts["head_share"] = round((max(hz) - min(hz)) / (zmax - zmin), 4)
print("FACTS_JSON " + json.dumps(facts))
"""


def find_blender():
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
    failures = []
    blender = find_blender()
    if not os.path.exists(blender):
        print("FAIL: blender executable not found")
        return 1
    if not os.path.exists(BLEND_PATH):
        print("FAIL: %s not found" % BLEND_PATH)
        return 1

    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
        fh.write(CHECKER)
        checker_path = fh.name
    try:
        proc = subprocess.run(
            [blender, "-b", "-P", checker_path, "--", BLEND_PATH],
            capture_output=True, text=True, timeout=600,
        )
    finally:
        os.unlink(checker_path)
    if proc.returncode != 0:
        print("FAIL: blender checker exit %d\n%s" % (proc.returncode, proc.stderr[-2000:]))
        return 1
    line = next((l for l in proc.stdout.splitlines() if l.startswith("FACTS_JSON ")), None)
    if line is None:
        print("FAIL: no FACTS_JSON in blender output")
        return 1
    facts = json.loads(line[len("FACTS_JSON "):])

    # rig integrity (twin-control chain depends on these bone names)
    missing = EXPECTED_BONES - set(facts["bones"])
    if missing:
        failures.append("rig missing bones: %s" % sorted(missing))
    if not facts["has_body"]:
        failures.append("mesh 'Humanoid_Body' not found")
    else:
        if "ARMATURE" not in facts["modifiers"]:
            failures.append("Humanoid_Body has no ARMATURE modifier (not skinned)")
        vg_missing = EXPECTED_BONES - set(facts["vgroups"])
        if vg_missing:
            failures.append("missing vertex groups: %s" % sorted(vg_missing))

    # robot palette (external, material-level evidence of the restyle)
    mats = facts["materials"]

    def has(pred):
        return any(pred(m) for m in mats)

    if not has(lambda m: m["metallic"] >= 0.6 and m["base"][0] > 0.25
                 and m["base"][0] > 2 * m["base"][1] and m["base"][0] > 2 * m["base"][2]):
        failures.append("no metallic red body material")
    if not has(lambda m: max(m["base"]) <= 0.05 and m["roughness"] <= 0.3):
        failures.append("no near-black glossy visor material")
    if not has(lambda m: m["emis_strength"] >= 1.0 and m["emis"][1] > 2 * m["emis"][0]
                 and m["emis"][1] > 2 * m["emis"][2]):
        failures.append("no emissive green eye material")
    if not has(lambda m: m["base"][0] > 0.5 and m["base"][1] > 0.4 and m["base"][2] < 0.5):
        failures.append("no warm cream/gold trim material")

    # chibi proportions
    if facts["head_share"] < 0.30:
        failures.append("head_share=%.3f < 0.30 (not chibi)" % facts["head_share"])
    if not (0.9 <= facts["height"] <= 1.6):
        failures.append("total height %.2f m out of chibi range" % facts["height"])

    # previews
    for path in PREVIEWS:
        if not os.path.exists(path):
            failures.append("missing preview %s" % path)
            continue
        size = png_size(path)
        if size != (1280, 1280):
            failures.append("preview %s size=%r, want (1280, 1280)" % (path, size))

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: humanoid model is the chibi robot with intact twin-control rig "
          "(head_share=%.2f, height=%.2fm)" % (facts["head_share"], facts["height"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
