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
  - visual-analyst gap fixes (2026-08-30 round 2), externally measured:
    * no dark gray sole plate under the feet (no DarkJoint verts at z<0.025);
    * hands are rounded with fingers (hand-region vert count well above the
      old polyhedral block);
    * visor is a curved shell hugging the helmet (every front-face Visor vert
      sits at a near-constant ellipsoid radius ratio, old flat box scored r
      up to 1.34) and carries a procedural scanline (wave) node;
    * emissive eyes hug the visor (eye ellipsoid radius ratio <= 1.10, old
      floating triangles scored ~1.15);
    * gold/cream rim present at the visor front sides (cream verts at the
      face border, absent in the old model);
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
         "vgroups": [], "materials": [], "head_share": 0.0, "height": 0.0,
         "visor_r": [0.0, 0.0], "eye_r_max": 0.0, "dark_low_count": 0,
         "hand_low_count": 0, "cream_front_count": 0, "visor_wave": False}
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
    # per-material vertex stats (external geometry evidence of gap fixes).
    # ellipsoid radius ratio vs the helmet ellipsoid (center 0,0,0.92;
    # radii 0.30/0.30/0.28): a shell hugging the head scores ~1.0-1.06.
    import math
    slot_of = {}
    for pi, poly in enumerate(body.data.polygons):
        sname = body.data.materials[poly.material_index].name \
            if body.data.materials[poly.material_index] else ""
        for li in poly.loop_indices:
            slot_of.setdefault(body.data.loops[li].vertex_index, sname)
    visor_rs, eye_r_max = [], 0.0
    dark_low = hand_low = cream_front = 0
    for v in body.data.vertices:
        x, y, z = v.co.x, v.co.y, v.co.z
        sname = slot_of.get(v.index, "")
        r = math.sqrt((x / 0.30) ** 2 + (y / 0.30) ** 2 + ((z - 0.92) / 0.28) ** 2)
        if sname == "Visor" and y < -0.24:  # front screen face only
            visor_rs.append(r)
        if sname == "EyeGlow":
            eye_r_max = max(eye_r_max, r)
        if sname == "DarkJoint" and z < 0.025:
            dark_low += 1
        if 0.17 <= z <= 0.235 and 0.13 <= abs(x) <= 0.25:
            hand_low += 1
        if (sname == "CreamTrim" and y < -0.25 and 0.15 <= abs(x) <= 0.25
                and 0.75 <= z <= 1.10):
            cream_front += 1
    if visor_rs:
        facts["visor_r"] = [round(min(visor_rs), 3), round(max(visor_rs), 3)]
    facts["eye_r_max"] = round(eye_r_max, 3)
    facts["dark_low_count"] = dark_low
    facts["hand_low_count"] = hand_low
    facts["cream_front_count"] = cream_front
    for slot in body.data.materials:
        if slot and slot.use_nodes and slot.name == "Visor":
            facts["visor_wave"] = any(n.type == "TEX_WAVE" for n in slot.node_tree.nodes)

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

    # visual-analyst gap fixes (round 2), external geometry evidence
    if facts["dark_low_count"] != 0:
        failures.append("gray sole plate under feet: %d dark verts at z<0.025"
                        % facts["dark_low_count"])
    if facts["hand_low_count"] < 300:
        failures.append("hands still polyhedral blocks: hand-region verts=%d < 300"
                        % facts["hand_low_count"])
    vr = facts["visor_r"]
    if not (0.99 <= vr[0] and vr[1] <= 1.10):
        failures.append("visor not a curved shell hugging the head: r-span=%s "
                        "(want within [0.99,1.10])" % (vr,))
    if facts["eye_r_max"] > 1.10:
        failures.append("eyes float off the visor: eye_r_max=%.3f > 1.10"
                        % facts["eye_r_max"])
    if facts["cream_front_count"] < 20:
        failures.append("no gold/cream rim at visor front: cream_front_count=%d"
                        % facts["cream_front_count"])
    if not facts["visor_wave"]:
        failures.append("visor material has no scanline (wave) node")

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
