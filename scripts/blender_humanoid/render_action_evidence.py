"""Render per-action evidence stills of the chibi humanoid digital twin.

Run:  blender -b -P scripts/blender_humanoid/render_action_evidence.py

Drives every key action (static poses relax/tpose/apose and time motions
idle/wave/walk/nod/look/run) through humanoid_control and renders one still
per action to assets/humanoid/action_<name>.png using ONE fixed front camera
(the same view as preview_front.png) so the actions are directly comparable.

For external verification it also prints one line per action:
    ACTION_FACTS <name> <json>
with externally observable rig facts (pose-bone local Euler rotations plus
world-space head/tail of the key bones) so the driven deformation can be
checked without looking at the image.
"""

import json
import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import humanoid_control as hc

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
BLEND = os.path.join(REPO_ROOT, "assets", "humanoid", "humanoid.blend")
OUT = os.path.join(REPO_ROOT, "assets", "humanoid")

# Uniform front camera, identical to build_humanoid.py preview_front.
CAM_LOC = (0.0, -2.3, 0.62)
TARGET_LOC = (0.0, 0.0, 0.60)

FACT_BONES = ["root", "spine", "chest", "head",
              "upper_arm.L", "upper_arm.R", "forearm.L", "forearm.R",
              "hand.L", "hand.R", "thigh.L", "thigh.R",
              "shin.L", "shin.R", "foot.L", "foot.R"]

# (name, driver, representative time in seconds)
ACTIONS = [
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


def setup_camera():
    """Fix the one comparison camera (front view, same as preview_front)."""
    scene = bpy.context.scene
    cam = bpy.data.objects.get("Camera")
    if cam is None:
        cam = bpy.data.objects.new("Camera", bpy.data.cameras.new("Camera"))
        scene.collection.objects.link(cam)
    cam.location = CAM_LOC
    scene.camera = cam
    empty = bpy.data.objects.get("Target")
    if empty is None:
        empty = bpy.data.objects.new("Target", None)
        scene.collection.objects.link(empty)
        track = cam.constraints.new("TRACK_TO")
        track.track_axis = "TRACK_NEGATIVE_Z"
        track.up_axis = "UP_Y"
    empty.location = TARGET_LOC
    for c in cam.constraints:
        if c.type == "TRACK_TO":
            c.target = empty


def setup_render():
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 1280
    scene.render.image_settings.file_format = "PNG"
    scene.view_settings.view_transform = "Standard"


def collect_facts(arm):
    bpy.context.view_layer.update()
    facts = {}
    for name in FACT_BONES:
        pb = arm.pose.bones.get(name)
        if pb is None:
            continue
        head = arm.matrix_world @ pb.head
        tail = arm.matrix_world @ pb.tail
        facts[name] = {
            "rot": [round(v, 4) for v in pb.rotation_euler],
            "loc": [round(v, 4) for v in pb.location],
            "head": [round(v, 4) for v in head],
            "tail": [round(v, 4) for v in tail],
        }
    return facts


def main():
    arm = hc.load_humanoid(BLEND)
    setup_camera()
    setup_render()
    for name, fn, t in ACTIONS:
        fn(arm, t)
        bpy.context.view_layer.update()
        facts = collect_facts(arm)
        print("ACTION_FACTS %s %s" % (name, json.dumps(facts)), flush=True)
        path = os.path.join(OUT, "action_%s.png" % name)
        bpy.context.scene.render.filepath = path
        bpy.ops.render.render(write_still=True)
        print("RENDERED action_%s.png" % name, flush=True)


if __name__ == "__main__":
    main()
