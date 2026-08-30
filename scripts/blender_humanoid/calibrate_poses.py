"""Calibration: render one still per pose so the pose axes can be verified.

Run:  blender -b -P scripts/blender_humanoid/calibrate_poses.py
Outputs stills to assets/humanoid/calib_*.png
"""

import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import humanoid_control as hc

BLEND = r"D:\Projects\aBot\assets\humanoid\humanoid.blend"
OUT_DIR = r"D:\Projects\aBot\assets\humanoid"


def setup_render():
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 640
    scene.render.resolution_y = 640
    scene.render.image_settings.file_format = "PNG"


def render_pose(name, fn, t=0.0):
    try:
        fn(bpy.data.objects["HumanoidRig"], t)
    except TypeError:
        fn(bpy.data.objects["HumanoidRig"])
    bpy.context.scene.frame_set(1)
    path = os.path.join(OUT_DIR, f"calib_{name}.png")
    bpy.context.scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    print(f"CALIB {name} -> {path}")


def main():
    hc.load_humanoid(BLEND)
    setup_render()
    render_pose("relax", hc.pose_relax)
    render_pose("tpose", hc.pose_tpose)
    render_pose("apose", hc.pose_apose)
    render_pose("wave", lambda a, t: hc.apply_wave(a, t), t=0.6)
    render_pose("walk", lambda a, t: hc.apply_walk(a, t), t=0.9)
    render_pose("nod", lambda a, t: hc.apply_nod(a, t), t=0.3)
    render_pose("look", lambda a, t: hc.apply_look(a, t), t=0.7)
    render_pose("run", lambda a, t: hc.apply_run(a, t), t=0.42)


if __name__ == "__main__":
    main()
