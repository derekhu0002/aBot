"""Render the twin in driven poses as visual proof of manipulation."""
import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import humanoid_control as hc

BLEND = r"D:\Projects\aBot\assets\humanoid\humanoid.blend"
OUT = r"D:\Projects\aBot\assets\humanoid"


def render(name, fn, t=0.0):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 900
    scene.render.resolution_y = 900
    scene.render.image_settings.file_format = "PNG"
    try:
        fn(bpy.data.objects["HumanoidRig"], t)
    except TypeError:
        fn(bpy.data.objects["HumanoidRig"])
    scene.frame_set(1)
    scene.render.filepath = os.path.join(OUT, name)
    bpy.ops.render.render(write_still=True)
    print("RENDERED", name)


hc.load_humanoid(BLEND)
render("driven_tpose.png", hc.pose_tpose)
render("driven_wave.png", lambda a, t: hc.apply_wave(a, t), t=0.6)
render("driven_relax.png", hc.pose_relax)
