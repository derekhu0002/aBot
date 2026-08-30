"""Procedurally build a stylized 3D humanoid character in Blender (headless).

Design decisions
----------------
- Blender 5.1 has no bundled human generator addon, so the character is built
  from primitives via the bpy Python API.
- Human-like proportions (approx. 1.75 m tall, feet at z=0, head top ~1.80).
- Subdivision surface + smooth shading for clean, low-poly-count visuals.
- Simple PBR-ish materials: skin, hair, eyes (white + iris + pupil), shirt,
  pants, shoes.
- A basic armature (root -> spine -> chest -> neck -> head, plus L/R limbs)
  is added and the joined mesh is parented with automatic weights so the
  character can be posed.

Run headless:
    blender -b -P scripts/blender_humanoid/build_humanoid.py
"""

import math

import bpy
from mathutils import Vector

# ---------------------------------------------------------------------------
# Output paths (edit as needed)
# ---------------------------------------------------------------------------
BLEND_OUT = r"D:\Projects\aBot\assets\humanoid\humanoid.blend"
RENDER_FRONT = r"D:\Projects\aBot\assets\humanoid\preview_front.png"
RENDER_THREEQUARTER = r"D:\Projects\aBot\assets\humanoid\preview_3quarter.png"

# ---------------------------------------------------------------------------
# Scene helpers
# ---------------------------------------------------------------------------
def clear_scene():
    """Remove all objects, meshes and materials, keep only default collection."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    for mat in list(bpy.data.materials):
        bpy.data.materials.remove(mat)
    # remove leftover empty collections
    for col in list(bpy.data.collections):
        if col.users == 0:
            bpy.data.collections.remove(col)


def make_material(name, color, roughness=0.45, metallic=0.0, emission=None):
    """Create a Principled BSDF material."""
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    if emission is not None:
        bsdf.inputs["Emission Color"].default_value = (*emission, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 1.0
    return mat


def make_primitive(kind, name, location, scale, rotation=(0.0, 0.0, 0.0),
                   segments=32, material=None, subdiv=2):
    """Create a primitive mesh object, position it, shade smooth, subdivide."""
    ops = {
        "sphere": lambda: bpy.ops.mesh.primitive_uv_sphere_add(
            radius=0.5, segments=segments, ring_count=max(segments // 2, 8),
            location=location, rotation=rotation),
        "cylinder": lambda: bpy.ops.mesh.primitive_cylinder_add(
            radius=0.5, depth=1.0, vertices=segments,
            location=location, rotation=rotation),
        "cube": lambda: bpy.ops.mesh.primitive_cube_add(
            size=1.0, location=location, rotation=rotation),
        "cone": lambda: bpy.ops.mesh.primitive_cone_add(
            radius1=0.5, radius2=0.35, depth=1.0, vertices=segments,
            location=location, rotation=rotation),
    }
    ops[kind]()
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = scale
    if material is not None:
        obj.data.materials.append(material)
    bpy.ops.object.shade_smooth()
    if subdiv and kind in ("sphere", "cylinder", "cone"):
        mod = obj.modifiers.new("Subdivision", "SUBSURF")
        mod.levels = subdiv
        mod.render_levels = subdiv
    return obj


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
SKIN = (0.96, 0.76, 0.64)
HAIR = (0.13, 0.09, 0.07)
EYE_WHITE = (0.95, 0.94, 0.92)
IRIS = (0.24, 0.42, 0.62)
PUPIL = (0.02, 0.02, 0.02)
SHIRT = (0.10, 0.40, 0.44)
PANTS = (0.13, 0.17, 0.30)
SHOES = (0.16, 0.16, 0.18)
MOUTH = (0.55, 0.35, 0.30)

# ---------------------------------------------------------------------------
# Build the humanoid
# ---------------------------------------------------------------------------
def build_humanoid():
    clear_scene()

    mat_skin = make_material("Skin", SKIN, roughness=0.55)
    mat_hair = make_material("Hair", HAIR, roughness=0.75)
    mat_eye_white = make_material("EyeWhite", EYE_WHITE, roughness=0.25)
    mat_iris = make_material("Iris", IRIS, roughness=0.20)
    mat_pupil = make_material("Pupil", PUPIL, roughness=0.10)
    mat_shirt = make_material("Shirt", SHIRT, roughness=0.60)
    mat_pants = make_material("Pants", PANTS, roughness=0.65)
    mat_shoes = make_material("Shoes", SHOES, roughness=0.35)
    mat_mouth = make_material("Mouth", MOUTH, roughness=0.50)

    parts = []

    def add(kind, name, loc, scale, material, **kw):
        obj = make_primitive(kind, name, Vector(loc), Vector(scale),
                             material=material, **kw)
        parts.append(obj)
        return obj

    # ---- Torso / hips / chest --------------------------------------------
    add("sphere", "Hips", (0.0, 0.0, 0.88), (0.115, 0.105, 0.12), mat_skin, subdiv=3)
    add("sphere", "Torso", (0.0, 0.0, 1.22), (0.135, 0.105, 0.17), mat_skin, subdiv=3)
    add("sphere", "Chest", (0.0, 0.0, 1.44), (0.125, 0.095, 0.10), mat_skin, subdiv=3)
    add("sphere", "Neck", (0.0, 0.0, 1.56), (0.045, 0.045, 0.05), mat_skin, subdiv=2)

    # ---- Head and face ----------------------------------------------------
    add("sphere", "Head", (0.0, 0.0, 1.70), (0.095, 0.105, 0.115), mat_skin, subdiv=3)
    # nose
    add("cone", "Nose", (0.0, 0.088, 1.70), (0.018, 0.025, 0.045), mat_skin,
        rotation=(math.radians(90), 0, 0), subdiv=0)
    # ears
    add("sphere", "Ear_L", (-0.098, 0.0, 1.70), (0.018, 0.028, 0.040), mat_skin, subdiv=0)
    add("sphere", "Ear_R", (0.098, 0.0, 1.70), (0.018, 0.028, 0.040), mat_skin, subdiv=0)

    # eyes (white sphere + iris + pupil)
    for side in (-1, 1):
        ex = side * 0.042
        ey = 0.093
        ez = 1.71
        add("sphere", f"EyeWhite{side:+d}", (ex, ey, ez), (0.020, 0.022, 0.020),
            mat_eye_white, subdiv=0)
        add("sphere", f"Iris{side:+d}", (ex, ey + 0.016, ez), (0.011, 0.004, 0.011),
            mat_iris, subdiv=0)
        add("sphere", f"Pupil{side:+d}", (ex, ey + 0.022, ez), (0.005, 0.003, 0.005),
            mat_pupil, subdiv=0)
    # brows
    for side in (-1, 1):
        add("cube", f"Brow{side:+d}", (side * 0.042, 0.100, 1.735),
            (0.040, 0.010, 0.010), mat_hair, subdiv=0)
    # mouth
    add("cube", "Mouth", (0.0, 0.097, 1.655), (0.050, 0.012, 0.008), mat_mouth, subdiv=0)

    # ---- Hair (half-sphere cap on top of head) ----------------------------
    hair = make_primitive("sphere", "HairCap", Vector((0.0, -0.01, 1.745)),
                          Vector((0.100, 0.110, 0.085)), material=mat_hair, subdiv=3)
    # keep only the top half: use a boolean-free approach — scale/flatten + clip
    bpy.context.view_layer.objects.active = hair
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="DESELECT")
    bpy.ops.object.mode_set(mode="OBJECT")
    # Remove bottom half vertices (z < hair center) via bmesh
    import bmesh
    me = hair.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.co.z < 0.0],
                     context="VERTS")
    bm.to_mesh(me)
    bm.free()
    bpy.ops.object.mode_set(mode="OBJECT")
    parts.append(hair)

    # ---- Arms (shoulder -> upper arm -> forearm -> hand) ------------------
    for side, sx in (("L", -1), ("R", 1)):
        add("sphere", f"Shoulder{side}", (sx * 0.16, 0.0, 1.44),
            (0.065, 0.065, 0.065), mat_skin, subdiv=2)
        add("cylinder", f"UpperArm{side}", (sx * 0.185, 0.0, 1.20),
            (0.045, 0.045, 0.20), mat_skin, subdiv=2)
        add("sphere", f"Elbow{side}", (sx * 0.185, 0.0, 0.99),
            (0.045, 0.045, 0.045), mat_skin, subdiv=2)
        add("cylinder", f"Forearm{side}", (sx * 0.185, 0.0, 0.82),
            (0.038, 0.038, 0.17), mat_skin, subdiv=2)
        add("sphere", f"Hand{side}", (sx * 0.185, 0.0, 0.68),
            (0.032, 0.028, 0.040), mat_skin, subdiv=0)

    # ---- Legs (thigh -> shin -> foot) -------------------------------------
    for side, sx in (("L", -1), ("R", 1)):
        add("cylinder", f"Thigh{side}", (sx * 0.095, 0.0, 0.64),
            (0.070, 0.070, 0.22), mat_skin, subdiv=2)
        add("sphere", f"Knee{side}", (sx * 0.095, 0.0, 0.43),
            (0.055, 0.055, 0.055), mat_skin, subdiv=2)
        add("cylinder", f"Shin{side}", (sx * 0.095, 0.0, 0.22),
            (0.048, 0.048, 0.22), mat_skin, subdiv=2)
        add("cube", f"Foot{side}", (sx * 0.095, 0.055, 0.035),
            (0.070, 0.16, 0.055), mat_shoes, subdiv=0)

    # ---- Clothes ----------------------------------------------------------
    # shirt (torso layer, slightly larger)
    add("sphere", "ShirtTop", (0.0, 0.0, 1.22), (0.145, 0.115, 0.19), mat_shirt, subdiv=3)
    add("sphere", "ShirtHip", (0.0, 0.0, 0.96), (0.125, 0.115, 0.10), mat_shirt, subdiv=3)
    # sleeves
    for side, sx in (("L", -1), ("R", 1)):
        add("cylinder", f"Sleeve{side}", (sx * 0.185, 0.0, 1.20),
            (0.050, 0.050, 0.14), mat_shirt, subdiv=2)
    # pants (over thighs + shins, slightly larger)
    for side, sx in (("L", -1), ("R", 1)):
        add("cylinder", f"Pant{side}", (sx * 0.098, 0.0, 0.60),
            (0.078, 0.078, 0.26), mat_pants, subdiv=2)
        add("cylinder", f"PantShin{side}", (sx * 0.098, 0.0, 0.22),
            (0.056, 0.056, 0.18), mat_pants, subdiv=2)

    # ---- Join all parts into a single mesh ---------------------------------
    for obj in parts:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = parts[0]
    bpy.ops.object.join()
    body = bpy.context.active_object
    body.name = "Humanoid_Body"
    return body


# ---------------------------------------------------------------------------
# Armature + auto-weight parenting
# ---------------------------------------------------------------------------
def add_armature(body):
    arm_data = bpy.data.armatures.new("HumanoidRig")
    arm = bpy.data.objects.new("HumanoidRig", arm_data)
    bpy.context.collection.objects.link(arm)

    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    eb = arm_data.edit_bones

    root = eb.new("root")
    root.head = (0.0, 0.0, 0.05)
    root.tail = (0.0, 0.0, 0.10)

    spine = eb.new("spine")
    spine.head = (0.0, 0.0, 0.85)
    spine.tail = (0.0, 0.0, 1.20)
    spine.parent = root

    chest = eb.new("chest")
    chest.head = (0.0, 0.0, 1.20)
    chest.tail = (0.0, 0.0, 1.45)
    chest.parent = spine

    neck = eb.new("neck")
    neck.head = (0.0, 0.0, 1.45)
    neck.tail = (0.0, 0.0, 1.56)
    neck.parent = chest

    head = eb.new("head")
    head.head = (0.0, 0.0, 1.56)
    head.tail = (0.0, 0.0, 1.72)
    head.parent = neck

    for side, sx in (("L", -1), ("R", 1)):
        shoulder = eb.new(f"shoulder.{side}")
        shoulder.head = (sx * 0.05, 0.0, 1.42)
        shoulder.tail = (sx * 0.20, 0.0, 1.42)
        shoulder.parent = chest

        uarm = eb.new(f"upper_arm.{side}")
        uarm.head = (sx * 0.20, 0.0, 1.42)
        uarm.tail = (sx * 0.20, 0.0, 1.00)
        uarm.parent = shoulder

        farm = eb.new(f"forearm.{side}")
        farm.head = (sx * 0.20, 0.0, 1.00)
        farm.tail = (sx * 0.20, 0.0, 0.75)
        farm.parent = uarm

        hand = eb.new(f"hand.{side}")
        hand.head = (sx * 0.20, 0.0, 0.75)
        hand.tail = (sx * 0.20, 0.0, 0.66)
        hand.parent = farm

        thigh = eb.new(f"thigh.{side}")
        thigh.head = (sx * 0.10, 0.0, 0.85)
        thigh.tail = (sx * 0.10, 0.0, 0.45)
        thigh.parent = root

        shin = eb.new(f"shin.{side}")
        shin.head = (sx * 0.10, 0.0, 0.45)
        shin.tail = (sx * 0.10, 0.0, 0.08)
        shin.parent = thigh

        foot = eb.new(f"foot.{side}")
        foot.head = (sx * 0.10, 0.0, 0.08)
        foot.tail = (sx * 0.10, 0.12, 0.03)
        foot.parent = shin

    bpy.ops.object.mode_set(mode="OBJECT")

    # Parent mesh to armature with automatic weights
    body.select_set(True)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.parent_set(type="ARMATURE_AUTO")
    # Fallback: ensure every bone has a vertex group with non-zero weights
    fix_weights(arm, body)
    # Fallback: ensure every bone has a vertex group with non-zero weights
    fix_weights(arm, body)
    return arm


def fix_weights(arm_obj, body):
    """Distance-falloff weight fallback for bones without any weights.

    Auto-weights (bone heat) can fail to find a solution for some bones when
    the mesh is built from many overlapping primitives. For any bone whose
    vertex group is missing or has zero weight, build a group using a Gaussian
    falloff from the bone segment.
    """
    import math
    from mathutils import Vector

    mesh = body.data
    vgs = body.vertex_groups
    sigma = 0.055

    for bone in arm_obj.data.bones:
        name = bone.name
        g = vgs.get(name)
        existing_ok = False
        if g is not None:
            for v in mesh.vertices:
                try:
                    if g.weight(v.index) > 0.001:
                        existing_ok = True
                        break
                except RuntimeError:
                    break
        if existing_ok:
            continue

        if g is not None:
            vgs.remove(g)
        g = vgs.new(name=name)

        p0 = Vector(bone.head_local)
        p1 = Vector(bone.tail_local)
        seg = p1 - p0
        seg_len2 = seg.length_squared

        for v in mesh.vertices:
            d = v.co - p0
            t = 0.0
            if seg_len2 > 1e-9:
                t = max(0.0, min(1.0, d.dot(seg) / seg_len2))
            closest = p0 + seg * t
            dist = (v.co - closest).length
            w = math.exp(-(dist * dist) / (2.0 * sigma * sigma))
            if w > 0.001:
                g.add([v.index], w, "REPLACE")


# ---------------------------------------------------------------------------
# Camera / lights / render
# ---------------------------------------------------------------------------
def setup_scene_and_render():
    scene = bpy.context.scene
    # Engine: EEVEE (Blender 5.x)
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 1280

    # Ground / subtle backdrop
    ground = make_primitive("cube", "Ground", (0.0, 0.0, -0.01),
                            (3.0, 3.0, 0.02), material=None, subdiv=0)
    gmat = make_material("Ground", (0.22, 0.24, 0.27), roughness=0.9)
    ground.data.materials.append(gmat)

    # Lights
    key = bpy.data.objects.new("KeyLight", bpy.data.lights.new("KeyLight", "AREA"))
    key.location = (2.4, -2.2, 3.2)
    key.rotation_euler = (math.radians(45), 0, math.radians(40))
    key.data.energy = 300
    scene.collection.objects.link(key)

    fill = bpy.data.objects.new("FillLight", bpy.data.lights.new("FillLight", "AREA"))
    fill.location = (-2.2, -1.4, 1.6)
    fill.rotation_euler = (math.radians(20), 0, math.radians(-60))
    fill.data.energy = 120
    scene.collection.objects.link(fill)

    rim = bpy.data.objects.new("RimLight", bpy.data.lights.new("RimLight", "AREA"))
    rim.location = (0.0, 3.0, 2.0)
    rim.rotation_euler = (math.radians(30), 0, math.radians(180))
    rim.data.energy = 80
    scene.collection.objects.link(rim)

    cam = bpy.data.objects.new("Camera", bpy.data.cameras.new("Camera"))
    scene.collection.objects.link(cam)
    scene.camera = cam

    # Track-to target
    empty = bpy.data.objects.new("Target", None)
    empty.location = (0.0, 0.0, 0.95)
    scene.collection.objects.link(empty)
    track = cam.constraints.new("TRACK_TO")
    track.target = empty
    track.track_axis = "TRACK_NEGATIVE_Z"
    track.up_axis = "UP_Y"

    def render_angle(name, cam_loc, out_path):
        cam.location = cam_loc
        scene.render.filepath = out_path
        bpy.ops.render.render(write_still=True)
        print(f"RENDERED {name}: {out_path}")

    render_angle("front", (0.0, -3.6, 1.05), RENDER_FRONT)
    render_angle("3quarter", (2.4, -2.7, 1.35), RENDER_THREEQUARTER)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("== building humanoid ==")
    body = build_humanoid()
    print("== adding armature ==")
    add_armature(body)
    print("== scene + render ==")
    setup_scene_and_render()
    bpy.ops.wm.save_as_mainfile(filepath=BLEND_OUT)
    print(f"SAVED: {BLEND_OUT}")


if __name__ == "__main__":
    main()
