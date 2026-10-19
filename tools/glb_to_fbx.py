import sys

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
src, dst = argv[0], argv[1]
fps = float(argv[2]) if len(argv) > 2 else 30.0

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = int(round(fps))
scene.render.fps_base = scene.render.fps / fps
bpy.ops.import_scene.gltf(filepath=src)
arm = next(o for o in scene.objects if o.type == "ARMATURE")
arm.name = "Armature"
for o in list(scene.objects):
    if o.type not in {"ARMATURE", "MESH"}:
        bpy.data.objects.remove(o, do_unlink=True)
actions = list(bpy.data.actions)
for a in actions:
    a.use_fake_user = True
    print(f"FULCRUM action {a.name} frames {a.frame_range[0]:.0f}-{a.frame_range[1]:.0f}", flush=True)
ad = arm.animation_data or arm.animation_data_create()
for t in list(ad.nla_tracks):
    ad.nla_tracks.remove(t)
ad.action = actions[0] if actions else None
for o in scene.objects:
    if o.type == "MESH" and o.animation_data:
        o.animation_data_clear()
end = max((a.frame_range[1] for a in actions), default=1)
scene.frame_start, scene.frame_end = 0, int(end)
bpy.ops.object.select_all(action="DESELECT")
for o in scene.objects:
    o.select_set(True)
bpy.context.view_layer.objects.active = arm
bpy.ops.export_scene.fbx(
    filepath=dst,
    use_selection=True,
    object_types={"ARMATURE", "MESH"},
    add_leaf_bones=False,
    bake_anim=True,
    bake_anim_use_all_actions=True,
    bake_anim_use_nla_strips=False,
    bake_anim_force_startend_keying=True,
    bake_anim_simplify_factor=0.0,
)
print(f"FULCRUM wrote {dst} with {len(actions)} animations", flush=True)
