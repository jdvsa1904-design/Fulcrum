import os
import runpy
import sys

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
skel_path, frames_path, out_path = argv[:3]
factor = float(argv[3]) if len(argv) > 3 else 1.0
base = os.path.splitext(out_path)[0]
full_path = base + ".full.fbx"

sys.argv = [sys.argv[0], "--", skel_path, frames_path, full_path]
runpy.run_path(os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames_to_fbx.py"), run_name="__main__")

act = bpy.data.actions.get("Fulcrum")
before = 0
if act is not None:
    try:
        curves = list(act.fcurves)
    except AttributeError:
        curves = [c for layer in act.layers for strip in layer.strips for bag in strip.channelbags for c in bag.fcurves]
    before = sum(len(c.keyframe_points) for c in curves)
print(f"FULCRUM reduce_keys factor={factor} source_keys={before}", flush=True)

bpy.ops.object.select_all(action="DESELECT")
for o in bpy.context.scene.objects:
    o.select_set(True)
bpy.context.view_layer.objects.active = next(o for o in bpy.context.scene.objects if o.type == "ARMATURE")
bpy.ops.export_scene.fbx(
    filepath=out_path,
    use_selection=True,
    object_types={"ARMATURE", "MESH"},
    add_leaf_bones=False,
    bake_anim=True,
    bake_anim_use_all_actions=False,
    bake_anim_use_nla_strips=False,
    bake_anim_force_startend_keying=True,
    bake_anim_simplify_factor=factor,
)
print(f"FULCRUM wrote reduced {out_path} (unreduced copy {full_path})", flush=True)
