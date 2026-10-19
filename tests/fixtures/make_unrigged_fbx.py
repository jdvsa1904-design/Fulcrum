"""Generate a disposable unrigged upright humanoid FBX for import smoke tests.

Run with Blender: blender -b --factory-startup -P this_file -- output.fbx
"""
import sys
import bpy

out = sys.argv[sys.argv.index("--") + 1]
bpy.ops.wm.read_factory_settings(use_empty=True)
parts = [
    ((0, 0, .98), (.30, .18, .42)),
    ((0, 0, 1.52), (.12, .12, .16)),
    ((-.21, 0, .99), (.13, .13, .15)),
    ((.21, 0, .99), (.13, .13, .15)),
    ((-.19, 0, .53), (.105, .11, .44)),
    ((.19, 0, .53), (.105, .11, .44)),
    ((-.19, .09, .09), (.13, .22, .09)),
    ((.19, .09, .09), (.13, .22, .09)),
    ((-.49, 0, 1.29), (.29, .09, .085)),
    ((.49, 0, 1.29), (.29, .09, .085)),
]
for center, half_size in parts:
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = bpy.context.object
    obj.scale = tuple(v * 2 for v in half_size)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
bpy.ops.object.select_all(action="SELECT")
bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
bpy.ops.object.join()
bpy.context.object.name = "UnriggedHumanoidSmoke"
bpy.ops.export_scene.fbx(filepath=out, use_selection=True, object_types={"MESH"}, axis_forward="-Z", axis_up="Y")
print("AUTORIG_FIXTURE", out, flush=True)
