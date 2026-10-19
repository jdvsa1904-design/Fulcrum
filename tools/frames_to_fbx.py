import json
import re
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix

argv = sys.argv[sys.argv.index("--") + 1:]
skel_path, frames_path, out_path = argv
req = json.load(open(frames_path))

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=skel_path) if Path(skel_path).suffix.lower() == ".glb" else bpy.ops.import_scene.fbx(filepath=skel_path, use_anim=False)
arm = next(o for o in bpy.context.scene.objects if o.type == "ARMATURE")
for o in list(bpy.context.scene.objects):
    if o.type not in {"ARMATURE", "MESH"}:
        bpy.data.objects.remove(o, do_unlink=True)


def key(n):
    return re.sub(r"[\[\]\.:/]", "", re.sub(r"\s", "_", n)).lower()


bones = {key(b.name): b for b in arm.pose.bones}
names = [n for n in req["bones"] if key(n) in bones]
idx = {n: i for i, n in enumerate(req["bones"])}
print(f"FULCRUM matched {len(names)}/{len(req['bones'])} bones", flush=True)

P = np.array([req["rest"][idx[n]][12:15] for n in names])
Q = np.array([list(arm.matrix_world @ bones[key(n)].bone.head_local) for n in names])
pc, qc = P.mean(0), Q.mean(0)
H = (P - pc).T @ (Q - qc)
U, S, Vt = np.linalg.svd(H)
d = np.sign(np.linalg.det(Vt.T @ U.T))
D = np.diag([1, 1, d])
R = Vt.T @ D @ U.T
s = (S * np.diag(D)).sum() / ((P - pc) ** 2).sum()
t = qc - s * R @ pc
resid = np.linalg.norm((s * (R @ P.T)).T + t - Q, axis=1)
print(f"FULCRUM axis fit scale={s:.5f} max_resid={resid.max():.5f} mean_resid={resid.mean():.5f}", flush=True)

C = Matrix.Identity(4)
for i in range(3):
    for j in range(3):
        C[i][j] = s * R[i, j]
    C[i][3] = t[i]
Ci = C.inverted()
AWi = arm.matrix_world.inverted()


def mat(a):
    return Matrix([a[0:4], a[4:8], a[8:12], a[12:16]]).transposed()


corr = {}
for n in names:
    pb = bones[key(n)]
    Bref = arm.matrix_world @ pb.bone.matrix_local
    corr[n] = mat(req["rest"][idx[n]]).inverted() @ Ci @ Bref

order = []
seen = set()


def visit(pb):
    order.append(pb)
    for c in pb.children:
        visit(c)


for pb in arm.pose.bones:
    if pb.parent is None:
        visit(pb)
name_of = {key(n): n for n in names}

scene = bpy.context.scene
fps = req["fps"]
scene.render.fps = int(round(fps))
scene.render.fps_base = scene.render.fps / fps
n_frames = len(req["frames"])
scene.frame_start = 0
scene.frame_end = n_frames - 1
props = []
for prop in req.get("props", []):
    if prop.get("driverIndex") is None:
        raise ValueError("Prop driver is unmapped: " + prop["name"])
    anchor = bpy.data.objects.new("FulcrumProp_" + prop["name"], None)
    scene.collection.objects.link(anchor)
    anchor.rotation_mode = "QUATERNION"
    source = Path(frames_path).parent / Path(prop.get("source", "missing.fbx")).name
    if prop.get("source") and source.is_file():
        before = set(scene.objects)
        bpy.ops.import_scene.fbx(filepath=str(source), use_anim=False)
        loaded = [o for o in scene.objects if o not in before]
        points = [o.matrix_world @ __import__('mathutils').Vector(c) for o in loaded if o.type == "MESH" for c in o.bound_box]
        if not points:
            raise ValueError("Weapon asset has no mesh: " + prop["name"])
        lo = np.min(np.array([list(p) for p in points]), axis=0)
        hi = np.max(np.array([list(p) for p in points]), axis=0)
        size = max(hi - lo)
        factor = prop.get("length", .8) * req.get("upm", 100) / max(size, 1e-8)
        for o in loaded:
            if o.parent not in loaded:
                mw = o.matrix_world.copy()
                o.parent = anchor
                o.matrix_parent_inverse = Matrix.Identity(4)
                o.matrix_basis = Matrix.Scale(factor, 4) @ mw
    else:
        bpy.ops.mesh.primitive_cube_add(size=1)
        mesh = bpy.context.object
        mesh.name = prop["name"] + "_grip_proxy"
        mesh.parent = anchor
        mesh.scale = (prop.get("length", .8) * req.get("upm", 100), .06 * req.get("upm", 100), .10 * req.get("upm", 100))
    props.append((anchor, prop))
arm.animation_data_clear()
arm.animation_data_create()
act = bpy.data.actions.new("Fulcrum")
arm.animation_data.action = act
for pb in arm.pose.bones:
    pb.rotation_mode = "QUATERNION"

nb = len(req["bones"])
prev_q = {}
for f, flat in enumerate(req["frames"]):
    for anchor, prop in props:
        i = prop["driverIndex"]
        anchor.matrix_world = C @ mat(flat[i * 16:(i + 1) * 16]) @ mat(prop["offset"])
        anchor.keyframe_insert("location", frame=f)
        anchor.keyframe_insert("rotation_quaternion", frame=f)
        anchor.keyframe_insert("scale", frame=f)
    target = {}
    for pb in order:
        k = key(pb.name)
        if k not in name_of:
            continue
        n = name_of[k]
        i = idx[n]
        Mw = mat(flat[i * 16:(i + 1) * 16])
        target[pb.name] = AWi @ C @ Mw @ corr[n]
    for pb in order:
        if pb.name not in target:
            continue
        B = target[pb.name]
        if pb.parent and pb.parent.name in target:
            rest_rel = pb.parent.bone.matrix_local.inverted() @ pb.bone.matrix_local
            basis = rest_rel.inverted() @ target[pb.parent.name].inverted() @ B
        elif pb.parent:
            continue
        else:
            basis = pb.bone.matrix_local.inverted() @ B
        loc, rot, sca = basis.decompose()
        pb.location = loc
        pq = prev_q.get(pb.name)
        if pq is not None and pq.dot(rot) < 0:
            rot = -rot
        prev_q[pb.name] = rot.copy()
        pb.rotation_quaternion = rot
        pb.keyframe_insert("location", frame=f)
        pb.keyframe_insert("rotation_quaternion", frame=f)
for pb in arm.pose.bones:
    pb.location = (0, 0, 0)
    pb.rotation_quaternion = (1, 0, 0, 0)
    pb.scale = (1, 1, 1)
scene.frame_set(0)
print(f"FULCRUM keyed {n_frames} frames at {fps} fps", flush=True)

arm.name = "Armature"
bpy.ops.object.select_all(action="DESELECT")
for o in scene.objects:
    o.select_set(True)
bpy.context.view_layer.objects.active = arm
bpy.ops.export_scene.fbx(
    filepath=out_path,
    use_selection=True,
    object_types={"ARMATURE", "MESH", "EMPTY"},
    add_leaf_bones=False,
    bake_anim=True,
    bake_anim_use_all_actions=False,
    bake_anim_use_nla_strips=False,
    bake_anim_force_startend_keying=True,
    bake_anim_simplify_factor=0.0,
)
print(f"FULCRUM wrote {out_path}", flush=True)
