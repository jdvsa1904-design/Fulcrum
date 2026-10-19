import json
import os
import sys
import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
JOBS = [("v6", os.path.join(HERE, "..", "exports", "FRT_fulcrum_v6.fbx"))]
MAJOR = ["root", "pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05",
         "thigh_l", "thigh_r", "calf_l", "calf_r", "foot_l", "foot_r", "ball_l", "ball_r",
         "clavicle_l", "clavicle_r", "upperarm_l", "upperarm_r", "lowerarm_l", "lowerarm_r",
         "hand_l", "hand_r", "neck_01", "neck_02", "head"]


def log(m):
    unreal.log_warning("FULCHK " + str(m))


def do_import(tag, path):
    dest = "/Game/Check_" + tag
    ui = unreal.FbxImportUI()
    ui.import_mesh = True
    ui.import_as_skeletal = True
    ui.import_animations = True
    ui.import_materials = False
    ui.import_textures = False
    ui.mesh_type_to_import = unreal.FBXImportType.FBXIT_SKELETAL_MESH
    ad = ui.anim_sequence_import_data
    for k, v in (("animation_length", "FBXALIT_EXPORTED_TIME"), ("import_bone_tracks", True), ("remove_redundant_keys", False), ("do_not_import_curve_with_zero", False), ("snap_to_closest_frame_boundary", False)):
        try:
            if isinstance(v, str):
                ad.set_editor_property(k, getattr(unreal.FBXAnimationLengthImportType, v))
            else:
                ad.set_editor_property(k, v)
        except Exception as e:
            log(f"opt {k} skipped {e!r}")
    try:
        log("anim props " + str(sorted(p for p in dir(ad) if not p.startswith("_"))[:80]))
    except Exception:
        pass
    task = unreal.AssetImportTask()
    task.filename = path
    task.destination_path = dest
    task.automated = True
    task.replace_existing = True
    task.save = False
    task.options = ui
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    paths = list(task.get_editor_property("imported_object_paths") or [])
    log(f"{tag} imported_object_paths={paths}")
    reg = unreal.AssetRegistryHelpers.get_asset_registry()
    assets = reg.get_assets_by_path(dest, recursive=True)
    seqs = []
    for a in assets:
        cls = str(a.asset_class_path.asset_name)
        log(f"{tag} asset {a.package_name} {cls}")
        if cls == "AnimSequence":
            seqs.append(unreal.load_asset(str(a.package_name)))
    return seqs, [str(a.asset_class_path.asset_name) for a in assets]


def sample(tag, seq):
    nf = unreal.AnimationLibrary.get_num_frames(seq)
    length = seq.get_play_length()
    try:
        rate = unreal.AnimationLibrary.get_frame_rate(seq)
    except Exception:
        rate = "?"
    try:
        keys = unreal.AnimationLibrary.get_num_keys(seq)
    except Exception:
        keys = nf + 1
    log(f"{tag} seq={seq.get_path_name()} num_frames={nf} num_keys={keys} length={length} rate={rate}")
    opts = unreal.AnimPoseEvaluationOptions()
    p0 = unreal.AnimPoseExtensions.get_anim_pose_at_frame(seq, 0, opts)
    allnames = [str(n) for n in unreal.AnimPoseExtensions.get_bone_names(p0)]
    bones = [b for b in allnames]
    parents = {}
    for b in bones:
        path = [str(x) for x in unreal.AnimationLibrary.find_bone_path_to_root(seq, b)]
        parents[b] = path[1] if len(path) > 1 else None
    loc_q = {b: [] for b in bones}
    cs_q = {b: [] for b in bones}
    cs_t = {b: [] for b in bones}
    for f in range(keys):
        pose = unreal.AnimPoseExtensions.get_anim_pose_at_frame(seq, f, opts)
        for b in bones:
            tl = unreal.AnimPoseExtensions.get_bone_pose(pose, b, unreal.AnimPoseSpaces.LOCAL)
            tw = unreal.AnimPoseExtensions.get_bone_pose(pose, b, unreal.AnimPoseSpaces.WORLD)
            ql = tl.rotation
            qw = tw.rotation
            loc_q[b].append([ql.x, ql.y, ql.z, ql.w])
            cs_q[b].append([qw.x, qw.y, qw.z, qw.w])
            cs_t[b].append([tw.translation.x, tw.translation.y, tw.translation.z])
    out = {"tag": tag, "seq": seq.get_path_name(), "num_frames": nf, "num_keys": keys, "length": length,
           "rate": [rate.numerator, rate.denominator] if hasattr(rate, "numerator") else str(rate),
           "bones": bones, "parents": parents, "major": [b for b in MAJOR if b in bones],
           "local": loc_q, "component": cs_q, "component_t": cs_t}
    with open(os.path.join(HERE, f"ue_samples_{tag}.json"), "w") as fh:
        json.dump(out, fh)
    log(f"{tag} wrote samples bones={len(bones)} frames={keys}")


status = {}
for tag, path in JOBS:
    try:
        seqs, classes = do_import(tag, path)
        status[tag] = {"classes": classes, "seqs": [s.get_path_name() for s in seqs]}
        if not seqs:
            log(f"{tag} NO ANIMSEQUENCE")
            continue
        sample(tag, seqs[0])
    except Exception as e:
        import traceback
        log(f"{tag} ERROR {e!r} {traceback.format_exc()}")
        status[tag] = {"error": repr(e)}
with open(os.path.join(HERE, "ue_status.json"), "w") as fh:
    json.dump(status, fh, indent=1)
log("DONE")
