import json
import os
import sys
import unreal

HERE = "C:/Projects/Fulcrum/ue_check"
JOBS = [("v3", "C:/Projects/Fulcrum/exports/FRT_fulcrum_v3.fbx"), ("v2", "C:/Projects/Fulcrum/exports/FRT_fulcrum_v2.fbx")]
MAJOR = ["root", "pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05",
         "thigh_l", "thigh_r", "calf_l", "calf_r", "foot_l", "foot_r", "ball_l", "ball_r",
         "clavicle_l", "clavicle_r", "upperarm_l", "upperarm_r", "lowerarm_l", "lowerarm_r",
         "hand_l", "hand_r", "neck_01", "neck_02", "head"]


def log(m):
    unreal.log_warning("FULCHK " + str(m))


def do_import(tag, path):
    dest = "/Game/Half_" + tag
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
    opts = unreal.AnimPoseEvaluationOptions()
    bones = [b for b in MAJOR]
    out = {"tag": tag, "times": [], "local": {b: [] for b in bones}, "component": {b: [] for b in bones}}
    for h in range(2 * nf + 1):
        t = h / 48.0
        pose = unreal.AnimPoseExtensions.get_anim_pose_at_time(seq, t, opts)
        out["times"].append(t)
        for b in bones:
            tl = unreal.AnimPoseExtensions.get_bone_pose(pose, b, unreal.AnimPoseSpaces.LOCAL)
            tw = unreal.AnimPoseExtensions.get_bone_pose(pose, b, unreal.AnimPoseSpaces.WORLD)
            out["local"][b].append([tl.rotation.x, tl.rotation.y, tl.rotation.z, tl.rotation.w])
            out["component"][b].append([tw.rotation.x, tw.rotation.y, tw.rotation.z, tw.rotation.w])
    with open(os.path.join(HERE, f"ue_half_{tag}.json"), "w") as fh:
        json.dump(out, fh)
    log(f"{tag} wrote half-frame samples n={len(out['times'])}")


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
with open(os.path.join(HERE, "ue_status_half.json"), "w") as fh:
    json.dump(status, fh, indent=1)
log("DONE")
