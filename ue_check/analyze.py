import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.join(os.path.dirname(HERE), "exports")
JUMP = 25.0
TOL = 5.0
MAJOR = ["root", "pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05",
         "thigh_l", "thigh_r", "calf_l", "calf_r", "foot_l", "foot_r", "ball_l", "ball_r",
         "clavicle_l", "clavicle_r", "upperarm_l", "upperarm_r", "lowerarm_l", "lowerarm_r",
         "hand_l", "hand_r", "neck_01", "neck_02", "head"]


def polar(m):
    u, _, vt = np.linalg.svd(m)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return r


def quat_to_mat(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ang(a, b):
    r = a.T @ b
    c = np.clip((np.trace(r) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


def steps(rs):
    return np.array([ang(rs[i], rs[i + 1]) for i in range(len(rs) - 1)])


def from0(rs):
    return np.array([ang(rs[0], r) for r in rs])


def source_rots(path):
    d = json.load(open(path))
    names = d["bones"]
    fr = np.array(d["frames"], dtype=np.float64).reshape(len(d["frames"]), len(names), 4, 4)
    fr = np.transpose(fr, (0, 1, 3, 2))
    rot = {}
    for i, n in enumerate(names):
        rot[n] = [polar(fr[f, i, :3, :3]) for f in range(fr.shape[0])]
    return d, rot


def ue_rots(path):
    u = json.load(open(path))
    loc = {b: [quat_to_mat(np.array(q)) for q in u["local"][b]] for b in u["bones"]}
    cs = {b: [quat_to_mat(np.array(q)) for q in u["component"][b]] for b in u["bones"]}
    return u, loc, cs


def run(tag):
    src_path = os.path.join(EXP, f"FRT_fulcrum_{tag}.frames.json")
    ue_path = os.path.join(HERE, f"ue_samples_{tag}.json")
    d, sw = source_rots(src_path)
    u, uloc, ucs = ue_rots(ue_path)
    parents = u["parents"]
    n_src = len(d["frames"])
    n_ue = len(u["local"][u["bones"][0]])
    n = min(n_src, n_ue)
    sl = {}
    for b in sw:
        p = parents.get(b)
        if p and p in sw:
            sl[b] = [sw[p][f].T @ sw[b][f] for f in range(n_src)]
        else:
            sl[b] = sw[b]
    rows = []
    flags = []
    for b in MAJOR:
        if b not in ucs or b not in sw:
            rows.append({"bone": b, "note": "missing in " + ("ue" if b not in ucs else "source")})
            continue
        s_w = steps(sw[b][:n])
        s_l = steps(sl[b][:n])
        u_c = steps(ucs[b][:n])
        u_l = steps(uloc[b][:n])
        dc = np.abs(u_c - s_w)
        dl = np.abs(u_l - s_l)
        drift_c = np.abs(from0(ucs[b][:n]) - from0(sw[b][:n]))
        drift_l = np.abs(from0(uloc[b][:n]) - from0(sl[b][:n]))
        rows.append({
            "bone": b, "parent": parents.get(b),
            "ue_local_max_step": round(float(u_l.max()), 3), "ue_local_max_frame": int(u_l.argmax()) + 1,
            "ue_comp_max_step": round(float(u_c.max()), 3), "ue_comp_max_frame": int(u_c.argmax()) + 1,
            "src_local_max_step": round(float(s_l.max()), 3), "src_local_max_frame": int(s_l.argmax()) + 1,
            "src_world_max_step": round(float(s_w.max()), 3), "src_world_max_frame": int(s_w.argmax()) + 1,
            "max_abs_step_diff_local": round(float(dl.max()), 4),
            "max_abs_step_diff_comp": round(float(dc.max()), 4),
            "max_from0_diff_local": round(float(drift_l.max()), 4),
            "max_from0_diff_comp": round(float(drift_c.max()), 4),
            "ue_steps_over_25_local": int((u_l > JUMP).sum()),
            "ue_steps_over_25_comp": int((u_c > JUMP).sum()),
            "src_steps_over_25_local": int((s_l > JUMP).sum()),
            "src_steps_over_25_world": int((s_w > JUMP).sum()),
            "slide_kneel_140_215_ue_local_max": round(float(u_l[139:215].max()), 3),
            "slide_kneel_140_215_ue_comp_max": round(float(u_c[139:215].max()), 3),
            "slide_kneel_140_215_src_local_max": round(float(s_l[139:215].max()), 3),
            "slide_kneel_140_215_src_world_max": round(float(s_w[139:215].max()), 3),
        })
        for space, ue_s, src_s in (("local", u_l, s_l), ("component", u_c, s_w)):
            for i in np.nonzero((ue_s > JUMP) | (src_s > JUMP))[0]:
                flags.append({"bone": b, "space": space, "frame_from": int(i), "frame_to": int(i + 1),
                              "ue_step": round(float(ue_s[i]), 3), "src_step": round(float(src_s[i]), 3),
                              "artifact": bool(ue_s[i] - src_s[i] > TOL)})
    allb = [b for b in u["bones"] if b in sw]
    worst_all = []
    for b in allb:
        u_l = steps(uloc[b][:n])
        s_l = steps(sl[b][:n])
        u_c = steps(ucs[b][:n])
        s_w = steps(sw[b][:n])
        worst_all.append((b, float(np.abs(u_l - s_l).max()), float(np.abs(u_c - s_w).max()),
                          int(((u_l > JUMP) & (u_l - s_l > TOL)).sum() + ((u_c > JUMP) & (u_c - s_w > TOL)).sum())))
    keys = [k for k in rows[0].keys()] if rows and "note" not in rows[0] else None
    keys = keys or sorted({k for r in rows for k in r})
    with open(os.path.join(HERE, f"steps_{tag}.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=sorted({k for r in rows for k in r}, key=lambda k: (k != "bone", k)))
        w.writeheader()
        for r in rows:
            w.writerow(r)
    with open(os.path.join(HERE, f"flags_{tag}.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["bone", "space", "frame_from", "frame_to", "ue_step", "src_step", "artifact"])
        w.writeheader()
        for r in flags:
            w.writerow(r)
    summary = {
        "tag": tag, "src_frames": n_src, "ue_keys": n_ue, "ue_num_frames": u["num_frames"], "ue_length_s": u["length"],
        "ue_rate": u["rate"], "ue_bones": len(u["bones"]), "src_bones": len(d["bones"]),
        "bones_compared_all": len(allb),
        "all_bones_max_local_step_diff": max(worst_all, key=lambda x: x[1])[:2],
        "all_bones_max_comp_step_diff": max(worst_all, key=lambda x: x[2])[::2][:2],
        "all_bones_artifact_jumps": sum(x[3] for x in worst_all),
        "major_flags": len(flags), "major_artifacts": sum(1 for f in flags if f["artifact"]),
    }
    json.dump({"summary": summary, "rows": rows, "flags": flags}, open(os.path.join(HERE, f"result_{tag}.json"), "w"), indent=1)
    print(json.dumps(summary, indent=1))
    return summary


if __name__ == "__main__":
    for t in sys.argv[1:] or ["v3", "v2"]:
        run(t)
