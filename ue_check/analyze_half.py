import json
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
from analyze import HERE, ang, quat_to_mat


def run(tag):
    h = json.load(open(os.path.join(HERE, f"ue_half_{tag}.json")))
    k = json.load(open(os.path.join(HERE, f"ue_samples_{tag}.json")))
    worst = {"excess": (0.0, None, None, None), "key_mismatch": (0.0, None, None, None)}
    rows = []
    for space in ("local", "component"):
        for b, qs in h[space].items():
            rs = [quat_to_mat(np.array(q)) for q in qs]
            ks = [quat_to_mat(np.array(q)) for q in k[space][b]]
            n = (len(rs) - 1) // 2
            ex = []
            for f in range(n):
                a, m, c = rs[2 * f], rs[2 * f + 1], rs[2 * f + 2]
                full = ang(a, c)
                excess = ang(a, m) + ang(m, c) - full
                ex.append(excess)
                km = ang(a, ks[f])
                if excess > worst["excess"][0]:
                    worst["excess"] = (excess, b, space, f)
                if km > worst["key_mismatch"][0]:
                    worst["key_mismatch"] = (km, b, space, f)
            ex = np.array(ex)
            rows.append({"bone": b, "space": space, "max_midframe_excess_deg": round(float(ex.max()), 4),
                         "frame": int(ex.argmax()), "frames_excess_over_1deg": int((ex > 1).sum())})
    out = {"tag": tag, "half_samples": len(h["times"]), "worst": worst, "rows": rows}
    json.dump(out, open(os.path.join(HERE, f"half_result_{tag}.json"), "w"), indent=1)
    print(tag, len(h["times"]), worst, sum(r["frames_excess_over_1deg"] for r in rows))


for t in sys.argv[1:] or ["v3", "v2"]:
    run(t)
