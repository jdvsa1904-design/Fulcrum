import json, traceback
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/ckr_out.json"
app = FBApplication(); app.FileNew()
fm = FBFilterManager(); res = {}
FPS = 24.0
def T(fr): t = FBTime(); t.SetSecondDouble(fr / FPS); return t
cnt = [0]
def run(prop, vals, ch=0, keep=True):
    cnt[0] += 1
    m = FBModelNull("mbprobe:c%d" % cnt[0])
    p = getattr(m, prop); p.SetAnimated(True)
    fc = p.GetAnimationNode().Nodes[ch].FCurve
    for i, v in enumerate(vals): fc.KeyAdd(T(i), v)
    f = fm.CreateFilter("Constant Key Reducer")
    f.PropertyList.Find("Keep At Least One Keyframe").Data = keep
    f.Apply(p.GetAnimationNode(), True)
    r = [[round(k.Time.GetSecondDouble() * FPS, 3), k.Value] for k in fc.Keys]
    m.FBDelete()
    return r
try:
    for prop, base in (("Translation", 0.0), ("Rotation", 0.0), ("Scaling", 1.0)):
        out = {}
        for e in (1e-6, 1e-5, 5e-5, 8e-5, 9e-5, 9.5e-5, 1e-4, 2e-4, 5e-4, 1e-3, 4e-3, 5e-3, 8e-3, 8.9e-3, 9e-3, 9.1e-3, 1e-2):
            out["%g" % e] = len(run(prop, [base] * 3 + [base + e] + [base] * 3))
        res["bump_" + prop] = out
    res["interior"] = run("Translation", [0, 1, 1, 1, 1, 1, 0, 2])
    res["interior_noise"] = run("Translation", [0, 1, 1.00001, 1, 1.00001, 1, 0, 2])
    res["interior_noise_R"] = run("Rotation", [0, 1, 1.004, 1, 1.004, 1, 0, 2])
    res["interior_drift_R"] = run("Rotation", [0, 1, 1.004, 1.008, 1.012, 1.016, 0, 2])
    res["two_flat"] = run("Translation", [0, 1, 1, 0])
    res["three_flat"] = run("Translation", [0, 1, 1, 1, 0])
    res["start_flat2"] = run("Translation", [0, 0, 5])
    res["start_flat3"] = run("Translation", [0, 0, 0, 5])
except Exception:
    res["exc"] = traceback.format_exc()
json.dump(res, open(OUT, "w"), indent=1)
print("ok")
