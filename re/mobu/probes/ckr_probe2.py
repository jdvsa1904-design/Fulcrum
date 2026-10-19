import json, traceback
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/ckr2_out.json"
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
    res["S_fine"] = {"%g" % e: len(run("Scaling", [1.0] * 3 + [1.0 + e] + [1.0] * 3)) for e in (0.0041, 0.0043, 0.0045, 0.0047, 0.0049)}
    res["S_fine0"] = {"%g" % e: len(run("Scaling", [0.0] * 3 + [e] + [0.0] * 3)) for e in (0.0039, 0.0040, 0.0041, 0.0045)}
    res["T_fine"] = {"%g" % e: len(run("Translation", [0.0] * 3 + [e] + [0.0] * 3)) for e in (9.0e-5, 9.1e-5, 9.2e-5, 9.4e-5)}
    res["R_ramp008"] = run("Rotation", [0.008 * i for i in range(8)])
    res["R_ramp008_x10"] = run("Rotation", [10 + 0.008 * i for i in range(8)])
except Exception:
    res["exc"] = traceback.format_exc()
json.dump(res, open(OUT, "w"), indent=1)
print("ok")
