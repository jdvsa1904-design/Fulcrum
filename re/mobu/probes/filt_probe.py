import json, traceback
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/filt_out.json"
app = FBApplication(); app.FileNew()
fm = FBFilterManager(); res = {}
FPS = 24.0
def T(fr): t = FBTime(); t.SetSecondDouble(fr / FPS); return t
cnt = [0]
def curve(prop, keys, ch=0):
    cnt[0] += 1
    m = FBModelNull("mbprobe:f%d" % cnt[0])
    p = getattr(m, prop); p.SetAnimated(True)
    fc = p.GetAnimationNode().Nodes[ch].FCurve
    fc.EditClear()
    for fr, v in keys: fc.KeyAdd(T(fr), v)
    return m, p, fc
def keys(fc): return [[round(k.Time.GetSecondDouble() * FPS, 4), round(k.Value, 6)] for k in fc.Keys]
def run(name, prop, ks, props=None, ch=0, node=False):
    m, p, fc = curve(prop, ks, ch)
    f = fm.CreateFilter(name)
    for k, v in (props or {}).items(): f.PropertyList.Find(k).Data = v
    before = keys(fc)
    ev = [round(fc.Evaluate(T(x / 2.0)), 6) for x in range(0, int(2 * max(k[0] for k in ks)) + 1)]
    ok = f.Apply(p.GetAnimationNode(), True) if node else f.Apply(fc)
    return {"ok": ok, "before": before, "after": keys(fc), "eval_before_halfframes": ev}
try:
    seq = lambda vals: [(i, v) for i, v in enumerate(vals)]
    ckr = "Constant Key Reducer"
    res["ckr_const"] = run(ckr, "Translation", seq([1.0] * 6), node=True)
    res["ckr_const_nokeep"] = run(ckr, "Translation", seq([1.0] * 6), {"Keep At Least One Keyframe": False}, node=True)
    res["ckr_noise_T"] = run(ckr, "Translation", seq([0, 0.005, 0.0, 0.005, 0, 1, 1, 1, 1.008, 1, 1]), node=True)
    res["ckr_step008_T"] = run(ckr, "Translation", seq([0.008 * i for i in range(8)]), node=True)
    res["ckr_step010_T"] = run(ckr, "Translation", seq([0.010 * i for i in range(8)]), node=True)
    res["ckr_step0089_T"] = run(ckr, "Translation", seq([0.0089 * i for i in range(8)]), node=True)
    res["ckr_step0091_T"] = run(ckr, "Translation", seq([0.0091 * i for i in range(8)]), node=True)
    res["ckr_bump_T"] = run(ckr, "Translation", seq([0, 0, 0, 0.0085, 0, 0, 0]), node=True)
    res["ckr_bump2_T"] = run(ckr, "Translation", seq([0, 0, 0, 0.0095, 0, 0, 0]), node=True)
    res["ckr_bump_R"] = run(ckr, "Rotation", seq([0, 0, 0, 0.0085, 0, 0, 0]), node=True)
    res["ckr_bump2_R"] = run(ckr, "Rotation", seq([0, 0, 0, 0.0095, 0, 0, 0]), node=True)
    res["ckr_bump_S"] = run(ckr, "Scaling", seq([1, 1, 1, 1.00008, 1, 1, 1]), node=True)
    res["ckr_bump2_S"] = run(ckr, "Scaling", seq([1, 1, 1, 1.0001, 1, 1, 1]), node=True)
    res["ckr_flatends"] = run(ckr, "Translation", seq([0, 0, 0, 0, 5, 5, 5, 5]), node=True)
    tri = [(0, 0.0), (10, 10.0), (20, 0.0)]
    res["resample24"] = run("Resample", "Translation", tri)
    res["resample12"] = run("Resample", "Translation", tri, {"Frame Rate": 12.0})
    res["resample_noint"] = run("Resample", "Translation", tri, {"With interpolation": False, "Frame Rate": 12.0})
    res["resample_frac"] = run("Resample", "Translation", [(0.3, 0.0), (10.6, 10.0)])
    res["kof"] = run("Keys on Frame", "Translation", [(0, 0.0), (2.4, 3.0), (5.6, 1.0), (10, 10.0)])
    res["kof12"] = run("Keys on Frame", "Translation", [(0, 0.0), (2.4, 3.0), (5.6, 1.0), (10, 10.0)], {"Frame Rate": 12.0})
    res["tss"] = run("Time Shift And Scale", "Translation", [(0, 0.0), (5, 5.0), (10, 0.0)], {"Shift": T(5), "Scale": 2.0})
    res["tss_late"] = run("Time Shift And Scale", "Translation", [(10, 0.0), (15, 5.0), (20, 0.0)], {"Shift": T(0), "Scale": 2.0})
    res["tss_late_half"] = run("Time Shift And Scale", "Translation", [(10, 0.0), (15, 5.0), (20, 0.0)], {"Shift": T(3), "Scale": 0.5})
except Exception:
    res["exc"] = traceback.format_exc()
json.dump(res, open(OUT, "w"), indent=1)
print("ok")
