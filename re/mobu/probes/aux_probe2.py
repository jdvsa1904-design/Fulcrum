import json, traceback
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/aux2_out.json"
sysm = FBSystem(); res = {}
try:
    ch = [c for c in sysm.Scene.Characters if c.LongName == "mbprobe:C"][0]
    cs = ch.GetCurrentControlSet()
    E = FBEffectorId.kFBLeftWristEffectorId
    main = cs.GetIKEffectorModel(E, 0); aux = cs.GetIKEffectorModel(E, 1)
    hand = FBFindModelByLabelName("mbprobe:LeftHand"); hips = FBFindModelByLabelName("mbprobe:Hips"); spine = FBFindModelByLabelName("mbprobe:Spine")
    T = FBModelTransformationType.kModelTranslation; R = FBModelTransformationType.kModelRotation
    def G(mdl, t=T):
        v = FBVector3d(); mdl.GetVector(v, t, True); return [round(x, 4) for x in v]
    def setE(e, pos=None, rot=None, rt=None, rr=None, pull=None, pivot=None, ph=None):
        if pos is not None: e.SetVector(FBVector3d(*pos), T, True)
        if rot is not None: e.SetVector(FBVector3d(*rot), R, True)
        if rt is not None: e.PropertyList.Find("IK Reach Translation").Data = rt
        if rr is not None: e.PropertyList.Find("IK Reach Rotation").Data = rr
        if pull is not None: e.PropertyList.Find("IK Pull").Data = pull
        if ph is not None: e.PropertyList.Find("IK Pull Hips").Data = ph
        if pivot is not None: e.PropertyList.Find("IKPivot").Data = FBVector3d(*pivot)
    def solve():
        for _ in range(3): sysm.Scene.Evaluate()
        return {"hand": G(hand), "handR": G(hand, R), "hips": G(hips), "spineR": G(spine, R)}
    setE(main, pivot=(0, 0, 0), pull=0)
    A = (55, 130, 20); B = (50, 150, -20)
    setE(aux, B, (0, 90, 0), rt=0, rr=0)
    avg = (52.5, 140, 0)
    for r in (25, 50, 75, 100):
        setE(main, avg, (0, 45, 0), rt=r, rr=r)
        res["single_avg_%d" % r] = solve()
    setE(main, (51.25, 145, -10), (0, 0, 0), rt=75, rr=0)
    res["single_w2575_r75"] = solve()
    setE(main, (51.25, 145, -10), rt=100)
    res["single_w2575_r100"] = solve()
    setE(main, (51.25, 145, -10), rt=50)
    res["single_w2575_r50"] = solve()
    setE(main, A, (0, 0, 0), rt=50, rr=50); setE(aux, B, (0, 90, 0), rt=50, rr=50)
    res["both50"] = solve()
    setE(main, A, (0, 0, 0), rt=25, rr=25); setE(aux, B, (0, 90, 0), rt=75, rr=75)
    res["m25_a75"] = solve()
    setE(aux, rt=0, rr=0)
    setE(main, (150, 145, 0), (0, 0, 0), rt=100, rr=0)
    for pv, ph in ((0, 0), (100, 0), (0, 100), (100, 100), (50, 100)):
        setE(main, pull=pv, ph=ph)
        res["pull_%d_hips_%d" % (pv, ph)] = solve()
    setE(main, A, (0, 0, 0), rt=100, rr=100, pull=0, ph=0)
    res["reset"] = solve()
except Exception:
    res["exc"] = traceback.format_exc()
json.dump(res, open(OUT, "w"), indent=1)
print("ok")
