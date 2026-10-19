import json
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/story_out.json"
app = FBApplication(); app.FileNew()
sysm = FBSystem(); pc = FBPlayerControl()
F = lambda f: FBTime(0, 0, 0, f)
m = FBModelNull("mbprobe:n")
m.Translation.SetAnimated(True)
def mktake(name, fn, n=100):
    tk = sysm.CurrentTake.CopyTake(name)
    sysm.CurrentTake = tk
    tk.ClearAllProperties(False)
    m.Translation.SetAnimated(True)
    nodes = m.Translation.GetAnimationNode().Nodes
    for i in range(3):
        nodes[i].FCurve.EditClear()
    for f in range(0, n + 1):
        v = fn(f)
        for i in range(3):
            nodes[i].FCurve.KeyAdd(F(f), v[i])
    return tk
tA = mktake("mbA", lambda f: [0.0, 0.0, 0.0])
tB = mktake("mbB", lambda f: [100.0, 0.0, 0.0])
tR = mktake("mbR", lambda f: [float(f), 0.0, 0.5 * f])
tE = sysm.CurrentTake.CopyTake("mbE")
sysm.CurrentTake = tE
tE.ClearAllProperties(False)
def val(f):
    pc.Goto(F(f)); sysm.Scene.Evaluate()
    v = FBVector3d(); m.GetVector(v, FBModelTransformationType.kModelTranslation, True)
    return list(v)
res = {"fps": FBPlayerControl().GetTransportFpsValue()}
track = FBStoryTrack(FBStoryTrackType.kFBStoryTrackAnimation)
track.Details.append(m)
span = FBTimeSpan(F(0), F(100))
def cp(tk):
    sysm.CurrentTake = tk
    c = track.CopyTakeIntoTrack(span, tk)
    sysm.CurrentTake = tE
    return c
cA = cp(tA)
cB = cp(tB)
cB.Start = F(80)
res["A"] = [cA.Start.GetFrame(), cA.Stop.GetFrame()]
res["B"] = [cB.Start.GetFrame(), cB.Stop.GetFrame()]
cA.Stop = F(100)
cB.Start = F(80)
res["preB"] = [cB.PropertyList.Find("PreBlendStart").Data.GetFrame(), cB.PropertyList.Find("PreBlendStop").Data.GetFrame()]
res["postA"] = [cA.PropertyList.Find("PostBlendStart").Data.GetFrame(), cA.PropertyList.Find("PostBlendStop").Data.GetFrame()]
blends = {}
for k in range(1, 5):
    cB.PropertyList.Find("InInterpolator").Data = k
    cA.PropertyList.Find("OutInterpolator").Data = k
    blends[k] = [val(f)[0] for f in range(78, 103)]
    cA.PropertyList.Find("OutInterpolator").Data = 1
    blends["in%d_outlin" % k] = [val(f)[0] for f in range(78, 103)]
res["blends"] = blends
cA.FBDelete(); cB.FBDelete()
cR = cp(tR)
res["R_check"] = [val(f)[0] for f in (0, 50, 100)]
res["R_props"] = {p.Name: str(p.Data) for p in cR.PropertyList if p.Name in ("ClipTranslation", "ClipRotation", "LoopTranslation", "Start", "Stop", "FirstLoopMarkIn", "LastLoopMarkOut")}
warps = {}
for k in range(1, 9):
    cR.PropertyList.Find("WarpInterpolator").Data = k
    warps[k] = [val(f)[0] for f in range(0, 101, 5)]
cR.PropertyList.Find("WarpInterpolator").Data = 1
res["warps"] = warps
cR.Speed = 2.0
res["speed2"] = [val(f) for f in (0, 10, 25, 49, 50, 60, 80)]
cR.Speed = 1.0
cR.Offset = F(10)
res["offset10"] = [val(f) for f in (0, 10, 50)]
cR.Offset = F(0)
cR.Loop = True
cR.PropertyList.Find("LoopEnable").Data = True
cR.Stop = F(250)
res["loop"] = [[f] + val(f) for f in (0, 50, 99, 100, 101, 102, 150, 201, 202, 250)]
res["srcmap"] = [[f] + [str(x) for x in cR.GetSourceTimeFromDestinationTime(F(f))] for f in (50, 100, 101, 150, 202)]
cR.LoopTranslation = FBVector3d(0, 0, 0)
res["loop_noLT"] = [[f] + val(f) for f in (50, 150)]
json.dump(res, open(OUT, "w"), indent=1)
print("ok")
