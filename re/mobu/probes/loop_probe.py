import json
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/loop_out.json"
app = FBApplication(); app.FileNew()
sysm = FBSystem(); pc = FBPlayerControl()
F = lambda f: FBTime(0, 0, 0, f)
m = FBModelNull("mbprobe:n")
m.Translation.SetAnimated(True)
src = sysm.CurrentTake.CopyTake("mbR"); sysm.CurrentTake = src; src.ClearAllProperties(False)
m.Translation.SetAnimated(True)
nodes = m.Translation.GetAnimationNode().Nodes
for f in range(0, 101):
    for i, v in enumerate([10.0 + f, 2.0 + 0.1 * f, 5.0 + 0.5 * f]):
        nodes[i].FCurve.KeyAdd(F(f), v)
tE = sysm.CurrentTake.CopyTake("mbE"); sysm.CurrentTake = tE; tE.ClearAllProperties(False)
def val(f):
    pc.Goto(F(f)); sysm.Scene.Evaluate()
    v = FBVector3d(); m.GetVector(v, FBModelTransformationType.kModelTranslation, True)
    return [round(x, 5) for x in v]
track = FBStoryTrack(FBStoryTrackType.kFBStoryTrackAnimation)
track.Details.append(m)
sysm.CurrentTake = src
c = track.CopyTakeIntoTrack(FBTimeSpan(F(0), F(100)), src)
sysm.CurrentTake = tE
P = lambda n: c.PropertyList.Find(n)
def props():
    return {n: (str(P(n).Data) if not isinstance(P(n).Data, FBTime) else P(n).Data.GetFrame()) for n in ("Start", "Stop", "FirstLoopMarkIn", "LastLoopMarkOut", "ExtractStart", "ExtractStop", "ClipTranslation", "ClipRotation", "LoopTranslation", "AutoLoop", "LoopEnable")} | {"SourceMarkIn": c.SourceMarkIn.GetFrame(), "SourceMarkOut": c.SourceMarkOut.GetFrame(), "Offset": c.Offset.GetFrame()}
res = {"init": props(), "init_vals": [val(f) for f in (0, 50, 100)]}
c.Loop = True
res["afterLoop"] = props()
try:
    c.LastLoopMarkOut = F(250)
    res["setLLMO"] = props()
    res["setLLMO_vals"] = [[f] + val(f) for f in (0, 50, 99, 100, 101, 150, 200, 201, 250)]
except Exception as e:
    res["setLLMO_err"] = str(e)
try:
    c.Stop = F(250)
    res["setStop"] = props()
    res["setStop_vals"] = [[f] + val(f) for f in (0, 50, 99, 100, 101, 150, 200, 201, 250)]
    res["srcmap"] = [[f, c.GetSourceTimeFromDestinationTime(F(f))[0].GetFrame(), c.GetSourceTimeFromDestinationTime(F(f))[1]] for f in (50, 100, 101, 150, 201, 250)]
except Exception as e:
    res["setStop_err"] = str(e)
c.Offset = F(20)
res["offset20"] = props()
res["offset20_vals"] = [[f] + val(f) for f in (0, 50, 80, 81, 150)]
json.dump(res, open(OUT, "w"), indent=1)
print("ok")
