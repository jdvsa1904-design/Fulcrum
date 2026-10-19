import json
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/loop2_out.json"
FBX = r"C:/Projects/Fulcrum/re/mobu/probes/mbprobe_clip.fbx"
sysm = FBSystem(); pc = FBPlayerControl(); st = FBStory()
F = lambda f: FBTime(0, 0, 0, f)
m = FBFindModelByLabelName("mbprobe:n")
track = st.RootFolder.Tracks[0]
c = track.Clips[0]
c.Stop = F(100)
res = {}
res["export"] = c.ExportToFile(FBX)
c.FBDelete()
r = FBStoryClip(FBX, track, F(0))
res["readonly"] = r.GetReadOnly() if hasattr(r, "GetReadOnly") else None
def val(f):
    pc.Goto(F(f)); sysm.Scene.Evaluate()
    v = FBVector3d(); m.GetVector(v, FBModelTransformationType.kModelTranslation, True)
    return [round(x, 5) for x in v]
P = lambda n: r.PropertyList.Find(n)
def props():
    d = {}
    for n in ("Start", "Stop", "FirstLoopMarkIn", "LastLoopMarkOut", "ClipTranslation", "ClipRotation", "LoopTranslation", "AutoLoop", "LoopEnable"):
        x = P(n).Data
        d[n] = x.GetFrame() if isinstance(x, FBTime) else str(x)
    d["SourceMarkIn"] = r.SourceMarkIn.GetFrame(); d["SourceMarkOut"] = r.SourceMarkOut.GetFrame(); d["Offset"] = r.Offset.GetFrame()
    return d
res["init"] = props()
res["init_vals"] = [[f] + val(f) for f in (0, 50, 100)]
r.Loop = True
r.Stop = F(250)
res["loop"] = props()
fs = (0, 50, 99, 100, 101, 102, 150, 200, 201, 202, 250)
res["loop_vals"] = [[f] + val(f) for f in fs]
res["srcmap"] = [[f, r.GetSourceTimeFromDestinationTime(F(f))[0].GetFrame(), r.GetSourceTimeFromDestinationTime(F(f))[1]] for f in fs]
r.LoopTranslation = FBVector3d(0, 0, 0)
res["noLT_vals"] = [[f] + val(f) for f in (50, 150, 201)]
r.LoopTranslation = FBVector3d(100, 0, 50)
r.SourceMarkIn = F(20)
res["smi20"] = props()
res["smi20_vals"] = [[f] + val(f) for f in fs]
res["smi20_src"] = [[f, r.GetSourceTimeFromDestinationTime(F(f))[0].GetFrame(), r.GetSourceTimeFromDestinationTime(F(f))[1]] for f in fs]
r.SourceMarkIn = F(0)
r.Offset = F(30)
res["off30"] = props()
res["off30_vals"] = [[f] + val(f) for f in fs]
res["off30_src"] = [[f, r.GetSourceTimeFromDestinationTime(F(f))[0].GetFrame(), r.GetSourceTimeFromDestinationTime(F(f))[1]] for f in fs]
r.Offset = F(0)
r.Speed = 2.0
res["sp2"] = props()
res["sp2_src"] = [[f, r.GetSourceTimeFromDestinationTime(F(f))[0].GetFrame(), r.GetSourceTimeFromDestinationTime(F(f))[1]] for f in (0, 25, 50, 51, 75, 100, 101)]
r.Speed = 1.0
json.dump(res, open(OUT, "w"), indent=1)
print("ok")
