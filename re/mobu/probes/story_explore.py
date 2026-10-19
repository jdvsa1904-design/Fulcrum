from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/story_explore.txt"
app = FBApplication(); app.FileNew()
sysm = FBSystem(); take = sysm.CurrentTake
m = FBModelNull("mbprobe:n")
m.Translation.SetAnimated(True)
nx = m.Translation.GetAnimationNode().Nodes
for f in range(0, 101, 1):
    for i in range(3):
        nx[i].FCurve.KeyAdd(FBTime(0,0,0,f), [float(f), 0.0, float(f)*0.5][i])
track = FBStoryTrack(FBStoryTrackType.kFBStoryTrackAnimation)
track.Details.append(m)
clip = track.CopyTakeIntoTrack(FBTimeSpan(FBTime(0,0,0,0), FBTime(0,0,0,100)), take)
lines = []
for p in clip.PropertyList:
    try:
        v = p.Data
    except Exception:
        v = "?"
    try:
        en = [p.EnumList(i) for i in range(20) if p.EnumList(i)] if p.GetPropertyType() == FBPropertyType.kFBPT_enum else ""
    except Exception:
        en = ""
    lines.append("%s | %s | %s | %s" % (p.Name, p.GetPropertyTypeName(), v, en))
lines.append(str([a for a in dir(clip) if not a.startswith("_")]))
open(OUT, "w").write("\n".join(lines))
print("ok")
