import json, math
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/layers_out.json"
app = FBApplication()
app.FileNew()
sysm = FBSystem()
take = sysm.CurrentTake
m = FBModelNull("mbprobe:n")
m.Translation.SetAnimated(True)
m.Rotation.SetAnimated(True)
t0 = FBTime(0, 0, 0, 0)
def put(prop, vals):
    for i, c in enumerate(prop.GetAnimationNode().Nodes):
        c.FCurve.EditClear()
        c.FCurve.KeyAdd(t0, vals[i])
take.SetCurrentLayer(0)
put(m.Translation, [1.0, 1.0, 1.0])
put(m.Rotation, [30.0, 0.0, 0.0])
take.CreateNewLayer()
lay = take.GetLayer(take.GetLayerCount() - 1)
take.SetCurrentLayer(take.GetLayerCount() - 1)
m.Translation.SetAnimated(True)
m.Rotation.SetAnimated(True)
put(m.Translation, [10.0, 10.0, 10.0])
put(m.Rotation, [0.0, 60.0, 20.0])
take.SetCurrentLayer(0)
def ev():
    FBPlayerControl().Goto(FBTime(0, 0, 0, 1))
    FBPlayerControl().Goto(t0)
    sysm.Scene.Evaluate()
    T = FBVector3d(); R = FBVector3d()
    m.GetVector(T, FBModelTransformationType.kModelTranslation, False)
    m.GetVector(R, FBModelTransformationType.kModelRotation, False)
    return [list(T), list(R)]
res = {}
modes = {"add": FBLayerMode.kFBLayerModeAdditive, "ovr": FBLayerMode.kFBLayerModeOverride, "ovp": FBLayerMode.kFBLayerModeOverridePassthrough}
rms = {"eul": FBLayerRotationMode.kFBLayerRotationModeEulerRotation, "quat": FBLayerRotationMode.kFBLayerRotationModeQuaternionRotation}
for mn, mv in modes.items():
    for rn, rv in rms.items():
        for w in (100.0, 50.0, 25.0, 0.0):
            lay.LayerMode = mv
            lay.LayerRotationMode = rv
            lay.Weight = w
            res["%s_%s_%d" % (mn, rn, int(w))] = ev()
take.CreateNewLayer()
child = take.GetLayer(take.GetLayerCount() - 1)
try:
    child.SetParentLayer(lay)
    lay.LayerMode = modes["add"]; lay.Weight = 50.0
    child.Weight = 50.0
    take.SetCurrentLayer(take.GetLayerCount() - 1)
    put(m.Translation, [4.0, 4.0, 4.0])
    take.SetCurrentLayer(0)
    res["parentchild"] = ev()
except Exception as e:
    res["parentchild_err"] = str(e)
json.dump(res, open(OUT, "w"), indent=1)
print("ok")
