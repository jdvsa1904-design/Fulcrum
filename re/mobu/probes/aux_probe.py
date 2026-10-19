import json, traceback
from pyfbsdk import *
OUT = r"C:/Projects/Fulcrum/re/mobu/probes/aux_out.json"
app = FBApplication(); app.FileNew()
sysm = FBSystem(); pc = FBPlayerControl()
res = {}
try:
    J = {}
    def j(name, parent, p):
        s = FBModelSkeleton("mbprobe:" + name)
        s.Show = True
        if parent: s.Parent = J[parent]
        s.SetVector(FBVector3d(*p), FBModelTransformationType.kModelTranslation, True)
        J[name] = s
    j("Hips", None, (0, 100, 0))
    j("Spine", "Hips", (0, 110, 0))
    j("Head", "Spine", (0, 160, 0))
    for sd, sx in (("Left", 1), ("Right", -1)):
        j(sd + "UpLeg", "Hips", (10 * sx, 95, 0))
        j(sd + "Leg", sd + "UpLeg", (10 * sx, 52, 2))
        j(sd + "Foot", sd + "Leg", (10 * sx, 8, 0))
        j(sd + "Arm", "Spine", (20 * sx, 145, 0))
        j(sd + "ForeArm", sd + "Arm", (45 * sx, 145, -1))
        j(sd + "Hand", sd + "ForeArm", (70 * sx, 145, 0))
    sysm.Scene.Evaluate()
    ch = FBCharacter("mbprobe:C")
    for k, mdl in J.items():
        ch.PropertyList.Find(k + "Link").append(mdl)
    res["characterize"] = ch.SetCharacterizeOn(True)
    res["err"] = ch.GetCharacterizeError()
    ch.CreateControlRig(True)
    ch.ActiveInput = True
    cs = ch.GetCurrentControlSet()
    E = FBEffectorId.kFBLeftWristEffectorId
    main = cs.GetIKEffectorModel(E, 0)
    res["aux_created"] = ch.CreateAuxiliary(E, False, 100, 100)
    aux = cs.GetIKEffectorModel(E, 1)
    res["names"] = [main.Name, aux.Name if aux else None]
    res["mainprops"] = [p.Name for p in main.PropertyList if "IK" in p.Name or "Pull" in p.Name or "Reach" in p.Name]
    hand = J["LeftHand"]; hips = J["Hips"]
    G = lambda mdl, t=FBModelTransformationType.kModelTranslation: (lambda v: (mdl.GetVector(v, t, True), [round(x, 4) for x in v])[1])(FBVector3d())
    def setE(e, pos=None, rot=None, rt=None, rr=None, pull=None, pivot=None):
        if pos is not None: e.SetVector(FBVector3d(*pos), FBModelTransformationType.kModelTranslation, True)
        if rot is not None: e.SetVector(FBVector3d(*rot), FBModelTransformationType.kModelRotation, True)
        if rt is not None: e.PropertyList.Find("IK Reach Translation").Data = rt
        if rr is not None: e.PropertyList.Find("IK Reach Rotation").Data = rr
        if pull is not None: e.PropertyList.Find("IK Pull").Data = pull
        if pivot is not None: e.PropertyList.Find("IKPivot").Data = FBVector3d(*pivot)
    def solve():
        for _ in range(3):
            sysm.Scene.Evaluate()
        return {"hand": G(hand), "handR": G(hand, FBModelTransformationType.kModelRotation), "hips": G(hips)}
    res["pullRange"] = [main.PropertyList.Find("IK Pull").GetMin(), main.PropertyList.Find("IK Pull").GetMax()]
    res["fk"] = solve()
    A = (55, 130, 20); B = (50, 150, -20)
    setE(main, A, (0, 0, 0)); setE(aux, B, (0, 90, 0))
    for rm, ra in ((100, 0), (0, 100), (100, 100), (100, 50), (50, 50), (50, 0), (0, 50), (25, 75)):
        setE(main, rt=rm, rr=rm); setE(aux, rt=ra, rr=ra)
        res["T_%d_%d" % (rm, ra)] = solve()
    setE(aux, rt=0, rr=0)
    setE(main, (150, 145, 0), (0, 0, 0), rt=100, rr=0)
    for pv in (0, 25, 50, 100):
        setE(main, pull=pv)
        res["pull_%d" % pv] = solve()
    setE(main, A, (0, 0, 0), rt=100, rr=100, pull=0)
    res["piv0"] = solve()
    setE(main, pivot=(10, 0, 0))
    res["piv10"] = solve()
    setE(main, rot=(0, 90, 0))
    res["piv10_rot90"] = solve()
    res["eff_after"] = G(main)
    json.dump(res, open(OUT, "w"), indent=1)
except Exception:
    res["exc"] = traceback.format_exc()
    json.dump(res, open(OUT, "w"), indent=1)
print("ok")
