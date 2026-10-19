import json, random
import maya.cmds as mc
import maya.api.OpenMaya as om, maya.api.OpenMayaAnim as oma
rnd = random.Random(5)
NS = 'fulcrumRE'
prev_ns = mc.namespaceInfo(cur=True)
if not mc.namespace(exists=NS): mc.namespace(add=NS)
mc.namespace(set=NS)
out = dict(curves=[], rot=[], skin=[])
fps = {'film':24.0,'ntsc':30.0,'pal':25.0,'ntscf':60.0}.get(mc.currentUnit(q=True, time=True), 24.0)
try:
    for w in (False, True):
        for tt in ('spline','linear','flat','clamped','plateau','auto','autoease','automix'):
            c = mc.createNode('animCurveTU')
            mc.keyTangent(c, e=True, weightedTangents=w)
            t = 0.0
            for i in range(6):
                t += rnd.choice([2,3,5,8]); mc.setKeyframe(c, t=t, v=round(rnd.uniform(-10,10),3), itt=tt, ott=tt)
            sel = om.MSelectionList(); sel.add(c); fn = oma.MFnAnimCurve(sel.getDependNode(0))
            mt = om.MTime.uiUnit()
            keys = []
            for i in range(fn.numKeys):
                ia, iw = fn.getTangentAngleWeight(i, True); oa, ow = fn.getTangentAngleWeight(i, False)
                keys.append(dict(t=fn.input(i).asUnits(mt), v=fn.value(i), ia=ia.asDegrees(), iw=iw, oa=oa.asDegrees(), ow=ow, itt=tt, ott=tt))
            ts = [keys[0]['t'] + (keys[-1]['t']-keys[0]['t'])*k/200.0 for k in range(201)]
            out['curves'].append(dict(weighted=w, kind=tt, fps=fps, keys=keys, samples=[[x, fn.evaluate(om.MTime(x, mt))] for x in ts]))
    for i in range(60):
        n = mc.createNode('animBlendNodeAdditiveRotation')
        a = [rnd.uniform(-170,170) for _ in range(3)]; b = [rnd.uniform(-170,170) for _ in range(3)]
        wa, wb = rnd.uniform(0,1.2), rnd.uniform(0,1.2)
        m = i % 3; ro = i % 6
        mc.setAttr(n+'.accumulationMode', m); mc.setAttr(n+'.rotateOrder', ro)
        mc.setAttr(n+'.inputA', *a); mc.setAttr(n+'.inputB', *b); mc.setAttr(n+'.weightA', wa); mc.setAttr(n+'.weightB', wb)
        out['rot'].append(dict(mode=m, rotateOrder=ro, a=a, b=b, wa=wa, wb=wb, out=list(mc.getAttr(n+'.output')[0])))
    for method in (0, 1):
        mesh = mc.polySphere(r=3, sx=8, sy=6, ch=False)[0]
        mc.setAttr(mesh+'.t', 0.3, -0.2, 0.5)
        mc.select(cl=True)
        js = [mc.joint(p=(rnd.uniform(-1,1), -3+2*k, 0)) for k in range(3)]
        sc = mc.skinCluster(js, mesh, toSelectedBones=True, normalizeWeights=1)[0]
        shape = mc.listRelatives(mesh, s=True, f=True)[0]
        mc.setAttr(sc+'.skinningMethod', method)
        sel = om.MSelectionList(); sel.add(shape); fm = om.MFnMesh(sel.getDagPath(0))
        mc.setAttr(sc+'.envelope', 0); pts = [[p.x,p.y,p.z] for p in fm.getPoints(om.MSpace.kObject)]; mc.setAttr(sc+'.envelope', 1)
        for j in js: mc.setAttr(j+'.r', rnd.uniform(-60,60), rnd.uniform(-60,60), rnd.uniform(-60,60))
        n = len(pts)
        out['skin'].append(dict(method=method, points=pts,
            weights=[[mc.getAttr('%s.weightList[%d].weights[%d]'%(sc,v,k)) for k in range(3)] for v in range(n)],
            bindPre=[list(mc.getAttr('%s.bindPreMatrix[%d]'%(sc,k))) for k in range(3)],
            matrix=[list(mc.getAttr('%s.matrix[%d]'%(sc,k))) for k in range(3)],
            geomMatrix=list(mc.getAttr(sc+'.geomMatrix')),
            out=[[p.x,p.y,p.z] for p in fm.getPoints(om.MSpace.kObject)]))
finally:
    mc.namespace(set=prev_ns if prev_ns.startswith(':') else ':' + prev_ns)
    mc.namespace(removeNamespace=NS, deleteNamespaceContent=True)
open(r'C:/Users/Gaban/AppData/Local/Temp/claude/C--/adc48759-14d4-4fc3-81cb-f65ce1652545/scratchpad/maya/live.json','w').write(json.dumps(out))
globals()['FULCRUM_RE_LIVE'] = 'ok %d %d %d' % (len(out['curves']), len(out['rot']), len(out['skin']))
