import json, math, random, sys
import maya.standalone
maya.standalone.initialize(name='python')
import maya.cmds as mc
import maya.api.OpenMaya as om

OUT = sys.argv[1]
rnd = random.Random(99)


def mlist(m):
    return [m[i] for i in range(16)]


def getm(plug):
    return list(mc.getAttr(plug))


def build(nj=4, scaled=False):
    mc.file(new=True, force=True)
    mesh = mc.polySphere(r=3, sx=8, sy=6, ch=False)[0]
    shape = mc.listRelatives(mesh, s=True)[0]
    mc.setAttr(mesh + '.t', 0.3, -0.2, 0.5)
    mc.setAttr(mesh + '.r', 10, 20, -5)
    mc.setAttr(mesh + '.s', 1.1, 0.9, 1.2)
    mc.select(cl=True)
    joints = []
    for i in range(nj):
        j = mc.joint(p=(rnd.uniform(-1, 1), -3 + 2 * i, rnd.uniform(-1, 1)))
        joints.append(j)
    for j in joints:
        mc.setAttr(j + '.jointOrient', rnd.uniform(-30, 30), rnd.uniform(-30, 30), rnd.uniform(-30, 30))
    return mesh, shape, joints


def weights_random(sc, shape, joints, n, mode):
    mc.setAttr(sc + '.normalizeWeights', 0)
    rows = []
    for v in range(n):
        if v == 0 and mode == 'post':
            w = [0.0] * len(joints)
        else:
            k = rnd.randint(1, len(joints))
            chosen = rnd.sample(range(len(joints)), k)
            w = [0.0] * len(joints)
            for c in chosen:
                w[c] = rnd.random()
            s = sum(w)
            if mode == 'unit':
                w = [x / s for x in w]
            elif mode == 'post':
                w = [x * rnd.uniform(0.3, 2.0) for x in w]
            else:
                w = [x * rnd.uniform(0.3, 1.6) for x in w]
        for i, x in enumerate(w):
            mc.setAttr('%s.weightList[%d].weights[%d]' % (sc, v, i), x)
        rows.append(w)
    return rows


def pose(joints, scaled):
    for j in joints:
        mc.setAttr(j + '.r', rnd.uniform(-70, 70), rnd.uniform(-70, 70), rnd.uniform(-70, 70))
        if rnd.random() < 0.5:
            mc.setAttr(j + '.t', *[mc.getAttr(j + '.t')[0][k] + rnd.uniform(-0.5, 0.5) for k in range(3)])
        if scaled:
            mc.setAttr(j + '.s', rnd.uniform(0.6, 1.5), rnd.uniform(0.6, 1.5), rnd.uniform(0.6, 1.5))


def points(shape, space):
    sel = om.MSelectionList()
    sel.add(shape)
    fn = om.MFnMesh(sel.getDagPath(0))
    return [[p.x, p.y, p.z] for p in fn.getPoints(space)]


cases = []
configs = []
for method in (0, 1, 2):
    for scaled in (False, True):
        for nr in ((False, True) if method else (False,)):
            for norm in ('unit', 'post', 'none'):
                configs.append((method, scaled, nr, norm))

for (method, scaled, nr, norm) in configs:
    mesh, shape, joints = build(4, scaled)
    orig_local = points(shape, om.MSpace.kObject)
    sc = mc.skinCluster(joints, mesh, toSelectedBones=True, normalizeWeights=1, maximumInfluences=4)[0]
    n = len(orig_local)
    w = weights_random(sc, shape, joints, n, norm)
    mc.setAttr(sc + '.normalizeWeights', {'unit': 1, 'post': 2, 'none': 0}[norm])
    mc.setAttr(sc + '.skinningMethod', method)
    mc.setAttr(sc + '.dqsSupportNonRigid', nr)
    blends = []
    if method == 2:
        for v in range(n):
            b = rnd.random()
            mc.setAttr('%s.blendWeights[%d]' % (sc, v), b)
            blends.append(b)
    pose(joints, scaled)
    readw = [[mc.getAttr('%s.weightList[%d].weights[%d]' % (sc, v, i)) for i in range(len(joints))] for v in range(n)]
    cases.append(dict(
        method=method, scaled=scaled, supportNonRigid=nr, normalize=norm,
        geomMatrix=getm(sc + '.geomMatrix'),
        bindPre=[getm('%s.bindPreMatrix[%d]' % (sc, i)) for i in range(len(joints))],
        matrix=[getm('%s.matrix[%d]' % (sc, i)) for i in range(len(joints))],
        meshWorld=getm(mesh + '.worldMatrix[0]'),
        dqsScale=list(mc.getAttr(sc + '.dqsScale')[0]),
        points=orig_local, weights=readw, blendWeights=blends,
        outObject=points(shape, om.MSpace.kObject), outWorld=points(shape, om.MSpace.kWorld)))

norm_cases = []
mesh, shape, joints = build(5)
sc = mc.skinCluster(joints, mesh, toSelectedBones=True, normalizeWeights=1, maximumInfluences=5)[0]
for trial in range(25):
    v = trial
    w = [rnd.random() for _ in joints]
    s = sum(w)
    w = [x / s for x in w]
    mc.setAttr(sc + '.normalizeWeights', 0)
    for i, x in enumerate(w):
        mc.setAttr('%s.weightList[%d].weights[%d]' % (sc, v, i), x)
    mc.setAttr(sc + '.normalizeWeights', 1)
    locks = [rnd.random() < 0.3 for _ in joints]
    target = rnd.randrange(len(joints))
    locks[target] = False
    for j, l in zip(joints, locks):
        mc.setAttr(j + '.liw', l)
    val = rnd.choice([rnd.random(), 0.0, 1.0, rnd.uniform(0, 0.3)])
    mode = trial % 2
    if mode == 0:
        mc.skinPercent(sc, '%s.vtx[%d]' % (mesh, v), transformValue=[(joints[target], val)])
    else:
        mc.skinPercent(sc, '%s.vtx[%d]' % (mesh, v), relative=True, transformValue=[(joints[target], val - 0.5)])
    after = [mc.getAttr('%s.weightList[%d].weights[%d]' % (sc, v, i)) for i in range(len(joints))]
    norm_cases.append(dict(before=w, locks=locks, target=target, value=val if mode == 0 else val - 0.5, relative=mode == 1, after=after))
    for j in joints:
        mc.setAttr(j + '.liw', False)

json.dump(dict(maya=mc.about(version=True), cases=cases, normalize=norm_cases), open(OUT, 'w'))
print('ok', len(cases), len(norm_cases))
maya.standalone.uninitialize()
