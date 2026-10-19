import json, random, sys
import maya.standalone
maya.standalone.initialize(name='python')
import maya.cmds as mc
import maya.api.OpenMaya as om

OUT = sys.argv[1]
rnd = random.Random(2027)


def getm(plug):
    return list(mc.getAttr(plug))


def points(shape, space):
    sel = om.MSelectionList()
    sel.add(shape)
    return [[p.x, p.y, p.z] for p in om.MFnMesh(sel.getDagPath(0)).getPoints(space)]


def build(label, method, norm, nonRigid, setup):
    mc.file(new=True, force=True)
    mesh = mc.polyCube(w=4, h=4, d=4, sx=2, sy=2, sz=2, ch=False)[0]
    shape = mc.listRelatives(mesh, s=True)[0]
    mc.setAttr(mesh + '.t', 0.3, -0.2, 0.5)
    mc.setAttr(mesh + '.r', 10, 20, -5)
    mc.select(cl=True)
    grp = mc.group(em=True, n='P')
    js = []
    for i in range(3):
        mc.select(grp if i == 0 else js[-1])
        js.append(mc.joint(p=(rnd.uniform(-1, 1), -2 + 1.5 * i, rnd.uniform(-1, 1))))
    setup('bind', grp, js)
    sc = mc.skinCluster(js, mesh, tsb=True)[0]
    mc.setAttr(sc + '.normalizeWeights', 0)
    orig = points(shape, om.MSpace.kObject)
    W = []
    for v in range(len(orig)):
        w = [rnd.uniform(0, 1.3) if rnd.random() < 0.7 else 0.0 for _ in js]
        if not any(w):
            w[rnd.randrange(3)] = rnd.uniform(0.2, 1.3)
        for i, x in enumerate(w):
            mc.setAttr('%s.weightList[%d].weights[%d]' % (sc, v, i), x)
        W.append(w)
    mc.setAttr(sc + '.normalizeWeights', {'post': 2, 'none': 0}[norm])
    mc.setAttr(sc + '.skinningMethod', method)
    mc.setAttr(sc + '.dqsSupportNonRigid', nonRigid)
    blends = []
    if method == 2:
        for v in range(len(orig)):
            b = rnd.random()
            mc.setAttr('%s.blendWeights[%d]' % (sc, v), b)
            blends.append(b)
    for j in js:
        mc.setAttr(j + '.r', *[rnd.uniform(-60, 60) for _ in range(3)])
    mc.setAttr(grp + '.t', 0.5, -0.3, 0.2)
    setup('pose', grp, js)
    return dict(label=label, method=method, normalize=norm, supportNonRigid=bool(nonRigid),
                geomMatrix=getm(sc + '.geomMatrix'),
                bindPre=[getm('%s.bindPreMatrix[%d]' % (sc, i)) for i in range(3)],
                matrix=[getm('%s.matrix[%d]' % (sc, i)) for i in range(3)],
                points=orig, weights=W, blendWeights=blends,
                outObject=points(shape, om.MSpace.kObject))


def rs():
    return [rnd.uniform(0.5, 1.8) for _ in range(3)]


def only(stage, fn):
    return lambda s, g, js: fn(g, js) if s == stage else None


SETUPS = {
    'rigid': lambda s, g, js: None,
    'uniform': only('pose', lambda g, js: [mc.setAttr(j + '.s', *([rnd.uniform(0.5, 1.8)] * 3)) for j in js]),
    'nonUniform': only('pose', lambda g, js: [mc.setAttr(j + '.s', *rs()) for j in js]),
    'oneJoint': only('pose', lambda g, js: mc.setAttr(js[1] + '.s', *rs())),
    'parent': only('pose', lambda g, js: (mc.setAttr(g + '.s', *rs()), mc.setAttr(g + '.r', 25, -30, 15))),
    'shear': only('pose', lambda g, js: mc.setAttr(js[2] + '.shear', 0.3, -0.2, 0.1)),
    'shearBelowGate': only('pose', lambda g, js: mc.setAttr(js[2] + '.shear', 5e-5, 0, 0)),
    'scaleBelowGate': only('pose', lambda g, js: mc.setAttr(js[2] + '.s', 1 + 1e-11, 1, 1)),
    'bindScaledKept': only('bind', lambda g, js: mc.setAttr(js[0] + '.s', 1.6, 0.8, 1.2)),
    'bindScaledReset': lambda s, g, js: mc.setAttr(js[0] + '.s', *((1.6, 0.8, 1.2) if s == 'bind' else (1, 1, 1))),
}

cases = []
for label, setup in SETUPS.items():
    for method in (1, 2):
        for norm in ('none', 'post'):
            for nr in (1, 0):
                cases.append(build(label, method, norm, nr, setup))
json.dump(dict(maya=mc.about(version=True), cases=cases), open(OUT, 'w'))
print('ok', len(cases))
maya.standalone.uninitialize()
