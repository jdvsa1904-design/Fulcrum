import json, random, sys
import maya.standalone
maya.standalone.initialize(name='python')
import maya.cmds as mc

OUT = sys.argv[1]
rnd = random.Random(7)
res = dict(maya=mc.about(version=True), layerWeights=[], nested=[], nodes=[])

j = mc.createNode('joint', n='j')
mc.setKeyframe(j, at='tx', t=0, v=1)
for ov in (False, True):
    for pt in (False, True):
        for mute in (False, True):
            for w in (0.0, 0.25, 0.7, 1.0):
                for ram in (0, 1):
                    L = mc.animLayer('L', override=ov)
                    mc.setAttr(L + '.passthrough', pt)
                    mc.setAttr(L + '.mute', mute)
                    mc.setAttr(L + '.weight', w)
                    mc.setAttr(L + '.rotationAccumulationMode', ram)
                    res['layerWeights'].append(dict(override=ov, passthrough=pt, mute=mute, weight=w, rotAcc=ram,
                        fg=mc.getAttr(L + '.foregroundWeight'), bg=mc.getAttr(L + '.backgroundWeight'),
                        outRotAcc=mc.getAttr(L + '.outRotationAccumulationMode')))
                    mc.delete(L)

for pov in (False, True):
    for cov in (False, True):
        P = mc.animLayer('P', override=pov)
        C = mc.animLayer('C', override=cov, parent=P)
        mc.setAttr(P + '.weight', 0.6)
        mc.setAttr(C + '.weight', 0.3)
        res['nested'].append(dict(parentOverride=pov, childOverride=cov, parentWeight=0.6, childWeight=0.3,
            child=dict(fg=mc.getAttr(C + '.foregroundWeight'), bg=mc.getAttr(C + '.backgroundWeight'), pw=mc.getAttr(C + '.parentWeight')),
            parent=dict(fg=mc.getAttr(P + '.foregroundWeight'), bg=mc.getAttr(P + '.backgroundWeight'))))
        mc.delete(C, P)


def wpair():
    r = rnd.random()
    if r < 0.15:
        return 1.0, 1.0
    if r < 0.3:
        w = rnd.random()
        return 1.0 - w, w
    if r < 0.4:
        return 0.0, rnd.random()
    return rnd.uniform(-0.2, 1.3), rnd.uniform(-0.2, 1.3)


for i in range(80):
    n = mc.createNode('animBlendNodeAdditiveDL')
    a, b = rnd.uniform(-50, 50), rnd.uniform(-50, 50)
    wa, wb = wpair()
    im = i % 2
    mc.setAttr(n + '.interpolateMode', im)
    mc.setAttr(n + '.inputA', a); mc.setAttr(n + '.inputB', b)
    mc.setAttr(n + '.weightA', wa); mc.setAttr(n + '.weightB', wb)
    res['nodes'].append(dict(type='DL', mode=im, a=a, b=b, wa=wa, wb=wb, out=mc.getAttr(n + '.output')))

for i in range(80):
    n = mc.createNode('animBlendNodeAdditiveScale')
    a, b = rnd.uniform(0.05, 4), rnd.uniform(0.05, 4)
    if i % 7 == 0:
        a = -a
    wa, wb = wpair()
    am = i % 2
    mc.setAttr(n + '.accumulationMode', am)
    mc.setAttr(n + '.inputA', a); mc.setAttr(n + '.inputB', b)
    mc.setAttr(n + '.weightA', wa); mc.setAttr(n + '.weightB', wb)
    res['nodes'].append(dict(type='Scale', mode=am, a=a, b=b, wa=wa, wb=wb, out=mc.getAttr(n + '.output')))

for i in range(40):
    n = mc.createNode('animBlendNodeBoolean')
    a, b = rnd.random() < 0.5, rnd.random() < 0.5
    wa, wb = wpair()
    mc.setAttr(n + '.inputA', a); mc.setAttr(n + '.inputB', b)
    mc.setAttr(n + '.weightA', wa); mc.setAttr(n + '.weightB', wb)
    res['nodes'].append(dict(type='Boolean', a=a, b=b, wa=wa, wb=wb, out=mc.getAttr(n + '.output')))

for i in range(600):
    n = mc.createNode('animBlendNodeAdditiveRotation')
    big = 400 if i % 5 == 0 else 120
    a = [rnd.uniform(-big, big) for _ in range(3)]
    b = [rnd.uniform(-big, big) for _ in range(3)]
    if i % 9 == 0:
        b = [0.0, 0.0, 0.0]
    wa, wb = wpair()
    am = i % 3
    ri = (i // 3) % 2
    ro = (i // 6) % 6
    mc.setAttr(n + '.accumulationMode', am)
    mc.setAttr(n + '.rotationInterpolation', ri)
    mc.setAttr(n + '.rotateOrder', ro)
    mc.setAttr(n + '.inputA', *a); mc.setAttr(n + '.inputB', *b)
    mc.setAttr(n + '.weightA', wa); mc.setAttr(n + '.weightB', wb)
    res['nodes'].append(dict(type='Rotation', mode=am, interp=ri, rotateOrder=ro, a=a, b=b, wa=wa, wb=wb,
                             out=list(mc.getAttr(n + '.output')[0])))

res['stacks'] = []
for trial in range(24):
    mc.file(new=True, force=True)
    j = mc.createNode('joint', n='j')
    ro = trial % 6
    mc.setAttr(j + '.rotateOrder', ro)
    base = dict(t=rnd.uniform(-5, 5), r=[rnd.uniform(-170, 170) for _ in range(3)], s=rnd.uniform(0.5, 2), v=rnd.random() < 0.5)
    mc.setKeyframe(j, at='tx', t=0, v=base['t'])
    for k, a in enumerate('xyz'):
        mc.setKeyframe(j, at='r' + a, t=0, v=base['r'][k])
    mc.setKeyframe(j, at='sx', t=0, v=base['s'])
    layers = []
    for li in range(rnd.randint(1, 3)):
        ov = rnd.random() < 0.5
        L = mc.animLayer('L%d' % li, override=ov)
        pt = rnd.random() < 0.6
        w = rnd.choice([1.0, rnd.random()])
        ram = rnd.randint(0, 1)
        sam = rnd.randint(0, 1)
        mute = rnd.random() < 0.15
        mc.setAttr(L + '.passthrough', pt)
        mc.setAttr(L + '.weight', w)
        mc.setAttr(L + '.rotationAccumulationMode', ram)
        mc.setAttr(L + '.scaleAccumulationMode', sam)
        mc.setAttr(L + '.mute', mute)
        mc.animLayer(L, e=True, attribute=[j + '.tx', j + '.rx', j + '.ry', j + '.rz', j + '.sx'])
        val = dict(t=rnd.uniform(-5, 5), r=[rnd.uniform(-170, 170) for _ in range(3)], s=rnd.uniform(0.5, 2))
        mc.setKeyframe(j, at='tx', t=0, v=val['t'], animLayer=L)
        for k, a in enumerate('xyz'):
            mc.setKeyframe(j, at='r' + a, t=0, v=val['r'][k], animLayer=L)
        mc.setKeyframe(j, at='sx', t=0, v=val['s'], animLayer=L)
        layers.append(dict(override=ov, passthrough=pt, weight=w, rotAcc=ram, scaleAcc=sam, mute=mute, value=val))
    mc.currentTime(0)

    def cv(layer, attr):
        c = mc.animLayer(layer, q=True, findCurveForPlug=j + '.' + attr)
        return mc.getAttr(c[0] + '.output') if c else None

    for li, L in enumerate(['L%d' % i for i in range(len(layers))]):
        layers[li]['value'] = dict(t=cv(L, 'tx'), r=[cv(L, 'rx'), cv(L, 'ry'), cv(L, 'rz')], s=cv(L, 'sx'))
    base = dict(t=cv('BaseAnimation', 'tx'), r=[cv('BaseAnimation', a) for a in ('rx', 'ry', 'rz')], s=cv('BaseAnimation', 'sx'))
    res['stacks'].append(dict(rotateOrder=ro, base=base, layers=layers,
        out=dict(t=mc.getAttr(j + '.tx'), r=list(mc.getAttr(j + '.r')[0]), s=mc.getAttr(j + '.sx'))))

json.dump(res, open(OUT, 'w'), indent=0)
print('ok', len(res['nodes']))
maya.standalone.uninitialize()
