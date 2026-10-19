import json, math, random, sys
import maya.standalone
maya.standalone.initialize(name='python')
import maya.cmds as mc
import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma

OUT = sys.argv[1]
rnd = random.Random(20261015)
INF = {'constant': oma.MFnAnimCurve.kConstant, 'linear': oma.MFnAnimCurve.kLinear, 'cycle': oma.MFnAnimCurve.kCycle,
       'cycleRelative': oma.MFnAnimCurve.kCycleRelative, 'oscillate': oma.MFnAnimCurve.kOscillate}
UNIT = {'film': (om.MTime.kFilm, 24.0), 'ntsc': (om.MTime.kNTSCFrame, 30.0), 'ntscf': (om.MTime.kNTSCField, 60.0)}


def fnof(c):
    sel = om.MSelectionList()
    sel.add(c)
    return oma.MFnAnimCurve(sel.getDependNode(0))


def dump(c, unit, samples):
    fn = fnof(c)
    mt, fps = UNIT[unit]
    keys = []
    for i in range(fn.numKeys):
        ia, iw = fn.getTangentAngleWeight(i, True)
        oa, ow = fn.getTangentAngleWeight(i, False)
        ix, iy = fn.getTangentXY(i, True)
        ox, oy = fn.getTangentXY(i, False)
        keys.append(dict(t=fn.input(i).asUnits(mt), v=fn.value(i),
                         itt=mc.keyTangent(c, index=(i, i), q=True, itt=True)[0],
                         ott=mc.keyTangent(c, index=(i, i), q=True, ott=True)[0],
                         ia=ia.asDegrees(), iw=iw, oa=oa.asDegrees(), ow=ow,
                         ix=ix, iy=iy, ox=ox, oy=oy))
    vals = [[t, fn.evaluate(om.MTime(t, mt))] for t in samples]
    return dict(weighted=fn.isWeighted, fps=fps, unit=unit,
                pre=mc.getAttr(c + '.preInfinity'), post=mc.getAttr(c + '.postInfinity'),
                keys=keys, samples=vals)


def sample_times(t0, t1, n, pad=0.0):
    out = [t0 - pad + (t1 - t0 + 2 * pad) * i / (n - 1) for i in range(n)]
    out += [rnd.uniform(t0 - pad, t1 + pad) for _ in range(n // 2)]
    return out


def make(times, values, itt, ott, weighted, unit='film'):
    mc.currentUnit(time=unit)
    c = mc.createNode('animCurveTU')
    mc.keyTangent(c, e=True, weightedTangents=weighted)
    for t, v in zip(times, values):
        mc.setKeyframe(c, t=t, v=v, itt=itt, ott=ott)
    return c


def rand_keys(n, frac=False, flat_pairs=False):
    t, times, vals = 0.0, [], []
    for i in range(n):
        t += rnd.choice([1, 2, 3, 5, 8, 13]) + (rnd.random() if frac else 0)
        times.append(round(t, 3))
        if flat_pairs and vals and rnd.random() < 0.3:
            vals.append(vals[-1] + rnd.uniform(-0.04, 0.04))
        else:
            vals.append(round(rnd.uniform(-10, 10), 3))
    return times, vals


cases = []

for weighted in (False, True):
    for k in range(10):
        times, vals = rand_keys(rnd.randint(2, 7), frac=k % 2 == 1)
        c = make(times, vals, 'spline', 'spline', weighted)
        n = len(times)
        for i in range(n):
            ia = rnd.uniform(-85, 85)
            oa = ia if rnd.random() < 0.5 else rnd.uniform(-85, 85)
            mc.keyTangent(c, index=(i, i), e=True, lock=False)
            if weighted:
                mc.keyTangent(c, index=(i, i), e=True, weightLock=False)
                big = rnd.random() < 0.35
                iw = rnd.uniform(0.1, 40 if big else 6)
                ow = rnd.uniform(0.1, 40 if big else 6)
                mc.keyTangent(c, index=(i, i), e=True, inAngle=ia, outAngle=oa, inWeight=iw, outWeight=ow)
            else:
                mc.keyTangent(c, index=(i, i), e=True, inAngle=ia, outAngle=oa)
        if k == 3:
            mc.keyTangent(c, index=(0, 0), e=True, ott='step')
        if k == 5 and n > 2:
            mc.keyTangent(c, index=(1, 1), e=True, ott='stepnext')
        pre = rnd.choice(list(INF))
        post = rnd.choice(list(INF))
        fn = fnof(c)
        fn.setPreInfinityType(INF[pre])
        fn.setPostInfinityType(INF[post])
        d = dump(c, 'film', sample_times(times[0], times[-1], 61, pad=(times[-1] - times[0]) * 1.3 + 3))
        d.update(kind='fixed', preName=pre, postName=post)
        cases.append(d)

for unit in ('film', 'ntsc'):
    for weighted in (False, True):
        for tt in ('spline', 'linear', 'flat', 'clamped', 'plateau', 'auto', 'autoease', 'automix', 'autocustom', 'step', 'stepnext'):
            for k in range(6):
                times, vals = rand_keys(rnd.randint(2, 8), frac=k == 5, flat_pairs=tt in ('clamped', 'plateau'))
                itt = 'linear' if tt in ('step', 'stepnext') else tt
                c = make(times, vals, itt, tt, weighted, unit)
                d = dump(c, unit, sample_times(times[0], times[-1], 41))
                d.update(kind=tt)
                cases.append(d)

mc.currentUnit(time='film')
for weighted in (False, True):
    for tt in ('spline', 'clamped', 'plateau', 'auto', 'autoease', 'automix'):
        times, vals = rand_keys(9, flat_pairs=True)
        c = mc.createNode('animCurveTU')
        mc.keyTangent(c, e=True, weightedTangents=weighted)
        mc.setKeyframe(c, t=[times[0], times[-1]], v=0)
        fn = fnof(c)
        fn.remove(1)
        fn.remove(0)
        tts = {'spline': oma.MFnAnimCurve.kTangentSmooth, 'clamped': oma.MFnAnimCurve.kTangentClamped,
               'plateau': oma.MFnAnimCurve.kTangentPlateau, 'auto': oma.MFnAnimCurve.kTangentAuto,
               'autoease': getattr(oma.MFnAnimCurve, 'kTangentAutoEase', oma.MFnAnimCurve.kTangentAuto),
               'automix': getattr(oma.MFnAnimCurve, 'kTangentAutoMix', oma.MFnAnimCurve.kTangentAuto)}[tt]
        fn.addKeys([om.MTime(t, om.MTime.kFilm) for t in times], vals, tts, tts)
        d = dump(c, 'film', sample_times(times[0], times[-1], 41))
        d.update(kind=tt, block=True)
        cases.append(d)

json.dump(dict(maya=mc.about(version=True), cases=cases), open(OUT, 'w'), indent=0)
print('cases', len(cases))
maya.standalone.uninitialize()
