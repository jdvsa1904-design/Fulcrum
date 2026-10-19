import test from 'node:test';
import assert from 'node:assert/strict';
import {
  CHANNELS, sortCurves, prepareCurve, evalCurve, makeView, toScreen, toWorld, panView, zoomView, frameAll,
  hitKey, hitHandle, handleScreen, keysInBox, insertKey, deleteKeys, moveKeys, setTangentType, setHandle,
  createHistory, layerToCurves, curvesToLayer, selKey, curveId,
} from '../ui/grapheditor.js';
import { compileLayers, evalTranslation, setLayerKey } from '../wire/animlayers.js';
import { evaluate } from '../re/maya/curves.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b}`);
const K = (t, v, type = 'auto') => ({ t, v, inType: type, outType: type, inAngle: 0, outAngle: 0, inWeight: 1, outWeight: 1 });
const curve = (keys, channel = 'rotateX', bone = 'Hips') => ({ bone, channel, keys });
const V = makeView(0, 100, -10, 10, 200, 100);

test('toScreen and toWorld are inverse', () => {
  const [x, y] = toScreen(V, 25, 5);
  near(x, 50); near(y, 25);
  const [t, v] = toWorld(V, x, y);
  near(t, 25); near(v, 5);
});

test('panView moves the world under the cursor by the pixel delta', () => {
  const p = panView(V, 20, 10);
  const [t, v] = toWorld(p, 70, 35);
  const [t0, v0] = toWorld(V, 50, 25);
  near(t, t0); near(v, v0);
});

test('zoomView keeps the cursor point fixed and scales spans', () => {
  const z = zoomView(V, 50, 25, 0.5);
  const [t, v] = toWorld(z, 50, 25);
  near(t, 25); near(v, 5);
  near(z.x1 - z.x0, 50); near(z.y1 - z.y0, 10);
  const zx = zoomView(V, 50, 25, 0.5, 1);
  near(zx.y1 - zx.y0, 20);
});

test('frameAll covers every key and the evaluated overshoot', () => {
  const cs = [curve([K(10, 0, 'spline'), K(20, 10, 'spline'), K(30, 0, 'spline')]), curve([K(5, -3, 'linear'), K(40, -3, 'linear')], 'rotateY')];
  const v = frameAll(cs, 200, 100);
  assert.ok(v.x0 < 5 && v.x1 > 40 && v.y0 < -3 && v.y1 >= 10);
  const vt = frameAll(cs, 200, 118, { top: 18 });
  const [, y] = toScreen(vt, 0, 10);
  assert.ok(y > 18);
});

test('frameAll falls back to the frame range when there are no keys', () => {
  const v = frameAll([curve([])], 200, 100, { frameCount: 50, pad: 0 });
  near(v.x0, 0); near(v.x1, 49);
});

test('evalCurve matches re/maya/curves evaluation and passes through keys', () => {
  const c = curve([K(0, 0), K(10, 5), K(20, 1)]);
  near(evalCurve(c, 10), 5);
  near(evalCurve(c, 7), evaluate(prepareCurve(c), 7));
  const lin = curve([K(0, 0, 'linear'), K(10, 10, 'linear')]);
  near(evalCurve(lin, 3), 3);
  const st = curve([K(0, 0, 'step'), K(10, 10, 'step')]);
  near(evalCurve(st, 9.9), 0);
});

test('hitKey finds the nearest key within radius and respects visibility', () => {
  const cs = [curve([K(25, 5)]), curve([K(26, 5)], 'rotateY')];
  const [x, y] = toScreen(V, 25, 5);
  assert.deepEqual(hitKey(cs, V, x, y), { ci: 0, ki: 0 });
  assert.equal(hitKey(cs, V, x + 50, y), null);
  assert.deepEqual(hitKey(cs, V, x, y, 6, new Set(['Hips.rotateY'])), { ci: 1, ki: 0 });
});

test('keysInBox selects keys inside the rectangle in either drag direction', () => {
  const cs = [curve([K(10, 0), K(50, 0), K(90, 0)])];
  const s = keysInBox(cs, V, 190, 0, 30, 100);
  assert.deepEqual([...s].sort(), [selKey(0, 1), selKey(0, 2)].sort());
});

test('handles sit along the tangent and are hit-testable', () => {
  const cs = [curve([K(0, 0, 'linear'), K(50, 5, 'linear'), K(100, 10, 'linear')])];
  const p = prepareCurve(cs[0]);
  const [kx, ky] = toScreen(V, 50, 5), [hx, hy] = handleScreen(p, 1, 'out', V, 40);
  near(Math.hypot(hx - kx, hy - ky), 40, 1e-6);
  assert.ok(hx > kx && hy < ky);
  const [ix] = handleScreen(p, 1, 'in', V, 40);
  assert.ok(ix < kx);
  assert.deepEqual(hitHandle(cs, new Set([selKey(0, 1)]), V, hx, hy), { ci: 0, ki: 1, side: 'out' });
  assert.equal(hitHandle(cs, new Set(), V, hx, hy), null);
});

test('insertKey adds a key on the curve value without changing the shape there', () => {
  const cs = [curve([K(0, 0), K(20, 10)])];
  const before = evalCurve(cs[0], 10);
  const r = insertKey(cs, 0, 10);
  assert.equal(r.curves[0].keys.length, 3);
  assert.equal(r.ki, 1);
  near(r.curves[0].keys[1].v, before);
  assert.equal(cs[0].keys.length, 2);
  const r2 = insertKey(r.curves, 0, 10, { v: 3 });
  assert.equal(r2.curves[0].keys.length, 3);
  near(r2.curves[0].keys[1].v, 3);
});

test('deleteKeys removes only selected keys', () => {
  const cs = [curve([K(0, 0), K(10, 1), K(20, 2)]), curve([K(0, 0)], 'rotateY')];
  const out = deleteKeys(cs, new Set([selKey(0, 1), selKey(1, 0)]));
  assert.deepEqual(out[0].keys.map(k => k.t), [0, 20]);
  assert.equal(out[1].keys.length, 0);
  assert.equal(cs[0].keys.length, 3);
});

test('moveKeys snaps time, re-sorts and remaps selection', () => {
  const cs = [curve([K(0, 0), K(10, 1), K(20, 2)])];
  const r = moveKeys(cs, new Set([selKey(0, 0)]), 14.6, 1);
  assert.deepEqual(r.curves[0].keys.map(k => k.t), [10, 15, 20]);
  assert.deepEqual([...r.sel], [selKey(0, 1)]);
  near(r.curves[0].keys[1].v, 1);
  const r2 = moveKeys(cs, new Set([selKey(0, 0)]), 10, 0);
  assert.deepEqual(r2.curves[0].keys.map(k => k.t), [10, 20]);
  near(r2.curves[0].keys[0].v, 0);
  const r3 = moveKeys(cs, new Set([selKey(0, 1)]), 0.3, 0, { snap: false });
  near(r3.curves[0].keys[1].t, 10.3);
});

test('setTangentType changes both sides or one side', () => {
  const cs = [curve([K(0, 0), K(10, 1)])];
  const a = setTangentType(cs, new Set([selKey(0, 0)]), 'flat');
  assert.equal(a[0].keys[0].inType, 'flat'); assert.equal(a[0].keys[0].outType, 'flat');
  const b = setTangentType(cs, new Set([selKey(0, 1)]), 'step', 'out');
  assert.equal(b[0].keys[1].inType, 'auto'); assert.equal(b[0].keys[1].outType, 'step');
  near(evalCurve(a[0], 0.01), 0, 1e-3);
});

test('setHandle fixes the tangent angle in both directions when unified', () => {
  const cs = [curve([K(0, 0, 'flat'), K(10, 0, 'flat'), K(20, 0, 'flat')])];
  const out = setHandle(cs, 0, 1, 'out', 1, 1);
  const k = out[0].keys[1];
  assert.equal(k.outType, 'fixed'); assert.equal(k.inType, 'fixed');
  near(k.outAngle, 45); near(k.inAngle, 45);
  assert.ok(evalCurve(out[0], 11) > 0 && evalCurve(out[0], 9) < 0);
  const brk = setHandle(cs, 0, 1, 'in', -1, 1, { unified: false });
  near(brk[0].keys[1].inAngle, -45); near(brk[0].keys[1].outAngle, 0);
});

test('setHandle sets weights on weighted curves', () => {
  const cs = [{ ...curve([K(0, 0, 'flat'), K(10, 0, 'flat')]), weighted: true }];
  const out = setHandle(cs, 0, 0, 'out', 3, 0);
  near(out[0].keys[0].outWeight, 1); near(out[0].keys[0].outAngle, 0);
});

test('history undo and redo walk states and new pushes clear redo', () => {
  const h = createHistory();
  assert.equal(h.undo('a'), null);
  h.push('a'); h.push('b');
  assert.equal(h.undo('c'), 'b');
  assert.equal(h.undo('b'), 'a');
  assert.ok(!h.canUndo && h.canRedo);
  assert.equal(h.redo('a'), 'b');
  h.push('x');
  assert.ok(!h.canRedo);
  const small = createHistory(2);
  small.push(1); small.push(2); small.push(3);
  assert.equal(small.undo(4), 3); assert.equal(small.undo(3), 2); assert.equal(small.undo(2), null);
});

test('sortCurves orders by bone then channel', () => {
  const s = sortCurves([curve([], 'translateX', 'B'), curve([], 'rotateZ', 'A'), curve([], 'rotateX', 'A')]);
  assert.deepEqual(s.map(curveId), ['A.rotateX', 'A.rotateZ', 'B.translateX']);
  assert.equal(CHANNELS.length, 6);
});

test('layer round trip: layerToCurves then curvesToLayer preserves keys and evaluation', () => {
  const layer = { name: 'L', tangent: 'auto', bones: { Spine: [{ f: 0, e: [0, 1, 2] }, { f: 10, e: [10, 5, -2], tangent: 'linear' }, { f: 20, e: [0, 0, 0] }] } };
  const cs = layerToCurves(layer);
  assert.deepEqual(cs.map(c => c.channel), ['rotateX', 'rotateY', 'rotateZ']);
  assert.equal(cs[0].keys[1].outType, 'linear');
  const back = curvesToLayer(layer, cs);
  assert.deepEqual(back.bones.Spine, layer.bones.Spine);
  const moved = moveKeys(cs, new Set([selKey(1, 1)]), 0, 3).curves;
  const L2 = curvesToLayer(layer, moved);
  near(L2.bones.Spine[1].e[1], 8);
  const comp = compileLayers([L2], () => 0).get(0)[0].curves;
  near(evaluate(comp[1], 10), 8);
});

test('curvesToLayer unions key times across x/y/z and drops emptied bones', () => {
  const layer = { tangent: 'auto', bones: { A: [{ f: 0, e: [0, 0, 0] }], B: [{ f: 0, e: [1, 1, 1] }] } };
  const cs = [curve([K(0, 0), K(10, 4)], 'rotateX', 'A'), curve([K(0, 0)], 'rotateY', 'A'), curve([K(0, 0)], 'rotateZ', 'A'), curve([], 'rotateX', 'B'), curve([], 'rotateY', 'B'), curve([], 'rotateZ', 'B'), curve([K(0, 9)], 'translateX', 'A')];
  const out = curvesToLayer(layer, cs);
  assert.deepEqual(out.bones.A.map(k => k.f), [0, 10]);
  near(out.bones.A[1].e[0], 4);
  assert.ok(!('B' in out.bones));
});

test('layerToCurves filters by bone list', () => {
  const layer = { bones: { A: [{ f: 0, e: [0, 0, 0] }], B: [{ f: 0, e: [0, 0, 0] }] } };
  assert.deepEqual([...new Set(layerToCurves(layer, ['B']).map(c => c.bone))], ['B']);
});

test('dragged fixed handles survive curvesToLayer, layerToCurves and compileLayers', () => {
  const layer = { tangent: 'auto', bones: { Spine: [{ f: 0, e: [0, 0, 0] }, { f: 10, e: [20, 0, 0] }, { f: 20, e: [0, 0, 0] }] } };
  const cs = setHandle(layerToCurves(layer), 0, 1, 'out', 3, 9, { unified: true });
  const want = [4, 7, 13, 16].map(t => evalCurve(cs[0], t));
  const L = curvesToLayer(layer, cs);
  assert.equal(L.bones.Spine[1].tan[0].outType, 'fixed');
  const back = layerToCurves(L);
  near(back[0].keys[1].outAngle, cs[0].keys[1].outAngle);
  const comp = compileLayers([L], () => 0).get(0)[0].curves[0];
  [4, 7, 13, 16].forEach((t, i) => near(evaluate(comp, t), want[i], 1e-6));
  assert.notEqual(Math.round(want[2] * 1e3), Math.round(evalCurve(layerToCurves(layer)[0], 13) * 1e3));
});

test('weighted handle weights survive the layer trip', () => {
  const layer = { tangent: 'auto', bones: { Spine: [{ f: 0, e: [0, 0, 0] }, { f: 10, e: [20, 0, 0] }, { f: 20, e: [0, 0, 0] }] } };
  const cs = layerToCurves(layer); cs[0] = { ...cs[0], weighted: true };
  const moved = setHandle(cs, 0, 1, 'out', 6, 0, { unified: false });
  const L = curvesToLayer(layer, moved);
  assert.deepEqual(L.weighted, ['Spine.rotateX']);
  const want = evalCurve(moved[0], 14);
  near(evaluate(compileLayers([L], () => 0).get(0)[0].curves[0], 14), want, 1e-6);
  near(layerToCurves(L)[0].keys[1].outWeight, moved[0].keys[1].outWeight);
});

test('translate curves survive the layer trip and stack additively', () => {
  const layer = { tangent: 'auto', bones: { Hips: [{ f: 0, e: [0, 0, 0] }, { f: 10, e: [0, 0, 0] }] } };
  let cs = layerToCurves(layer);
  cs = [...cs, curve([K(0, 0), K(10, 5)], 'translateY', 'Hips')];
  const L = curvesToLayer(layer, cs);
  assert.deepEqual(L.bones.Hips.map(k => k.p[1]), [0, 5]);
  assert.equal(layerToCurves(L).length, 6);
  const ent = compileLayers([L], () => 0).get(0);
  near(evalTranslation([L], ent, [1, 2, 3], 10)[1], 7);
  near(evalTranslation([{ ...L, weight: 0.5 }], ent, [1, 2, 3], 10)[1], 4.5);
  near(evalTranslation([{ ...L, override: true }], ent, [1, 2, 3], 10)[1], 5);
  const re = setLayerKey(L, 'Hips', 10, [1, 2, 3]);
  assert.deepEqual(re.bones.Hips.find(k => k.f === 10).p, [0, 5, 0]);
});
