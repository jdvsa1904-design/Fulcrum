import test from 'node:test';
import assert from 'node:assert/strict';
import { json, near } from './helpers.mjs';
import {
  layerWeights, rotationAccumulation, blendAdditive, blendMultiply, blendScale, blendBoolean, blendRotation,
  eulerToQuat, quatToEuler, evaluateLayerStack,
} from '../re/maya/layers.js';

const fx = json(new URL('./fixtures/maya_layers.json', import.meta.url));
const ACC = ['component', 'layerCumulative', 'layerBlended'];

test('layer foreground/background weights match Maya for every layer flag combination', () => {
  for (const r of fx.layerWeights) {
    const w = layerWeights({ override: r.override, passthrough: r.passthrough, mute: r.mute, weight: r.weight });
    near(w.fg, r.fg, 1e-12, JSON.stringify(r));
    near(w.bg, r.bg, 1e-12, JSON.stringify(r));
    const mode = rotationAccumulation({ override: r.override, rotationAccumulationMode: r.rotAcc ? 'layer' : 'component' });
    assert.equal(mode, ACC[r.outRotAcc]);
  }
});

test('nested layers multiply foreground by the parent weight; override background ignores it', () => {
  for (const r of fx.nested) {
    const w = layerWeights({ override: r.childOverride, passthrough: true, weight: r.childWeight, parentWeight: r.parentWeight });
    near(w.fg, r.child.fg, 1e-12);
    near(w.bg, r.child.bg, 1e-12);
  }
});

test('scalar, scale and boolean blend nodes match Maya', () => {
  for (const r of fx.nodes) {
    if (r.type === 'DL') near(r.mode ? blendMultiply(r.a, r.b, r.wa, r.wb) : blendAdditive(r.a, r.b, r.wa, r.wb), r.out, 1e-9 * Math.max(1, Math.abs(r.out)));
    if (r.type === 'Scale') near(blendScale(r.a, r.b, r.wa, r.wb, r.mode ? 'multiply' : 'additive'), r.out, 1e-9 * Math.max(1, Math.abs(r.out)));
    if (r.type === 'Boolean') assert.equal(blendBoolean(r.a, r.b, r.wa, r.wb), r.out);
  }
});

test('rotation blend node matches Maya in all three accumulation modes, all rotate orders', () => {
  let n = 0;
  for (const r of fx.nodes.filter(r => r.type === 'Rotation')) {
    const out = blendRotation(r.a, r.b, r.wa, r.wb, { mode: ACC[r.mode], rotateOrder: r.rotateOrder });
    for (let i = 0; i < 3; i++) near(out[i], r.out[i], 5e-5, `mode ${r.mode} ro ${r.rotateOrder} wa ${r.wa} wb ${r.wb}`);
    n++;
  }
  assert.equal(n, 600);
});

test('rotationInterpolation has no effect: by-component is always Euler, by-layer always quaternion', () => {
  const byInterp = fx.nodes.filter(r => r.type === 'Rotation' && r.mode === 0 && r.interp === 1);
  assert.ok(byInterp.length > 50);
  for (const r of byInterp) for (let i = 0; i < 3; i++) near(r.out[i], r.wa * r.a[i] + r.wb * r.b[i], 1e-9);
});

test('full layer stacks on a joint match Maya (translate, rotate, scale)', () => {
  for (const s of fx.stacks) {
    const L = s.layers.map(l => ({ override: l.override, passthrough: l.passthrough, weight: l.weight, mute: l.mute, rotationAccumulationMode: l.rotAcc ? 'layer' : 'component', scaleAccumulationMode: l.scaleAcc ? 'multiply' : 'additive' }));
    near(evaluateLayerStack(s.base.t, L.map((l, i) => ({ ...l, value: s.layers[i].value.t }))), s.out.t, 1e-9);
    near(evaluateLayerStack(s.base.s, L.map((l, i) => ({ ...l, value: s.layers[i].value.s })), 'scale'), s.out.s, 1e-9);
    const r = evaluateLayerStack(s.base.r, L.map((l, i) => ({ ...l, value: s.layers[i].value.r })), 'rotation', { rotateOrder: s.rotateOrder });
    for (let i = 0; i < 3; i++) near(r[i], s.out.r[i], 1e-8);
  }
});

test('quatToEuler round-trips orientation in every rotate order', () => {
  for (let ro = 0; ro < 6; ro++) for (const e of [[10, 20, 30], [170, -80, 45], [-120, 95, 200], [0, 90, 0]]) {
    const q = eulerToQuat(e, ro), q2 = eulerToQuat(quatToEuler(q, ro), ro);
    near(Math.abs(q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3]), 1, 1e-12);
  }
});
