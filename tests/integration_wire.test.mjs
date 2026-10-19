import test from 'node:test';
import assert from 'node:assert/strict';
import { near, nearVec, q4, deg } from './helpers.mjs';
import { planFootsteps, footstepContacts, midStanceOffset, balanceShift } from '../wire/footsteps.js';
import { gaitTiming, MAX } from '../solvers/biped/footsteps.js';
import { compileLayers, evalRotation, solveLayerKey, setLayerKey, normalizeLayer, unwrapEuler, layerIncludesBone } from '../wire/animlayers.js';
import { makeCurve, evaluate } from '../solvers/maya/curves.js';
import { eulerToQuat } from '../solvers/maya/layers.js';
import { applyStoryBlends, storyWeight, storyWindow, cutIndex, insertIndex } from '../wire/story.js';
import { floorTargets, soleMarkerRig, soleMarkers } from '../wire/floor.js';

function inPlaceRun({ n = 120, fps = 24, period = 16, S = 0.87, ground = 0 }) {
  const ball = { l: new Float64Array(n * 3), r: new Float64Array(n * 3) };
  for (let f = 0; f < n; f++) for (const [side, ph] of [['l', 0], ['r', 0.5]]) {
    const u = ((f / period + ph) % 1 + 1) % 1, h = ground + 0.12 * S * (1 - Math.cos(2 * Math.PI * u)) / 2;
    ball[side][f * 3] = (side === 'l' ? -1 : 1) * 0.1; ball[side][f * 3 + 1] = h; ball[side][f * 3 + 2] = 0.25 * Math.sin(2 * Math.PI * u);
  }
  return ball;
}

test('Biped footsteps: contacts sit on the capture footfalls and last 0.6 leg / speed (Max run timing)', () => {
  const fps = 24, S = 0.87, speed = 4, ball = inPlaceRun({ fps, S });
  const r = footstepContacts({ ball, a: 10, b: 110, fps, S, speed });
  assert.equal(r.cycle, 16);
  const T = gaitTiming('run', speed * 8 / fps, S, 8, fps);
  assert.equal(r.contactFrames, Math.max(2, Math.round(T.contact)));
  near(r.contactFrames, 0.6 * S / speed * fps, 1, 'contact frames');
  for (const c of r.contacts) { assert.ok(c.b - c.a + 1 <= r.contactFrames); assert.ok(c.m >= 10 && c.m <= 110); }
  const sides = r.contacts.map(c => c.side).join('');
  assert.ok(!/ll|rr/.test(sides), 'feet alternate ' + sides);
});

test('Biped footsteps: footprints are 0.1 S either side and m = 0.9595 v + 0.0107 S behind the COM at mid-stance', () => {
  const fps = 24, S = 0.87, speed = 4, v = speed / fps, ball = inPlaceRun({ fps, S });
  const com = f => [0.02 * Math.sin(f), 0.95, v * f], fwd = () => [0, 0, 1], right = () => [-1, 0, 0];
  const plan = planFootsteps({ ball, a: 10, b: 110, fps, S, speed, com, fwd, right, vPerFrame: v, ground: 0, lift: 0.01 });
  const m = midStanceOffset(v, S, fps);
  near(m, MAX.midOffsetTime * v * 30 / fps + MAX.midOffsetConst * S, 1e-12);
  for (const k of plan.footprints) {
    const c = com(k.m);
    near(c[2] - k.pos[2], m, 1e-9, 'behind COM');
    near(k.pos[0] - c[0], k.side === 'l' ? 0.1 * S : -0.1 * S, 1e-9, 'half width');
    near(k.pos[1], 0.01, 1e-12);
  }
  const L = plan.footprints.filter(k => k.side === 'l');
  for (let i = 1; i < L.length; i++) near(L[i].pos[2] - L[i - 1].pos[2], v * (L[i].m - L[i - 1].m), 1e-9, 'stride follows speed');
});

test('Biped balance shift is the measured -0.794 bf horizontal(spine_04 - spine_02)', () => {
  nearVec(balanceShift([0, 1, 0], [0.1, 1.3, -0.05], 1), [-0.0794, 0, 0.0397], 1e-12);
  nearVec(balanceShift([0, 1, 0], [0.1, 1.3, -0.05], 0), [0, 0, 0], 1e-12);
});

const BASE = q4.axis([0.3, 1, 0.2], 0.7);
const idx = name => ({ spine_01: 0, head: 1 })[name];

test('animation layer masks include explicit joints and named regions while preserving unrestricted default', () => {
  assert.equal(layerIncludesBone(normalizeLayer({}), 'hand_l', 4), true);
  assert.equal(layerIncludesBone(normalizeLayer({ mask: ['hand_l'] }), 'hand_l', 4), true);
  assert.equal(layerIncludesBone(normalizeLayer({ mask: { regions: ['upperbody'] } }), 'spine_01', 0), true);
  assert.equal(layerIncludesBone(normalizeLayer({ mask: { regions: ['upperbody'] } }), 'thigh_l', 8), false);
  assert.equal(layerIncludesBone(normalizeLayer({ mask: { hierarchy: { root: 0, depth: 1 } } }), 'hand_l', 4, [-1, 0, 1, 2, 3]), false);
});

test('Maya layers: an additive key solved for a target reproduces it exactly, by layer and by component', () => {
  const target = q4.mul(q4.axis([1, 0, 0], 0.4), BASE);
  for (const accumulation of ['layer', 'component']) {
    let L = normalizeLayer({ name: 'A', accumulation });
    const e = solveLayerKey([L], [], 0, BASE, target, 10);
    L = setLayerKey(L, 'spine_01', 10, e);
    const comp = compileLayers([L], idx);
    const out = evalRotation([L], comp.get(0), BASE, 10);
    assert.ok(deg(q4.angle(out, target)) < 1e-4, accumulation + ' ' + deg(q4.angle(out, target)));
  }
});

test('Maya layers: weight 0 and mute give the base pose; override at weight 1 gives the key', () => {
  const key = q4.axis([0, 0, 1], 0.9);
  let L = setLayerKey(normalizeLayer({ name: 'O', override: true }), 'spine_01', 5, solveLayerKey([normalizeLayer({ override: true })], [], 0, BASE, key, 5));
  let out = evalRotation([L], compileLayers([L], idx).get(0), BASE, 5);
  assert.ok(deg(q4.angle(out, key)) < 1e-4);
  for (const patch of [{ weight: 0 }, { mute: true }]) {
    const M = { ...L, ...patch };
    out = evalRotation([M], compileLayers([M], idx).get(0), BASE, 5);
    assert.ok(deg(q4.angle(out, BASE)) < 1e-4, JSON.stringify(patch));
  }
});

test('Maya layers: between keys the channels follow Maya tangent curves (auto never overshoots, step holds)', () => {
  const keys = [{ f: 0, e: [0, 0, 0] }, { f: 10, e: [30, 0, 0] }, { f: 20, e: [30, -10, 0] }];
  for (const tangent of ['auto', 'step', 'linear', 'spline']) {
    const L = normalizeLayer({ name: 'T', tangent, bones: { spine_01: keys } });
    const comp = compileLayers([L], idx, 24).get(0);
    const ref = makeCurve(keys.map(k => ({ t: k.f, v: k.e[0], inType: tangent, outType: tangent })), { fps: 24 });
    for (let f = 0; f <= 20; f++) near(comp[0].curves[0] && evaluate(comp[0].curves[0], f), evaluate(ref, f), 1e-12);
    if (tangent === 'auto') for (let f = 10; f <= 20; f++) near(evaluate(comp[0].curves[0], f), 30, 1e-9);
    if (tangent === 'step') near(evaluate(comp[0].curves[0], 9), 0, 1e-12);
  }
  nearVec(unwrapEuler([179, 0, -179], [-179, 0, 179]), [-181, 0, 181], 1e-12);
});

test('Maya layers stack in order: two additive layers compose, a later override replaces them', () => {
  const a = q4.axis([1, 0, 0], 0.3), b = q4.axis([0, 1, 0], 0.2);
  let A = normalizeLayer({ name: 'A' }), B = normalizeLayer({ name: 'B' });
  A = setLayerKey(A, 'spine_01', 0, solveLayerKey([A], [], 0, BASE, q4.mul(BASE, a), 0));
  const layers = [A, B];
  B = setLayerKey(B, 'spine_01', 0, solveLayerKey(layers, compileLayers(layers, idx).get(0), 1, BASE, q4.mul(q4.mul(BASE, a), b), 0));
  const both = [A, B];
  assert.ok(deg(q4.angle(evalRotation(both, compileLayers(both, idx).get(0), BASE, 0), q4.mul(q4.mul(BASE, a), b))) < 1e-4);
  const O = setLayerKey(normalizeLayer({ name: 'O', override: true }), 'spine_01', 0, [0, 0, 0]);
  const all = [A, B, O];
  assert.ok(deg(q4.angle(evalRotation(all, compileLayers(all, idx).get(0), BASE, 0), [0, 0, 0, 1])) < 1e-4);
});

function takeOf(n, nb, fn) {
  const q = new Float32Array(n * nb * 4), p = new Float32Array(n * nb * 3);
  for (let f = 0; f < n; f++) for (let i = 0; i < nb; i++) { const r = fn(f, i); q.set(r.q, (f * nb + i) * 4); p.set(r.p, (f * nb + i) * 3); }
  return { q, p, n };
}

test('Story cut: the outgoing clip plays on into the cut frames and crossfades into the incoming one', () => {
  const nb = 2, n = 100, src = takeOf(n, nb, (f, i) => ({ q: q4.axis([0, 0, 1], i ? 0.02 * f : 0), p: i ? [0, 0, 0] : [0, 1, 0.05 * f] }));
  const a = 40, b = 59, k = b - a + 1;
  const srcIdx = cutIndex(Int32Array.from({ length: n }, (_, f) => f), a, b);
  const q = new Float32Array((n - k) * nb * 4), p = new Float32Array((n - k) * nb * 3);
  for (let f = 0; f < n - k; f++) { q.set(src.q.subarray(srcIdx[f] * nb * 4, (srcIdx[f] + 1) * nb * 4), f * nb * 4); p.set(src.p.subarray(srcIdx[f] * nb * 3, (srcIdx[f] + 1) * nb * 3), f * nb * 3); }
  const fx = { type: 'story', at: a, pre: 6, post: 6, curve: 'ease' };
  const rep = applyStoryBlends({ q, p, nb, n: n - k, src, srcIdx, fixes: [fx], root: 0, up: [0, 1, 0] });
  assert.equal(rep[0].outgoingFrom, a);
  assert.equal(rep[0].incomingFrom, b + 1);
  const { s, e } = storyWindow(fx, n - k);
  assert.equal(storyWeight(fx, s, n - k), 0);
  near(storyWeight(fx, e + 1, n - k), 1, 1e-12);
  for (let f = 1; f < n - k; f++) {
    const dz = p[(f * nb) * 3 + 2] - p[((f - 1) * nb) * 3 + 2];
    near(dz, 0.05, 1e-5, 'pelvis keeps its speed at f' + f);
    const d = deg(q4.angle(Array.from(q.subarray((f * nb + 1) * 4, (f * nb + 1) * 4 + 4)), Array.from(q.subarray(((f - 1) * nb + 1) * 4, ((f - 1) * nb + 1) * 4 + 4))));
    assert.ok(d < 1.15 * deg(0.02) * 4, `joint step ${d.toFixed(2)} deg at f${f}`);
  }
  near(Math.hypot(...rep[0].matchOffset), 0.05 * k, 1e-5);
});

test('Story index maps follow cuts and inserts', () => {
  const id = Int32Array.from({ length: 10 }, (_, f) => f);
  assert.deepEqual([...cutIndex(id, 3, 5)], [0, 1, 2, 6, 7, 8, 9]);
  assert.deepEqual([...insertIndex(id, 8, 1, 2)], [0, 1, 2, 3, 4, 5, 6, 7, 1, 2, 8, 9]);
});

function footTake(n) {
  const nb = 2, W = new Float32Array(n * nb * 16);
  for (let f = 0; f < n; f++) {
    const y = f < n / 2 ? 0.075 : 0.06;
    const foot = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, y, 0, 1], ball = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0.02, 0.13, 1];
    W.set(foot, (f * nb) * 16); W.set(ball, (f * nb + 1) * 16);
  }
  return { W, nb };
}

test('HumanIK contact markers: contact size scales the drawn markers only, never the floor solve', () => {
  const n = 20, { W, nb } = footTake(n), legs = { l: { foot: 0, ball: 1 } };
  const base = floorTargets({ W, nb, n, upm: 1, ground: 0, legs });
  const rig = soleMarkerRig({ W, nb, n, upm: 1, ground: 0, legs });
  const m1 = soleMarkers(rig, W, nb, 15, 100).l, m2 = soleMarkers(rig, W, nb, 15, 200).l;
  const ank = [0, 0.06, 0];
  for (const k of Object.keys(m1)) nearVec(m2[k].map((v, i) => v - ank[i]), m1[k].map((v, i) => 2 * (v - ank[i])), 1e-6, k);
  const again = floorTargets({ W, nb, n, upm: 1, ground: 0, legs, markers: null });
  assert.deepEqual([...again.keys()], [...base.keys()]);
  for (const [f, ts] of base) nearVec(again.get(f)[0].ankleP, ts[0].ankleP, 1e-12);
});
