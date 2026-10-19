import test from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { json, exists } from './helpers.mjs';
import { detectFootPlants } from '../wire/plants.js';

const UPM = 100, FPS = 30, NB = 4;
const LEGS = { l: { foot: 0, ball: 1 }, r: { foot: 2, ball: 3 } };

function rig(frames) {
  const n = frames.length, W = new Float32Array(n * NB * 16);
  frames.forEach((fr, f) => {
    for (let b = 0; b < NB; b++) {
      const o = (f * NB + b) * 16;
      W[o] = W[o + 5] = W[o + 10] = W[o + 15] = 1;
      const p = fr[b];
      W[o + 12] = p[0]; W[o + 13] = p[1]; W[o + 14] = p[2];
    }
  });
  return { W, nb: NB, n, fps: FPS, upm: UPM, ground: 0, legs: LEGS, legLength_cm: 90, footLength_cm: 26 };
}

const AIR = [[0, 60, 0], [15, 55, 0]];
const foot = (x, ankle = 10, ball = 1, z = 0) => [[x, ankle, z], [x + 15, ball, z]];
const frame = (l, r = AIR) => [...l, ...r];

function finite(res) {
  for (const p of res) for (const v of Object.values(p)) if (typeof v === 'number') assert.ok(Number.isFinite(v), JSON.stringify(p));
}
const left = res => res.filter(p => p.side === 'l');

test('stance, swing, stance gives two still plants with anchors inside', () => {
  const fr = [];
  for (let i = 0; i < 15; i++) fr.push(frame(foot(0)));
  for (let k = 1; k <= 8; k++) fr.push(frame(foot(8 * k, 10 + 25 * Math.sin(Math.PI * k / 9), 1 + 25 * Math.sin(Math.PI * k / 9))));
  for (let i = 0; i < 15; i++) fr.push(frame(foot(72)));
  const res = detectFootPlants(rig(fr));
  finite(res);
  const l = left(res);
  assert.deepEqual(l.map(p => [p.a, p.b, p.sliding, p.touch]), [[0, 14, false, false], [23, 37, false, false]]);
  for (const p of l) assert.ok(p.anchor >= p.a && p.anchor <= p.b);
  assert.equal(res.filter(p => p.side === 'r').length, 0);
});

test('a one or two frame lift inside a stance is filled, a long lift splits it', () => {
  const build = lifts => Array.from({ length: 30 }, (_, i) => frame(foot(0, 10 + (lifts[i] ?? 0), 1 + (lifts[i] ?? 0))));
  assert.deepEqual(left(detectFootPlants(rig(build({ 12: 6, 13: 6 })))).map(p => [p.a, p.b]), [[0, 29]]);
  assert.deepEqual(left(detectFootPlants(rig(build({ 12: 6 })))).map(p => [p.a, p.b]), [[0, 29]]);
  const split = left(detectFootPlants(rig(build({ 10: 6, 11: 8, 12: 8, 13: 8, 14: 6 }))));
  assert.deepEqual(split.map(p => [p.a, p.b]), [[0, 9], [15, 29]]);
});

test('runs shorter than minFrames are dropped as plants and only kept as touches', () => {
  const fr = [];
  for (let i = 0; i < 10; i++) fr.push(frame(foot(0, 40, 30)));
  for (let i = 0; i < 3; i++) fr.push(frame(foot(0)));
  for (let i = 0; i < 10; i++) fr.push(frame(foot(0, 40, 30)));
  for (let i = 0; i < 12; i++) fr.push(frame(foot(0)));
  const res = left(detectFootPlants(rig(fr)));
  assert.deepEqual(res.map(p => [p.a, p.b, p.touch]), [[10, 12, true], [23, 34, false]]);
  const strict = left(detectFootPlants({ ...rig(fr), minTouch: 0 }));
  assert.deepEqual(strict.map(p => [p.a, p.b]), [[23, 34]]);
  assert.deepEqual(left(detectFootPlants({ ...rig(fr), minTouch: 0, minFrames: 3 })).map(p => [p.a, p.b]), [[10, 12], [23, 34]]);
});

test('a foot on the floor moving 1 m/s is a sliding plant, a slow creep is not', () => {
  const slide = Array.from({ length: 30 }, (_, i) => frame(foot(i * 100 / FPS)));
  const s = left(detectFootPlants(rig(slide)));
  assert.equal(s.length, 1);
  assert.deepEqual([s[0].a, s[0].b, s[0].sliding], [0, 29, true]);
  assert.ok(s[0].drift_cm > 13 && Math.abs(s[0].speed - 1) < 1e-3);
  const creep = Array.from({ length: 90 }, (_, i) => frame(foot(i * 20 / FPS)));
  const c = left(detectFootPlants(rig(creep)));
  assert.deepEqual(c.map(p => [p.a, p.b, p.sliding]), [[0, 89, false]]);
  assert.ok(c[0].drift_cm > 13);
});

test('fast push-off frames at the end of a stance are trimmed off the plant', () => {
  const fr = [];
  for (let i = 0; i < 20; i++) fr.push(frame(foot(0)));
  for (let k = 1; k <= 4; k++) fr.push(frame(foot(4 * k)));
  for (let k = 1; k <= 10; k++) fr.push(frame(foot(16 + 10 * k, 40, 30)));
  const l = left(detectFootPlants(rig(fr)));
  assert.equal(l.length, 1);
  assert.equal(l[0].a, 0);
  assert.ok(l[0].b >= 19 && l[0].b <= 20, `b ${l[0].b}`);
  assert.equal(l[0].sliding, false);
});

test('forefoot stance counts: ball on the floor with the heel 17 cm up', () => {
  const fr = [];
  for (let i = 0; i < 20; i++) fr.push(frame(foot(0)));
  for (let k = 1; k <= 8; k++) fr.push(frame(foot(5 * k, 40, 30)));
  for (let i = 0; i < 12; i++) fr.push(frame([[40, 27, 0], [52, 1, 0]]));
  const l = left(detectFootPlants(rig(fr)));
  assert.deepEqual(l.map(p => [p.a, p.b, p.point, p.sliding]), [[0, 19, 'ball', false], [28, 39, 'ball', false]]);
});

test('nothing to fix: feet always in the air give no plants, bad input gives []', () => {
  const fr = Array.from({ length: 40 }, (_, i) => frame([[i * 3, 60, 0], [i * 3 + 15, 55, 0]], [[i * 3, 70, 30], [i * 3 + 15, 65, 30]]));
  const air = rig(fr);
  assert.deepEqual(detectFootPlants(air), []);
  assert.deepEqual(detectFootPlants({ ...air, ground: -50 }), []);
  assert.deepEqual(detectFootPlants({ ...rig([]), n: 0 }), []);
  assert.deepEqual(detectFootPlants({ ...air, legs: {} }), []);
  assert.deepEqual(detectFootPlants({ ...air, fps: 0 }), []);
});

test('missing ball bone falls back to the ankle', () => {
  const fr = Array.from({ length: 20 }, () => frame(foot(0)));
  const res = detectFootPlants({ ...rig(fr), legs: { l: { foot: 0 } } });
  assert.deepEqual(res.map(p => [p.side, p.a, p.b, p.point]), [['l', 0, 19, 'foot']]);
});

const REAL = fileURLToPath(new URL('../exports/FRT_fulcrum_v6.frames.json', import.meta.url));
test('FRT_fulcrum_v6: long stand/kneel/stand plants, short alternating sprint contacts', { skip: !exists(REAL) }, () => {
  const d = json(REAL);
  const nb = d.bones.length, n = d.frames.length, W = new Float32Array(n * nb * 16);
  d.frames.forEach((F, f) => W.set(F, f * nb * 16));
  const I = k => d.bones.indexOf(k);
  const ys = [];
  for (let f = 0; f < n; f++) for (const s of 'lr') ys.push(Math.min(W[(f * nb + I('foot_' + s)) * 16 + 13], W[(f * nb + I('ball_' + s)) * 16 + 13]));
  ys.sort((a, b) => a - b);
  const upm = 91.33, ground = ys[Math.floor(ys.length * 0.03)];
  const res = detectFootPlants({ W, nb, n, fps: d.fps, upm, ground, legs: { l: { foot: I('foot_l'), ball: I('ball_l') }, r: { foot: I('foot_r'), ball: I('ball_r') } }, legLength_cm: 90, footLength_cm: 26 });
  finite(res);
  for (const p of res) console.log(`${p.side} f${p.a}-${p.b} (${p.b - p.a + 1}) ${p.touch ? 'touch' : 'plant'} ${p.sliding ? 'sliding' : 'planted'} anchor ${p.anchor} ${p.point} drift ${p.drift_cm.toFixed(1)}cm ${p.speed.toFixed(2)}m/s`);
  const still = res.filter(p => !p.sliding);
  const covers = (side, a, b) => still.some(p => p.side === side && p.a <= a && p.b >= b);
  for (const s of 'lr') {
    assert.ok(covers(s, 0, 50), `${s} stand f0-50`);
    assert.ok(covers(s, 297, 344), `${s} final stand`);
    assert.ok(covers(s, 180, 200), `${s} kneel`);
  }
  for (const p of still) {
    for (const k of [p.a, p.anchor, p.b]) {
      const ha = (W[(k * nb + I('foot_' + p.side)) * 16 + 13] - ground) / upm, hb = (W[(k * nb + I('ball_' + p.side)) * 16 + 13] - ground) / upm;
      assert.ok(ha < 0.16 || hb < 0.05, `${p.side} f${k} ankle ${ha} ball ${hb}`);
    }
    assert.ok(p.speed < 0.6, `${p.side} f${p.a} speed ${p.speed}`);
  }
  const sprint = res.filter(p => p.a > 60 && p.b < 290 && (p.a < 140 || p.a > 206));
  assert.ok(sprint.length >= 10);
  assert.ok(sprint.every(p => p.b - p.a + 1 <= 12));
  let alternations = 0;
  for (let i = 1; i < sprint.length; i++) if (sprint[i].side !== sprint[i - 1].side) alternations++;
  assert.ok(alternations >= sprint.length / 2, `alternations ${alternations}/${sprint.length}`);
  for (const p of res) for (const q of res) if (p !== q && p.side === q.side) assert.ok(p.b < q.a || q.b < p.a, 'no overlap per foot');
});
