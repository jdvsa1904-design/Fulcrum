import test from 'node:test';
import assert from 'node:assert/strict';
import { BONES, segmentPhases } from '../phases.js';

const FPS = 30, THIGH = 50, SHIN = 42;

function rest() {
  return frame({});
}

function legPose(hip, ankle) {
  const d = [0, 1, 2].map(k => ankle[k] - hip[k]), len = Math.hypot(...d);
  const reach = Math.min(len, THIGH + SHIN - 1e-6);
  const a = (THIGH * THIGH - SHIN * SHIN + reach * reach) / (2 * reach), h = Math.sqrt(Math.max(0, THIGH * THIGH - a * a));
  const u = d.map(v => v / len), fwd = [1, 0, 0], dot = fwd[0] * u[0] + fwd[1] * u[1] + fwd[2] * u[2];
  let p = fwd.map((v, k) => v - dot * u[k]);
  const pl = Math.hypot(...p) || 1;
  p = p.map(v => v / pl);
  return [0, 1, 2].map(k => hip[k] + u[k] * a + p[k] * h);
}

function frame({ x = 0, y = 0, z = 100, chest = [0, 0, 45], hand = null, lf = null, rf = null }) {
  const P = [x, y, z], r = BONES.map(() => [x, y, z]);
  r[0] = P;
  r[1] = P.map((v, k) => v + chest[k] / 3);
  r[2] = P.map((v, k) => v + chest[k] * 2 / 3);
  r[3] = P.map((v, k) => v + chest[k]);
  r[4] = r[3].map((v, k) => v + chest[k] / 5);
  r[5] = r[3].map((v, k) => v + chest[k] / 3);
  const hands = hand || [[x, y + 25, z + 5], [x, y - 25, z + 5]];
  for (const [s, c, ua, la, hd] of [[0, 6, 7, 8, 9], [1, 10, 11, 12, 13]]) {
    const sh = r[3].map((v, k) => v + [0, s ? -18 : 18, 0][k]);
    r[c] = sh; r[ua] = sh; r[hd] = hands[s];
    r[la] = sh.map((v, k) => (v + hands[s][k]) / 2);
  }
  const feet = [lf || [x, y + 10, 8], rf || [x, y - 10, 8]];
  for (const [s, th, ca, fo, ba] of [[0, 14, 15, 16, 17], [1, 18, 19, 20, 21]]) {
    const hip = [x, y + (s ? -10 : 10), z];
    r[th] = hip; r[fo] = feet[s]; r[ca] = legPose(hip, feet[s]);
    r[ba] = [feet[s][0] + 15, feet[s][1], feet[s][2] - 7];
  }
  return r;
}

const clip = B => ({ fps: FPS, B, rest: rest() });
const phasesOf = d => segmentPhases(d).map(s => s.phase);
const dominant = d => {
  const c = {};
  for (const s of segmentPhases(d)) c[s.phase] = (c[s.phase] || 0) + s.end - s.start + 1;
  return Object.entries(c).sort((a, b) => b[1] - a[1])[0][0];
};

function walk(n, { z = 100, speed = 120, stepHz = 1.6, stride = 25 } = {}) {
  const B = [];
  for (let f = 0; f < n; f++) {
    const t = f / FPS, x = speed * t, s = Math.sin(2 * Math.PI * stepHz * t);
    B.push(frame({ x, z, lf: [x + stride * s, 10, 8 + Math.max(0, 12 * s)], rf: [x - stride * s, -10, 8 + Math.max(0, -12 * s)] }));
  }
  return B;
}

test('a level walk stays a walk phase', () => {
  assert.equal(dominant(clip(walk(120))), 'locomotion');
});

test('prone crawl: pelvis low and torso horizontal is a prone phase', () => {
  const B = [];
  for (let f = 0; f < 90; f++) {
    const x = 30 * f / FPS, s = Math.sin(2 * Math.PI * f / FPS);
    B.push(frame({ x, z: 18, chest: [45, 0, 3], hand: [[x + 50 + 10 * s, 20, 2], [x + 50 - 10 * s, -20, 2]], lf: [x - 80 + 8 * s, 12, 6], rf: [x - 80 - 8 * s, -12, 6] }));
  }
  const d = clip(B);
  assert.equal(dominant(d), 'prone', phasesOf(d).join(' > '));
});

test('crouched walk with bent knees is a crouch phase, not a walk', () => {
  const d = clip(walk(120, { z: 85, speed: 70, stride: 8 }));
  assert.equal(dominant(d), 'crouch', phasesOf(d).join(' > '));
});

test('kneel is still detected with one knee on the floor', () => {
  const B = [];
  for (let f = 0; f < 30; f++) B.push(frame({}));
  for (let f = 0; f < 60; f++) {
    const r = frame({ z: 50, lf: [30, 10, 8], rf: [-40, -10, 6] });
    r[19] = [-5, -10, 4];
    B.push(r);
  }
  const d = clip(B);
  assert.ok(phasesOf(d).includes('kneel'), phasesOf(d).join(' > '));
});

test('standing with feet planted and arms busy is an upper-body phase', () => {
  const B = [];
  for (let f = 0; f < 90; f++) {
    const a = 2 * Math.PI * 1.5 * f / FPS;
    B.push(frame({ hand: [[30 + 25 * Math.cos(a), 25, 130 + 25 * Math.sin(a)], [30 + 25 * Math.sin(a), -25, 130 + 25 * Math.cos(a)]] }));
  }
  const d = clip(B);
  assert.deepEqual(phasesOf(d), ['upper_body']);
});

test('fast irregular footwork without travel is a freeform phase', () => {
  let seed = 7;
  const rnd = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
  const tgt = [[0, 10, 8], [0, -10, 8]], cur = [[0, 10, 8], [0, -10, 8]], B = [];
  for (let f = 0; f < 150; f++) {
    if (f % 5 === 0) for (const s of [0, 1]) tgt[s] = [(rnd() - .5) * 80, (s ? -10 : 10) + (rnd() - .5) * 40, 8 + rnd() * 25];
    for (const s of [0, 1]) cur[s] = cur[s].map((v, k) => v + (tgt[s][k] - v) * .5);
    const x = 10 * Math.sin(f / 9), y = 10 * Math.cos(f / 13);
    B.push(frame({ x, y, lf: [x + cur[0][0], y + cur[0][1], cur[0][2]], rf: [x + cur[1][0], y + cur[1][1], cur[1][2]] }));
  }
  const d = clip(B);
  assert.equal(dominant(d), 'freeform', phasesOf(d).join(' > '));
});

test('feet off the floor with a hand planted low is a vault', () => {
  const B = walk(40);
  const x0 = B[B.length - 1][0][0];
  for (let f = 0; f < 14; f++) {
    const x = x0 + 8 * f;
    B.push(frame({ x, lf: [x + 10, 10, 45], rf: [x - 10, -10, 45], hand: [[x0 + 60, 20, 55], [x + 10, -25, 110]] }));
  }
  for (const r of walk(40)) B.push(r.map(p => [p[0] + x0 + 112, p[1], p[2]]));
  const d = clip(B);
  assert.ok(phasesOf(d).includes('vault'), phasesOf(d).join(' > '));
});
