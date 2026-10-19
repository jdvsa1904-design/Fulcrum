import { fileURLToPath } from 'node:url';
import test from 'node:test';
import assert from 'node:assert/strict';
import { json, exists, near, v3, q4, deg } from './helpers.mjs';
import { floorTargets, kneeFloorTargets } from '../wire/floor.js';
import { footMarkers } from '../contacts.js';

const UPM = 91.33, U = UPM / 100, NB = 4, LEGS = { l: { thigh: 0, calf: 1, foot: 2, ball: 3 } };
const BOTTOM = 8 * U, MID = 15 * U, BALL_Y = 1 * U;
const OFFSET = q4.mul(q4.axis([0, 0, 1], Math.PI / 2), q4.axis([1, 0, 0], 0.3));

function mat(p, q, s = 100) {
  const c = [q4.rot(q, [1, 0, 0]), q4.rot(q, [0, 1, 0]), q4.rot(q, [0, 0, 1])];
  return [...c[0].map(x => x * s), 0, ...c[1].map(x => x * s), 0, ...c[2].map(x => x * s), 0, ...p, 1];
}

function footPose(ankleP, H) {
  const boneQ = q4.mul(H, q4.conj(OFFSET));
  const ballP = v3.add(ankleP, q4.rot(H, [0, -(BOTTOM - BALL_Y), MID]));
  const ballQ = q4.mul(H, q4.axis([0, 1, 0], 0.2));
  const hip = v3.add(ankleP, [0, 80 * U, -5 * U]), knee = v3.add(ankleP, [0, 40 * U, 5 * U]);
  return [mat(hip, [0, 0, 0, 1]), mat(knee, [0, 0, 0, 1]), mat(ankleP, boneQ), mat(ballP, ballQ)];
}

function build(poses) {
  const W = new Float32Array(poses.length * NB * 16);
  poses.forEach((p, f) => p.forEach((m, i) => W.set(m, (f * NB + i) * 16)));
  return W;
}

const yaw = a => q4.axis([0, 1, 0], a), pitch = a => q4.axis([1, 0, 0], a);
const REST = footPose([3, BOTTOM, 2], yaw(0));
const minY = mk => Math.min(...Object.values(mk).map(p => p[1]));
const finite = m => { for (const [, ts] of m) for (const t of ts) assert.ok([...t.ankleP, ...t.ankleQ, ...(t.ballQ || []), t.lift_cm].every(Number.isFinite)); };

test('calibration from a flat rest pose recovers the bone-to-ankle-frame offset and the rig foot size', () => {
  const W = build([footPose([0, BOTTOM + 20 * U, 0], yaw(0.4))]);
  const r = floorTargets({ W, nb: NB, n: 1, upm: UPM, ground: 0, legs: LEGS, rest: REST.map(m => Float32Array.from(m)) });
  assert.equal(r.size, 0);
  const cal = r.rig.l;
  assert.ok(deg(q4.angle(cal.offsetQ, OFFSET)) < 0.01, `offset ${deg(q4.angle(cal.offsetQ, OFFSET))}`);
  near(cal.markers.bottom, BOTTOM, 1e-4);
  near(cal.markers.middle, MID, 1e-4);
  near(cal.markers.front, MID * 7 / 13, 1e-4);
});

test('a flat foot pushed 3 cm below the floor comes back to touch it', () => {
  for (const [stiffness, pivot] of [[0, 'ankle'], [1, 'ankle'], [0.5, 'auto'], [0, 'toes']]) {
    const W = build([footPose([5, BOTTOM - 3 * U, 7], yaw(1.1))]);
    const r = floorTargets({ W, nb: NB, n: 1, upm: UPM, ground: 0, legs: LEGS, rest: REST, stiffness, pivot });
    assert.equal(r.size, 1);
    const t = r.get(0)[0];
    near(minY(t.markers), 0, 0.05 * U, `stiffness ${stiffness} min marker`);
    near(t.lift_cm, 3, 0.05, `lift ${stiffness}`);
    near(t.pen_cm, 3, 0.01);
    if (pivot === 'ankle') { near(t.ankleP[0], 5, 1e-9); near(t.ankleP[2], 7, 1e-9); }
    finite(r);
  }
});

test('a heel-down tilted foot is rotated flat with every marker at or above the floor', () => {
  const H = q4.mul(yaw(-0.7), pitch(-0.25));
  const W = build([footPose([0, BOTTOM - 3 * U, 0], H)]);
  const r = floorTargets({ W, nb: NB, n: 1, upm: UPM, ground: 0, legs: LEGS, rest: REST });
  const t = r.get(0)[0];
  near(minY(t.markers), 0, 0.05 * U);
  const W2 = build([[...footPose([0, 0, 0], H).slice(0, 2), mat(t.ankleP, t.ankleQ), footPose([0, 0, 0], H)[3]]]);
  const again = floorTargets({ W: W2, nb: NB, n: 1, upm: UPM, ground: 0, legs: LEGS, rest: REST, tol_cm: 0.05 });
  assert.equal(again.size, 0, 'applying the target clears the penetration');
});

test('a toe-down foot bends the ball joint so the front markers sit on the floor', () => {
  const H = pitch(0.5), ankleP = [0, BOTTOM + 3 * U, 0];
  const W = build([footPose(ankleP, H)]);
  const r = floorTargets({ W, nb: NB, n: 1, upm: UPM, ground: 0, legs: LEGS, rest: REST });
  const t = r.get(0)[0];
  assert.ok(t.ballQ, 'ballQ present when the toe bends');
  near(minY(t.markers), 0, 0.05 * U);
  near(Math.min(t.markers.frontIn[1], t.markers.frontOut[1]), 0, 0.05 * U);
});

test('a foot above the floor and a foot resting on it are untouched', () => {
  const W = build([footPose([0, BOTTOM + 10 * U, 0], yaw(0.3)), footPose([2, BOTTOM, 1], yaw(-2)), footPose([0, BOTTOM + 2 * U, 0], pitch(0.05))]);
  const r = floorTargets({ W, nb: NB, n: 3, upm: UPM, ground: 0, legs: LEGS, rest: REST });
  assert.equal(r.size, 0);
});

test('frame range limits the search and auto-calibration matches rest calibration', () => {
  const poses = [];
  for (let f = 0; f < 60; f++) poses.push(footPose([f * U, f % 20 === 5 ? BOTTOM - 2 * U : BOTTOM + (f % 10 < 5 ? 0 : 15 * U), 0], yaw(f * 0.05)));
  const W = build(poses);
  const all = floorTargets({ W, nb: NB, n: 60, upm: UPM, ground: 0, legs: LEGS });
  assert.equal(all.rig.l.source, 'take');
  assert.deepEqual([...all.keys()], [5, 25, 45]);
  assert.ok(deg(q4.angle(all.rig.l.offsetQ, OFFSET)) < 0.01);
  near(all.rig.l.markers.bottom, BOTTOM, 1e-3);
  const some = floorTargets({ W, nb: NB, n: 60, upm: UPM, ground: 0, legs: LEGS, frames: [10, 30] });
  assert.deepEqual([...some.keys()], [25]);
  for (const [, ts] of all) near(ts[0].lift_cm, 2, 0.05);
  finite(all);
});

test('kneeFloorTargets lifts a knee that sinks into the floor', () => {
  const W = build([footPose([0, BOTTOM, 0], yaw(0))]);
  W[1 * 16 + 13] = 2 * U;
  const r = kneeFloorTargets({ W, nb: NB, n: 1, upm: UPM, ground: 0, knees: { l: 1 } });
  near(r.get(0)[0].lift_cm, 4, 1e-4);
  near(r.get(0)[0].kneeP[1], 6 * U, 1e-4);
  W[1 * 16 + 13] = 20 * U;
  assert.equal(kneeFloorTargets({ W, nb: NB, n: 1, upm: UPM, ground: 0, knees: { l: 1 } }).size, 0);
});

const FRT = fileURLToPath(new URL('../exports/FRT_fulcrum_v6.frames.json', import.meta.url));
test('FRT v6: sole markers land on the floor on planted frames; trigger counts reported', { skip: !exists(FRT) }, () => {
  const d = json(FRT), nb = d.bones.length, n = d.frames.length, W = new Float32Array(n * nb * 16);
  d.frames.forEach((fr, f) => W.set(fr, f * nb * 16));
  const bi = k => d.bones.indexOf(k);
  const legs = Object.fromEntries(['l', 'r'].map(s => [s, { thigh: bi('thigh_' + s), calf: bi('calf_' + s), foot: bi('foot_' + s), ball: bi('ball_' + s) }]));
  const ys = [];
  for (let f = 0; f < n; f++) for (const s of ['l', 'r']) ys.push(Math.min(W[(f * nb + legs[s].foot) * 16 + 13], W[(f * nb + legs[s].ball) * 16 + 13]));
  ys.sort((a, b) => a - b);
  const ground = ys[Math.floor(ys.length * 0.03)];
  const auto = floorTargets({ W, nb, n, upm: UPM, ground, legs });
  const rest = floorTargets({ W, nb, n, upm: UPM, ground, legs, rest: d.rest });
  for (const s of ['l', 'r']) {
    const a = auto.rig[s], b = rest.rig[s];
    console.log(`FRT ${s}: bottom ${(a.markers.bottom / U).toFixed(2)} cm (rest ${(b.markers.bottom / U).toFixed(2)}), middle ${(a.markers.middle / U).toFixed(2)} cm (rest ${(b.markers.middle / U).toFixed(2)}), offset take vs rest ${deg(q4.angle(a.offsetQ, b.offsetQ)).toFixed(2)} deg`);
    assert.equal(b.source, 'rest');
    if (s === 'l') { assert.ok(deg(q4.angle(a.offsetQ, b.offsetQ)) < 3); near(a.markers.bottom, b.markers.bottom, 1 * U); }
    let worst = 0, edge = 0, cnt = 0;
    for (let f = 0; f < n; f++) {
      const o = (f * nb + legs[s].foot) * 16, ob = (f * nb + legs[s].ball) * 16;
      const f1 = Math.min(n - 1, f + 1), f0 = Math.max(0, f - 1);
      const sp = Math.hypot(W[(f1 * nb + legs[s].ball) * 16 + 12] - W[(f0 * nb + legs[s].ball) * 16 + 12], W[(f1 * nb + legs[s].ball) * 16 + 14] - W[(f0 * nb + legs[s].ball) * 16 + 14]) * d.fps / 2 / UPM;
      if (sp > 0.05 || W[ob + 13] - ground > 2 * U) continue;
      const c0 = v3.unit([W[o], W[o + 1], W[o + 2]]), c1 = v3.unit([W[o + 4], W[o + 5], W[o + 6]]), c2 = v3.unit([W[o + 8], W[o + 9], W[o + 10]]);
      const qb = quatFromBasis(c0, c1, c2), mk = footMarkers({ p: [W[o + 12], W[o + 13], W[o + 14]], q: q4.mul(qb, b.offsetQ) }, { ...b.markers, side: b.side });
      const lo = Math.min((mk.backIn[1] + mk.backOut[1]) / 2, (mk.midIn[1] + mk.midOut[1]) / 2);
      worst = Math.max(worst, Math.abs(lo - ground) / U);
      edge = Math.max(edge, (ground - Math.min(mk.backIn[1], mk.backOut[1], mk.midIn[1], mk.midOut[1])) / U);
      cnt++;
    }
    console.log(`FRT ${s}: ${cnt} planted frames, sole centre line within ${worst.toFixed(2)} cm of the floor, deepest edge marker ${edge.toFixed(2)} cm below it`);
    assert.ok(cnt > 10);
    assert.ok(worst < 2.5, `worst ${worst}`);
  }
  for (const [name, r] of [['auto', auto], ['rest', rest]]) {
    let maxLift = 0, toe = 0, maxRot = 0;
    const per = { l: 0, r: 0 }, deep = { l: 0, r: 0 }, maxPen = { l: 0, r: 0 };
    for (const [f, ts] of r) for (const t of ts) {
      per[t.side]++; if (t.pen_cm > 0.5) deep[t.side]++; maxPen[t.side] = Math.max(maxPen[t.side], t.pen_cm);
      maxLift = Math.max(maxLift, t.lift_cm); if (t.ballQ) toe++;
      const o = (f * nb + legs[t.side].foot) * 16;
      maxRot = Math.max(maxRot, deg(q4.angle(quatFromBasis(v3.unit([W[o], W[o + 1], W[o + 2]]), v3.unit([W[o + 4], W[o + 5], W[o + 6]]), v3.unit([W[o + 8], W[o + 9], W[o + 10]])), t.ankleQ)));
    }
    console.log(`FRT ${name}: ground ${(ground / U).toFixed(2)} cm, ${r.size}/${n} frames trigger (l ${per.l}, r ${per.r}); over 0.5 cm deep: l ${deep.l}, r ${deep.r}; deepest l ${maxPen.l.toFixed(2)} r ${maxPen.r.toFixed(2)} cm; toe bends ${toe}; max lift ${maxLift.toFixed(2)} cm; max ankle turn ${maxRot.toFixed(1)} deg`);
    finite(r);
  }
});

function quatFromBasis(c0, c1, c2) {
  const t = c0[0] + c1[1] + c2[2];
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(c1[2] - c2[1]) / s, (c2[0] - c0[2]) / s, (c0[1] - c1[0]) / s, s / 4]; }
  else if (c0[0] > c1[1] && c0[0] > c2[2]) { const s = Math.sqrt(1 + c0[0] - c1[1] - c2[2]) * 2; q = [s / 4, (c1[0] + c0[1]) / s, (c2[0] + c0[2]) / s, (c1[2] - c2[1]) / s]; }
  else if (c1[1] > c2[2]) { const s = Math.sqrt(1 + c1[1] - c0[0] - c2[2]) * 2; q = [(c1[0] + c0[1]) / s, s / 4, (c2[1] + c1[2]) / s, (c2[0] - c0[2]) / s]; }
  else { const s = Math.sqrt(1 + c2[2] - c0[0] - c1[1]) * 2; q = [(c2[0] + c0[2]) / s, (c2[1] + c1[2]) / s, s / 4, (c0[1] - c1[0]) / s]; }
  const l = Math.hypot(...q);
  return q.map(x => x / l);
}
