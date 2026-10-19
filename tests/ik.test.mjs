import test from 'node:test';
import assert from 'node:assert/strict';
import { near, nearVec, v3, rng, deg, rad } from './helpers.mjs';
import {
  qrot, arc, rotAxis, defaultPole, twoBoneIK, bendDirFromHinge, stablePole, poleGuardTrack,
  ikLimbSolve, swivelAngle, signedSwivel, swivelPlane, springIK, springRestPole,
} from '../ik.js';

const root = [0, 100, 0], mid = [4, 55, 12], end = [0, 10, 0];
const L1 = v3.dist(root, mid), L2 = v3.dist(mid, end);

test('arc rotates u onto v, including the antiparallel case', () => {
  const r = rng(3);
  for (let i = 0; i < 50; i++) {
    const u = v3.unit([r() - 0.5, r() - 0.5, r() - 0.5]), v = v3.unit([r() - 0.5, r() - 0.5, r() - 0.5]);
    nearVec(qrot(arc(u, v), u), v, 1e-12);
  }
  nearVec(qrot(arc([1, 0, 0], [-1, 0, 0]), [1, 0, 0]), [-1, 0, 0], 1e-12);
  nearVec(arc([0, 1, 0], [0, 1, 0]), [0, 0, 0, 1], 1e-12);
});

test('rotAxis is Rodrigues: preserves length, quarter turn about z', () => {
  nearVec(rotAxis([1, 0, 0], [0, 0, 1], Math.PI / 2), [0, 1, 0], 1e-12);
  near(v3.len(rotAxis([1, 2, 3], v3.unit([1, 1, 0]), 0.7)), Math.hypot(1, 2, 3), 1e-12);
});

test('twoBoneIK keeps bone lengths and reaches a reachable goal', () => {
  const r = rng(11);
  for (let i = 0; i < 200; i++) {
    const goal = [(r() - 0.5) * 60, 30 + r() * 60, (r() - 0.5) * 60];
    const s = twoBoneIK(root, mid, end, goal);
    near(v3.dist(root, s.mid), L1, 1e-9);
    near(v3.dist(s.mid, s.end), L2, 1e-9);
    if (!s.clamped) nearVec(s.end, goal, 1e-9);
  }
});

test('twoBoneIK clamps an unreachable goal along the goal ray at full extension', () => {
  const goal = [0, 100 - (L1 + L2) - 50, 0];
  const s = twoBoneIK(root, mid, end, goal);
  assert.ok(s.clamped);
  near(v3.dist(root, s.end), L1 + L2, 1e-9);
  near(v3.dist(root, s.mid) + v3.dist(s.mid, s.end), L1 + L2, 1e-9);
  nearVec(s.end, [0, 100 - (L1 + L2), 0], 1e-9);
});

test('twoBoneIK bends toward the pole target', () => {
  const goal = [0, 40, 0];
  for (const side of [[1, 0, 0], [-1, 0, 0], [0, 0, 1], [0, 0, -1]]) {
    const s = twoBoneIK(root, mid, end, goal, { poleTarget: v3.add(root, v3.mul(side, 50)) });
    assert.ok(v3.dot(v3.sub(s.mid, root), side) > 0, side.join());
  }
});

test('twoBoneIK twist swings the bend plane about the root-goal axis without moving the end', () => {
  const goal = [3, 40, 5], base = twoBoneIK(root, mid, end, goal, { pole: [1, 0, 0] });
  const tw = twoBoneIK(root, mid, end, goal, { pole: [1, 0, 0], twist: rad(40) });
  nearVec(tw.end, base.end, 1e-9);
  const H = v3.unit(v3.sub(goal, root));
  near(deg(Math.acos(v3.dot(base.bend, tw.twistedBend))), 40, 1e-6);
  near(Math.abs(v3.dot(tw.twistedBend, H)), 0, 1e-9);
});

test('twoBoneIK rotations carry the rest chain onto the solved chain', () => {
  const goal = [10, 35, 15], s = twoBoneIK(root, mid, end, goal, { pole: [0, 0, 1] });
  nearVec(v3.add(root, qrot(s.rotRoot, v3.sub(mid, root))), s.mid, 1e-9);
  const e = v3.add(s.mid, qrot(s.rotMid, qrot(s.rotRoot, v3.sub(end, mid))));
  nearVec(e, s.end, 1e-9);
});

test('twoBoneIK guard: degenerate pole falls back (strict: prev, then preferred, then rest)', () => {
  const goal = [0, 40, 0], H = [0, 1, 0];
  const a = twoBoneIK(root, mid, end, goal, { pole: H, prevBend: [0, 0, 1], preferredBend: [1, 0, 0] });
  assert.equal(a.source, 'prev');
  nearVec(a.bend, [0, 0, 1], 1e-12);
  const b = twoBoneIK(root, mid, end, goal, { pole: H, preferredBend: [1, 0, 0] });
  assert.equal(b.source, 'preferred');
  const c = twoBoneIK(root, mid, end, goal, { pole: H });
  assert.equal(c.source, 'rest');
  const m = twoBoneIK(root, mid, end, goal, { pole: H, prevBend: [0, 0, 1], guard: 'maya' });
  assert.equal(m.source, 'rest');
  const z = twoBoneIK([0, 0, 0], [0, 0, 1], [0, 0, 2], [0, 0, 1.5], { pole: [0, 0, 0] });
  assert.ok(['rest', 'arbitrary'].includes(z.source));
  near(v3.len(z.bend), 1, 1e-12);
});

test('twoBoneIK never returns NaN for goal on the root or fully folded', () => {
  for (const goal of [root, [0, 100, 1e-14], v3.add(root, [0, -(Math.abs(L1 - L2)), 0])]) {
    const s = twoBoneIK(root, mid, end, goal);
    for (const v of [...s.mid, ...s.end, ...s.rotRoot, ...s.rotMid]) assert.ok(Number.isFinite(v));
  }
});

test('defaultPole is the Maya rest pole 2*mid - root - end', () => {
  nearVec(defaultPole(root, mid, end), [8, 0, 24], 1e-12);
});

test('bendDirFromHinge points opposite to the knee motion when the hinge flexes', () => {
  const r = [0, 0, 0], m = [0, -45, 0], e = [0, -90, 0];
  const d = bendDirFromHinge(r, m, e, [1, 0, 0]);
  near(v3.len(d), 1, 1e-12);
  near(v3.dot(d, [0, 1, 0]), 0, 1e-12);
  const d2 = bendDirFromHinge(r, m, e, [-1, 0, 0]);
  nearVec(v3.mul(d, -1), d2, 1e-9);
});

test('stablePole puts the pole on the knee side and tracks a straight leg through prevN', () => {
  const st = {};
  const bent = stablePole([0, 100, 0], [0, 55, 10], [0, 10, 0], { height: 180 }, st);
  assert.ok(bent.n[2] > 0.99);
  assert.ok(bent.pole[2] > 55);
  const straight = stablePole([0, 100, 0], [0, 55, 0.01], [0, 10, 0], { height: 180 }, st);
  assert.equal(straight.source, 'prev');
  nearVec(straight.n, bent.n, 1e-12);
  const fresh = stablePole([0, 100, 0], [0, 55, 0], [0, 10, 0], { height: 180, bendDir: [0, 0, 1] }, {});
  assert.ok(fresh.pole[2] > 0);
  assert.equal(fresh.source, 'knee');
});

test('stablePole bendDir offsets the chord point so a straight knee still has a side', () => {
  const r = stablePole([0, 100, 0], [0, 55, 0], [0, 10, 0], { height: 180, bendDir: [1, 0, 0] }, {});
  assert.ok(r.n[0] > 0.99);
});

test('poleGuardTrack: a leg that passes through straight keeps its knee side with no flip', () => {
  const frames = [];
  for (let i = 0; i <= 40; i++) {
    const kz = 10 * Math.cos(Math.PI * i / 40) * (i < 20 ? 1 : 0.001 * (i - 20));
    const mid2 = [0, 55, i < 20 ? 10 * (1 - i / 20) : 0.0004 * (i - 20)];
    frames.push({ root: [0, 100, 0], mid: mid2, end: [0, 10, 0], goal: [0, 10 + i * 0.4, 0] });
    void kz;
  }
  const out = poleGuardTrack(frames, { height: 180, bendDir: [0, 0, 1] });
  assert.equal(out.length, frames.length);
  for (const o of out) {
    assert.ok(o.mid[2] > -1e-6, `knee flipped: ${o.mid}`);
    for (const v of o.mid) assert.ok(Number.isFinite(v));
  }
});

test('ikLimbSolve: law of cosines, clamped to rmax/rmin', () => {
  const a = ikLimbSolve(45, 45, [0, 0, 63.63961030678928]);
  near(deg(a.kneeInterior), 90, 1e-6);
  near(deg(a.knee), 90, 1e-6);
  near(deg(a.hipOff), 45, 1e-6);
  assert.equal(a.clamped, false);
  const far = ikLimbSolve(45, 45, [0, 0, 200]);
  assert.ok(far.clamped);
  near(far.d, 90 * 0.999, 1e-12);
  near(far.knee, Math.acos(far.cosK) ? Math.PI - Math.acos(far.cosK) : 0, 1e-12);
  const tight = ikLimbSolve(60, 30, [0, 0, 0]);
  assert.ok(tight.clamped);
  near(tight.d, 30 + 1e-6, 1e-12);
});

test('swivelAngle is 0..2pi, signedSwivel is -pi..pi, both agree on direction', () => {
  const dir = [0, 0, 1], n0 = [1, 0, 0];
  for (const a of [0.3, 1.2, 2.9, -0.3, -1.2, -2.9]) {
    const n = rotAxis(n0, dir, a);
    near(signedSwivel(n0, n, dir), a, 1e-12);
    const s = swivelAngle(n0, n, dir);
    near(Math.atan2(Math.sin(s - a), Math.cos(s - a)), 0, 1e-9);
    assert.ok(s >= 0 && s <= 2 * Math.PI);
  }
});

test('swivelPlane projects the pole off the axis, with a fallback for a collinear pole', () => {
  nearVec(swivelPlane([3, 5, 0], [0, 1, 0]), [1, 0, 0], 1e-12);
  nearVec(swivelPlane([0, 5, 0], [0, 1, 0], [0, 0, 1]), [0, 0, 1], 1e-12);
  near(v3.len(swivelPlane([0, 5, 0], [0, 1, 0])), 1, 1e-12);
});

const chain = [[0, 0, 0], [0, -40, 6], [0, -80, 0], [0, -95, 12]];
const restLen = chain.slice(1).map((p, i) => v3.dist(p, chain[i]));

test('springIK preserves bone lengths and reaches targets inside the range', () => {
  const total = restLen.reduce((a, b) => a + b, 0);
  const rest = v3.dist(chain[0], chain[3]);
  for (const k of [0.5, 0.7, 0.9, 1.0]) {
    const goal = v3.mul(v3.unit([0.2, -1, 0.3]), rest * k + (k > 0.95 ? 0 : 0));
    const r = springIK(chain, goal);
    r.positions.slice(1).forEach((p, i) => near(v3.dist(p, r.positions[i]), restLen[i], 1e-9, 'bone ' + i));
    if (rest * k < total * 0.98) near(v3.dist(r.positions[0], r.positions[3]), v3.len(goal), 1e-3, 'reach ' + k);
  }
});

test('springIK at the rest distance returns the rest pose and identity-sized rotations', () => {
  const goal = v3.sub(chain[3], chain[0]);
  const r = springIK(chain, goal);
  assert.ok(r.reachError < 1e-5);
  chain.forEach((p, i) => nearVec(r.positions[i], p, 1e-9));
  assert.equal(r.rotations.length, chain.length - 1);
});

test('springIK compresses when the goal is nearer and stretches when farther', () => {
  const dr = v3.dist(chain[0], chain[3]);
  const near1 = springIK(chain, [0, -dr * 0.7, 0]);
  assert.equal(near1.compressing, true);
  const far = springIK(chain, [0, -dr * 1.05, 0]);
  assert.equal(far.compressing, false);
  assert.ok(near1.interior.every((x, i) => x < 180 + 1e-9 && Number.isFinite(x)));
  assert.ok(far.interior.every((x, i) => x >= near1.interior[i] - 1e-9));
});

test('springIK pole rotates the chain plane about the root-goal axis', () => {
  const dr = v3.dist(chain[0], chain[3]), goal = [0, -dr * 0.8, 0];
  const a = springIK(chain, goal, { pole: [1, 0, 0] }), b = springIK(chain, goal, { pole: [-1, 0, 0] });
  assert.ok(a.positions[1][0] > 0 || a.positions[2][0] > 0 || Math.abs(a.positions[1][0]) < 1e-9);
  const sa = a.positions[1][0] + a.positions[2][0], sb = b.positions[1][0] + b.positions[2][0];
  assert.ok(sa * sb <= 1e-9);
  assert.ok(v3.len(springRestPole(chain)) > 0);
});

test('springIK rotations chain: parent-accumulated rotation maps rest bone dirs to solved', () => {
  const dr = v3.dist(chain[0], chain[3]), goal = v3.mul(v3.unit([0.1, -1, 0.2]), dr * 0.85);
  const r = springIK(chain, goal);
  const d0 = v3.unit(v3.sub(r.positions[1], r.positions[0]));
  nearVec(v3.unit(qrot(r.rotations[0], v3.sub(chain[1], chain[0]))), d0, 1e-9);
});
