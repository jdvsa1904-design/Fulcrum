import test from 'node:test';
import assert from 'node:assert/strict';
import { near, rng } from './helpers.mjs';
import * as M from '../mocap/math.js';
import * as S from '../mocap/skeleton.js';
import { solveFrame, restFrames, limbRotations } from '../mocap/solve.js';
import { solveTranslation, buildTracks, pickTrack, trackSeries, fillGaps, focalPx, IMG_STRIDE } from '../mocap/landmarks.js';
import { lowpass, cleanPoints } from '../mocap/clean.js';
import { lookAtCamera, synthCapture, staticPerson } from '../mocap/synth.js';
import { idleClip, gaitClip } from '../mocap/gait.js';

const randQ = (U, maxA) => M.qaxis(M.norm([U(-1, 1), U(-1, 1), U(-1, 1)]), U(-maxA, maxA));

test('math: frameRotation maps rest axes onto target axes and is rotation-equivariant', () => {
  const r = rng(1), U = (a, b) => a + (b - a) * r();
  for (let k = 0; k < 50; k++) {
    const R = randQ(U, Math.PI), p = M.norm([U(-1, 1), U(-1, 1), U(-1, 1)]), s = M.norm([U(-1, 1), U(-1, 1), U(-1, 1)]);
    const q = M.frameRotation(M.qrot(R, p), M.qrot(R, s), p, s);
    assert.ok(M.qangle(q, R) < 1e-9, 'recovers the rotation');
  }
});

test('math: swing/twist recomposes, slerp hits endpoints, eigSym3 sorts eigenpairs', () => {
  const r = rng(2), U = (a, b) => a + (b - a) * r();
  for (let k = 0; k < 30; k++) {
    const q = randQ(U, 3), ax = M.norm([U(-1, 1), U(-1, 1), U(-1, 1)]), { swing, twist } = M.swingTwist(q, ax);
    assert.ok(M.qangle(M.qmul(swing, twist), q) < 1e-9);
    assert.ok(M.len(M.cross(M.qrot(swing, ax), M.qrot(q, ax))) < 1e-9, 'swing carries the axis like q does');
    const b = randQ(U, 3);
    assert.ok(M.qangle(M.qslerp(q, b, 0), q) < 1e-9 && M.qangle(M.qslerp(q, b, 1), b) < 1e-9);
  }
  const e = M.eigSym3([[4, 1, 0], [1, 3, 0], [0, 0, 1]]);
  near(e[0].value, 1, 1e-9);
  near(Math.abs(e[0].vector[2]), 1, 1e-9);
  assert.ok(e[1].value < e[2].value);
});

test('skeleton: measuring synthetic landmarks returns the dimensions they were built from', () => {
  const dims = { ...S.DEFAULT_DIMS, thigh: 0.47, shin: 0.45, torso: 0.55, upperArm: 0.3, hipWidth: 0.2 };
  const skel = S.buildSkeleton(dims), L = S.landmarksFromPose(skel, S.restPose(skel)), m = S.measureFromLandmarks([L]);
  for (const k of Object.keys(dims)) near(m[k], dims[k], 2e-3, k);
  const sole = S.soleTrack(skel, S.restPose(skel), 'L');
  for (const p of Object.values(sole)) near(p[1], 0, 1e-9, 'rest soles touch the floor');
  const h = S.measureFromLandmarks([L], { height: 1.9 });
  near(S.buildSkeleton(h).dims.height, 1.9, 1e-9);
});

test('skeleton: FK keeps every bone length for random rotations', () => {
  const r = rng(3), U = (a, b) => a + (b - a) * r(), skel = S.buildSkeleton();
  for (let k = 0; k < 20; k++) {
    const local = new Float64Array(S.NB * 4);
    for (let b = 0; b < S.NB; b++) S.setQ(local, b, randQ(U, 2));
    assert.ok(S.boneLengthCheck(skel, S.fk(skel, [U(-1, 1), 1, U(-1, 1)], local)) < 1e-12);
  }
});

test('solve: random poses are recovered exactly from their own landmarks', () => {
  const r = rng(7), U = (a, b) => a + (b - a) * r(), skel = S.buildSkeleton({ ...S.DEFAULT_DIMS, thigh: 0.45 }), rf = restFrames(skel);
  let worstP = 0, worstQ = 0;
  for (let trial = 0; trial < 100; trial++) {
    const world = new Float64Array(S.NB * 4), qH = randQ(U, Math.PI), qC = M.qmul(qH, M.qaxis([0, 1, 0], U(-0.8, 0.8)));
    S.setQ(world, S.B.Hips, qH); S.setQ(world, S.B.Spine, M.qslerp(qH, qC, 1 / 3)); S.setQ(world, S.B.Spine1, M.qslerp(qH, qC, 2 / 3)); S.setQ(world, S.B.Spine2, qC);
    const qHd = M.qmul(qC, randQ(U, 0.8));
    S.setQ(world, S.B.Neck, M.qslerp(qC, qHd, 0.5)); S.setQ(world, S.B.Head, qHd);
    for (const side of ['L', 'R']) {
      const Sd = S.SIDES[side], d0 = [Sd.s, 0, 0], h0 = [0, -Sd.s, 0];
      S.setQ(world, Sd.clav, qC);
      const qA = M.qmul(qC, randQ(U, 1.2)), lo = M.qmul(qA, M.qaxis(h0, U(0.6, 2))), ta = U(-2, 2);
      const sw = M.qaxis(M.norm(M.perp([U(-1, 1), U(-1, 1), U(-1, 1)], d0)), U(-1, 1));
      S.setQ(world, Sd.arm, qA); S.setQ(world, Sd.fore, M.qmul(lo, M.qaxis(d0, ta / 2))); S.setQ(world, Sd.hand, M.qmul(lo, M.qmul(sw, M.qaxis(d0, ta))));
      const qU = M.qmul(qH, M.qmul(M.qaxis([0, 1, 0], U(-0.5, 0.5)), M.qaxis(M.norm([U(-1, 1), 0, U(-1, 1)]), U(0, 0.9)))), qK = M.qmul(qU, M.qaxis([1, 0, 0], U(0.6, 2)));
      const qF = M.qmul(qK, randQ(U, 0.8));
      S.setQ(world, Sd.up, qU); S.setQ(world, Sd.knee, qK); S.setQ(world, Sd.foot, qF); S.setQ(world, Sd.toe, qF);
    }
    const W = S.fk(skel, [U(-2, 2), U(0.5, 1.2), U(-3, 3)], S.worldToLocal(world)), res = solveFrame(skel, S.landmarksFromPose(skel, W), rf);
    const W2 = S.fk(skel, res.root, res.local);
    for (let b = 0; b < S.NB; b++) {
      worstP = Math.max(worstP, M.dist(S.getV(W.pos, b), S.getV(W2.pos, b)));
      worstQ = Math.max(worstQ, M.qangle(S.getQ(W.q, b), S.getQ(W2.q, b)));
    }
  }
  assert.ok(worstP < 1e-9, `position error ${worstP}`);
  assert.ok(worstQ < 1e-6, `rotation error ${worstQ}`);
});

test('solve: a straight leg with a backward-looking knee is held nearly straight, not bent backwards', () => {
  const r = limbRotations([0, -0.42, 0.02], [0, -0.41, -0.12].map((x, i) => i === 2 ? 0.14 : x), M.QI, [0, -1, 0], [1, 0, 0], { hint: [1, 0, 0], flipAlways: true });
  assert.ok(r.beta <= 0 && r.beta >= -0.09 - 1e-9, `beta ${r.beta}`);
  const shin = M.qrot(r.qLo, [0, -1, 0]), thigh = M.qrot(r.qUp, [0, -1, 0]);
  assert.ok(M.angle(shin, thigh) <= 0.09 + 1e-9);
});

test('landmarks: least-squares translation recovers the true camera-space position', () => {
  const skel = S.buildSkeleton(), cam = lookAtCamera([0, 1.4, 5], [0, 0.9, 0], { width: 1600, height: 900, hfov: 55 });
  const take = idleClip(skel, { seconds: 0.2 });
  for (let k = 0; k < take.n; k++) take.root[k * 3] += 0.8;
  const { capture } = synthCapture(skel, take, cam);
  const p = capture.frames[0].poses[0], f = focalPx(1600, 55), t = solveTranslation(p.img, p.world, 1600, 900, f);
  const W = S.fk(skel, S.getV(take.root, 0), take.rot.subarray(0, S.NB * 4)), L = S.landmarksFromPose(skel, W);
  const hip = M.mid(S.lmAt(L, S.LM.lHip), S.lmAt(L, S.LM.rHip)), d = M.sub(hip, cam.pos);
  const want = [M.dot(d, cam.r), M.dot(d, cam.dn), M.dot(d, cam.f)];
  for (let k = 0; k < 3; k++) near(t[k], want[k], 1e-4, 'axis ' + k);
});

test('landmarks: tracking picks the most prominent person, and a click picks the other one', () => {
  const skel = S.buildSkeleton(), cam = lookAtCamera([0, 1.4, 6], [0, 0.9, 0], { width: 1280, height: 720 });
  const walk = gaitClip(skel, { kind: 'walk', fps: 30 }), take = { n: 60, fps: 30, root: new Float64Array(180), rot: new Float64Array(60 * S.NB * 4) };
  for (let k = 0; k < 60; k++) {
    const src = k % walk.n;
    take.root.set([walk.root[src * 3] - 1 + k * 0.04, walk.root[src * 3 + 1], walk.root[src * 3 + 2] + 1.5], k * 3);
    take.rot.set(walk.rot.subarray(src * S.NB * 4, (src + 1) * S.NB * 4), k * S.NB * 4);
  }
  const statue = staticPerson(skel, cam, [2.6, 0, -6], { local: S.standLocal() });
  const { capture } = synthCapture(skel, take, cam, { extraPerson: statue, shuffle: true });
  const tracks = buildTracks(capture);
  assert.equal(tracks.length, 2);
  const main = pickTrack(tracks, capture);
  assert.equal(main.count, 60);
  const k = main.frames.get(10), img = capture.frames[10].poses[k].img;
  const hipX = (img[S.LM.lHip * IMG_STRIDE] + img[S.LM.rHip * IMG_STRIDE]) / 2;
  assert.ok(Math.abs(hipX - 0.5) < 0.25, 'the walker is near the middle of the frame');
  const other = statue(), sx = (other.img[S.LM.lHip * IMG_STRIDE] + other.img[S.LM.rHip * IMG_STRIDE]) / 2, sy = other.img[S.LM.nose * IMG_STRIDE + 1];
  const clicked = pickTrack(tracks, capture, { point: [sx, sy], frame: 10 });
  assert.notEqual(clicked.id, main.id);
  const series = trackSeries(capture, main, { trim: [5, 40] });
  assert.equal(series.first, 5);
  assert.equal(series.n, 36);
});

test('landmarks: gaps are filled by interpolation and the ends are held', () => {
  const arr = new Float64Array([0, 0, 0, 3, 0, 0, 6, 0]), valid = new Uint8Array([0, 0, 1, 0, 0, 1, 0, 0]);
  arr[2] = 2; arr[5] = 5;
  fillGaps(arr, valid, 1);
  assert.deepEqual(Array.from(arr), [2, 2, 2, 3, 4, 5, 5, 5]);
});

test('clean: low-pass keeps slow motion, removes jitter, and does not pin the ends to noise', () => {
  const r = rng(5), fps = 30, n = 120, x = new Float64Array(n), truth = new Float64Array(n);
  for (let i = 0; i < n; i++) { truth[i] = 0.3 * Math.sin(2 * Math.PI * 0.8 * i / fps) + 0.5 * i / fps; x[i] = truth[i] + (r() - 0.5) * 0.04; }
  x[0] += 0.08;
  x[n - 1] -= 0.08;
  const y = lowpass(x, fps, 4);
  let err = 0, raw = 0;
  for (let i = 0; i < n; i++) { err += (y[i] - truth[i]) ** 2; raw += (x[i] - truth[i]) ** 2; }
  assert.ok(err < raw * 0.4, `filtered error ${err} vs raw ${raw}`);
  assert.ok(Math.abs(y[0] - truth[0]) < 0.05 && Math.abs(y[n - 1] - truth[n - 1]) < 0.05, 'ends follow the trend, not the spike');
  const P = new Float64Array(n * S.NLM * 3).map((_, i) => Math.sin(i));
  const c = cleanPoints(P, n, fps, {});
  assert.ok(c.P.every(Number.isFinite));
});
