import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { parseBVH } from '../mocap/bvh.js';
import { clipRig, clipCanonical, resampleCanonical } from '../tools/motionlib_bvh.mjs';
import { decodeBank, encodeBank, rigFromCalibration, calibrateTakeRig, canonFrames, canonPose, chainPose, studioWorld, studioLocal, bankInbetween,
  qangle, qslerp, qmul, qyaw, BANK_BONES, BANK_PARENT, NB } from '../wire/posebank.js';
import { suggestPoses } from '../poseassist.js';

const ROOT = new URL('..', import.meta.url).pathname, LIB = ROOT + 'data/MotionLibrary/';
const meta = JSON.parse(readFileSync(ROOT + 'wire/pose_bank.json', 'utf8')), bin = readFileSync(ROOT + 'wire/pose_bank.bin');
const bank = decodeBank(meta, bin), index = JSON.parse(readFileSync(LIB + 'index/index.json', 'utf8')), byFile = new Map(index.clips.map(c => [c.file, c]));
const D = 180 / Math.PI, q4 = (A, k) => Array.from(A.subarray(k * 4, k * 4 + 4)), mean = a => a.reduce((s, x) => s + x, 0) / a.length;
const load = file => { const rec = byFile.get(file), cr = clipRig(parseBVH(readFileSync(LIB + 'Animations/' + file, 'utf8')), rec); return { rec, cr, rig: rigFromCalibration(cr.lookup, meta.rigs[cr.hierarchy].O) }; };
// leave-clip-out: the clip and every near duplicate / same-legs variant the catalogue lists for it
const heldOut = file => { const r = byFile.get(file); return [file, ...(r.near_duplicates ?? []).map(d => d.of), ...(r.same_legs_other_travel ?? [])]; };
const spread = (xs, k) => Array.from({ length: k }, (_, i) => xs[Math.floor(i * xs.length / k)]);

test('pose bank stays small and documents its provenance', () => {
  assert.ok(bin.length + JSON.stringify(meta).length < 5 * 1024 * 1024);
  assert.equal(meta.bones.join(), BANK_BONES.join());
  assert.equal(meta.provenance.clips_used, meta.clips.length);
  assert.ok(meta.clips.length > 350 && bank.N > 10000);
  assert.ok(meta.clips.every(c => byFile.get(c.file)?.usable && !byFile.get(c.file).duplicate_of));
  assert.equal(new Set(meta.clips.map(c => c.rig_family)).size, 3);
});

test('bank round trip: decoded poses match the source clips within 0.5 degrees', () => {
  let worst = 0, checked = 0;
  for (const c of spread(meta.clips, 24)) {
    const { cr, rig } = load(c.file), res = resampleCanonical(clipCanonical(cr, rig), bank.fps);
    assert.equal(res.length, c.n);
    for (let t = 0; t < c.n; t++) for (let k = 0; k < NB; k++) { worst = Math.max(worst, qangle(q4(res[t].L, k), q4(bank.C, (c.start + t) * NB + k))); checked++; }
  }
  assert.ok(worst * D < 0.5, `worst ${(worst * D).toFixed(3)} deg over ${checked} bone poses`);
  const tiny = encodeBank([{ L: new Float64Array(NB * 4).map((_, i) => i % 4 === 3 ? 1 : 0), vel: [1.234, 0, -0.5], yaw: 3, height: 0.9, contact: [1, 0], phase: 2 }]);
  const back = decodeBank({ ...meta, frames: 1, layout: tiny.layout, clips: [{ start: 0, n: 1 }] }, tiny.bytes);
  assert.ok(Math.abs(back.vel[0] - 1.234) < 1e-3 && Math.abs(back.yaw[0] - 3) < 1e-3 && back.contact[0] === 1 && back.phase[0] === 2);
});

// A studio rig made from a UE BVH clip: local rotations from the file, calibrated from its own take like the studio does.
function studioFrom(file) {
  const { cr } = load(file), fk = cr.fk, nb = fk.nj, n = fk.n, q = new Float32Array(fk.L), W = new Float32Array(n * nb * 16);
  for (let k = 0; k < n * nb; k++) { W[k * 16] = W[k * 16 + 5] = W[k * 16 + 10] = W[k * 16 + 15] = 1; W.set(fk.P.subarray(k * 3, k * 3 + 3), k * 16 + 12); }
  const parents = parseBVH(readFileSync(LIB + 'Animations/' + file, 'utf8')).joints.map(j => j.parent), rootParentQ = () => [0, 0, 0, 1];
  return { nb, n, q, W, parents, rootParentQ, rig: calibrateTakeRig({ lookup: cr.lookup, q, W, nb, n, parents, rootParentQ }), fps: cr.fps };
}

test('studio rig conversion round-trips canonical poses', () => {
  const st = studioFrom('clip_0015_C_Idle.bvh'), f = 10, W = studioWorld(st.q, st.nb, f, st.parents, st.rootParentQ);
  const F = canonFrames(st.rig, i => W[i]), back = studioLocal(st.rig, F, st.q, st.nb, f, st.parents, st.rootParentQ);
  for (let i = 0; i < st.nb; i++) assert.ok(qangle(q4(back, i), q4(st.q, f * st.nb + i)) < 2e-3);
});

test('held-out library pose retrieval (leave-clip-out) beats take-only suggestion', () => {
  const take = studioFrom('clip_0015_C_Idle.bvh'), guideNames = ['thigh_l', 'thigh_r', 'upperarm_l', 'upperarm_r', 'spine_03'];
  const guides = guideNames.map(b => take.rig.idx[BANK_BONES.indexOf(b)]), freeK = BANK_BONES.map((_, k) => k).filter(k => !guideNames.includes(BANK_BONES[k]));
  const targets = spread(meta.clips.filter(c => c.file !== 'clip_0015_C_Idle.bvh' && c.n >= 8), 30);
  const errTake = [], errLib = [], fromTake = [];
  for (const c of targets) {
    const { cr, rig } = load(c.file), can = clipCanonical(cr, rig), t = Math.floor(can.n / 2), f = Math.min(take.n - 1, 20);
    const W0 = studioWorld(take.q, take.nb, f, take.parents, take.rootParentQ), yaw = canonPose(canonFrames(take.rig, i => W0[i]), take.rig.anc).yaw;
    const Fh = chainPose(can.frames[t].L), F = new Float64Array(NB * 4);
    for (let k = 0; k < NB; k++) F.set(qmul(qyaw(yaw), q4(Fh, k)), k * 4);
    const truthPose = studioLocal(take.rig, F, take.q, take.nb, f, take.parents, take.rootParentQ), current = take.q.slice();
    current.set(truthPose, f * take.nb * 4);
    const truth = canonPose(canonFrames(take.rig, i => studioWorld(current, take.nb, f, take.parents, take.rootParentQ)[i]), take.rig.anc).L;
    const errOf = pose => { const cur = current.slice(); cur.set(pose, f * take.nb * 4); const Wq = studioWorld(cur, take.nb, f, take.parents, take.rootParentQ), L = canonPose(canonFrames(take.rig, i => Wq[i]), take.rig.anc).L; return mean(freeK.map(k => qangle(q4(L, k), q4(truth, k)))) * D; };
    const args = { source: take.q, current, nb: take.nb, n: take.n, f, guides };
    const only = suggestPoses(args), both = suggestPoses({ ...args, library: { bank, rig: take.rig, parents: take.parents, rootParentQ: take.rootParentQ, context: { vel: can.frames[t].vel, contact: can.frames[t].contact, height: can.frames[t].height }, exclude: heldOut(c.file) } });
    assert.ok(both.suggestions.some(s => s.source === 'library' && s.clip && s.clipFile !== c.file));
    errTake.push(errOf(only.suggestions[0].pose)); errLib.push(errOf(both.suggestions[0].pose)); fromTake.push(both.suggestions[0].source === 'take');
    for (const s of both.suggestions) for (const i of guides) assert.ok(qangle(q4(s.pose, i), q4(current, f * take.nb + i)) < 2e-3, 'guided joints are preserved');
  }
  const a = mean(errTake), b = mean(errLib);
  console.log(`retrieval over ${targets.length} held-out clips: take-only ${a.toFixed(1)} deg, with library ${b.toFixed(1)} deg mean error on unguided joints (take row kept first ${fromTake.filter(Boolean).length}x)`);
  assert.ok(b < a * 0.8, `library ${b.toFixed(1)} vs take ${a.toFixed(1)}`);
});

test('take-local suggestions stay first when the take already holds the guided pose', () => {
  const take = studioFrom('clip_0015_C_Idle.bvh'), f = 20, g = 70, guides = ['thigh_l', 'upperarm_r', 'spine_03'].map(b => take.rig.idx[BANK_BONES.indexOf(b)]), current = take.q.slice();
  for (const i of guides) current.set(take.q.subarray((g * take.nb + i) * 4, (g * take.nb + i) * 4 + 4), (f * take.nb + i) * 4);
  const t0 = performance.now();
  const r = suggestPoses({ source: take.q, current, nb: take.nb, n: take.n, f, guides, library: { bank, rig: take.rig, parents: take.parents, rootParentQ: take.rootParentQ, exclude: ['clip_0015_C_Idle.bvh'] } });
  const ms = performance.now() - t0;
  assert.equal(r.suggestions[0].source, 'take');
  assert.ok(r.suggestions[0].guidanceErrorDeg < 1 && r.suggestions.some(s => s.source === 'library'));
  assert.ok(ms < 1000, `search took ${ms.toFixed(0)} ms`);
  console.log(`take + library suggestion over ${bank.N} bank poses: ${ms.toFixed(0)} ms`);
});

test('library inbetween on a held-out walk gap reproduces the true motion better than slerp', () => {
  const walks = meta.clips.filter(c => c.category === 'walk' && (c.locomotion_speed_ms ?? 0) > 0.8 && c.src_frames >= 28);
  assert.ok(walks.length >= 3);
  const lib = [], sl = [];
  for (const c of spread(walks, 4)) {
    const { cr, rig } = load(c.file), can = clipCanonical(cr, rig), gap = 12;
    for (let a = 4; a + gap < can.n - 1; a += 9) {
      const A = can.frames[a], B = can.frames[a + gap];
      const res = bankInbetween(bank, { LA: A.L, LB: B.L, yawA: A.yaw, yawB: B.yaw, ctxA: A, ctxB: B, seconds: gap / can.fps, exclude: heldOut(c.file) });
      assert.ok(res.segments.length && !heldOut(c.file).includes(res.segments[0].clipFile));
      const frames = res.frames(gap - 1);
      for (let j = 1; j < gap; j++) {
        const truth = can.frames[a + j].L, u = j / gap;
        lib.push(mean(BANK_BONES.map((_, k) => qangle(q4(frames[j - 1].L, k), q4(truth, k)))) * D);
        sl.push(mean(BANK_BONES.map((_, k) => qangle(qslerp(q4(A.L, k), q4(B.L, k), u), q4(truth, k)))) * D);
      }
      const end = res.frames(1)[0];
      assert.ok(Number.isFinite(end.yaw));
    }
  }
  const a = mean(lib), b = mean(sl);
  console.log(`inbetween over ${lib.length} held-out walk frames (0.4 s gaps): library ${a.toFixed(2)} deg, slerp ${b.toFixed(2)} deg mean joint error`);
  assert.ok(a < b, `library ${a.toFixed(2)} vs slerp ${b.toFixed(2)}`);
});

test('inbetween frames meet both keys exactly', () => {
  const c = meta.clips.find(x => x.category === 'walk'), { cr, rig } = load(c.file), can = clipCanonical(cr, rig), A = can.frames[2], B = can.frames[14];
  const res = bankInbetween(bank, { LA: A.L, LB: B.L, yawA: A.yaw, yawB: B.yaw, seconds: 12 / can.fps, exclude: [c.file] }), seg = res.segments[0];
  const near = (m, j) => res.frames(m)[j].L, m = 199;
  for (let k = 0; k < NB; k++) { assert.ok(qangle(q4(near(m, 0), k), q4(A.L, k)) < 0.05); assert.ok(qangle(q4(near(m, m - 1), k), q4(B.L, k)) < 0.05); }
  assert.ok(seg.warp >= 0.7 && seg.warp <= 1.45 && BANK_PARENT[0] === -1);
});
