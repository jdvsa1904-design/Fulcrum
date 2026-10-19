// Library-learned motion prior (wire/prior.js + the committed wire/motion_prior.json): model shape and provenance,
// clean takes stay quiet, injected defects come out as 'unnatural_*' findings, the judge only adds them when given the
// model, and fixplan maps them to fixes or reports them.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import { makeTake, rotateLocal, harness, fk, smoothstep, NB, RIG, NAMES, PARENTS } from './fixrig.mjs';
import { loadPrior, priorFindings, priorFeatures, buildPrior, kneeFlex } from '../wire/prior.js';
import { computeFacts } from '../wire/facts.js';
import { judgeMotion } from '../wire/judge.js';
import { planCandidates, NO_AUTO_FIX } from '../wire/fixplan.js';

const FILE = new URL('../wire/motion_prior.json', import.meta.url);
const TEXT = readFileSync(FILE, 'utf8'), MODEL = loadPrior(JSON.parse(TEXT));
const sway = f => [0, 100 + 0.5 * Math.sin(2 * Math.PI * f / 45), 0.3 * Math.sin(2 * Math.PI * f / 60)];
const walk = (n = 120) => {
  const ank = (x, off) => f => { const ph = (f / 30 + off) % 1, z = 120 * f / 30, sw = ph < 0.4; return [x, 8 + (sw ? 10 * Math.sin(Math.PI * ph / 0.4) : 0), sw ? z - 30 + 75 * smoothstep(ph / 0.4) : z + 45 - 120 * (ph - 0.4)]; };
  return makeTake({ n, pelvis: f => [0, 96 + 2 * Math.cos(4 * Math.PI * f / 30), 120 * f / 30], ankle: { l: ank(10, 0), r: ank(-10, 0.5) } });
};
const factsOf = t => computeFacts({ W: t.W, nb: NB, n: t.n, fps: t.fps, upm: 100, rig: RIG, restW: harness(t).restW });
const prior = t => priorFindings({ model: MODEL, facts: factsOf(t), W: t.W, nb: NB });

test('committed model: version, size, provenance and the knee notes', () => {
  assert.ok(MODEL, 'loadPrior accepts the committed JSON');
  assert.ok(TEXT.length < 200 * 1024, `model is ${TEXT.length} bytes`);
  const p = MODEL.provenance;
  assert.match(p.source_sha256, /^[0-9a-f]{64}$/);
  assert.match(p.index_sha256, /^[0-9a-f]{64}$/);
  assert.ok(p.clips >= 150 && p.frames > 10000, `${p.clips} clips, ${p.frames} frames`);
  assert.ok(p.clips_per_phase.locomotion >= 30 && p.clips_per_phase.stand >= 20, JSON.stringify(p.clips_per_phase));
  assert.equal(p.excluded.retargeted > 0, true, 'the retargeted set is excluded');
  assert.ok(!('mixamo' in p.rig_families) || p.rig_families.autorigpro > 0);
  for (const k of ['knee_flex', 'speed_knee', 'snap_spine', 'pelvis_height', 'leg_motion', 'plant_drift']) assert.ok(MODEL.env[k]?.all, k);
  assert.equal(MODEL.features.knee_flex.check, 'hi', 'knee minimum is never a must-bend rule');
  assert.ok(MODEL.knee.library_p2_deg >= 0 && MODEL.knee.library_p2_deg < 1, `library knee p2 ${MODEL.knee.library_p2_deg} (the rig clamp)`);
  assert.ok(p.validation.held_clean_false_alarm < 0.05, `held-out false alarms ${p.validation.held_clean_false_alarm}`);
  assert.equal(p.validation.held_clean_blockers, 0);
  assert.equal(loadPrior({ version: 99 }), null);
  assert.equal(loadPrior(null), null);
});

test('clean synthetic stand and walk: no prior findings', () => {
  assert.deepEqual(prior(makeTake({ n: 90, pelvis: sway })).map(f => f.text), []);
  assert.deepEqual(prior(walk()).map(f => f.text), []);
});

test('a straight (unbent) knee is not flagged: the library bent-knee bias is no lower limit', () => {
  const t = makeTake({ n: 90, pelvis: f => [0, 102.8 + 0.2 * Math.sin(2 * Math.PI * f / 45), 0] });
  const F = priorFeatures({ W: t.W, nb: NB, n: t.n, fps: t.fps, rig: RIG, facts: factsOf(t) });
  assert.ok(Math.max(...F.tracks.knee_flex_l) < 12, 'legs nearly straight');
  assert.equal(prior(t).filter(f => /knee/.test(f.rule)).length, 0);
});

test('injected pop (40 deg, one frame) on the spine: snap blocker and speed warning', () => {
  const fs = prior(rotateLocal(makeTake({ n: 90, pelvis: sway }), 'spine_03', 45, [1, 0, 0], 40));
  const snap = fs.find(f => f.rule === 'unnatural_spine_snap');
  assert.ok(snap && snap.a <= 45 && snap.b >= 45, fs.map(f => f.rule).join());
  assert.equal(snap.severity, 'blocker');
  assert.equal(snap.source, 'prior');
  assert.match(snap.text, /^Spine snaps 40 deg/);
  assert.ok(fs.some(f => f.rule === 'unnatural_spine_speed' && f.severity === 'warning'));
});

test('injected hyperextended knee: unnatural_knee_hyperextension on that side', () => {
  let t = makeTake({ n: 90, pelvis: sway });
  for (let f = 40; f <= 55; f++) t = rotateLocal(t, 'calf_l', f, [1, 0, 0], -55 * Math.min(1, (f - 39) / 3, (56 - f) / 3));
  const fs = prior(t).filter(f => f.rule === 'unnatural_knee_hyperextension');
  assert.equal(fs.length, 1, prior(t).map(f => f.text).join(' | '));
  assert.equal(fs[0].side, 'l');
  assert.ok(fs[0].a >= 40 && fs[0].b <= 55 && fs[0].value < -10);
  assert.match(fs[0].text, /^Left knee bends backwards \d+ deg/);
});

test('kneeFlex signs a backwards knee from the toes, and keeps folded legs positive', () => {
  const W = new Float32Array(3 * 4 * 16), put = (f, i, p) => { W.set([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, ...p, 1], (f * 4 + i) * 16); };
  const pose = (f, knee, ball) => { put(f, 0, [0, 90, 0]); put(f, 1, knee); put(f, 2, [0, 8, 0]); put(f, 3, ball); };
  pose(0, [0, 50, 8], [0, 1, 13]);      // knee forward of the hip-ankle line, toes forward: bent ~20 deg
  pose(1, [0, 50, -8], [0, 1, 13]);     // knee behind the line, toes forward: hyperextended
  pose(2, [0, 50, -40], [0, 1, -13]);   // folded past 40 deg with the toes behind (kneeling on the instep): positive
  const k = kneeFlex(W, 4, 3, 0, 1, 2, 3);
  assert.ok(k[0] > 15 && k[0] < 30, String(k[0]));
  assert.ok(k[1] < -15 && k[1] > -30, String(k[1]));
  assert.ok(k[2] > 40, String(k[2]));
});

test('injected frozen leg in a walk: unnatural_frozen_limb', () => {
  const t = walk();
  for (let f = 40; f <= 80; f++) for (const k of ['thigh_l', 'calf_l', 'foot_l']) t.q.set(t.q.subarray((40 * NB + RIG[k]) * 4, (40 * NB + RIG[k]) * 4 + 4), (f * NB + RIG[k]) * 4);
  const fs = prior({ ...t, W: fk(t.q, t.p, t.n) }).filter(f => f.rule === 'unnatural_frozen_limb');
  assert.equal(fs.length, 1);
  assert.equal(fs[0].side, 'l');
  assert.ok(fs[0].a >= 40 && fs[0].b <= 80);
});

test('judgeMotion adds prior findings only when the model is supplied', () => {
  const t = rotateLocal(makeTake({ n: 90, pelvis: sway }), 'spine_03', 45, [1, 0, 0], 40), h = harness(t);
  const args = { motion: t, nb: NB, n: t.n, fps: t.fps, upm: 100, rig: RIG, names: NAMES, parents: PARENTS, restW: h.restW, shots: t.shots, ground: h.ground };
  const without = judgeMotion(args), withPrior = judgeMotion({ ...args, prior: MODEL });
  assert.equal(without.findings.filter(f => f.source === 'prior').length, 0);
  const extra = withPrior.findings.filter(f => f.source === 'prior');
  assert.ok(extra.some(f => f.rule === 'unnatural_spine_snap'));
  assert.ok(withPrior.cost > without.cost);
  assert.deepEqual(withPrior.findings.filter(f => f.source !== 'prior').map(f => f.rule), without.findings.map(f => f.rule));
  for (const f of extra) for (const k of ['rule', 'severity', 'a', 'b', 'side', 'phase', 'value', 'limit', 'unit', 'source', 'text']) assert.ok(k in f, k);
});

test('fixplan: prior speed spikes -> despike / knee spike, hyperextension -> knee extension, the rest report only', () => {
  const t = makeTake({ n: 90, pelvis: sway }), facts = factsOf(t), state = { fixes: [], locks: [], pins: [] };
  const mk = (rule, o = {}) => ({ rule, severity: 'warning', a: 40, b: 42, side: 'l', phase: 'stand', source: 'prior', text: rule, ...o });
  const noise = planCandidates({ findings: [mk('unnatural_knee_speed', { group: 'knee', bones: [RIG.calf_l] }), mk('unnatural_elbow_snap', { group: 'elbow', bones: [RIG.lowerarm_l] })], facts, state, stage: 'noise' });
  const knee = noise.find(c => c.rule === 'unnatural_knee_speed'), elbow = noise.find(c => c.rule === 'unnatural_elbow_snap');
  assert.equal(knee.change.fixes[0].type, 'knee');
  assert.equal(knee.change.fixes[0].mode, 'spike');
  assert.ok(knee.variants.some(v => v.fixes[0].type === 'despike'));
  assert.deepEqual(elbow.change.fixes[0], { type: 'despike', a: 38, b: 44, bones: [RIG.lowerarm_l] });
  assert.ok(planCandidates({ findings: [mk('unnatural_elbow_snap')], facts, state, stage: 'polish' }).some(c => c.change.fixes[0].type === 'smooth'));
  const ext = planCandidates({ findings: [mk('unnatural_knee_hyperextension')], facts, state, stage: 'contacts' });
  assert.equal(ext[0].change.fixes[0].mode, 'extension');
  for (const r of ['unnatural_pelvis_speed', 'unnatural_frozen_limb', 'unnatural_hip_flexion', 'unnatural_step_period']) {
    for (const stage of ['noise', 'body', 'contacts', 'polish']) assert.equal(planCandidates({ findings: [mk(r)], facts, state, stage }).length, 0, `${r} ${stage}`);
    assert.ok(NO_AUTO_FIX[r], r);
  }
});

test('buildPrior: phases with too few clips fall back to their family, then to all', () => {
  const t = makeTake({ n: 60, pelvis: sway }), facts = factsOf(t), F = priorFeatures({ W: t.W, nb: NB, n: t.n, fps: t.fps, rig: RIG, facts });
  const m = buildPrior([{ features: F }, { features: F }], { minClips: 3 });
  assert.ok(m.env.knee_flex.all && !m.env.knee_flex.stand);
  assert.equal(priorFindings({ model: m, facts, W: t.W, nb: NB, features: F }).filter(f => f.kind !== 'other').length, 0);
});

test('library clips: a clean held-out clip stays quiet, an injected arm pop is found', { skip: !existsSync(new URL('../data/MotionLibrary/index/index.json', import.meta.url)) && 'motion library not present' }, async () => {
  const { loadClip } = await import('../tools/motionlib_clip.mjs');
  const dir = new URL('../data/MotionLibrary/Animations/', import.meta.url).pathname;
  for (const file of ['clip_0508_W2_Walk_Aim_F_Loop.bvh', 'clip_0322_Rifle_CrouchLoop.bvh'].filter(f => existsSync(dir + f))) {
    const c = loadClip(dir, file), facts = computeFacts({ W: c.W, nb: c.nb, n: c.n, fps: c.fps, upm: c.upm, rig: c.rig, restW: c.restW });
    assert.deepEqual(priorFindings({ model: MODEL, facts, W: c.W, nb: c.nb }).map(f => f.text), [], file);
    const W = Float32Array.from(c.W), f0 = c.n >> 1, kids = i => c.parents.flatMap((p, j) => p === i ? [j, ...kids(j)] : []);
    const ua = c.rig.upperarm_l, o = (f0 * c.nb + ua) * 16, piv = [W[o + 12], W[o + 13], W[o + 14]], a = 60 * Math.PI / 180, ca = Math.cos(a), sa = Math.sin(a);
    for (const i of [ua, ...kids(ua)]) {   // 60 deg about world Y through the shoulder, one frame
      const q = (f0 * c.nb + i) * 16;
      for (const col of [0, 4, 8, 12]) { const x = W[q + col] - (col === 12 ? piv[0] : 0), z = W[q + col + 2] - (col === 12 ? piv[2] : 0); W[q + col] = ca * x + sa * z + (col === 12 ? piv[0] : 0); W[q + col + 2] = -sa * x + ca * z + (col === 12 ? piv[2] : 0); }
    }
    const f2 = computeFacts({ W, nb: c.nb, n: c.n, fps: c.fps, upm: c.upm, rig: c.rig, restW: c.restW });
    const fs = priorFindings({ model: MODEL, facts: f2, W, nb: c.nb });
    assert.ok(fs.some(f => /unnatural_shoulder_(snap|speed)/.test(f.rule) && f.a <= f0 + 1 && f.b >= f0 - 1), `${file}: ${fs.map(f => f.rule).join()}`);
  }
});
