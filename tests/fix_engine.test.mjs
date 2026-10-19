import test from 'node:test';
import assert from 'node:assert/strict';
import { rng } from './helpers.mjs';
import { makeTake, harness, rotateLocal, refreshW, smoothstep, qmul, qaxis, NB, NAMES, RIG, UPM } from './fixrig.mjs';
import { computeFacts, rigFromNames, estimateGround, legsOf, LEGACY_MAP, phaseOf } from '../wire/facts.js';
import { judgeMotion, physicsFlags, costOf, scopeByPhase, drivenBones, fidelityGate, findingMagnitude, SEVERITY_WEIGHT } from '../wire/judge.js';
import { planCandidates, applyChange, withoutFixIds, chainRootOffsets, normalizeFinding, changeKey, STAGES, NEW_FIX_TYPES } from '../wire/fixplan.js';
import { runFix, accept, flaggedRanges, describeChange } from '../wire/fixengine.js';
import { PHASES_SOURCE } from '../wire/phaseadapter.js';

const EMPTY = { fixes: [], locks: [], pins: [] };
const sway = f => [0, 100 + 0.5 * Math.sin(2 * Math.PI * f / 45), 0.3 * Math.sin(2 * Math.PI * f / 60)];
const hold = (f, a, b, e) => f < a ? smoothstep((f - (a - e)) / e) : f <= b ? 1 : smoothstep((b + e - f) / e);
const TAKES = {
  stand: () => makeTake({ n: 90, pelvis: sway }),
  slide: () => makeTake({ n: 90, pelvis: sway, ankle: { l: f => [10, 8, 20 * smoothstep((f - 40) / 10)], r: () => [-10, 8, 0] } }),
  float: () => makeTake({ n: 90, pelvis: sway, ankle: { l: f => [10, 8 + 13 * hold(f, 40, 49, 6), 0], r: f => [-10, 8 + 13 * hold(f, 40, 49, 6), 0] } }),
  pop: () => rotateLocal(makeTake({ n: 90, pelvis: sway }), 'spine_03', 45, [1, 0, 0], 40),
  seam: () => makeTake({ n: 120, pelvis: f => [f >= 60 ? 30 : 0, 100 + 0.5 * Math.sin(2 * Math.PI * f / 45), 0], ankle: { l: f => [10 + (f >= 60 ? 30 : 0), 8, 0], r: f => [-10 + (f >= 60 ? 30 : 0), 8, 0] }, shots: [{ name: 'A', start: 0, n: 60 }, { name: 'B', start: 60, n: 60 }] }),
  knee: () => rotateLocal(makeTake({ n: 90, pelvis: sway }), 'calf_l', 50, [1, 0, 0], 25),
  sink: () => makeTake({ n: 90, pelvis: f => [0, 100 - 6 * hold(f, 40, 55, 6), 0], ankle: { l: f => [10, 8 - 6 * hold(f, 40, 55, 6), 0], r: f => [-10, 8 - 6 * hold(f, 40, 55, 6), 0] } }),
  floatyJump: () => makeTake({ n: 90, pelvis: f => [0, 100 + 40 * hold(f, 36, 47, 6), 0], ankle: { l: f => [10, 8 + 40 * hold(f, 36, 47, 6), 0], r: f => [-10, 8 + 40 * hold(f, 36, 47, 6), 0] } }),
  cleanJump: () => { const a = 30, b = 51, T = (b - a) / 30, v0 = 981 * T / 2, h = f => f <= a || f >= b ? 0 : v0 * (f - a) / 30 - 981 * ((f - a) / 30) ** 2 / 2; return makeTake({ n: 90, pelvis: f => [0, 100 + h(f), 0], ankle: { l: f => [10, 8 + h(f), 0], r: f => [-10, 8 + h(f), 0] } }); },
  walk: () => {
    const v = 3, T = 30, st = 18, foot = (f, ph) => { const c = Math.floor((f + ph) / T), u = f + ph - c * T, z = c * v * T; if (u < st) return [z, 8]; const s = (u - st) / (T - st); return [z + v * T * smoothstep(s), 8 + 8 * Math.sin(Math.PI * s)]; };
    return makeTake({ n: 120, pelvis: f => [0, 95 + 1.5 * Math.cos(4 * Math.PI * f / T), v * f - 9 * v], ankle: { l: f => { const p = foot(f, 0); return [10, p[1], p[0]]; }, r: f => { const p = foot(f, T / 2); return [-10, p[1], p[0] - v * T / 2]; } } });
  },
  jitter: () => {
    const t = makeTake({ n: 90, pelvis: sway }), r = rng(7), g = () => { let s = 0; for (let k = 0; k < 6; k++) s += r(); return (s - 3) * Math.SQRT2; };
    const legs = new Set(['thigh_l', 'calf_l', 'foot_l', 'ball_l', 'thigh_r', 'calf_r', 'foot_r', 'ball_r']);
    for (let f = 20; f < 70; f++) for (let i = 0; i < NB; i++) if (!legs.has(NAMES[i])) { const o = (f * NB + i) * 4; t.q.set(qmul(Array.from(t.q.subarray(o, o + 4)), qaxis([g(), g(), g()], 1.5 * Math.PI / 180 * Math.abs(g()))), o); }
    return refreshW(t);
  },
};
const cache = new Map();
function setup(name) {
  if (!cache.has(name)) { const take = TAKES[name](), h = harness(take); cache.set(name, { take, h, j0: h.judge(EMPTY) }); }
  return cache.get(name);
}
const factsOf = take => computeFacts({ W: take.W, nb: NB, n: take.n, fps: take.fps, upm: UPM, rig: RIG, restW: harness(take).restW, shots: take.shots });
const rules = j => j.findings.map(f => `${f.severity}:${f.rule}`);
const has = (j, rule, sev) => j.findings.some(f => f.rule === rule && (!sev || f.severity === sev));
const blockers = j => j.findings.filter(f => f.severity === 'blocker');
async function fix(name, o = {}) {
  const { h } = setup(name), input = structuredClone(EMPTY);
  const r = await runFix({ state: input, apply: h.apply, evaluate: h.evaluate, ...o });
  assert.deepEqual(input, EMPTY, 'runFix must not mutate its input state');
  return r;
}
const keptTypes = r => r.kept.flatMap(k => [...(k.change.fixes || []).map(x => x.type), ...(k.change.locks || []).map(() => 'lock'), ...(k.change.pins || []).map(() => 'pin')]);
function noNewBlockers(r, j0) {
  const before = new Set(blockers(j0).map(f => f.rule));
  for (const f of r.findings) if (f.severity === 'blocker') assert.ok(before.has(f.rule), `new blocker ${f.rule}: ${f.text}`);
  assert.ok(r.end.blockers <= r.start.blockers);
}

// ---------- facts ----------
test('facts: phase code loads through the adapter (phases.js or tastebridge.js)', () => {
  assert.ok(PHASES_SOURCE === 'phases.js' || PHASES_SOURCE === 'tastebridge.js', String(PHASES_SOURCE));
});

test('facts: clean stand gives one ground, one lockable plant per foot, a stand phase and COM over the feet', () => {
  const { j0 } = setup('stand'), F = j0.facts;
  assert.ok(Math.abs(F.ground - 1) < 0.05 && Math.abs(F.groundLegacy - 1) < 0.05, `${F.ground} ${F.groundLegacy}`);
  for (const s of ['l', 'r']) {
    const ps = F.contacts.feet[s].plants;
    assert.equal(ps.length, 1);
    assert.deepEqual([ps[0].a, ps[0].b, ps[0].lockable, ps[0].sliding], [0, 89, true, false]);
    assert.ok(F.contacts.feet[s].mask.every(m => m === 1));
    assert.ok(Math.abs(F.contacts.feet[s].level.ball_cm) < 0.5 && Math.abs(F.contacts.feet[s].level.foot_cm - 7) < 0.5);
  }
  assert.deepEqual(F.phases.map(p => p.phase), ['stand']);
  assert.ok(F.contacts.feetDown.every(Boolean) && F.flights.length === 0 && F.seams.length === 0);
  assert.ok(Array.from(F.balance.marginCm).every(m => m > 5), 'COM inside the support polygon');
  assert.ok(Math.abs(F.knees.l.flex[0] - 29.7) < 0.5 && F.trust.every(v => v === 1) && F.missing.length === 0);
  assert.ok(Math.abs(F.legLength_cm - 90) < 1e-6 && F.footLength_cm > 20);
  assert.equal(j0.findings.length, 0, rules(j0).join(', '));
});

test('facts: the shared ground ignores a long sink; the legacy 3rd-percentile floor sinks with it', () => {
  const F = factsOf(TAKES.sink());
  assert.ok(Math.abs(F.ground - 1) < 0.2, `ground ${F.ground}`);
  assert.ok(F.groundLegacy < -4, `legacy ${F.groundLegacy}`);
  const g = estimateGround({ W: TAKES.stand().W, nb: NB, n: 90, fps: 30, upm: UPM, legs: legsOf(RIG) });
  assert.equal(g.method, 'stance-band');
});

test('facts: a jump is a plausible flight in a flight phase; seams come from shot boundaries; trust is clamped', () => {
  const F = factsOf(TAKES.cleanJump());
  assert.equal(F.flights.length, 1);
  assert.ok(F.flights[0].plausible && F.flights[0].flightPhase > 0.8 && Math.abs(F.flights[0].report.airtime_s - 0.7) < 0.1);
  assert.equal(phaseOf(F, 35, 45), 'flight');
  const S = factsOf(TAKES.seam());
  assert.deepEqual(S.seams.map(s => [s.at, s.name, s.prev]), [[60, 'B', 'A']]);
  assert.ok(S.nearSeam[60] && S.nearSeam[54] && !S.nearSeam[50]);
  const t = TAKES.stand(), T = computeFacts({ W: t.W, nb: NB, n: 90, fps: 30, upm: UPM, rig: RIG, trust: Array.from({ length: 90 }, (_, f) => f === 3 ? 5 : f === 4 ? -1 : f === 5 ? NaN : 0.5) });
  assert.deepEqual([T.trust[3], T.trust[4], T.trust[5], T.trust[6]], [1, 0, 1, 0.5]);
});

test('facts: hands on the floor are contacts with an anchor and slip; rig names resolve through namespaces', () => {
  const t = TAKES.stand(), W = Float32Array.from(t.W), hi = RIG.hand_l;
  for (let f = 20; f <= 40; f++) { const o = (f * NB + hi) * 16 + 12; W[o] = 40 + (f - 20) * 0.3; W[o + 1] = 4; W[o + 2] = 30; }
  const F = computeFacts({ W, nb: NB, n: 90, fps: 30, upm: UPM, rig: RIG });
  const r = F.contacts.hands.l.runs;
  assert.equal(r.length, 1);
  assert.ok(r[0].a <= 21 && r[0].b >= 39 && Math.abs(r[0].slip_cm - 6) < 0.7, JSON.stringify(r[0]));
  assert.ok(F.contacts.support[30] & 4);
  const j = judgeMotion({ motion: { W }, nb: NB, n: 90, fps: 30, upm: UPM, rig: RIG, names: NAMES, facts: F });
  assert.ok(has(j, 'hand_slip', 'warning'));
  const R = rigFromNames(NAMES.map(n => 'Mixamo:Rig|' + n.toUpperCase()));
  assert.deepEqual(R, RIG);
  assert.ok(Object.keys(LEGACY_MAP).length >= 8);
});

// ---------- judge: no false positives, then each injected defect ----------
test('judge: clean stand, clean walk and a clean ballistic jump have no findings', () => {
  for (const name of ['stand', 'walk', 'cleanJump']) assert.deepEqual(rules(setup(name).j0), [], name);
  assert.deepEqual(setup('walk').j0.facts.contacts.feet.l.plants.map(p => [p.a, p.b]), [[0, 18], [30, 48], [60, 78], [90, 108]]);
});

test('judge: physicsFlags is analyze(); its 10 cm contact and the landing smear are why the judge narrows them', () => {
  const { j0: walk } = setup('walk'), F = walk.facts, major = ['spine_03'].map(k => RIG[k]);
  const raw = physicsFlags({ com: F.com, contact: { any: F.contacts.feetDown, per: F.contacts.per, speed: F.contacts.speed }, q: null, nb: NB, n: F.n, fps: 30, upm: UPM, major });
  assert.ok(raw.flags.some(x => x.kind === 'slide'), 'analyze() reports low swing feet as sliding');
  const narrowed = judgeMotion({ motion: { W: setup('walk').take.W }, nb: NB, n: F.n, fps: 30, upm: UPM, rig: RIG, names: NAMES, facts: F, opts: { slideContact: 'height' } });
  assert.ok(has(narrowed, 'foot_slide', 'blocker'));
  const J = setup('cleanJump').j0.facts;
  const land = physicsFlags({ com: J.com, contact: { any: J.contacts.feetDown, per: J.contacts.per, speed: J.contacts.speed }, q: null, nb: NB, n: J.n, fps: 30, upm: UPM, edgeGuard: 0 });
  assert.ok(land.flags.some(x => x.kind === 'float'), 'exact analyze() flags the landing of a perfect arc');
  const guarded = physicsFlags({ com: J.com, contact: { any: J.contacts.feetDown, per: J.contacts.per, speed: J.contacts.speed }, q: null, nb: NB, n: J.n, fps: 30, upm: UPM, edgeGuard: 3 });
  assert.ok(!guarded.flags.some(x => x.kind === 'float'));
});

test('judge flags: foot slide, float, pop, seam jump, knee pop, penetration, floaty jump, jitter', () => {
  const at = (name, rule, sev, side, a, b) => {
    const j = setup(name).j0, f = j.findings.find(x => x.rule === rule && x.severity === sev);
    assert.ok(f, `${name}: no ${sev} ${rule} in ${rules(j)}`);
    if (side !== undefined) assert.equal(f.side, side, name);
    if (a !== undefined) assert.ok(f.a <= b && f.b >= a, `${name} ${rule} at ${f.a}-${f.b}, want overlap ${a}-${b}`);
    assert.ok(Number.isFinite(f.value) && Number.isFinite(f.limit) && f.value > f.limit && f.phase && f.text.length > 10, JSON.stringify(f));
    return f;
  };
  at('slide', 'foot_slide', 'blocker', 'l', 40, 50);
  at('slide', 'foot_skate', 'warning', 'l', 0, 89);
  assert.ok(setup('slide').j0.facts.contacts.feet.l.plants[0].drift_cm > 19);
  assert.equal(at('float', 'ground_float', 'blocker', null, 40, 49).phase === 'flight', false);
  assert.equal(at('pop', 'pop', 'blocker', undefined, 45, 46).bone, 'spine_03');
  at('seam', 'seam_jump', 'blocker', null, 59, 60);
  assert.ok(Math.abs(setup('seam').j0.findings.find(x => x.rule === 'seam_jump').value - 30) < 0.5);
  const k = at('knee', 'knee_pop', 'blocker', 'l', 50, 50);
  assert.ok(Math.abs(k.value - 25) < 1 && k.a === 50 && k.b === 50);
  for (const s of ['l', 'r']) assert.ok(setup('sink').j0.findings.some(f => f.rule === 'ground_sink' && f.side === s && f.value > 5.5 && f.a <= 40 && f.b >= 55));
  assert.equal(at('floatyJump', 'ground_float', 'blocker', null, 36, 47).phase, 'flight');
  at('jitter', 'jitter', 'warning', null, 20, 69);
});

test('judge: fidelity gate leaves driven bones out, scales by trust and splits rotation from pelvis penalty', () => {
  const { take } = setup('stand'), q = Float32Array.from(take.q), src = Float32Array.from(take.q), n = take.n;
  for (let f = 0; f < n; f++) { const o = (f * NB + RIG.calf_l) * 4; q.set(qmul(Array.from(q.subarray(o, o + 4)), qaxis([1, 0, 0], 0.2)), o); const s = (f * NB + RIG.spine_03) * 4; q.set(qmul(Array.from(q.subarray(s, s + 4)), qaxis([0, 1, 0], 0.05)), s); }
  const base = { q, srcQ: src, p: take.p, srcP: take.p, nb: NB, n, names: NAMES, rig: RIG, upm: UPM };
  const g0 = fidelityGate(base);
  assert.equal(g0.worstBone, 'calf_l'); assert.ok(Math.abs(g0.worst - 11.46) < 0.05 && !g0.pass);
  const g1 = fidelityGate({ ...base, state: { locks: [{ side: 'l', a: 0, b: 89 }] } });
  assert.equal(g1.worstBone, 'spine_03'); assert.ok(Math.abs(g1.worst - 2.86) < 0.05 && g1.pass);
  const g2 = fidelityGate({ ...base, state: { locks: [{ side: 'l', a: 0, b: 89 }] }, trust: new Float32Array(n).fill(0.5) });
  assert.ok(Math.abs(g2.worst - 1.43) < 0.05 && g2.rotPenalty < g1.rotPenalty && g2.pelvisPenalty === 0);
  assert.ok(drivenBones({ fixes: [{ type: 'knee', side: 'r' }, { type: 'level', side: 'l' }] }, RIG).has(RIG.calf_r));
});

test('judge: cost weighs severity, how far past the limit and how long, not just counts', () => {
  const f = (severity, value, limit, a = 0, b = 0) => ({ rule: 'x', severity, value, limit, a, b });
  assert.equal(costOf([f('blocker', 1, 1)]), SEVERITY_WEIGHT.blocker);
  assert.equal(costOf([f('warning', 3, 1)]), SEVERITY_WEIGHT.warning * 3);
  assert.equal(costOf([f('info', 50, 1, 0, 3)]), SEVERITY_WEIGHT.info * 10 * 2);
  assert.ok(costOf([f('blocker', 2, 1)]) > costOf([f('blocker', 1.2, 1)]));
  assert.equal(findingMagnitude(f('blocker', null, null)), 1);
});

test('judge: phase scope drops airborne foot rules and softens slides in a slide phase', () => {
  const facts = { n: 30, phaseAt: Array.from({ length: 30 }, (_, f) => f < 10 ? 'flight' : f < 20 ? 'slide' : 'stand') };
  const fs = [['foot_slide', 2, 5], ['foot_slide', 12, 15], ['foot_slide', 22, 25], ['pop', 3, 3]].map(([rule, a, b]) => ({ rule, severity: 'blocker', a, b, text: rule }));
  const out = scopeByPhase(fs, facts);
  assert.deepEqual(out.map(x => [x.rule, x.phase, x.severity]), [['foot_slide', 'slide', 'info'], ['foot_slide', 'stand', 'blocker'], ['pop', 'flight', 'blocker']]);
});

// ---------- planner ----------
test('plan: the phase picks the solver (flight -> ballistic, stand -> ground, slide phase -> moving ankle lock)', () => {
  const jump = setup('floatyJump'), fl = jump.j0.findings.filter(f => f.rule === 'ground_float');
  const body = planCandidates({ findings: fl, facts: jump.j0.facts, state: EMPTY, stage: 'body' });
  assert.equal(body[0].change.fixes[0].type, 'ballistic');
  assert.equal(planCandidates({ findings: fl, facts: jump.j0.facts, state: EMPTY, stage: 'contacts' }).length, 0);
  const hover = setup('float'), hc = planCandidates({ findings: hover.j0.findings, facts: hover.j0.facts, state: EMPTY, stage: 'contacts' });
  assert.deepEqual(hc[0].change.fixes.map(x => [x.type, x.side]), [['ground', 'l'], ['ground', 'r']]);
  assert.equal(planCandidates({ findings: hover.j0.findings, facts: hover.j0.facts, state: EMPTY, stage: 'body' }).length, 0);
  const sl = setup('slide'), f = sl.j0.findings.find(x => x.rule === 'foot_slide');
  const stand = planCandidates({ findings: [{ ...f, phase: 'stand' }], facts: sl.j0.facts, state: EMPTY, stage: 'contacts' })[0];
  assert.deepEqual(stand.change.locks, [{ side: 'l', a: 0, b: 89 }]);
  assert.ok(stand.variants.some(v => v.fixes?.[0].type === 'ankleLock'), 'moving-anchor lock is a variant');
  const slidePhase = planCandidates({ findings: [{ ...f, phase: 'slide' }], facts: sl.j0.facts, state: EMPTY, stage: 'contacts' })[0];
  assert.equal(slidePhase.change.fixes[0].type, 'ankleLock'); assert.ok(slidePhase.change.fixes[0].maxSlide > 0);
  assert.equal(planCandidates({ findings: [{ ...f, phase: 'flight' }], facts: sl.j0.facts, state: EMPTY, stage: 'contacts' }).length, 0);
});

test('plan: stages and existing fix formats (despike, smooth, seams, floor, knee, filter with parameter variants)', () => {
  const pop = setup('pop').j0;
  assert.deepEqual(planCandidates({ findings: pop.findings, facts: pop.facts, state: EMPTY, stage: 'noise' })[0].change.fixes, [{ type: 'despike', a: 43, b: 48, bones: 'all' }]);
  assert.equal(planCandidates({ findings: pop.findings, facts: pop.facts, state: EMPTY, stage: 'polish' })[0].change.fixes[0].type, 'smooth');
  const seam = setup('seam').j0;
  assert.deepEqual(planCandidates({ findings: seam.findings, facts: seam.facts, state: EMPTY, stage: 'structure' })[0].change.fixes, [{ type: 'seams', blend: 6, mode: 'carry', force: false }]);
  const sink = setup('sink').j0, fc = planCandidates({ findings: sink.findings, facts: sink.facts, state: EMPTY, stage: 'contacts' })[0];
  assert.deepEqual(Object.keys(fc.change.fixes[0]).sort(), ['a', 'b', 'pivot', 'side', 'stiffness', 'type']);
  assert.ok(fc.variants.some(v => v.fixes[0].pivot === 'toes') && fc.variants.some(v => v.fixes[0].stiffness === 0.5));
  const kn = setup('knee').j0, kc = planCandidates({ findings: kn.findings, facts: kn.facts, state: EMPTY, stage: 'noise' })[0];
  assert.deepEqual(kc.change.fixes, [{ type: 'knee', side: 'l', a: 50, b: 50, mode: 'spike' }]);
  assert.ok(kc.variants.some(v => v.fixes[0].type === 'despike' && Array.isArray(v.fixes[0].bones)));
  const jt = setup('jitter').j0, jc = planCandidates({ findings: jt.findings, facts: jt.facts, state: EMPTY, stage: 'noise' }).find(c => c.rule === 'jitter');
  assert.deepEqual([jc.change, ...jc.variants].map(v => v.fixes[0].fc), [8, 6, 4]);
  assert.ok(jc.change.fixes.every(x => x.type === 'filter' && Array.isArray(x.bones)));
  assert.ok(!jc.change.fixes[0].bones.includes(RIG.calf_l), 'planted legs are not filtered');
  assert.deepEqual(STAGES, ['preflight', 'noise', 'structure', 'body', 'contacts', 'polish', 'stop']);
  assert.ok(NEW_FIX_TYPES.knee && NEW_FIX_TYPES.ankleLock);
});

test('plan: taste-style findings normalise and map like the old autofix planner', () => {
  const f = normalizeFinding({ rule: 'ground_float', body_part: 'left foot', severity: 'warn', frame_start: 10, frame_end: 14, phase: 'stand' });
  assert.deepEqual([f.rule, f.side, f.severity, f.a, f.b], ['ground_float', 'l', 'warning', 10, 14]);
  const facts = setup('stand').j0.facts;
  const c = planCandidates({ findings: [f], facts, state: EMPTY, stage: 'contacts' });
  assert.deepEqual(c[0].change.fixes, [{ type: 'ground', a: 7, b: 17, side: 'l' }]);
  const roll = planCandidates({ findings: [{ rule: 'heel_raise_pct', severity: 'warning', a: 10, b: 20, phase: 'stand' }], facts, state: { fixes: [], locks: [{ side: 'r', a: 0, b: 40 }], pins: [] }, stage: 'contacts' });
  assert.deepEqual(roll[0].change.fixes, [{ type: 'roll', side: 'r', a: 0, b: 40, start: 30, end: 60 }]);
  const tr = planCandidates({ findings: [{ rule: 'stride_ratio', severity: 'warning', a: 0, b: 60, phase: 'locomotion' }], facts, state: EMPTY, stage: 'body' });
  assert.ok(tr[0].change.fixes[0].type === 'travel' && tr[0].change.fixes[0].footsteps && tr[0].variants.length === 2);
});

test('plan: applyChange is pure, tags every added item and a muted layer drops exactly its items', () => {
  const st = { fixes: [{ type: 'level', side: null }], locks: [{ side: 'l', a: 0, b: 9 }], pins: [] };
  const snap = structuredClone(st);
  const a = applyChange(st, { fixes: [{ type: 'despike', a: 1, b: 5 }], locks: [{ side: 'r', a: 3, b: 8 }], dropLocks: [{ side: 'l', a: 0, b: 9 }] }, { id: 'fix-1' });
  const b = applyChange(a, { edits: [{ index: 0, patch: { side: 'l' } }], pins: [{ a: 0, b: 3, ref: 1, world: new Array(16).fill(0) }] }, { id: 'fix-2' });
  assert.deepEqual(st, snap);
  assert.deepEqual(a.locks, [{ side: 'r', a: 3, b: 8, fixId: 'fix-1' }]);
  assert.deepEqual(b.fixes.map(x => [x.type, x.fixId ?? null, x.side ?? null]), [['level', 'fix-2', 'l'], ['despike', 'fix-1', null]]);
  const m = withoutFixIds(b, ['fix-1']);
  assert.deepEqual([m.fixes.length, m.locks.length, m.pins.length], [1, 0, 1]);
  assert.equal(changeKey({ fixes: [{ type: 'x', fixId: 'a', hits: 3 }] }), changeKey({ fixes: [{ type: 'x' }] }));
});

test('plan: root offsets are chained (each solver sees the previous one), not summed from one pass', () => {
  const n = 4, base = 10;
  const pass = acc => Float64Array.from({ length: n }, (_, f) => base + (acc[f]?.[0] ?? 0));
  const shift = { name: 'seams', fn: () => Array.from({ length: n }, () => [5, 0, 0]) };
  const recentre = { name: 'fulcrum', fn: W => Array.from(W, x => [-x, 0, 0]) };
  const r = chainRootOffsets({ n, producers: [recentre, shift], pass });
  assert.deepEqual(r.used, ['seams', 'fulcrum']);
  assert.ok(Array.from(r.W).every(x => Math.abs(x) < 1e-12), 'chained: the pivot solve sees the seam shift');
  const p1 = pass(new Array(n).fill(null)), summed = Array.from({ length: n }, (_, f) => base + 5 - p1[f]);
  assert.ok(summed.every(x => x === 5), 'summed from one pass double-corrects');
});

// ---------- engine: real defects, pure test apply() ----------
const DEFECTS = [
  ['slide', 'foot_slide', 'lock', 'locks the plant through the slide'],
  ['float', 'ground_float', 'ground', 'grounds both floating feet'],
  ['pop', 'pop', 'despike', 'despikes the single-frame pop'],
  ['seam', 'seam_jump', 'seams', 'matches the shot seam'],
  ['knee', 'knee_pop', 'knee', 'removes the knee pop with the knee type'],
  ['sink', 'ground_sink', 'floor', 'lifts the sunk feet with floor contact'],
  ['floatyJump', 'ground_float', 'ballistic', 'puts the floaty jump on a ballistic arc'],
  ['jitter', 'jitter', 'filter', 'filters the jitter'],
];
for (const [name, rule, type, what] of DEFECTS) {
  test(`runFix ${name}: ${what}, no new blockers, stops before export`, async () => {
    const { j0 } = setup(name), r = await fix(name);
    assert.ok(has(j0, rule), `${name} must start with ${rule}`);
    assert.ok(!r.findings.some(f => f.rule === rule && f.severity !== 'info'), `${rule} left: ${rules(r.judgement)}`);
    assert.equal(r.end.blockers, 0, rules(r.judgement).join(', '));
    noNewBlockers(r, j0);
    assert.ok(keptTypes(r).includes(type), `${name} kept ${keptTypes(r)}`);
    assert.ok(r.judgement.fidelity.worst <= 8, `fidelity ${r.judgement.fidelity.worst}`);
    assert.ok(r.report.some(l => l.startsWith('Stopped before export')) && r.report.at(-1).startsWith('Done:'));
    for (const k of r.kept) {
      assert.ok(k.id.startsWith('fix-') && k.label && k.detail && k.after.score < k.before.score);
      const tagged = [...r.state.fixes, ...r.state.locks, ...r.state.pins].filter(x => x.fixId === k.id);
      assert.ok(tagged.length >= 1, `items tagged ${k.id}`);
    }
    assert.ok(r.trials <= 80 && r.flagged.length === 0);
  });
}

test('runFix: the filter parameter search keeps the best-scoring cut-off of 8, 6 and 4 Hz', async () => {
  const { h, j0 } = setup('jitter'), r = await fix('jitter');
  const k = r.kept.find(x => x.rule === 'jitter'), fc = k.change.fixes[0].fc;
  const cand = planCandidates({ findings: j0.findings, facts: j0.facts, state: EMPTY, stage: 'noise' }).find(c => c.rule === 'jitter');
  const scores = [cand.change, ...cand.variants].map(ch => { const s = applyChange(EMPTY, ch); const j = h.evaluate(s, h.apply(s)); return { fc: ch.fixes[0].fc, score: j.score, ok: accept(j, j0).ok }; });
  const best = scores.filter(x => x.ok).sort((a, b) => a.score - b.score)[0];
  assert.equal(fc, best.fc, JSON.stringify(scores));
});

test('runFix: report is plain English, budget is respected and a clean take is left alone', async () => {
  const r = await fix('stand');
  assert.deepEqual([r.kept.length, r.reverted.length, r.trials, r.end.blockers, r.end.warnings], [0, 0, 0, 0, 0]);
  const s = await fix('seam', { maxTrials: 1 });
  assert.equal(s.trials, 1); assert.equal(s.reason, 'max_trials');
  assert.ok(s.report.some(l => /budget/.test(l)));
  const p = await fix('pop');
  assert.ok(p.report.some(l => /^Noise: Pose snaps .* Remove the pop at f45-46 \(despike f43-48\)\.$/.test(l)), p.report.join('\n'));
});

// ---------- engine: keep/revert rules on a toy judge ----------
function toy(defects, effects) {
  // effects: fix type -> { removes: [rule], adds: [finding], fid: {worst, penalty, pelvisCm} }
  const apply = st => st;
  const evaluate = st => {
    let fs = defects.map(d => ({ ...d })), fid = { worst: 0, rotPenalty: 0, penalty: 0, pelvisCm: 0 };
    for (const x of [...st.fixes, ...st.locks.map(() => ({ type: 'lock' }))]) {
      const e = effects[x.fc ? `${x.type}:${x.fc}` : x.type];
      if (!e) continue;
      fs = fs.filter(f => !(e.removes || []).includes(f.rule)).concat((e.adds || []).map(f => ({ ...f })));
      if (e.fid) fid = { ...fid, ...e.fid, penalty: (e.fid.rotPenalty ?? 0) + (e.fid.pelvisCm ?? 0) * 0.5 };
    }
    return { findings: fs, cost: costOf(fs), fidelity: fid };
  };
  return { apply, evaluate };
}
const toyPlan = cands => ({ stage }) => cands.filter(c => c.stage === stage).map(c => ({ variants: [], priority: 1, severity: 'blocker', finding: { rule: c.rule, text: c.rule, a: 0, b: 0 }, label: c.rule, ...c, key: changeKey(c.change) }));

test('runFix rules: revert a change that adds a blocker or breaks a lock; keep the one that helps', async () => {
  const pop = { rule: 'pop', severity: 'blocker', a: 5, b: 5, value: 2, limit: 1 };
  const t = toy([pop], { despike: { removes: ['pop'], adds: [{ rule: 'foot_slide', severity: 'blocker', a: 1, b: 1 }, { rule: 'lock_residual', severity: 'blocker', a: 1, b: 1 }] }, median: { removes: ['pop'] } });
  const r = await runFix({ state: EMPTY, ...t, plan: toyPlan([{ stage: 'noise', rule: 'pop', change: { fixes: [{ type: 'despike', a: 3, b: 7 }] } }, { stage: 'noise', rule: 'pop', change: { fixes: [{ type: 'median', a: 3, b: 7 }] } }]) });
  assert.deepEqual(r.state.fixes.map(x => x.type), ['median']);
  assert.equal(r.reverted.length, 1); assert.match(r.reverted[0].tried[0].why, /raised blockers/);
  assert.equal(r.end.blockers, 0);
});

test('runFix rules: fidelity cap and gate limit reject over-edits; pelvis moves only count against the score', async () => {
  const fl = { rule: 'ground_float', severity: 'blocker', a: 0, b: 9, value: 1, limit: 1 };
  const plan = toyPlan([{ stage: 'body', rule: 'ground_float', change: { fixes: [{ type: 'ballistic', a: 0, b: 9 }] }, variants: [{ fixes: [{ type: 'momentum', a: 0, b: 9 }] }, { fixes: [{ type: 'ground', a: 0, b: 9 }] }] }]);
  const t = toy([fl], { ballistic: { removes: ['ground_float'], fid: { worst: 9.5, rotPenalty: 10 } }, momentum: { removes: ['ground_float'], fid: { worst: 3, rotPenalty: 20 } }, ground: { removes: ['ground_float'], fid: { worst: 1, rotPenalty: 2, pelvisCm: 30 } } });
  const r = await runFix({ state: EMPTY, ...t, plan });
  assert.deepEqual(r.state.fixes.map(x => x.type), ['ground']);
  assert.ok(r.kept[0].variant);
  const why = r.kept[0].rejected.map(x => x.detail + ': ' + x.why);
  assert.match(why[0], /^ballistic arc f0-9: .*past the fidelity limit/, 'ballistic moves an untouched joint past the 8 deg gate');
  assert.match(why[1], /^momentum f0-9: .*untouched joints too much/, 'momentum raises the rotation penalty past the cap');
  const strict = await runFix({ state: EMPTY, ...t, plan, caps: { pelvisRiseCm: 10 } });
  assert.deepEqual(strict.state.fixes, []);
  assert.match(strict.reverted[0].tried.map(x => x.why).join(' | '), /pelvis 30 cm/);
});

test('runFix rules: warnings are worked after blockers, info only on request; flagged ranges merge', async () => {
  const b = { rule: 'pop', severity: 'blocker', a: 50, b: 50, value: 2, limit: 1 }, w = { rule: 'foot_skate', severity: 'warning', a: 10, b: 20, value: 6, limit: 4 };
  const t = toy([w, b], { lock: { removes: ['foot_skate'] }, despike: { removes: ['pop'] } });
  const plan = ({ findings, stage }) => stage !== 'contacts' ? [] : findings.map(f => f.rule === 'foot_skate'
    ? { stage, rule: f.rule, finding: f, severity: f.severity, label: 'lock', change: { locks: [{ side: 'l', a: 10, b: 20 }] }, variants: [], key: 'k1' }
    : { stage, rule: f.rule, finding: f, severity: f.severity, label: 'despike', change: { fixes: [{ type: 'despike', a: 48, b: 52 }] }, variants: [], key: 'k2' });
  const r = await runFix({ state: EMPTY, ...t, plan });
  assert.deepEqual(r.kept.map(k => k.rule), ['pop', 'foot_skate'], 'blocker first, then the warning');
  assert.deepEqual([r.end.blockers, r.end.warnings], [0, 0]);
  const facts = setup('slide').j0.facts, info = [{ rule: 'foot_skate', severity: 'info', side: 'l', a: 0, b: 89, phase: 'stand' }];
  assert.equal(planCandidates({ findings: info, facts, state: EMPTY, stage: 'contacts' }).length, 0);
  assert.ok(planCandidates({ findings: info, facts, state: EMPTY, stage: 'contacts', includeInfo: true }).length > 0);
  assert.deepEqual(flaggedRanges([{ rule: 'a', severity: 'warning', a: 1, b: 3 }, { rule: 'b', severity: 'blocker', a: 3, b: 6 }, { rule: 'c', severity: 'info', a: 9, b: 9 }]), [{ a: 1, b: 6, severity: 'blocker', rules: ['a', 'b'] }]);
  assert.equal(describeChange({ dropLocks: [{ side: 'l', a: 0, b: 4 }], fixes: [{ type: 'ground', side: 'l', a: 0, b: 4 }] }), 'unlock left f0-4, ground left foot f0-4');
});
