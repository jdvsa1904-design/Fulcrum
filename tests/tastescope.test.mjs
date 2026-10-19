import test from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { join } from 'node:path';
import { BONES, segmentPhases, phasePlan, phaseScope, phasedTasteInput, groupByPhase, ENVELOPE_RULES, GAIT_PHASES } from '../tastebridge.js';

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

test('a level walk stays a walk phase judged with the gait envelope', () => {
  const d = clip(walk(120));
  assert.equal(dominant(d), 'locomotion');
  const plan = phasePlan(segmentPhases(d), d.B.length), loco = plan.find(p => p.phase === 'locomotion');
  assert.ok(loco.rules.includes('envelope'));
  for (const r of ENVELOPE_RULES) assert.ok(loco.judged_with.includes(r), r);
  assert.ok(loco.label.startsWith('judged with: '));
});

test('prone crawl: pelvis low and torso horizontal is a prone phase without gait or foot-drag rules', () => {
  const B = [];
  for (let f = 0; f < 90; f++) {
    const x = 30 * f / FPS, s = Math.sin(2 * Math.PI * f / FPS);
    B.push(frame({ x, z: 18, chest: [45, 0, 3], hand: [[x + 50 + 10 * s, 20, 2], [x + 50 - 10 * s, -20, 2]], lf: [x - 80 + 8 * s, 12, 6], rf: [x - 80 - 8 * s, -12, 6] }));
  }
  const d = clip(B);
  assert.equal(dominant(d), 'prone', phasesOf(d).join(' > '));
  const sc = phaseScope('prone', 90);
  assert.ok(!sc.judged_with.includes('foot_slide') && !sc.judged_with.includes('planted_toe_out_deg'));
  assert.ok(sc.skipped.some(x => x.rule === 'foot_slide' && /drag/.test(x.why)));
  assert.ok(sc.judged_with.includes('plant_pop') && sc.judged_with.includes('pop') && sc.judged_with.includes('ground_sink'));
});

test('crouched walk with bent knees is a crouch phase, not a walk', () => {
  const d = clip(walk(120, { z: 85, speed: 70, stride: 8 }));
  assert.equal(dominant(d), 'crouch', phasesOf(d).join(' > '));
  const plan = phasePlan(segmentPhases(d), d.B.length);
  for (const p of plan.filter(p => p.phase === 'crouch')) {
    assert.ok(!p.rules.includes('envelope') && !p.rules.includes('bounce'));
    assert.deepEqual(p.rules, ['foot_slide', 'plant_pop', 'ground_float']);
    assert.ok(p.skipped.some(x => x.rule === 'knee_flex_max_deg'));
  }
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
  const sc = phaseScope('upper_body', 90);
  assert.deepEqual(sc.judged_with.slice(0, 3), ['foot_slide', 'plant_pop', 'ground_float']);
  assert.ok(sc.skipped.some(x => x.rule === 'bounce') && sc.skipped.some(x => x.rule === 'stride_ratio'));
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
  assert.ok(!phaseScope('freeform', 150).judged_with.includes('min_stance_width'));
});

test('feet off the floor with a hand planted low is a vault, judged only by whole-take rules', () => {
  const B = walk(40);
  const x0 = B[B.length - 1][0][0];
  for (let f = 0; f < 14; f++) {
    const x = x0 + 8 * f;
    B.push(frame({ x, lf: [x + 10, 10, 45], rf: [x - 10, -10, 45], hand: [[x0 + 60, 20, 55], [x + 10, -25, 110]] }));
  }
  for (const r of walk(40)) B.push(r.map(p => [p[0] + x0 + 112, p[1], p[2]]));
  const d = clip(B);
  assert.ok(phasesOf(d).includes('vault'), phasesOf(d).join(' > '));
  const sc = phaseScope('vault', 14);
  assert.deepEqual(sc.judged_with, ['pop', 'crossing', 'freeze', 'ground_sink']);
  assert.ok(sc.skipped.some(x => x.rule === 'ground_float'));
});

test('jump flight lists the ballistic check and every phase reports what was not checked', () => {
  const sc = phaseScope('flight', 12);
  assert.ok(sc.judged_with.includes('ballistic_flight') && !sc.judged_with.includes('ground_float'));
  for (const p of ['prone', 'crouch', 'kneel', 'upper_body', 'freeform', 'vault', 'flight', 'slide', 'stand']) {
    const s = phaseScope(p, 90);
    assert.ok(!GAIT_PHASES.includes(p));
    for (const r of [...ENVELOPE_RULES, 'bounce', 'stride_ratio']) assert.ok(s.skipped.some(x => x.rule === r), `${p} reports ${r} as not checked`);
    assert.ok(s.skipped_label.startsWith('not checked: '));
    assert.ok(s.judged_with.includes('pop') && s.judged_with.includes('crossing') && s.judged_with.includes('ground_sink'), `${p} keeps the whole-take rules`);
  }
  const g = groupByPhase({ phases: [{ phase: 'prone', start: 0, end: 59 }], findings: [] });
  assert.ok(g.groups[0].label.startsWith('judged with:') && g.groups[0].skipped.length > 0, 'server phases without scope get one filled in');
});

const TASTE = fileURLToPath(new URL('../../LocomotionTaste', import.meta.url));
const SERVER = fileURLToPath(new URL('../server.py', import.meta.url));
const PY = process.env.PYTHON || 'python';
const GAIT = new Set([...ENVELOPE_RULES, 'bounce', 'stride_ratio']);
const cmuSkip = !existsSync(join(TASTE, 'data', 'cmu', 'data', '136', '136_09.bvh.gz')) && 'CMU data not found';

function py(args, input, cwd = TASTE) {
  const r = spawnSync(PY, ['-B', ...args], { input, cwd, env: { ...process.env, PYTHONDONTWRITEBYTECODE: '1' }, maxBuffer: 1 << 28, encoding: 'utf8' });
  assert.ok(r.stdout && r.stdout.trim().startsWith('{'), 'python failed: ' + (r.stderr || '').slice(-2000));
  return JSON.parse(r.stdout);
}

const LOAD = 'import json,sys;from taste import human as H;rows={n:f for f,n,d in H.index_rows()};B,R=H.cmu_run(rows[sys.argv[1]],30);print(json.dumps({"fps":30,"B":B.round(2).tolist(),"rest":R.round(2).tolist()}))';
const cmu = name => py(['-c', LOAD, name]);

test('CMU non-locomotion clips get their phases and no gait-envelope findings', { skip: cmuSkip, timeout: 300000 }, () => {
  const cases = { '136_09': 'crouch', '136_13': 'crouch', '90_34': 'prone', '49_04': 'flight', '86_15': 'upper_body', '69_70': 'crouch' };
  for (const [name, want] of Object.entries(cases)) {
    const d = cmu(name), body = phasedTasteInput(d);
    assert.ok(body.phases.some(p => p.phase === want), `${name} has a ${want} phase: ${body.phases.map(p => p.phase).join(' > ')}`);
    const res = py([SERVER, '--taste-phases'], JSON.stringify(body));
    const bad = res.findings.filter(f => GAIT.has(f.rule) && !GAIT_PHASES.includes(f.phase));
    assert.deepEqual(bad.map(f => f.rule + '@' + f.phase), [], `${name}: no gait rule outside walk and run`);
    if (name.startsWith('136')) assert.deepEqual(res.findings.filter(f => GAIT.has(f.rule)).map(f => f.rule), [], `${name}: crouched walk has no envelope false positive`);
  }
});

test('CMU clean walk stays one walk phase and a labelled tiptoe walk is still caught', { skip: cmuSkip, timeout: 300000 }, () => {
  const walkPlan = phasedTasteInput(cmu('16_15'));
  assert.deepEqual(walkPlan.phases.map(p => p.phase), ['locomotion']);
  const tip = phasedTasteInput(cmu('136_05'));
  const res = py([SERVER, '--taste-phases'], JSON.stringify(tip));
  assert.ok(res.findings.some(f => f.rule === 'heel_raise_pct' && f.phase === 'locomotion'), 'tiptoe walk still warns');
});
