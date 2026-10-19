import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import { BONES, boneIndices, tasteInput, summarize, segmentPhases, phasePlan, phasedTasteInput, groupByPhase, mergeShort, INTEGRITY_RULES, ENVELOPE_MIN_FRAMES, markInPlace, phaseRules } from '../tastebridge.js';

const mat = (x, y, z) => { const m = new Array(16).fill(0); m[0] = m[5] = m[10] = m[15] = 1; m[12] = x; m[13] = y; m[14] = z; return m; };

test('bone mapping reports missing bones and keeps taste order', () => {
  const have = new Map(BONES.slice(0, 20).map((b, i) => [b, i]));
  const r = boneIndices(k => have.get(k));
  assert.deepEqual(r.missing, BONES.slice(20));
  assert.equal(r.map.length, 20);
});

test('tasteInput converts Y-up metres-scale units to Z-up centimetres with (x,-z,y)', () => {
  const nb = BONES.length, n = 2, upm = 50, W = new Float32Array(n * nb * 16);
  for (let f = 0; f < n; f++) for (let i = 0; i < nb; i++) { const m = mat(1 + i, 2 + f, 3); W.set(m, (f * nb + i) * 16); }
  const map = BONES.map((_, i) => i), restW = BONES.map((_, i) => mat(i, 0, 0));
  const t = tasteInput({ W, nb, n, fps: 24, upm, map, restW });
  assert.equal(t.fps, 24);
  assert.equal(t.B.length, 2);
  assert.equal(t.B[1].length, nb);
  assert.deepEqual(t.B[1][0], [2, -6, 6]);
  assert.deepEqual(t.rest[3], [6, 0, 0]);
  const part = tasteInput({ W, nb, n, fps: 24, upm, map, restW, from: 1, to: 1 });
  assert.equal(part.B.length, 1);
  assert.equal(part.frame_offset, 1);
});

test('summarize ranks blockers first', () => {
  const s = summarize({ verdict: 'fail', confidence: 0.6, findings: [{ severity: 'info' }, { severity: 'blocker' }, { severity: 'warn' }] });
  assert.equal(s.findings[0].severity, 'blocker');
  assert.equal(s.blockers.length, 1);
});

const TAKE = new URL('../exports/taste_in.json', import.meta.url);

function synthRest() {
  const r = BONES.map(() => [0, 0, 120]);
  r[0] = [0, 0, 100]; r[15] = [10, 0, 50]; r[19] = [-10, 0, 50];
  r[16] = [10, 0, 9]; r[17] = [10, -15, 1]; r[20] = [-10, 0, 9]; r[21] = [-10, -15, 1];
  return r;
}

function synthFrame({ px = 0, pz = 100, lz = 0, rz = 0, lx = 0, rx = 0, kl = 50, kr = 50 }) {
  const r = BONES.map(() => [px, 0, pz + 40]);
  r[0] = [px, 0, pz];
  r[15] = [lx, 10, kl]; r[16] = [lx, 10, lz + 8]; r[17] = [lx, -5, lz];
  r[19] = [rx, -10, kr]; r[20] = [rx, -10, rz + 8]; r[21] = [rx, -25, rz];
  return r;
}

function synthDrill() {
  const fps = 24, B = [];
  let x = 0;
  for (let f = 0; f < 40; f++) B.push(synthFrame({ px: x, lx: x, rx: x }));
  for (let f = 0; f < 60; f++) {
    const s = Math.sin(2 * Math.PI * 1.5 * f / fps);
    x += 120 / fps;
    B.push(synthFrame({ px: x, pz: 98, lz: Math.max(0, 15 * s), rz: Math.max(0, -15 * s), lx: x + 20 * s, rx: x - 20 * s }));
  }
  for (let f = 0; f < 15; f++) { x += 200 / fps; B.push(synthFrame({ px: x, pz: 100 + 18 * Math.sin(Math.PI * f / 14), lz: 30, rz: 30, lx: x, rx: x })); }
  for (let f = 0; f < 10; f++) { x += 150 / fps; B.push(synthFrame({ px: x, pz: 100 - 4.5 * (f + 1), lz: 4, rz: 8, lx: x + 30, rx: x - 30, kl: 50, kr: 50 - 4 * (f + 1) })); }
  for (let f = 0; f < 50; f++) B.push(synthFrame({ px: x, pz: 55, lz: 1, rz: 10, lx: x + 30, rx: x - 30, kl: 45, kr: 8 }));
  for (let f = 0; f < 15; f++) B.push(synthFrame({ px: x, pz: 55 + 3 * (f + 1), lz: 1, rz: 4, lx: x + 30, rx: x - 30, kl: 50, kr: 8 + 2.8 * (f + 1) }));
  for (let f = 0; f < 50; f++) B.push(synthFrame({ px: x, lx: x + 10, rx: x - 10 }));
  return { fps, B, rest: synthRest(), frame_offset: 0 };
}

const order = segs => segs.map(s => s.phase);
const coverOk = (segs, n) => {
  assert.equal(segs[0].start, 0);
  assert.equal(segs[segs.length - 1].end, n - 1);
  for (let i = 1; i < segs.length; i++) assert.equal(segs[i].start, segs[i - 1].end + 1);
};
const isSubsequence = (want, have) => { let i = 0; for (const h of have) if (h === want[i]) i++; return i === want.length; };

test('mergeShort folds fragments under 6 frames into neighbours', () => {
  const m = mergeShort([{ phase: 'stand', start: 0, end: 19 }, { phase: 'plant', start: 20, end: 22 }, { phase: 'stand', start: 23, end: 40 }, { phase: 'flight', start: 41, end: 43 }, { phase: 'locomotion', start: 44, end: 80 }]);
  assert.deepEqual(m.map(s => [s.phase, s.start, s.end]), [['stand', 0, 43], ['locomotion', 44, 80]]);
});

test('segmentPhases labels a synthetic stand, walk, jump, drop, kneel, rise, stand', () => {
  const d = synthDrill(), n = d.B.length, segs = segmentPhases(d);
  coverOk(segs, n);
  assert.ok(segs.every(s => s.end - s.start + 1 >= 6), 'no fragment under 6 frames');
  assert.ok(isSubsequence(['stand', 'locomotion', 'flight', 'kneel', 'rise', 'stand'], order(segs)), order(segs).join(' > '));
  const flight = segs.find(s => s.phase === 'flight'), kneel = segs.find(s => s.phase === 'kneel');
  assert.ok(flight.start >= 98 && flight.start <= 104 && flight.end >= 110 && flight.end <= 120, `flight ${flight.start}-${flight.end}`);
  assert.ok(kneel.start >= 118 && kneel.start <= 130 && kneel.end >= 170 && kneel.end <= 182, `kneel ${kneel.start}-${kneel.end}`);
  assert.ok(segs.some(s => (s.phase === 'slide' || s.phase === 'plant') && s.start >= 110 && s.end <= 130), 'drop into the kneel is a slide or plant: ' + order(segs).join(' > '));
});

test('segmentPhases on a flat standing clip is one stand phase', () => {
  const B = Array.from({ length: 60 }, () => synthFrame({}));
  assert.deepEqual(order(segmentPhases({ fps: 24, B, rest: synthRest() })), ['stand']);
});

test('segmentPhases on the FRT drill export finds slide, kneel, rise in order and no fake flight', { skip: !existsSync(TAKE) && 'exports/taste_in.json missing' }, () => {
  const d = JSON.parse(readFileSync(TAKE, 'utf8')), n = d.B.length, segs = segmentPhases(d);
  coverOk(segs, n);
  assert.ok(segs.every(s => s.end - s.start + 1 >= 6), 'no fragment under 6 frames');
  assert.ok(!segs.some(s => s.phase === 'flight'), 'the in-place sprint is airborne far longer than its pelvis rise allows, so it is not flight: ' + order(segs).join(' > '));
  const kneel = segs.find(s => s.phase === 'kneel');
  assert.ok(kneel && kneel.start >= 145 && kneel.start <= 165 && kneel.end >= 195 && kneel.end <= 215, 'kneel around f150-205: ' + JSON.stringify(kneel));
  assert.equal(segs[0].phase, 'stand');
  assert.ok(isSubsequence(['stand', 'in_place', 'slide', 'kneel', 'rise', 'in_place', 'stand'], order(segs)), order(segs).join(' > '));
  const sprint = segs.find(s => s.start <= 100 && s.end >= 100);
  assert.equal(sprint.phase, 'in_place', 'the sprint is run on the spot: pelvis far slower than the feet');
  assert.ok(segs.filter(s => s.phase === 'kneel' || s.phase === 'slide').every(s => s.start >= 140 && s.end <= 215), 'low phases stay inside f140-215');
});

test('phasePlan picks rules per phase and pads short windows', () => {
  const phases = [{ phase: 'stand', start: 0, end: 39 }, { phase: 'locomotion', start: 40, end: 99 }, { phase: 'flight', start: 100, end: 114 }, { phase: 'slide', start: 115, end: 124 },
    { phase: 'kneel', start: 125, end: 174 }, { phase: 'rise', start: 175, end: 189 }, { phase: 'locomotion', start: 190, end: 209 }, { phase: 'stand', start: 210, end: 239 }];
  const plan = phasePlan(phases, 240), by = i => plan[i];
  assert.ok(by(1).rules.includes('envelope') && by(1).end - by(1).start + 1 >= ENVELOPE_MIN_FRAMES);
  assert.ok(!by(6).rules.includes('envelope'), 'locomotion under 30 frames skips the envelope');
  for (const i of [0, 3, 4, 5, 7]) assert.ok(!by(i).rules.includes('envelope'), plan[i].phase + ' has no envelope');
  assert.deepEqual(by(2).rules, [], 'flight judges nothing per phase');
  assert.ok(!by(3).rules.includes('foot_slide') && by(3).rules.includes('ground_float'), 'slide skips foot_slide');
  for (const p of plan) for (const r of INTEGRITY_RULES) assert.ok(!p.rules.includes(r), 'integrity rules are whole-take only');
  assert.equal(by(3).to - by(3).from + 1, 15, 'short slide window padded to 15 frames');
  assert.ok(by(3).from <= 115 && by(3).to >= 124);
  const kneelContact = by(4).passes.find(p => p.rules.includes('foot_slide'));
  assert.equal(kneelContact.from, 125, 'contact pass does not reach back into the slide');
  assert.equal(kneelContact.to, 180, 'contact pass pads 6 frames into the rise');
  const lastContact = by(7).passes.find(p => p.rules.includes('foot_slide'));
  assert.ok(lastContact.to === 239 && lastContact.to - lastContact.from + 1 > 60, 'edge contact pass keeps more than 60 frames so clip-edge contacts are skipped like the whole take');
});

test('phasedTasteInput keeps the taste contract and adds phases and integrity', () => {
  const d = synthDrill(), body = phasedTasteInput(d);
  assert.equal(body.B, d.B);
  assert.equal(body.fps, 24);
  assert.deepEqual(body.integrity.rules, INTEGRITY_RULES);
  assert.ok(body.phases.length >= 5 && body.phases.every(p => Array.isArray(p.passes)));
});

test('groupByPhase files findings under their phase and whole-take findings first', () => {
  const g = groupByPhase({ verdict: 'fail', phases: [{ phase: 'stand', start: 0, end: 9 }, { phase: 'flight', start: 10, end: 20 }],
    findings: [{ rule: 'pop', severity: 'blocker', phase_index: 1, scope: 'whole take' }, { rule: 'foot_slide', severity: 'warn', phase_index: 0 }, { rule: 'x', severity: 'info' }] });
  assert.deepEqual(g.groups.map(x => [x.phase, x.findings.length]), [['whole take', 1], ['stand', 1], ['flight', 1]]);
  assert.equal(g.summary.blockers.length, 1);
});

test('markInPlace relabels locomotion only when the pelvis is much slower than the feet', () => {
  const leg = 100, segs = [{ phase: 'locomotion', start: 0, end: 19 }, { phase: 'locomotion', start: 20, end: 39 }, { phase: 'kneel', start: 40, end: 45 }];
  const speed = Array.from({ length: 46 }, (_, f) => f < 20 ? 30 : 180), footSpeed = [Array(46).fill(200), Array(46).fill(150)];
  const out = markInPlace(segs, { leg, speed, footSpeed });
  assert.deepEqual(out.map(s => s.phase), ['in_place', 'locomotion', 'kneel']);
  const still = markInPlace([{ phase: 'locomotion', start: 0, end: 9 }], { leg, speed: Array(10).fill(1), footSpeed: [Array(10).fill(10), Array(10).fill(10)] });
  assert.equal(still[0].phase, 'locomotion', 'feet barely moving is not running in place');
  assert.deepEqual(phaseRules('in_place', 90), ['foot_slide', 'plant_pop', 'ground_float', 'bounce'], 'no gait envelope in place');
});

