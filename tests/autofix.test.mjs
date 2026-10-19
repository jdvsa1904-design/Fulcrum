import test from 'node:test';
import assert from 'node:assert/strict';
import { planFixes, autoFixLoop, sideOf, score, fulcrumFindings } from '../wire/autofix.js';

const F = (rule, a, b, extra = {}) => ({ rule, body_part: 'left foot', severity: 'blocker', frame_start: a, frame_end: b, phase: 'in_place', ...extra });
const ctx = { fixes: [], locks: [], n: 300, fps: 24 };

test('sideOf maps body parts', () => {
  assert.equal(sideOf('left foot'), 'l');
  assert.equal(sideOf('right foot'), 'r');
  assert.equal(sideOf('feet'), null);
});

test('float becomes padded ground on that side', () => {
  const p = planFixes([F('ground_float', 69, 77)], ctx);
  assert.deepEqual(p.fixes[0], { type: 'ground', a: 66, b: 80, side: 'l', anchor: 'ankle' });
});

test('float padding clamps to the take', () => {
  const p = planFixes([F('ground_float', 1, 298, { body_part: 'feet' })], ctx);
  assert.equal(p.fixes[0].a, 0); assert.equal(p.fixes[0].b, 299); assert.equal(p.fixes[0].side, null);
});

test('sink becomes floor', () => {
  assert.equal(planFixes([F('ground_sink', 10, 20)], ctx).fixes[0].type, 'floor');
});

test('pop becomes padded despike', () => {
  assert.deepEqual(planFixes([F('pop', 100, 100)], ctx).fixes[0], { type: 'despike', a: 98, b: 102 });
});

test('slide locks the matching plant', () => {
  const p = planFixes([F('foot_slide', 40, 45)], { ...ctx, plants: [{ side: 'r', a: 30, b: 50 }, { side: 'l', a: 35, b: 52 }] });
  assert.deepEqual(p.locks, [{ side: 'l', a: 35, b: 52 }]);
});

test('slide without plants locks the finding range; covered slide is skipped', () => {
  assert.deepEqual(planFixes([F('foot_slide', 40, 45)], ctx).locks, [{ side: 'l', a: 40, b: 45 }]);
  const p = planFixes([F('foot_slide', 40, 45)], { ...ctx, locks: [{ side: 'l', a: 38, b: 60 }] });
  assert.equal(p.locks.length, 1); assert.equal(p.changes.length, 0);
});

test('slide with no side and no plants locks both feet', () => {
  const p = planFixes([F('foot_slide', 40, 45, { body_part: 'feet' })], ctx);
  assert.deepEqual(p.locks, [{ side: 'l', a: 40, b: 45 }, { side: 'r', a: 40, b: 45 }]);
  const q = planFixes([F('foot_slide', 40, 45, { body_part: 'feet' })], { ...ctx, locks: [{ side: 'l', a: 30, b: 60 }] });
  assert.deepEqual(q.locks.slice(1), [{ side: 'r', a: 40, b: 45 }]);
});

test('toe-out in locomotion edits the existing travel toe without mutating input', () => {
  const fixes = [{ type: 'travel', a: 0, b: 200, speed: 4 }];
  const p = planFixes([F('planted_toe_out_deg', 50, 60, { phase: 'locomotion', value: 30, expected_range: [null, 18] })], { ...ctx, fixes });
  assert.equal(p.fixes.length, 1); assert.equal(p.fixes[0].toe, -12);
  assert.equal(fixes[0].toe, undefined);
});

test('ankle in locomotion sets travel pitch', () => {
  const p = planFixes([F('planted_ankle_deg', 50, 60, { phase: 'locomotion', value: 25, expected_range: [-10, 15] })], { ...ctx, fixes: [{ type: 'travel', a: 0, b: 200 }] });
  assert.equal(p.fixes[0].pitch, 15);
});

test('toe-out without travel adds a single level', () => {
  const p = planFixes([F('planted_toe_out_deg', 5, 9), F('planted_ankle_deg', 20, 30)], ctx);
  assert.deepEqual(p.fixes, [{ type: 'level', side: 'l' }]);
});

test('crossing warning is note only, crossing blocker smooths', () => {
  const w = planFixes([F('crossing', 10, 12, { severity: 'warning' })], ctx);
  assert.equal(w.fixes.length, 0); assert.match(w.notes[0], /note only/);
  assert.equal(planFixes([F('crossing', 10, 12)], ctx).fixes[0].type, 'smooth');
});

test('unknown rules note only and duplicates collapse', () => {
  const p = planFixes([F('bounce', 1, 2), F('pop', 100, 100), F('pop', 100, 100)], ctx);
  assert.equal(p.fixes.length, 1); assert.equal(p.notes.length, 3);
});

test('score counts blockers and warnings', () => {
  assert.deepEqual(score([F('pop', 1, 1), F('pop', 1, 1, { severity: 'warning' }), F('pop', 1, 1, { severity: 'info' })]), { blockers: 1, warnings: 1 });
});

function fakeJudge(base, cures) {
  return async st => base.filter(f => !cures.some(([rule, ok]) => f.rule === rule && st.fixes.concat(st.locks).some(ok)));
}

test('loop keeps helpful fixes and stops clean', async () => {
  const base = [F('pop', 100, 100), F('ground_float', 69, 77)];
  const judge = fakeJudge(base, [['pop', x => x.type === 'despike'], ['ground_float', x => x.type === 'ground']]);
  const applied = [];
  const r = await autoFixLoop({ judge, apply: s => applied.push(s), n: 300, fps: 24 });
  assert.equal(r.reason, 'clean'); assert.equal(r.end.blockers, 0); assert.equal(r.kept.length, 2);
  assert.equal(r.fixes.length, 2); assert.ok(applied.length >= 3);
});

test('loop reverts a fix that does not help', async () => {
  const base = [F('pop', 100, 100), F('ground_float', 69, 77)];
  const judge = fakeJudge(base, [['pop', x => x.type === 'despike']]);
  const applied = [];
  const r = await autoFixLoop({ judge, apply: s => applied.push(s), n: 300 });
  assert.equal(r.reason, 'no_progress'); assert.equal(r.end.blockers, 1);
  assert.deepEqual(r.fixes.map(x => x.type), ['despike']);
  assert.ok(r.reverted.some(x => /ground/.test(x.note)));
  assert.deepEqual(applied.at(-1).fixes.map(x => x.type), ['despike']);
});

test('loop reverts a fix that makes things worse', async () => {
  const judge = async st => st.fixes.some(x => x.type === 'despike') ? [F('pop', 1, 1), F('bounce', 1, 1)] : [F('pop', 1, 1)];
  const r = await autoFixLoop({ judge, apply: () => {}, n: 300 });
  assert.equal(r.fixes.length, 0); assert.equal(r.reason, 'no_progress'); assert.equal(r.rounds, 1);
});

test('loop respects maxRounds and accepts the {findings} judge shape', async () => {
  let k = 0;
  const judge = async () => ({ findings: [F('pop', 10 + k, 10 + k)].concat(Array.from({ length: Math.max(0, 5 - k++) }, () => F('bounce', 1, 1))) });
  const r = await autoFixLoop({ judge, apply: () => {}, maxRounds: 2, n: 300 });
  assert.equal(r.rounds, 2); assert.equal(r.reason, 'max_rounds');
});

test('loop with no blockers does nothing', async () => {
  const r = await autoFixLoop({ judge: async () => [F('pop', 1, 1, { severity: 'warning' })], apply: () => {}, n: 300 });
  assert.equal(r.rounds, 0); assert.equal(r.reason, 'clean');
});

const contactOf = (n, slideL) => ({ per: { l: new Uint8Array(n).fill(1), r: new Uint8Array(n).fill(1) }, speed: { l: Float32Array.from({ length: n }, (_, f) => slideL(f) ? 0.8 : 0), r: new Float32Array(n) } });

test('fulcrumFindings maps Fulcrum flags, motion quality and sliding plants to findings', () => {
  const flags = [{ kind: 'slide', a: 40, b: 45 }, { kind: 'float', a: 69, b: 77 }, { kind: 'snap', a: 100, b: 100 }, { kind: 'overload', a: 5, b: 9 }];
  const quality = { findings: [
    { kind: 'ground-penetration', severity: 'warning', frame: 10, bone: 'foot_r', cm: 3.5 }, { kind: 'ground-penetration', severity: 'warning', frame: 11, bone: 'foot_r', cm: 4.25 },
    { kind: 'ground-penetration', severity: 'warning', frame: 30, bone: 'foot_r', cm: 3.1 },
    { kind: 'constraint-residual', severity: 'blocker', frame: 50, bone: 'hand_l', errorCm: 6 }, { kind: 'constraint-residual', severity: 'blocker', frame: 51, bone: 'hand_l', errorCm: 7 },
  ] };
  const fs = fulcrumFindings({ flags, quality, slidingPlants: [{ side: 'r', a: 200, b: 210, drift_cm: 12.345 }], contact: contactOf(300, f => f >= 40 && f <= 45) });
  const pick = r => fs.filter(f => f.rule === r).map(({ rule, severity, frame_start, frame_end, body_part, value }) => [rule, severity, frame_start, frame_end, body_part, value]);
  assert.deepEqual(pick('foot_slide'), [['foot_slide', 'blocker', 40, 45, 'foot_l', null], ['foot_slide', 'warning', 200, 210, 'foot_r', 12.35]]);
  assert.deepEqual(pick('ground_float'), [['ground_float', 'blocker', 69, 77, 'feet', null]]);
  assert.deepEqual(pick('pop'), [['pop', 'blocker', 100, 100, 'whole body', null]]);
  assert.deepEqual(pick('ground_sink'), [['ground_sink', 'warning', 10, 11, 'foot_r', 4.25], ['ground_sink', 'warning', 30, 30, 'foot_r', 3.1]]);
  assert.deepEqual(pick('constraint_residual'), [['constraint_residual', 'blocker', 50, 51, 'hand_l', 7]]);
  assert.equal(fs.length, 7, 'momentum overload has no finding');
  assert.ok(fs.every(f => 'phase' in f) && fs.findIndex(f => f.severity === 'warning') > fs.findLastIndex(f => f.severity === 'blocker'), 'blockers first');
  assert.deepEqual(score(fs), { blockers: 4, warnings: 3 });
});

test('fulcrumFindings leaves the slide side open when both feet slide, and plans the existing fixes', () => {
  const fs = fulcrumFindings({ flags: [{ kind: 'slide', a: 40, b: 45 }], contact: { per: { l: new Uint8Array(60).fill(1), r: new Uint8Array(60).fill(1) }, speed: { l: new Float32Array(60).fill(1), r: new Float32Array(60).fill(1) } } });
  assert.equal(fs[0].body_part, 'feet');
  const p = planFixes(fulcrumFindings({ flags: [{ kind: 'float', a: 69, b: 77 }, { kind: 'snap', a: 100, b: 100 }], quality: { findings: [{ kind: 'ground-penetration', severity: 'warning', frame: 10, bone: 'foot_l', cm: 4 }, { kind: 'constraint-residual', severity: 'blocker', frame: 50, bone: 'foot_l', errorCm: 5 }] }, slidingPlants: [{ side: 'l', a: 20, b: 30 }] }), ctx);
  assert.deepEqual(p.fixes.map(x => x.type).sort(), ['despike', 'floor', 'ground']);
  assert.deepEqual(p.locks, [{ side: 'l', a: 20, b: 30 }]);
  assert.ok(p.notes.some(n => /constraint_residual .*no auto fix/.test(n)));
  assert.deepEqual(fulcrumFindings(), []);
});
