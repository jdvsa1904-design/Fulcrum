import test from 'node:test';
import assert from 'node:assert/strict';
import { json } from './helpers.mjs';
import * as F from '../re/biped/footsteps.js';
import * as C from '../contacts.js';

const M = json(new URL('./fixtures/biped_max_measured.json', import.meta.url));
const B = json(new URL('./fixtures/biped_max_balance.json', import.meta.url));
const D = json(new URL('./fixtures/biped_max_adapt_diag.json', import.meta.url));
const byTag = (src, tag) => M.cases.find(c => c.src === src && c.tag === tag);

test('balance factor shifts the pelvis by -bf * 0.794 * horizontal(spine3 - spine1)', () => {
  let worst = 0, peak = 0;
  for (const c of B.cases) {
    for (const r of c.rows) {
      const s1 = r.slice(0, 3), s3 = r.slice(3, 6), p0 = r.slice(6, 9), p = r.slice(9, 12);
      const o = F.balanceOffset(s1, s3, c.bf, 2);
      const meas = [p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]];
      worst = Math.max(worst, Math.abs(o[0] - meas[0]), Math.abs(o[1] - meas[1]), Math.abs(o[2] - meas[2]));
      peak = Math.max(peak, Math.hypot(meas[0], meas[1]));
    }
  }
  console.log(`balance offset worst error ${worst.toFixed(4)} units on a peak shift of ${peak.toFixed(3)}`);
  assert.ok(worst < 0.01, `worst ${worst}`);
  assert.ok(peak > 1);
});

function keyDeltas(base, edit) {
  return base.hk.map((k, i) => [k[0], edit.hk[i][1] - k[1], edit.hk[i][2] - k[2]]);
}

function check(gait, src, baseTag, editTag, delta, editIndex, tol) {
  const base = byTag(src, baseTag), edit = byTag(src, editTag);
  assert.ok(base && edit, `${baseTag} ${editTag}`);
  const plan = F.generateFootsteps({ gait, count: 10, stride: gait === 'walk' ? 0.75 : 1.5, cycle: 15, height: 170 });
  const keys = F.comKeys(plan);
  const adapted = F.adaptCOM(plan, keys, editIndex, delta);
  const meas = keyDeltas(base, edit);
  let worst = 0;
  for (const k of adapted.horizontal) {
    if (k.foot == null) continue;
    const before = keys.horizontal.find(x => x.t === k.t);
    const m = meas.find(x => Math.abs(x[0] - k.t) < 1e-6);
    if (!m) continue;
    worst = Math.max(worst, Math.abs(k.x - before.x - m[1]), Math.abs(k.y - before.y - m[2]));
  }
  assert.ok(worst < tol, `${editTag} worst ${worst}`);
  return worst;
}

test('footstep edit adaptation: along-path and lateral key responses match Max', () => {
  const r = [
    check('walk', 's5', 'walk_c15_base', 'walk_c15_x30', [30, 0], 5, 0.1),
    check('walk', 's5', 'walk_c15_base', 'walk_c15_x60', [60, 0], 5, 0.15),
    check('walk', 's5', 'walk_c15_base', 'walk_c15_y10', [0, 10], 5, 0.1),
    check('run', 's5', 'run_c15_base', 'run_c15_x30', [30, 0], 5, 0.1),
    check('run', 's5', 'run_c15_base', 'run_c15_y10', [0, 10], 5, 0.6)
  ];
  console.log('adaptation worst key errors (units):', r.map(x => x.toFixed(3)).join(' '));
});

function diagError(c) {
  const base = D.cases.find(b => b.gait === c.gait && b.width === c.width && b.fs === 0);
  const plan = F.generateFootsteps({ gait: c.gait, count: 10, stride: c.gait === 'walk' ? 0.75 : 1.5, cycle: 15, height: 170, width: c.width });
  const keys = F.comKeys(plan);
  const before = F.adaptCOM(plan, keys, [], null).horizontal;
  const after = F.adaptCOM(plan, keys, c.fs - 1, c.delta).horizontal;
  let worst = 0;
  after.forEach((k, i) => {
    if (k.foot == null) return;
    const bi = base.hk.findIndex(x => Math.abs(x[0] - k.t * 160) < 1);
    if (bi < 0) return;
    const mx = c.hk[bi][1] - base.hk[bi][1], my = c.hk[bi][2] - base.hk[bi][2];
    worst = Math.max(worst, Math.hypot(k.x - before[i].x - mx, k.y - before[i].y - my));
  });
  return worst / Math.hypot(...c.delta);
}

test('diagonal footstep edits: regenerate plus Biped key smoothing matches Max', () => {
  const worst = {};
  for (const c of D.cases) {
    if (c.fs === 0) continue;
    const k = `${c.gait} w${c.width}`;
    worst[k] = Math.max(worst[k] ?? 0, diagError(c));
  }
  console.log('diagonal adaptation worst error / |delta|:', Object.entries(worst).map(([k, v]) => `${k} ${(100 * v).toFixed(1)}%`).join(', '));
  assert.ok(worst['walk w1'] < 0.01 && worst['walk w2'] < 0.01);
  assert.ok(worst['run w1'] < 0.12 && worst['run w2'] < 0.16);
});

test('smoothKeys leaves a straight key line alone and pulls a bent key toward its neighbours', () => {
  const line = [0, 1, 2, 3, 4].map(i => ({ x: 10 * i, y: 0 }));
  assert.deepEqual(F.smoothKeys(line, 5).map(k => k.y), [0, 0, 0, 0, 0]);
  const bent = [{ x: 0, y: 0 }, { x: 10, y: 10 }, { x: 20, y: 0 }, { x: 30, y: 0 }];
  const s = F.smoothKeys(bent, 100);
  assert.ok(Math.abs(s[1].y - (10 - 0.48 * Math.hypot(10, 10))) < 1e-9);
  assert.ok(Math.abs(F.smoothKeys(bent, 2)[1].y - 8) < 1e-9);
});

test('footstep height edits leave the COM keys alone, as in Max', () => {
  for (const g of ['walk', 'run']) {
    const d = keyDeltas(byTag('s5', `${g}_c15_base`), byTag('s5', `${g}_c15_z10`));
    for (const k of d) assert.ok(Math.abs(k[1]) < 0.02 && Math.abs(k[2]) < 0.02);
  }
});

function track(frames) {
  return new Float64Array(frames.flatMap(p => p));
}

test('detectFootstepsMax implements Biped Rule A, including the clip-edge cases', () => {
  const still = (n, x) => Array.from({ length: n }, () => [x, 0, 0]);
  const moving = (n, x0) => Array.from({ length: n }, (_, i) => [x0 + (i + 1) * 10, 5, 0]);
  const pos = track([...still(2, 0), ...moving(5, 0), ...still(3, 60), ...moving(5, 60), ...still(10, 120), [130, 0, 0], [130.01, 0, 0], ...moving(4, 200), [300, 0, 0], [300, 0, 0]]);
  const r = F.detectFootstepsMax(pos, { legLength: 1, tol: 0.07 });
  assert.deepEqual(r.runs.map(x => [x.start, x.end]), [[0, 1], [15, 26], [31, 32]]);
  const old = C.detectPlants(pos, { legLength: 1, tol: 0.07 });
  assert.deepEqual(old.runs.map(x => [x.s, x.e]), [[15, 26]]);
});

test('Rule A gap filling and short-contact removal', () => {
  const p = [];
  for (let i = 0; i < 30; i++) p.push([i < 10 ? 0 : 100 + i * 5, 0, 0]);
  p[5] = [3, 0, 0];
  const r = F.detectFootstepsMax(new Float64Array(p.flat()), { legLength: 1, tol: 0.07 });
  assert.equal(r.runs[0].start, 0);
  assert.equal(r.runs[0].end, 9);
  const v = F.detectFootstepsMax(new Float64Array(p.flat()), { legLength: 1, tol: 0.07, verticalTol: 0.5, floor: 10 });
  assert.deepEqual(v.runs.map(x => [x.start, x.end]), [[0, 0]]);
});
