import test from 'node:test';
import assert from 'node:assert/strict';
import { near, nearVec } from './helpers.mjs';
import { fulcrumOffsets, combineOffsets, detectFulcrums, pivotSlip } from '../fulcrum.js';

function world(frames) {
  const n = frames.length, nb = frames[0].length, W = new Float32Array(n * nb * 16);
  frames.forEach((row, f) => row.forEach((p, i) => { const o = (f * nb + i) * 16; W[o] = W[o + 5] = W[o + 10] = W[o + 15] = 1; W[o + 12] = p[0]; W[o + 13] = p[1]; W[o + 14] = p[2]; }));
  return { W, nb, n };
}
const smooth = t => t * t * (3 - 2 * t);

test('fulcrumOffsets: the root offset holds the pivot at its slowest frame and fades out over the blend', () => {
  const xs = [0, 0, 0, 4, 7, 9, 10, 10.2, 11.5, 13, 16, 16, 16, 16];
  const { W, nb, n } = world(xs.map(x => [[0, 90, 0], [x, 2, 1]]));
  const off = fulcrumOffsets(W, nb, 1, 3, 10, 'auto', 2);
  for (let f = 3; f <= 10; f++) near(xs[f] + off[f][0], 10, 1e-5, `pivot held at f${f}`);
  for (const f of [3, 10]) assert.ok(Math.abs(off[f][1]) < 1e-6 && Math.abs(off[f][2]) < 1e-6);
  near(off[2][0], (10 - xs[3]) * smooth(1 - 1 / 3), 1e-5); near(off[1][0], (10 - xs[3]) * smooth(1 - 2 / 3), 1e-5);
  near(off[11][0], (10 - xs[10]) * smooth(1 - 1 / 3), 1e-5);
  assert.equal(off[0], null); assert.equal(off[13], null);
  const fixed = fulcrumOffsets(W, nb, 1, 3, 10, 9, 0);
  for (let f = 3; f <= 10; f++) near(xs[f] + fixed[f][0], xs[9], 1e-5);
  assert.equal(fixed[2], null);
  assert.ok(fulcrumOffsets(W, nb, 1, 9, 4).every(o => o === null), 'empty range');
  const clamp = fulcrumOffsets(W, nb, 1, -5, 99, 'auto', 3);
  assert.ok(clamp.every(o => o !== null) && clamp.length === n);
});

test('combineOffsets: per-frame sums, nulls skipped, longest length wins, the yaw slot is dropped', () => {
  const r = combineOffsets([[1, 2, 3, 0.5], null], null, [[1, 1, 1], [2, 0, 0], [0, 0, 1]]);
  assert.deepEqual(r, [[2, 3, 4], [2, 0, 0], [0, 0, 1]]);
  assert.deepEqual(combineOffsets(), []);
  assert.deepEqual(combineOffsets([null, null]), [null, null]);
});

test('detectFulcrums: low and slow runs of at least minFrames, per candidate, sorted by start', () => {
  const rows = [];
  for (let f = 0; f < 20; f++) rows.push([[0, 90, 0], [f < 8 ? 5 : 5 + (f - 8) * 4, 3, 0], [0, f >= 12 ? 4 : 30, 0], [f * 0.1, 15, 0]]);
  const { W, nb, n } = world(rows);
  const c = [{ name: 'foot', idx: 1 }, { name: 'hand', idx: 2 }, { name: 'high', idx: 3 }, { name: 'none', idx: undefined }];
  const runs = detectFulcrums(W, nb, n, 30, 100, c, 0, 0.25, 0.08, 4);
  assert.deepEqual(runs.map(r => [r.pivot, r.a, r.b]), [['foot', 0, 7], ['hand', 12, 19]]);
  assert.deepEqual(detectFulcrums(W, nb, n, 30, 100, c, 0, 0.25, 0.08, 9), [], 'runs shorter than minFrames are dropped');
  assert.deepEqual(detectFulcrums(W, nb, n, 30, 100, c, -10, 0.25, 0.08, 4), [], 'ground far below');
});

test('pivotSlip: the widest horizontal spread of the pivot inside the run, in cm', () => {
  const { W, nb } = world([[[0, 0, 0]], [[3, 9, 0]], [[1, 0, 4]], [[0, 0, 0]]]);
  near(pivotSlip(W, nb, 0, 0, 3, 100), Math.hypot(2, 4), 1e-6);
  near(pivotSlip(W, nb, 0, 0, 1, 100), 3, 1e-6);
  near(pivotSlip(W, nb, 0, 0, 1, 1), 300, 1e-4, 'metres in, cm out');
  assert.equal(pivotSlip(W, nb, 0, 2, 2, 100), 0);
});
