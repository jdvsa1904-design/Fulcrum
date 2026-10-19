import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as S from '../re/mobu/story.js';

const P = (f) => JSON.parse(readFileSync(new URL(`../re/mobu/probes/${f}`, import.meta.url)));
const story = P('story_out.json');
const loop = P('loop2_out.json');
const maxErr = (pairs) => Math.max(...pairs.map(([a, b]) => Math.abs(a - b)));

test('transition curves match MotionBuilder', (t) => {
  let worst = 0;
  for (const k of [1, 2, 3, 4]) {
    const got = story.blends[k];
    const pairs = got.map((v, i) => [v, 100 * S.transition(78 + i, { stop: 100 }, { start: 80, inType: k }).wIn]);
    worst = Math.max(worst, maxErr(pairs));
    assert.deepEqual(story.blends[`in${k}_outlin`], got);
  }
  t.diagnostic(`blend max error ${worst.toExponential(2)} (percent of weight)`);
  assert.ok(worst < 1e-4);
});

test('time warp curves match MotionBuilder', (t) => {
  let worst = 0;
  for (let k = 1; k <= 8; k++) {
    const pairs = story.warps[k].map((v, i) => [v, 100 * S.timeWarp(k, (5 * i) / 100)]);
    worst = Math.max(worst, maxErr(pairs));
  }
  t.diagnostic(`time warp max error ${worst.toFixed(4)} (percent of clip)`);
  assert.ok(worst < 0.1);
});

const clipBase = { start: 0, sourceMarkIn: 0, sourceMarkOut: 100, loop: true };
const src = (s) => [10 + s, 2 + 0.1 * s, 5 + 0.5 * s];

function checkMap(rows, clip) {
  for (const [f, time, n] of rows) assert.deepEqual(S.sourceTime(f, clip), { time, loop: n }, `frame ${f}`);
}

test('loop, offset and speed source mapping match MotionBuilder', () => {
  checkMap(loop.srcmap, clipBase);
  checkMap(loop.smi20_src, { ...clipBase, sourceMarkIn: 20, offset: -20 });
  checkMap(loop.off30_src, { ...clipBase, offset: 30 });
  checkMap(loop.sp2_src, { ...clipBase, speed: 2 });
});

test('loop translation values match MotionBuilder', (t) => {
  let worst = 0;
  const cases = [
    ['loop_vals', { ...clipBase, loopTranslation: [100, 0, 50] }],
    ['smi20_vals', { ...clipBase, sourceMarkIn: 20, offset: -20, loopTranslation: [100, 0, 50] }],
    ['off30_vals', { ...clipBase, offset: 30, loopTranslation: [100, 0, 50] }],
    ['noLT_vals', { ...clipBase, loopTranslation: [0, 0, 0] }],
  ];
  for (const [key, clip] of cases) {
    for (const [f, ...v] of loop[key]) {
      const m = S.loopedValue(f, clip, src);
      worst = Math.max(worst, ...m.map((x, i) => Math.abs(x - v[i])));
    }
  }
  t.diagnostic(`loop value max error ${worst.toExponential(2)}`);
  assert.ok(worst < 1e-3);
});

test('non-looping clip holds its last source frame', () => {
  assert.deepEqual(S.sourceTime(80, { start: 0, sourceMarkIn: 0, sourceMarkOut: 100, speed: 2 }), { time: 100, loop: 0 });
});

test('clip frame and auto loop translation match MotionBuilder', () => {
  const f = S.clipFrame(src(0), src(100));
  assert.deepEqual(f.t, [10, 0, 5]);
  assert.ok(Math.abs(f.yawDeg - -26.5651) < 1e-4);
  assert.deepEqual(S.autoLoopTranslation(src(0), src(100)), [100, 0, 50]);
});

test('gravity match keeps own height and yaws only', () => {
  const yaw = (a) => [0, Math.sin(a / 2), 0, Math.cos(a / 2)];
  const A = { t: [5, 1, 7], q: yaw(Math.PI / 2) };
  const B = { t: [0, 0.9, 0], q: yaw(0) };
  const M = S.matchPose(A, B);
  assert.deepEqual(M.t, [5, 0.9, 7]);
  const P2 = S.applyOffset(S.matchOffset(A, B), B);
  P2.t.forEach((x, i) => assert.ok(Math.abs(x - M.t[i]) < 1e-9));
  const pitch = [0, 0, Math.sin(0.3), Math.cos(0.3)];
  const g = S.matchPose({ t: [0, 0, 0], q: pitch }, { t: [0, 0, 0], q: [0, 0, 0, 1] }, 0, S.MATCH_R.GRAVITY);
  assert.ok(Math.abs(g.q[3] - 1) < 1e-9);
});
