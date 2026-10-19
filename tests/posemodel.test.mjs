import { test } from 'node:test';
import assert from 'node:assert/strict';
import { parsePoseModel, forwardPoseModel, buildPoseInput, predictNextPose, rollPoseForward, normalizeQuats } from '../wire/posemodel.js';
import { buildTakeDataset, trainPoseModel, oneStepError } from '../wire/posetrain.js';

const NB = 2, N = 200, FPS = 30;

function syntheticTake() {
  const p = new Float32Array(N * 3);
  const q = new Float32Array(N * NB * 4);
  for (let f = 0; f < N; f++) {
    p[f * 3] = Math.sin(f * 0.05) * 10;
    p[f * 3 + 1] = 90;
    p[f * 3 + 2] = f * 0.2;
    const a = Math.sin(f * 0.1) * 0.6;
    q.set([Math.cos(a / 2), 0, Math.sin(a / 2), 0], f * NB * 4);
    q.set([1, 0, 0, 0], f * NB * 4 + 4);
  }
  return { n: N, p, q, nb: NB, fps: FPS };
}

test('a pose model with another format tag is refused', () => {
  assert.throws(() => parsePoseModel({ format: 'onnx' }), /Unsupported pose model format/);
});

test('a pose model whose shapes disagree with its dimensions is refused', () => {
  const bad = {
    format: 'motioncore-mlp-v1', inputDim: 2, hiddenDim: 1, outputDim: 1,
    weights: { w1: [[1]], b1: [0], w2: [[1]], b2: [0] }, xMean: [0, 0], xStd: [1, 1], yMean: [0], yStd: [1],
  };
  assert.throws(() => parsePoseModel(bad), /shapes do not match/);
});

test('a hand-computed forward pass matches', () => {
  const m = parsePoseModel({
    format: 'motioncore-mlp-v1', inputDim: 2, hiddenDim: 1, outputDim: 1,
    weights: { w1: [[1, 0]], b1: [0], w2: [[2]], b2: [0.5] }, xMean: [0, 0], xStd: [1, 1], yMean: [10], yStd: [2],
  });
  const expected = (2 * Math.tanh(0.5) + 0.5) * 2 + 10;
  assert.ok(Math.abs(forwardPoseModel(m, [0.5, 9])[0] - expected) < 1e-12);
});

test('the input row has path, velocity and previous-pose parts in order', () => {
  const t = syntheticTake();
  const row = buildPoseInput({ p: t.p, q: t.q, nb: NB, n: N, f: 50, fps: FPS });
  assert.equal(row.length, 13 * 2 + 2 + NB * 4);
  const prevPose = Array.from(t.q.subarray(49 * NB * 4, 50 * NB * 4));
  assert.deepEqual(row.slice(-NB * 4), prevPose);
});

test('a root track of the wrong size is refused', () => {
  const t = syntheticTake();
  assert.throws(() => buildPoseInput({ p: new Float32Array(5), q: t.q, nb: NB, n: N, f: 3, fps: FPS }), /3 values per frame/);
});

test('predicted poses are unit quaternions, one per bone', () => {
  const t = syntheticTake();
  const dataset = buildTakeDataset(t, { fps: FPS, frameEnd: 150 });
  const model = parsePoseModel(trainPoseModel({ X: dataset.X, Y: dataset.Y, hiddenDim: 8, epochs: 60, seed: 4 }));
  const pose = predictNextPose(model, { p: t.p, q: t.q, nb: NB, n: N, f: 160, fps: FPS });
  assert.equal(pose.length, NB * 4);
  for (let i = 0; i < NB; i++) {
    const len = Math.hypot(pose[i * 4], pose[i * 4 + 1], pose[i * 4 + 2], pose[i * 4 + 3]);
    assert.ok(Math.abs(len - 1) < 1e-5);
  }
});

test('training beats the mean-pose baseline on frames it did not see', () => {
  const t = syntheticTake();
  const train = buildTakeDataset(t, { fps: FPS, frameEnd: 150 });
  const test = buildTakeDataset(t, { fps: FPS, frameStart: 151, frameEnd: 199 });
  const model = parsePoseModel(trainPoseModel({ X: train.X, Y: train.Y, hiddenDim: 16, epochs: 200, seed: 5 }));
  const err = oneStepError(model, test.X, test.Y);
  assert.ok(err.model < err.baseline, `model ${err.model} vs baseline ${err.baseline}`);
});

test('a roll-forward returns one pose per step, keeping quaternion signs continuous', () => {
  const t = syntheticTake();
  const dataset = buildTakeDataset(t, { fps: FPS, frameEnd: 150 });
  const model = parsePoseModel(trainPoseModel({ X: dataset.X, Y: dataset.Y, hiddenDim: 8, epochs: 60, seed: 6 }));
  const frames = rollPoseForward(model, { p: t.p, q: t.q, nb: NB, n: N, f: 160, fps: FPS, steps: 5 });
  assert.equal(frames.length, 5);
  let prev = Array.from(t.q.subarray(160 * NB * 4, 161 * NB * 4));
  for (const pose of frames) {
    for (let i = 0; i < NB; i++) {
      const dot = pose.slice(i * 4, i * 4 + 4).reduce((s, v, k) => s + v * prev[i * 4 + k], 0);
      assert.ok(dot >= -1e-6);
    }
    prev = Array.from(pose);
  }
});

test('a roll-forward from outside the take is refused', () => {
  const t = syntheticTake();
  const small = buildTakeDataset(t, { fps: FPS, frameEnd: 5 });
  const model = parsePoseModel(trainPoseModel({ X: small.X, Y: small.Y, hiddenDim: 2, epochs: 2, seed: 7 }));
  assert.throws(() => rollPoseForward(model, { p: t.p, q: t.q, nb: NB, n: N, f: N, fps: FPS, steps: 2 }), /outside the take/);
});

test('normalizeQuats returns unit length for each bone', () => {
  const out = normalizeQuats([2, 0, 0, 0, 0, 3, 0, 0], 2);
  assert.ok(Math.abs(out[0] - 1) < 1e-9);
  assert.ok(Math.abs(out[5] - 1) < 1e-9);
});

import { existsSync, readFileSync } from 'node:fs';

const FRT_TAKE = 'exports/frt_take_raw.json';
const FRT_MODEL = 'exports/frt_pose_model_h16-e150.json';

test('the trained FRT model predicts a valid next pose for the real take', { skip: !existsSync(FRT_TAKE) || !existsSync(FRT_MODEL) }, () => {
  const src = JSON.parse(readFileSync(FRT_TAKE, 'utf8'));
  const model = parsePoseModel(JSON.parse(readFileSync(FRT_MODEL, 'utf8')));
  const { n, nb, fps } = src;
  const pose = predictNextPose(model, { p: Float32Array.from(src.root), q: Float32Array.from(src.q), nb, n, f: 200, fps });
  assert.equal(pose.length, nb * 4);
  for (let i = 0; i < nb; i++) {
    const len = Math.hypot(pose[i * 4], pose[i * 4 + 1], pose[i * 4 + 2], pose[i * 4 + 3]);
    assert.ok(Math.abs(len - 1) < 1e-4);
  }
});
