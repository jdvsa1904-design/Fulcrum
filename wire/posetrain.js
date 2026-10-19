import { POSE_MODEL_FORMAT, buildPoseInput, forwardPoseModel } from './posemodel.js';

function rng(seed) {
  let s = seed >>> 0;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

function statsOf(rows) {
  const n = rows.length, dim = rows[0].length;
  const mean = new Array(dim).fill(0);
  for (const r of rows) for (let i = 0; i < dim; i++) mean[i] += r[i] / n;
  const std = new Array(dim).fill(0);
  for (const r of rows) for (let i = 0; i < dim; i++) std[i] += (r[i] - mean[i]) ** 2 / n;
  return { mean, std: std.map(v => Math.max(Math.sqrt(v), 1e-8)) };
}

export function buildTakeDataset(take, { fps, frameStart = 1, frameEnd = take.n - 1 }) {
  const { n, p, q, nb } = take;
  const X = [], Y = [];
  for (let f = Math.max(1, frameStart); f <= Math.min(n - 1, frameEnd); f++) {
    X.push(buildPoseInput({ p, q, nb, n, f, fps }));
    Y.push(Array.from(q.subarray(f * nb * 4, (f + 1) * nb * 4)));
  }
  return { X, Y };
}

export function trainPoseModel({ X, Y, hiddenDim = 16, epochs = 150, learningRate = 0.05, seed = 1 }) {
  if (!X.length || X.length !== Y.length) throw new Error('Training rows and targets must be non-empty and equal length.');
  const inputDim = X[0].length, outputDim = Y[0].length;
  const xs = statsOf(X), ys = statsOf(Y);
  const xn = X.map(r => r.map((v, i) => (v - xs.mean[i]) / xs.std[i]));
  const yn = Y.map(r => r.map((v, i) => (v - ys.mean[i]) / ys.std[i]));
  const rand = rng(seed);
  const s1 = 1 / Math.sqrt(inputDim), s2 = 1 / Math.sqrt(hiddenDim);
  const w1 = Array.from({ length: hiddenDim }, () => Array.from({ length: inputDim }, () => (rand() * 2 - 1) * s1));
  const b1 = new Array(hiddenDim).fill(0);
  const w2 = Array.from({ length: outputDim }, () => Array.from({ length: hiddenDim }, () => (rand() * 2 - 1) * s2));
  const b2 = new Array(outputDim).fill(0);
  const n = X.length;
  for (let e = 0; e < epochs; e++) {
    const gw1 = w1.map(r => r.map(() => 0)), gb1 = b1.map(() => 0);
    const gw2 = w2.map(r => r.map(() => 0)), gb2 = b2.map(() => 0);
    for (let s = 0; s < n; s++) {
      const x = xn[s];
      const h = b1.map((b, j) => { let a = b; for (let i = 0; i < inputDim; i++) a += w1[j][i] * x[i]; return Math.tanh(a); });
      const out = b2.map((b, k) => { let a = b; for (let j = 0; j < hiddenDim; j++) a += w2[k][j] * h[j]; return a; });
      const dOut = out.map((o, k) => (2 * (o - yn[s][k])) / n);
      for (let k = 0; k < outputDim; k++) { gb2[k] += dOut[k]; for (let j = 0; j < hiddenDim; j++) gw2[k][j] += dOut[k] * h[j]; }
      const dH = h.map((hj, j) => { let g = 0; for (let k = 0; k < outputDim; k++) g += dOut[k] * w2[k][j]; return g * (1 - hj * hj); });
      for (let j = 0; j < hiddenDim; j++) { gb1[j] += dH[j]; for (let i = 0; i < inputDim; i++) gw1[j][i] += dH[j] * x[i]; }
    }
    for (let j = 0; j < hiddenDim; j++) { b1[j] -= learningRate * gb1[j]; for (let i = 0; i < inputDim; i++) w1[j][i] -= learningRate * gw1[j][i]; }
    for (let k = 0; k < outputDim; k++) { b2[k] -= learningRate * gb2[k]; for (let j = 0; j < hiddenDim; j++) w2[k][j] -= learningRate * gw2[k][j]; }
  }
  return { format: POSE_MODEL_FORMAT, inputDim, hiddenDim, outputDim, weights: { w1, b1, w2, b2 }, xMean: xs.mean, xStd: xs.std, yMean: ys.mean, yStd: ys.std };
}

export function oneStepError(model, X, Y) {
  let total = 0, base = 0, count = 0;
  const meanY = Y[0].map((_, k) => Y.reduce((s, r) => s + r[k], 0) / Y.length);
  X.forEach((x, s) => {
    const pred = forwardPoseModel(model, x);
    pred.forEach((v, k) => {
      total += (v - Y[s][k]) ** 2;
      base += (Y[s][k] - meanY[k]) ** 2;
      count++;
    });
  });
  return { model: total / count, baseline: base / count };
}
