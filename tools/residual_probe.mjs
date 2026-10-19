import { readFileSync } from 'node:fs';
import { buildTakeDataset, trainPoseModel } from '../wire/posetrain.js';
import { forwardPoseModel } from '../wire/posemodel.js';
const src = JSON.parse(readFileSync('exports/frt_take_raw.json', 'utf8'));
const { n, nb, fps } = src;
const root = Float32Array.from(src.root), q = Float32Array.from(src.q);
for (let i = 0; i < nb; i++) for (let f = 1; f < n; f++) {
  const o = f * nb * 4 + i * 4, po = (f - 1) * nb * 4 + i * 4;
  const d = q[o] * q[po] + q[o + 1] * q[po + 1] + q[o + 2] * q[po + 2] + q[o + 3] * q[po + 3];
  if (d < 0) for (let k = 0; k < 4; k++) q[o + k] = -q[o + k];
}
const take = { n, p: root, q, nb, fps };
const split = Math.floor(n * 0.75), nq = nb * 4;
const tr = buildTakeDataset(take, { fps, frameStart: 1, frameEnd: split - 1 });
const te = buildTakeDataset(take, { fps, frameStart: split, frameEnd: n - 1 });
const prevOf = (X) => X.map(x => x.slice(-nq));
const dY = (r) => r.Y.map((y, s) => y.map((v, k) => v - prevOf(r.X)[s][k]));
const trainY = dY(tr);
for (const [hidden, epochs, lr] of [[16, 150, 0.05], [8, 150, 0.02], [16, 400, 0.02]]) {
  const m = trainPoseModel({ X: tr.X, Y: trainY, hiddenDim: hidden, epochs, learningRate: lr, seed: 1 });
  let t = 0, c = 0;
  te.X.forEach((x, s) => {
    const pred = forwardPoseModel(m, x), prev = x.slice(-nq);
    pred.forEach((v, k) => { t += (prev[k] + v - te.Y[s][k]) ** 2; c++; });
  });
  let zero = 0, zc = 0;
  te.X.forEach((x, s) => { const prev = x.slice(-nq); prev.forEach((v, k) => { zero += (v - te.Y[s][k]) ** 2; zc++; }); });
  console.log(JSON.stringify({ hidden, epochs, lr, residualTestMse: t / c, persistenceTestMse: zero / zc, ratio: (t / c) / (zero / zc) }));
}
