import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { buildTakeDataset, trainPoseModel, oneStepError } from '../wire/posetrain.js';
import { POSE_MODEL_FORMAT } from '../wire/posemodel.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const exportsDir = path.join(here, '..', 'exports');
const src = JSON.parse(readFileSync(path.join(exportsDir, 'frt_take_raw.json'), 'utf8'));
const { n, nb, fps } = src;
const HIDDEN = Number(process.env.HIDDEN ?? 16);
const EPOCHS = Number(process.env.EPOCHS ?? 150);
const LR = Number(process.env.LR ?? 0.05);
const SEED = Number(process.env.SEED ?? 1);
const SPLIT_FRACTION = Number(process.env.SPLIT ?? 0.75);

const root = Float32Array.from(src.root);
const q = Float32Array.from(src.q);
let flips = 0;
for (let i = 0; i < nb; i++) {
  for (let f = 1; f < n; f++) {
    const o = f * nb * 4 + i * 4, po = (f - 1) * nb * 4 + i * 4;
    const d = q[o] * q[po] + q[o + 1] * q[po + 1] + q[o + 2] * q[po + 2] + q[o + 3] * q[po + 3];
    if (d < 0) { flips++; for (let k = 0; k < 4; k++) q[o + k] = -q[o + k]; }
  }
}

const take = { n, p: root, q, nb, fps };
const split = Math.floor(n * SPLIT_FRACTION);
const train = buildTakeDataset(take, { fps, frameStart: 1, frameEnd: split - 1 });
const test = buildTakeDataset(take, { fps, frameStart: split, frameEnd: n - 1 });

const t0 = Date.now();
const model = trainPoseModel({ X: train.X, Y: train.Y, hiddenDim: HIDDEN, epochs: EPOCHS, learningRate: LR, seed: SEED });
const seconds = (Date.now() - t0) / 1000;
const nbq = nb * 4;
const persist = (rows) => { let t = 0, c = 0; for (const [i, r] of rows.entries()) { const prev = r.X.slice(-nbq); for (let k = 0; k < nbq; k++) { t += (prev[k] - r.Y[k]) ** 2; c++; } } return t / c; };
const persistTest = (() => { let t = 0, c = 0; for (let f = split; f < n; f++) { for (let k = 0; k < nbq; k++) { t += (q[(f - 1) * nbq + k] - q[f * nbq + k]) ** 2; c++; } } return t / c; })();
const trainErr = oneStepError(model, train.X, train.Y);
const testErr = oneStepError(model, test.X, test.Y);

const tag = `h${HIDDEN}-e${EPOCHS}`;
const modelPath = path.join(exportsDir, `frt_pose_model_${tag}.json`);
const report = {
  source: 'exports/frt_take_raw.json (FRT drill, Checkpoint soldier)',
  frames: n, bones: nb, fps, splitFrame: split,
  signFlipsFixed: flips,
  trainRows: train.X.length, testRows: test.X.length,
  inputDim: model.inputDim, outputDim: model.outputDim,
  hiddenDim: HIDDEN, epochs: EPOCHS, learningRate: LR, seed: SEED,
  seconds,
  trainMse: trainErr.model, trainBaselineMse: trainErr.baseline,
  testMse: testErr.model, testBaselineMse: testErr.baseline,
  testVsBaseline: testErr.model / testErr.baseline,
  persistenceTestMse: persistTest,
  validated: testErr.model < persistTest,
  model: modelPath,
};
writeFileSync(modelPath, JSON.stringify({ format: POSE_MODEL_FORMAT, ...model, bones: src.bones, pelvis: src.pelvis, fps, validated: testErr.model < persistTest }));
writeFileSync(path.join(exportsDir, `frt_pose_model_${tag}.report.json`), JSON.stringify(report, null, 2));
console.log(JSON.stringify(report, null, 2));
