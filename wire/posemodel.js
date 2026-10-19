export const POSE_MODEL_FORMAT = 'motioncore-mlp-v1';
export const PATH_POINTS = 13;
export const PATH_SECONDS = 1.0;

export function parsePoseModel(text) {
  const d = typeof text === 'string' ? JSON.parse(text) : text;
  if (d.format !== POSE_MODEL_FORMAT) throw new Error(`Unsupported pose model format: ${d.format}`);
  const { inputDim, hiddenDim, outputDim, weights, xMean, xStd, yMean, yStd } = d;
  const shapeOk = weights.w1.length === hiddenDim && weights.w1.every(r => r.length === inputDim)
    && weights.w2.length === outputDim && weights.w2.every(r => r.length === hiddenDim)
    && weights.b1.length === hiddenDim && weights.b2.length === outputDim
    && xMean.length === inputDim && xStd.length === inputDim && yMean.length === outputDim && yStd.length === outputDim;
  if (!shapeOk) throw new Error('Pose model shapes do not match its declared dimensions.');
  if (xStd.some(s => !(s > 0)) || yStd.some(s => !(s > 0))) throw new Error('Pose model std values must be positive.');
  return { inputDim, hiddenDim, outputDim, weights, xMean, xStd, yMean, yStd };
}

export function forwardPoseModel(model, raw) {
  if (raw.length !== model.inputDim) throw new Error(`Pose model input must have ${model.inputDim} values, got ${raw.length}.`);
  const x = raw.map((v, i) => (v - model.xMean[i]) / model.xStd[i]);
  const h = model.weights.b1.map((b, j) => {
    let s = b;
    for (let i = 0; i < model.inputDim; i++) s += model.weights.w1[j][i] * x[i];
    return Math.tanh(s);
  });
  return model.weights.b2.map((b, k) => {
    let s = b;
    for (let j = 0; j < model.hiddenDim; j++) s += model.weights.w2[k][j] * h[j];
    return s * model.yStd[k] + model.yMean[k];
  });
}

function heading(p, f, n) {
  const a = Math.max(0, f - 1), b = Math.min(n - 1, f + 1);
  const dx = p[b * 3] - p[a * 3], dz = p[b * 3 + 2] - p[a * 3 + 2];
  return Math.hypot(dx, dz) < 1e-9 ? 0 : Math.atan2(dx, dz);
}

export function buildPoseInput({ p, q, nb, n, f, fps, pathPoints = PATH_POINTS, pathSeconds = PATH_SECONDS }) {
  if (p.length !== n * 3) throw new Error('Root track must hold 3 values per frame.');
  if (q.length !== n * nb * 4) throw new Error('Pose track must hold 4 values per bone per frame.');
  const h = heading(p, f, n);
  const c = Math.cos(-h), s = Math.sin(-h);
  const ox = p[f * 3], oz = p[f * 3 + 2];
  const row = [];
  for (let k = 0; k < pathPoints; k++) {
    const t = (k / (pathPoints - 1)) * pathSeconds - pathSeconds / 2;
    const idx = Math.min(n - 1, Math.max(0, Math.round(f + t * fps)));
    const dx = p[idx * 3] - ox, dz = p[idx * 3 + 2] - oz;
    row.push(c * dx - s * dz, s * dx + c * dz);
  }
  const prev = Math.max(0, f - 1);
  const vx = (p[f * 3] - p[prev * 3]) * fps, vz = (p[f * 3 + 2] - p[prev * 3 + 2]) * fps;
  row.push(c * vx - s * vz, s * vx + c * vz);
  for (let i = 0; i < nb * 4; i++) row.push(q[prev * nb * 4 + i]);
  return row;
}

export function normalizeQuats(values, nb) {
  const out = new Float32Array(nb * 4);
  for (let i = 0; i < nb; i++) {
    const o = i * 4;
    const len = Math.hypot(values[o], values[o + 1], values[o + 2], values[o + 3]) || 1;
    for (let k = 0; k < 4; k++) out[o + k] = values[o + k] / len;
  }
  return out;
}

export function alignToPrevious(pose, prev, nb) {
  for (let i = 0; i < nb; i++) {
    const o = i * 4;
    const d = pose[o] * prev[o] + pose[o + 1] * prev[o + 1] + pose[o + 2] * prev[o + 2] + pose[o + 3] * prev[o + 3];
    if (d < 0) for (let k = 0; k < 4; k++) pose[o + k] = -pose[o + k];
  }
  return pose;
}

export function predictNextPose(model, { p, q, nb, n, f, fps }) {
  const raw = forwardPoseModel(model, buildPoseInput({ p, q, nb, n, f, fps }));
  const prev = q.subarray(Math.max(0, f - 1) * nb * 4, (Math.max(0, f - 1) + 1) * nb * 4);
  return alignToPrevious(normalizeQuats(raw, nb), prev, nb);
}

export function rollPoseForward(model, { p, q, nb, n, f, fps, steps }) {
  if (f < 0 || f >= n) throw new Error('Start frame is outside the take.');
  const work = Float32Array.from(q);
  const frames = [];
  for (let cur = f; cur < Math.min(n - 1, f + steps); cur++) {
    const next = predictNextPose(model, { p, q: work, nb, n, f: cur + 1, fps });
    work.set(next, (cur + 1) * nb * 4);
    frames.push(next);
  }
  return frames;
}
