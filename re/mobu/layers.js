export const MODE = { ADDITIVE: 0, OVERRIDE: 1, OVERRIDE_PASSTHROUGH: 2 };
export const ROT = { EULER: 0, QUAT: 1 };

const DEG = Math.PI / 180;

export function effectiveWeight(layer, byId) {
  let w = (layer.weight ?? 100) / 100;
  let p = layer.parent != null ? byId.get(layer.parent) : null;
  while (p) { w *= (p.weight ?? 100) / 100; p = p.parent != null ? byId.get(p.parent) : null; }
  return w;
}

export function blendScalar(below, value, mode, w) {
  if (mode === MODE.ADDITIVE) return below + w * value;
  if (mode === MODE.OVERRIDE_PASSTHROUGH) return below + w * (value - below);
  return w * value;
}

export function evalScalar(layers) {
  const byId = new Map(layers.map((l) => [l.id, l]));
  let v = 0;
  layers.forEach((l, i) => {
    if (l.value == null || l.mute) return;
    if (i === 0) { v = l.value; return; }
    v = blendScalar(v, l.value, l.mode ?? MODE.ADDITIVE, effectiveWeight(l, byId));
  });
  return v;
}

export function eulerToQuat(e) {
  const [x, y, z] = e.map((a) => a * DEG / 2);
  const qx = [Math.sin(x), 0, 0, Math.cos(x)];
  const qy = [0, Math.sin(y), 0, Math.cos(y)];
  const qz = [0, 0, Math.sin(z), Math.cos(z)];
  return qmul(qz, qmul(qy, qx));
}

export function quatToEuler(q) {
  const [x, y, z, w] = q;
  const sy = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  return [
    Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) / DEG,
    Math.asin(sy) / DEG,
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) / DEG,
  ];
}

export function qmul(a, b) {
  return [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
}

export function slerp(a, b, t) {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  let bb = b;
  if (d < 0) { d = -d; bb = b.map((x) => -x); }
  if (d > 0.9995) {
    const r = a.map((x, i) => x + t * (bb[i] - x));
    const l = Math.hypot(...r);
    return r.map((x) => x / l);
  }
  const th = Math.acos(d);
  const s = Math.sin(th);
  const ka = Math.sin((1 - t) * th) / s;
  const kb = Math.sin(t * th) / s;
  return a.map((x, i) => ka * x + kb * bb[i]);
}

const I = [0, 0, 0, 1];

export function blendRotation(below, value, mode, w, rotMode = ROT.EULER) {
  if (rotMode === ROT.EULER) return below.map((b, i) => blendScalar(b, value[i], mode, w));
  const qb = eulerToQuat(below);
  const qv = eulerToQuat(value);
  let q;
  if (mode === MODE.ADDITIVE) q = qmul(qb, slerp(I, qv, w));
  else if (mode === MODE.OVERRIDE_PASSTHROUGH) q = slerp(qb, qv, w);
  else q = slerp(I, qv, w);
  return quatToEuler(q);
}

export function evalRotation(layers) {
  const byId = new Map(layers.map((l) => [l.id, l]));
  let v = [0, 0, 0];
  layers.forEach((l, i) => {
    if (l.value == null || l.mute) return;
    if (i === 0) { v = [...l.value]; return; }
    v = blendRotation(v, l.value, l.mode ?? MODE.ADDITIVE, effectiveWeight(l, byId), l.rotMode ?? ROT.EULER);
  });
  return v;
}

export const CKR_THRESHOLD = { T: 9e-5, R: 0.009, S: 0.004 };

export function constantKeyReducer(keys, opts = {}) {
  const thr = (opts.threshold ?? CKR_THRESHOLD.R) * (1 + 1e-4);
  const keepOne = opts.keepOneKey ?? true;
  const n = keys.length;
  const flat = (a, b) => {
    const ks = keys.slice(Math.max(0, a), Math.min(n, b + 1)).map((k) => k.v);
    return Math.max(...ks) - Math.min(...ks) <= thr;
  };
  const out = keys.filter((k, i) => !(flat(i - 2, i) && flat(i - 1, i + 1) && flat(i, i + 2)));
  if (out.length === 0 && keepOne && n) return [keys[0]];
  return out;
}

export function resample(evaluate, t0, t1, fps) {
  const out = [];
  const step = 1 / fps;
  for (let i = 0; t0 + i * step < t1 - 1e-9; i++) {
    const t = t0 + i * step;
    out.push({ t, v: evaluate(t) });
  }
  out.push({ t: t1, v: evaluate(t1) });
  return out;
}

export function timeShiftAndScale(keys, shift = 0, scale = 1) {
  return keys.map((k) => ({ ...k, t: (k.t + shift) * scale }));
}

export function keysOnFrame(keys, fps) {
  return keys.map((k) => ({ ...k, t: Math.round(k.t * fps) / fps }));
}
