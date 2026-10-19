const D2R = Math.PI / 180;

export const ROTATE_ORDERS = ['xyz', 'yzx', 'zxy', 'xzy', 'yxz', 'zyx'];
export const ROTATION_ACCUMULATION = ['component', 'layerCumulative', 'layerBlended'];

export function layerWeights({ override = false, passthrough = true, mute = false, weight = 1, parentWeight = 1 } = {}) {
  if (mute) return { fg: 0, bg: 1 };
  const fg = weight * parentWeight;
  if (!override) return { fg, bg: 1 };
  return { fg, bg: passthrough ? 1 - weight : 0 };
}

export function rotationAccumulation({ override = false, rotationAccumulationMode = 'component' } = {}) {
  if (rotationAccumulationMode === 'component' || rotationAccumulationMode === 0) return 'component';
  return override ? 'layerBlended' : 'layerCumulative';
}

export function blendAdditive(a, b, wa, wb) {
  return wa * a + wb * b;
}

function signedPow(x, w) {
  if (w === 0) return 1;
  return Math.sign(x) * Math.pow(Math.abs(x), w);
}

export function blendMultiply(a, b, wa, wb) {
  return signedPow(a, wa) * signedPow(b, wb);
}

export function blendScale(a, b, wa, wb, mode = 'multiply') {
  return mode === 'multiply' || mode === 1 ? blendMultiply(a, b, wa, wb) : blendAdditive(a, b, wa, wb);
}

export function blendBoolean(a, b, wa, wb) {
  return wb > 0 ? b : a;
}

export function qmul(a, b) {
  return [a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0], a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
}

function axisQuat(i, ang) {
  const q = [0, 0, 0, Math.cos(ang / 2)];
  q[i] = Math.sin(ang / 2);
  return q;
}

export function eulerToQuat(e, rotateOrder = 0) {
  const order = typeof rotateOrder === 'string' ? rotateOrder : ROTATE_ORDERS[rotateOrder];
  let q = [0, 0, 0, 1];
  for (const c of order) {
    const i = 'xyz'.indexOf(c);
    q = qmul(axisQuat(i, e[i] * D2R), q);
  }
  return q;
}

export function quatToEulerPrincipal(q, rotateOrder = 0) {
  const order = typeof rotateOrder === 'string' ? rotateOrder : ROTATE_ORDERS[rotateOrder];
  const [x, y, z, w] = q;
  const m = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]];
  const i = 'xyz'.indexOf(order[0]), j = 'xyz'.indexOf(order[1]), k = 'xyz'.indexOf(order[2]);
  const par = (j - i + 3) % 3 === 1 ? 1 : -1;
  const e = [0, 0, 0];
  const cy = Math.hypot(m[i][i], m[j][i]);
  e[j] = Math.atan2(-par * m[k][i], cy);
  if (cy > 1e-12) {
    e[i] = Math.atan2(par * m[k][j], m[k][k]);
    e[k] = Math.atan2(par * m[j][i], m[i][i]);
  } else {
    e[i] = Math.atan2(-par * m[j][k], m[j][j]);
    e[k] = 0;
  }
  return e.map(v => v / D2R);
}

export function alternateEuler(e, rotateOrder = 0) {
  const order = typeof rotateOrder === 'string' ? rotateOrder : ROTATE_ORDERS[rotateOrder];
  const i = 'xyz'.indexOf(order[0]), j = 'xyz'.indexOf(order[1]), k = 'xyz'.indexOf(order[2]);
  const r = [0, 0, 0];
  r[i] = e[i] + 180;
  r[j] = 180 - e[j];
  r[k] = e[k] + 180;
  return r;
}

export function quatToEuler(q, rotateOrder = 0) {
  const e = quatToEulerPrincipal(q, rotateOrder), a = alternateEuler(e, rotateOrder);
  const l1 = v => Math.abs(v[0]) + Math.abs(v[1]) + Math.abs(v[2]);
  return l1(a) < l1(e) ? a : e;
}

export function quatPow(q, w) {
  const [x, y, z, s] = q;
  const n = Math.hypot(x, y, z);
  if (n < 1e-15) return [0, 0, 0, 1];
  const th = Math.atan2(n, s) * w, k = Math.sin(th) / n;
  return [x * k, y * k, z * k, Math.cos(th)];
}

export function slerpShortest(a, b, t) {
  const dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  const bb = dot < 0 ? b.map(v => -v) : b;
  return qmul(a, quatPow(qmul([-a[0], -a[1], -a[2], a[3]], bb), t));
}

export function blendRotationQuat(qa, qb, wa, wb, mode) {
  if (mode === 'layerCumulative') return qmul(quatPow(qa, wa), quatPow(qb, wb));
  const S = Math.abs(wa) + Math.abs(wb);
  if (S === 0) return [0, 0, 0, 1];
  const sa = wa < 0 ? -1 : 1, sb = wb < 0 ? -1 : 1;
  return slerpShortest(quatPow(qa, S * sa), quatPow(qb, S * sb), Math.abs(wb) / S);
}

export function blendRotation(a, b, wa, wb, { mode = 'component', rotateOrder = 0 } = {}) {
  const m = typeof mode === 'number' ? ROTATION_ACCUMULATION[mode] : mode;
  if (m === 'component') return [0, 1, 2].map(i => wa * a[i] + wb * b[i]);
  const q = blendRotationQuat(eulerToQuat(a, rotateOrder), eulerToQuat(b, rotateOrder), wa, wb, m);
  return quatToEuler(q, rotateOrder);
}

export function evaluateLayerStack(base, layers, kind = 'scalar', opts = {}) {
  let out = base;
  for (const layer of layers) {
    if (layer.value === undefined) continue;
    const { fg, bg } = layerWeights(layer);
    if (kind === 'rotation') out = blendRotation(out, layer.value, bg, fg, { mode: rotationAccumulation(layer), rotateOrder: opts.rotateOrder ?? 0 });
    else if (kind === 'scale') out = blendScale(out, layer.value, bg, fg, layer.scaleAccumulationMode ?? 'multiply');
    else if (kind === 'boolean') out = blendBoolean(out, layer.value, bg, fg);
    else out = blendAdditive(out, layer.value, bg, fg);
  }
  return out;
}

export function qinv(q) {
  return [-q[0], -q[1], -q[2], q[3]];
}

export function invertAdditiveB(a, t, wa, wb) {
  return (t - wa * a) / wb;
}

export function invertAdditiveA(b, t, wa, wb) {
  return (t - wb * b) / wa;
}

export function invertMultiplyB(a, t, wa, wb) {
  return signedPow(t / signedPow(a, wa), 1 / wb);
}

export function invertMultiplyA(b, t, wa, wb) {
  return signedPow(t / signedPow(b, wb), 1 / wa);
}

export function invertRotationQuatB(qa, qt, wa, wb, mode) {
  if (mode === 'layerCumulative') return quatPow(qmul(qinv(quatPow(qa, wa)), qt), 1 / wb);
  const S = Math.abs(wa) + Math.abs(wb);
  const sa = wa < 0 ? -1 : 1, sb = wb < 0 ? -1 : 1;
  const P = quatPow(qa, S * sa);
  const dot = P[0] * qt[0] + P[1] * qt[1] + P[2] * qt[2] + P[3] * qt[3];
  const T = dot < 0 ? qt.map(v => -v) : qt;
  const Q = qmul(P, quatPow(qmul(qinv(P), T), S / Math.abs(wb)));
  return quatPow(Q, 1 / (S * sb));
}

export function invertRotationQuatA(qb, qt, wa, wb, mode) {
  if (mode === 'layerCumulative') return quatPow(qmul(qt, qinv(quatPow(qb, wb))), 1 / wa);
  const S = Math.abs(wa) + Math.abs(wb);
  const sa = wa < 0 ? -1 : 1, sb = wb < 0 ? -1 : 1;
  const Q = quatPow(qb, S * sb);
  const dot = Q[0] * qt[0] + Q[1] * qt[1] + Q[2] * qt[2] + Q[3] * qt[3];
  const T = dot < 0 ? qt.map(v => -v) : qt;
  const P = qmul(Q, quatPow(qmul(qinv(Q), T), S / Math.abs(wa)));
  return quatPow(P, 1 / (S * sa));
}
