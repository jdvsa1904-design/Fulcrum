export function matMul(a, b) {
  const o = new Array(16);
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += a[r * 4 + k] * b[k * 4 + c];
    o[r * 4 + c] = s;
  }
  return o;
}

export function matInverse(m) {
  const a = m.slice(), inv = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  for (let c = 0; c < 4; c++) {
    let p = c;
    for (let r = c + 1; r < 4; r++) if (Math.abs(a[r * 4 + c]) > Math.abs(a[p * 4 + c])) p = r;
    if (p !== c) for (let k = 0; k < 4; k++) {
      [a[c * 4 + k], a[p * 4 + k]] = [a[p * 4 + k], a[c * 4 + k]];
      [inv[c * 4 + k], inv[p * 4 + k]] = [inv[p * 4 + k], inv[c * 4 + k]];
    }
    const d = a[c * 4 + c];
    for (let k = 0; k < 4; k++) { a[c * 4 + k] /= d; inv[c * 4 + k] /= d; }
    for (let r = 0; r < 4; r++) if (r !== c) {
      const f = a[r * 4 + c];
      if (f) for (let k = 0; k < 4; k++) { a[r * 4 + k] -= f * a[c * 4 + k]; inv[r * 4 + k] -= f * inv[c * 4 + k]; }
    }
  }
  return inv;
}

export function xformPoint(p, m) {
  const x = p[0], y = p[1], z = p[2];
  const w = x * m[3] + y * m[7] + z * m[11] + m[15];
  return [(x * m[0] + y * m[4] + z * m[8] + m[12]) / w, (x * m[1] + y * m[5] + z * m[9] + m[13]) / w, (x * m[2] + y * m[6] + z * m[10] + m[14]) / w];
}

export function influenceMatrices(bindPre, matrix) {
  return bindPre.map((b, i) => matMul(b, matrix[i]));
}

export function normalizedWeights(w, mode) {
  if (mode !== 'post') return w;
  let s = 0;
  for (const x of w) s += x;
  if (s === 0) return w;
  return w.map(x => x / s);
}

export function linearPoint(pWorld, w, mats) {
  const o = [0, 0, 0];
  for (let i = 0; i < w.length; i++) {
    if (!w[i]) continue;
    const q = xformPoint(pWorld, mats[i]);
    o[0] += w[i] * q[0]; o[1] += w[i] * q[1]; o[2] += w[i] * q[2];
  }
  return o;
}

function qmul(a, b) {
  return [a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0], a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
}

function qrot(q, v) {
  const t = qmul(qmul(q, [v[0], v[1], v[2], 0]), [-q[0], -q[1], -q[2], q[3]]);
  return [t[0], t[1], t[2]];
}

function quatFromRotMatrix(r) {
  const [m00, m01, m02, m10, m11, m12, m20, m21, m22] = r;
  const tr = m00 + m11 + m22;
  let q;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4];
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    q = [s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s];
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    q = [(m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s];
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    q = [(m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s];
  }
  const n = Math.hypot(...q);
  return q.map(x => x / n);
}

export function decompose(m) {
  const X = [m[0], m[1], m[2]], Y = [m[4], m[5], m[6]], Z = [m[8], m[9], m[10]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const sub = (a, b, s) => [a[0] - b[0] * s, a[1] - b[1] * s, a[2] - b[2] * s];
  const sx = Math.hypot(...X);
  const x = X.map(v => v / sx);
  let shXY = dot(x, Y);
  let y = sub(Y, x, shXY);
  const sy = Math.hypot(...y);
  y = y.map(v => v / sy);
  shXY /= sy;
  let shXZ = dot(x, Z);
  let z = sub(Z, x, shXZ);
  let shYZ = dot(y, z);
  z = sub(z, y, shYZ);
  let sz = Math.hypot(...z);
  z = z.map(v => v / sz);
  shXZ /= sz; shYZ /= sz;
  const det = x[0] * (y[1] * z[2] - y[2] * z[1]) - x[1] * (y[0] * z[2] - y[2] * z[0]) + x[2] * (y[0] * z[1] - y[1] * z[0]);
  let scale = [sx, sy, sz], rx = x, ry = y, rz = z;
  if (det < 0) { scale = [-sx, -sy, -sz]; rx = x.map(v => -v); ry = y.map(v => -v); rz = z.map(v => -v); }
  const rot = [rx[0], ry[0], rz[0], rx[1], ry[1], rz[1], rx[2], ry[2], rz[2]];
  return { scale, shear: [shXY, shXZ, shYZ], rotation: quatFromRotMatrix(rot), translation: [m[12], m[13], m[14]] };
}

export function composeSRT(scale, shear, q, t) {
  const S = [scale[0], 0, 0, 0, 0, scale[1], 0, 0, 0, 0, scale[2], 0, 0, 0, 0, 1];
  const H = [1, 0, 0, 0, shear[0], 1, 0, 0, shear[1], shear[2], 1, 0, 0, 0, 0, 1];
  const [x, y, z, w] = q;
  const R = [1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0,
    2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0,
    2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0,
    t[0], t[1], t[2], 1];
  return matMul(matMul(S, H), R);
}

export function dualQuat(q, t) {
  const d = qmul([t[0], t[1], t[2], 0], q).map(v => v * 0.5);
  return { r: q.slice(), d };
}

export function affinePoint(p, m) {
  return [p[0] * m[0] + p[1] * m[4] + p[2] * m[8] + m[12], p[0] * m[1] + p[1] * m[5] + p[2] * m[9] + m[13], p[0] * m[2] + p[1] * m[6] + p[2] * m[10] + m[14]];
}

export function dqsTerms(bindPre, matrix, supportNonRigid = false) {
  if (!supportNonRigid) {
    const dec = decompose(matMul(bindPre, matrix));
    return { dq: dualQuat(dec.rotation, dec.translation), pre: composeSRT(dec.scale, [0, 0, 0], [0, 0, 0, 1], [0, 0, 0]) };
  }
  const W = decompose(matrix), Bw = decompose(matInverse(bindPre));
  const pre = matMul(bindPre, composeSRT(W.scale, W.shear, Bw.rotation, Bw.translation));
  const rigid = decompose(matMul(bindPre, composeSRT(Bw.scale, Bw.shear, W.rotation, W.translation)));
  return { dq: dualQuat(rigid.rotation, rigid.translation), pre };
}

export function isNonRigid(matrix) {
  const d = decompose(matrix);
  return d.scale.some(x => Math.abs(Math.abs(x) - 1) > 1e-10) || d.shear.some(x => Math.abs(x) > 1e-4);
}

export function dqPoint(pWorld, w, terms, scaleBySum = false) {
  const first = w.findIndex(x => x);
  if (first < 0) return pWorld.slice();
  const ref = terms[first].dq.r;
  const r = [0, 0, 0, 0], d = [0, 0, 0, 0], pre = new Array(16).fill(0);
  for (let i = 0; i < w.length; i++) {
    if (!w[i]) continue;
    const t = terms[i];
    const k = (t.dq.r[0] * ref[0] + t.dq.r[1] * ref[1] + t.dq.r[2] * ref[2] + t.dq.r[3] * ref[3]) < 0 ? -w[i] : w[i];
    for (let c = 0; c < 4; c++) { r[c] += k * t.dq.r[c]; d[c] += k * t.dq.d[c]; }
    for (let c = 0; c < 16; c++) pre[c] += w[i] * t.pre[c];
  }
  if (scaleBySum) {
    const s = w.reduce((a, x) => a + x, 0);
    for (let c = 0; c < 16; c++) pre[c] *= s;
  }
  const n = Math.hypot(r[0], r[1], r[2], r[3]);
  const rn = r.map(v => v / n), dn = d.map(v => v / n);
  const rotated = qrot(rn, affinePoint(pWorld, pre));
  const tq = qmul(dn, [-rn[0], -rn[1], -rn[2], rn[3]]);
  return [rotated[0] + 2 * tq[0], rotated[1] + 2 * tq[1], rotated[2] + 2 * tq[2]];
}

export function skinPoints(input) {
  const { points, weights, bindPre, matrix, geomMatrix, method = 'linear', normalize = 'interactive', blendWeights = [], supportNonRigid = false } = input;
  const mats = influenceMatrices(bindPre, matrix);
  const geom = geomMatrix ?? [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const geomInv = matInverse(geom);
  const nonRigid = supportNonRigid && matrix.some(isNonRigid);
  const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const terms = method === 'linear' ? null : bindPre.map((b, i) => {
    const t = dqsTerms(b, matrix[i], supportNonRigid);
    return supportNonRigid && !nonRigid ? { ...t, pre: identity } : t;
  });
  return points.map((p, v) => {
    const pw = xformPoint(p, geom);
    const w = normalizedWeights(weights[v], normalize);
    const empty = !w.some(x => x);
    const lin = () => (empty ? p.slice() : xformPoint(linearPoint(pw, w, mats), geomInv));
    const dq = () => (empty ? pw : xformPoint(dqPoint(pw, w, terms, nonRigid), geomInv));
    if (method === 'linear') return lin();
    if (method === 'dq') return dq();
    const b = blendWeights[v] ?? 0, l = lin(), q = dq();
    return [0, 1, 2].map(k => (1 - b) * l[k] + b * q[k]);
  });
}

export function setInfluenceWeight(weights, index, value, locks = [], relative = false) {
  const n = weights.length, w = weights.slice();
  const target = relative ? w[index] + value : Math.min(1, Math.max(0, value));
  let lockedSum = 0;
  for (let i = 0; i < n; i++) if (i !== index && locks[i]) lockedSum += w[i];
  const avail = Math.max(0, 1 - lockedSum);
  const t = Math.min(target, avail);
  const others = [];
  let otherSum = 0;
  for (let i = 0; i < n; i++) if (i !== index && !locks[i]) { others.push(i); otherSum += w[i]; }
  const rest = avail - t;
  w[index] = t;
  if (!others.length) { w[index] = avail; return w; }
  if (otherSum > 0) for (const i of others) w[i] = w[i] / otherSum * rest;
  else for (const i of others) w[i] = rest / others.length;
  return w;
}
