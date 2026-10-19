export const TPS = 141120000;
const PI_T = 3.1415926535;
const TAU = 2 * Math.PI;
const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
const pmod = (a, m) => ((a % m) + m) % m;

const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qinv = q => [-q[0], -q[1], -q[2], q[3]];
const qdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
const qaxis = (k, a) => { const s = Math.sin(a / 2); return [k === 0 ? s : 0, k === 1 ? s : 0, k === 2 ? s : 0, Math.cos(a / 2)]; };
export function qlog(q) {
  const s = Math.hypot(q[0], q[1], q[2]);
  if (s < 1e-12) return [q[0], q[1], q[2]];
  const k = Math.atan2(s, q[3]) / s;
  return [q[0] * k, q[1] * k, q[2] * k];
}
export function qexp(v) {
  const t = Math.hypot(v[0], v[1], v[2]);
  if (t < 1e-12) return qnorm([v[0], v[1], v[2], 1]);
  const k = Math.sin(t) / t;
  return [v[0] * k, v[1] * k, v[2] * k, Math.cos(t)];
}
const getQ = (tr, f) => [tr[f * 4], tr[f * 4 + 1], tr[f * 4 + 2], tr[f * 4 + 3]];
const setQ = (tr, f, q) => { tr[f * 4] = q[0]; tr[f * 4 + 1] = q[1]; tr[f * 4 + 2] = q[2]; tr[f * 4 + 3] = q[3]; };

export function readTrack(arr, nb, i, comps) {
  const n = arr.length / (nb * comps), out = new Float64Array(n * comps);
  for (let f = 0; f < n; f++) for (let k = 0; k < comps; k++) out[f * comps + k] = arr[(f * nb + i) * comps + k];
  return out;
}
export function writeTrack(arr, nb, i, comps, track) {
  const n = track.length / comps;
  for (let f = 0; f < n; f++) for (let k = 0; k < comps; k++) arr[(f * nb + i) * comps + k] = track[f * comps + k];
  return arr;
}

export function butterworthCoefficients(fc, dt, order = 4) {
  let wc = dt * PI_T * fc;
  if (Math.abs(wc) > 2 * PI_T) { const k = Math.trunc(wc * 0.5 / PI_T); wc -= 2 * k * PI_T; }
  const t = Math.tan(wc), t2 = t * t, off = order % 2 === 0 ? 0.5 : 0, secs = [];
  for (let i = 1; i <= order >> 1; i++) {
    const c = Math.cos((i - off) * PI_T / order) * t, d = 2 * c + 1 + t2;
    secs.push({ fo: false, a1: -2 * (t2 - 1) / d, a2: -(1 - 2 * c + t2) / d, g: t2 / d });
  }
  if (order % 2) secs.push({ fo: true, a1: (1 - t) / (1 + t), a2: 0, g: t / (1 + t) });
  return secs;
}

export function butterworthRun(secs, xs, prime = false) {
  const st = secs.map(s => { const w = prime ? xs[0] / (1 - s.a1 - s.a2) : 0; return [w, w]; });
  const out = new Float64Array(xs.length);
  for (let i = 0; i < xs.length; i++) {
    let v = xs[i];
    for (let j = 0; j < secs.length; j++) {
      const s = secs[j], m = st[j], w = v + s.a1 * m[0] + s.a2 * m[1];
      v = s.fo ? (w + m[0]) * s.g : (w + 2 * m[0] + m[1]) * s.g;
      m[1] = m[0]; m[0] = w;
    }
    out[i] = v;
  }
  return out;
}

export function butterworthCurve(f, t0, t1, o = {}) {
  const fc = o.fc ?? 7, sr = o.sr ?? 24, kof = o.kof ?? true, rate = o.internalRate ?? 240, order = o.order ?? 4;
  const mobu = o.variant === 'mobu', prime = o.prime ?? mobu;
  const T0 = Math.round(t0 * TPS), T1 = Math.round(t1 * TPS);
  const first = o.first === undefined ? T0 : Math.round(o.first * TPS), last = o.last === undefined ? T1 : Math.round(o.last * TPS);
  const din = Math.round(TPS / rate), dout = Math.round(TPS / sr), F = tk => f(tk / TPS);
  let s = T0, e = T1;
  if (kof && pmod(s, dout) !== 0) { const q = s - pmod(s, dout); if (q > 0) s = q + dout; }
  s += dout; e -= dout;
  if (e <= s) return null;
  const N = Math.floor((e - s) / din) + 1, P = o.pad ?? (mobu ? 250 : 3 * N), len = 2 * P + N;
  const buf = new Float64Array(len);
  let v0, sl;
  if (s - din < first) { v0 = F(s); sl = F(s + din) - v0; } else { v0 = F(s); sl = v0 - F(s - din); }
  const b0 = v0 - P * sl;
  for (let i = 0; i < P; i++) buf[i] = i * sl + b0;
  let ve;
  if (last < e + din) { ve = F(e); sl = ve - F(e - din); } else { ve = F(e); sl = F(e + din) - ve; }
  const b1 = ve - (N + P - 1) * sl;
  for (let i = N + P; i < len; i++) buf[i] = i * sl + b1;
  let t = s;
  for (let k = 0; k < N; k++) { if (t > e) break; buf[P + k] = F(t); t += din; }
  const secs = butterworthCoefficients(fc, 1 / rate, order);
  const fw = butterworthRun(secs, buf, prime).reverse();
  const bw = butterworthRun(secs, fw, prime).reverse();
  const pts = [[T0 / TPS, F(T0)]];
  for (let k = 0; k < N; k++) pts.push([(s + k * din) / TPS, bw[P + k]]);
  pts.push([T1 / TPS, F(T1)]);
  pts.ticks = [T0, ...Array.from({ length: N }, (_, k) => s + k * din), T1];
  return pts;
}

export function sampleCurve(pts, t) { return evalPts(pts, Math.round(t * TPS)); }

function evalPts(pts, tk) {
  const T = pts.ticks;
  let lo = 0, hi = T.length - 1;
  if (tk <= T[0]) return pts[0][1];
  if (tk >= T[hi]) return pts[hi][1];
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (T[m] <= tk) lo = m; else hi = m; }
  if (T[lo] === tk) return pts[lo][1];
  const u = (tk - T[lo]) / (T[hi] - T[lo]);
  return pts[lo][1] + (pts[hi][1] - pts[lo][1]) * u;
}

export function butterworth(values, fps, o = {}) {
  const n = values.length, a = Math.max(0, o.a ?? 0), b = Math.min(n - 1, o.b ?? n - 1), out = Float64Array.from(values);
  if (b - a < 2) return out;
  const f = sec => {
    const x = sec * fps;
    if (x <= a) return values[a];
    if (x >= b) return values[b];
    const i = Math.floor(x), u = x - i;
    return u === 0 ? values[i] : values[i] + (values[i + 1] - values[i]) * u;
  };
  const pts = butterworthCurve(f, a / fps, b / fps, { ...o, sr: o.sr ?? fps, kof: o.kof ?? true });
  if (!pts) return out;
  for (let k = a + 1; k < b; k++) out[k] = evalPts(pts, Math.round(k / fps * TPS));
  return out;
}

export function butterworthVec(track, comps, fps, o = {}) {
  const n = track.length / comps, out = Float64Array.from(track), ch = new Float64Array(n);
  for (let k = 0; k < comps; k++) {
    for (let f = 0; f < n; f++) ch[f] = track[f * comps + k];
    const r = butterworth(ch, fps, o);
    for (let f = 0; f < n; f++) out[f * comps + k] = r[f];
  }
  return out;
}

export function quatContinuity(track) {
  const n = track.length / 4, out = Float64Array.from(track);
  let flips = 0;
  for (let f = 1; f < n; f++) {
    if (qdot(getQ(out, f), getQ(out, f - 1)) < 0) { for (let k = 0; k < 4; k++) out[f * 4 + k] = -out[f * 4 + k]; flips++; }
  }
  out.flips = flips;
  return out;
}

function relLogs(track, a, b, ref) {
  const al = quatContinuity(track), v = new Float64Array((b - a + 1) * 3);
  const r = qdot(getQ(al, (a + b) >> 1), ref) < 0 ? ref.map(x => -x) : ref, ir = qinv(r);
  for (let f = a; f <= b; f++) v.set(qlog(qmul(ir, getQ(al, f))), (f - a) * 3);
  return { v, ref: r };
}

export function butterworthQuat(track, fps, o = {}) {
  const n = track.length / 4, a = Math.max(0, o.a ?? 0), b = Math.min(n - 1, o.b ?? n - 1), out = Float64Array.from(track);
  if (b - a < 2) return out;
  const ref = o.ref ?? getQ(track, (a + b) >> 1);
  const { v, ref: r } = relLogs(track, a, b, ref);
  const fv = butterworthVec(v, 3, fps, { ...o, a: 0, b: b - a });
  for (let f = a + 1; f < b; f++) {
    let q = qmul(r, qexp([fv[(f - a) * 3], fv[(f - a) * 3 + 1], fv[(f - a) * 3 + 2]]));
    if (qdot(q, getQ(track, f)) < 0) q = q.map(x => -x);
    setQ(out, f, q);
  }
  return out;
}

function peakStep(v, t, a, b, tt, vt) {
  const m = b - a;
  if (m < 3) return [];
  let mdt = 0;
  for (let i = a + 1; i < b; i++) mdt += (t(i) - t(i - 1)) ** 2;
  mdt /= m;
  const lim = mdt * tt, sMin = new Float64Array(m), sMax = new Float64Array(m);
  let msp = 0;
  for (let i = a; i < b; i++) {
    let lo = Infinity, hi = -Infinity;
    for (let j = i - 1; j >= a && (t(j) - t(i)) ** 2 < lim; j--) { const s = (v[j] - v[i]) / (t(j) - t(i)); if (s < lo) lo = s; if (s > hi) hi = s; }
    for (let j = i + 1; j < b && (t(j) - t(i)) ** 2 < lim; j++) { const s = (v[j] - v[i]) / (t(j) - t(i)); if (s < lo) lo = s; if (s > hi) hi = s; }
    if (lo === Infinity) { lo = 0; hi = 0; }
    sMin[i - a] = lo; sMax[i - a] = hi;
    msp += (hi - lo) ** 2;
  }
  msp /= m;
  const thr = msp * vt, over = i => (sMax[i - a] - sMin[i - a]) ** 2 > thr, done = new Uint8Array(m), spikes = [];
  for (let i = a; i < b; i++) {
    if (done[i - a] || !over(i) || !(sMin[i - a] * sMax[i - a] < 0)) continue;
    let L = i - 1, R = i + 1;
    while (L >= a && over(L)) L--;
    while (R < b && over(R)) R++;
    if (L < a || R >= b) continue;
    const tL = t(L), tR = t(R), vL = v[L], vR = v[R];
    for (let k = L + 1; k < R; k++) { v[k] = vR - ((tR - t(k)) / (tR - tL)) * (vR - vL); done[k - a] = 1; }
    spikes.push(i);
  }
  return spikes;
}

export function peakRemoval(values, o = {}) {
  const v = Float64Array.from(values), n = v.length, times = o.times, t = times ? i => times[i] : i => i;
  const a = Math.max(0, o.a ?? 0), b = Math.min(n, o.b ?? n), passes = o.passes ?? 20;
  const removed = [];
  let p = 0;
  for (; p < passes; p++) {
    const r = peakStep(v, t, a, b, o.timeTolerance ?? 3, o.valueTolerance ?? 3);
    if (!r.length) break;
    removed.push(...r);
  }
  return { values: v, removed, passes: p };
}

export function peakRemovalVec(track, comps, o = {}) {
  const n = track.length / comps, out = Float64Array.from(track), ch = new Float64Array(n), removed = new Set();
  for (let k = 0; k < comps; k++) {
    for (let f = 0; f < n; f++) ch[f] = track[f * comps + k];
    const r = peakRemoval(ch, o);
    for (let f = 0; f < n; f++) out[f * comps + k] = r.values[f];
    r.removed.forEach(i => removed.add(i));
  }
  out.removed = [...removed].sort((x, y) => x - y);
  return out;
}

export function peakRemovalQuat(track, o = {}) {
  const n = track.length / 4, ref = o.ref ?? getQ(track, n >> 1), { v, ref: r } = relLogs(track, 0, n - 1, ref);
  const fv = peakRemovalVec(v, 3, o), out = Float64Array.from(track);
  for (let f = 0; f < n; f++) {
    let q = qmul(r, qexp([fv[f * 3], fv[f * 3 + 1], fv[f * 3 + 2]]));
    if (qdot(q, getQ(track, f)) < 0) q = q.map(x => -x);
    setQ(out, f, q);
  }
  out.removed = fv.removed;
  return out;
}

const AXI = { x: 0, y: 1, z: 2 };
const orderIdx = order => { const o = order.toLowerCase(); return [AXI[o[0]], AXI[o[1]], AXI[o[2]]]; };
function quatMat(q) {
  const [x, y, z, w] = q;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]];
}

export function eulerFromQuat(q, order = 'xyz') {
  const [r, m, p] = orderIdx(order), M = quatMat(qnorm(q)), s = (m - p + 3) % 3 === 1 ? 1 : -1, e = [0, 0, 0];
  const sb = clamp(s * M[p][r], -1, 1);
  e[m] = Math.asin(sb);
  if (Math.abs(sb) < 0.9999999) { e[p] = Math.atan2(-s * M[m][r], M[r][r]); e[r] = Math.atan2(-s * M[p][m], M[p][p]); }
  else { e[p] = Math.atan2(s * M[r][m], M[m][m]); e[r] = 0; }
  return e;
}

export function quatFromEuler(e, order = 'xyz') {
  const [r, m, p] = orderIdx(order);
  return qmul(qmul(qaxis(p, e[p]), qaxis(m, e[m])), qaxis(r, e[r]));
}

export function eulerEquivalent(e, order = 'xyz') {
  const [r, m, p] = orderIdx(order), o = e.slice();
  o[m] = Math.PI - e[m]; o[p] = e[p] + Math.PI; o[r] = e[r] + Math.PI;
  return o;
}

const cut = (c, p) => c + TAU * Math.round((p - c) / TAU);
const bound = c => c - TAU * Math.round(c / TAU);
const dist = (a, b) => Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]) + Math.abs(a[2] - b[2]);

export function eulerFilter(track, o = {}) {
  const mode = o.mode ?? 'solution', order = o.order ?? 'xyz', n = track.length / 3, out = new Float64Array(track.length);
  let prev = o.initial ?? null;
  for (let f = 0; f < n; f++) {
    const e = [track[f * 3], track[f * 3 + 1], track[f * 3 + 2]];
    let c;
    if (mode === 'bound') c = e.map(bound);
    else if (!prev) c = e;
    else {
      const c1 = e.map((x, k) => cut(x, prev[k]));
      if (mode === 'cut') c = c1;
      else { const c2 = eulerEquivalent(e, order).map((x, k) => cut(x, prev[k])); c = dist(c1, prev) <= dist(c2, prev) ? c1 : c2; }
    }
    out.set(c, f * 3);
    prev = c;
  }
  return out;
}

export function unrollQuats(track, o = {}) {
  const n = track.length / 4, order = o.order ?? 'xyz', e = new Float64Array(n * 3);
  for (let f = 0; f < n; f++) e.set(eulerFromQuat(getQ(track, f), order), f * 3);
  return eulerFilter(e, { ...o, mode: o.mode ?? 'solution', order });
}

export function unrollQuality(e1, e2, order = 'xyz') {
  const q = qmul(quatFromEuler(e1, order), qinv(quatFromEuler(e2, order)));
  const th = 2 * Math.acos(Math.min(1, Math.abs(q[3]))) * 180 / Math.PI;
  const de = ((e1[0] - e2[0]) ** 2 + (e1[1] - e2[1]) ** 2 + (e1[2] - e2[2]) ** 2) * (180 / Math.PI) ** 2;
  return de <= 1e-12 ? 1 : th * th / de;
}

export function unrollFlags(eulerTrack, o = {}) {
  const n = eulerTrack.length / 3, order = o.order ?? 'xyz', tol = o.tolerance ?? 0.5, out = [];
  for (let f = 1; f < n; f++) {
    const qf = unrollQuality(Array.from(eulerTrack.subarray(f * 3, f * 3 + 3)), Array.from(eulerTrack.subarray(f * 3 - 3, f * 3)), order);
    if (qf < tol) out.push({ f, qf });
  }
  return out;
}

export const KEY_THRESHOLDS = {
  mobu: { translation: 0.009, rotation: 0.009, scaling: 9e-5, default: 0.009, derivative: 0.004 },
  mayaFbx: { translation: 0.0001, rotation: 0.009, scaling: 0.004 },
};

export function constantKeyReducer(values, o = {}) {
  const v = values, n = v.length, t = o.times ? i => o.times[i] : i => i;
  const thr = o.threshold ?? 0.009, dtol = o.derivativeTolerance ?? 0.004, kfl = o.keepFirstAndLast ?? true, k1 = o.keepOneKey ?? true;
  if (n === 0) return [];
  if (n === 1) return k1 ? [0] : [];
  const kept = [0];
  for (let i = 1; i < n - 1; i++) {
    const L = kept[kept.length - 1];
    const dl = (v[i] - v[i - 1]) / (t(i) - t(i - 1)), dr = (v[i + 1] - v[i]) / (t(i + 1) - t(i));
    if (Math.abs(v[i] - v[L]) < thr && Math.abs(v[i + 1] - v[i]) < thr && Math.abs(dl - dr) < dtol) continue;
    kept.push(i);
  }
  kept.push(n - 1);
  if (kfl) return kept;
  const res = kept.slice();
  if (res.length >= 2 && Math.abs(v[res[res.length - 1]] - v[res[res.length - 2]]) < thr) res.pop();
  if (res.length >= 2 && Math.abs(v[res[1]] - v[res[0]]) < thr) res.shift();
  if (res.length === 1 && !k1) return [];
  return res;
}

export function evalLinearKeys(values, kept, i, times) {
  const t = times ? k => times[k] : k => k, x = t(i);
  let j = 0;
  while (j < kept.length - 1 && t(kept[j + 1]) < x) j++;
  if (kept.length === 1) return values[kept[0]];
  const A = kept[j], B = kept[Math.min(j + 1, kept.length - 1)];
  if (x <= t(A)) return values[A];
  if (x >= t(B)) return values[B];
  return values[A] + (values[B] - values[A]) * (x - t(A)) / (t(B) - t(A));
}

function hermite(A, B, x) {
  const h = B.t - A.t, s = (x - A.t) / h, s2 = s * s, s3 = s2 * s;
  return (2 * s3 - 3 * s2 + 1) * A.v + (s3 - 2 * s2 + s) * h * A.dr + (-2 * s3 + 3 * s2) * B.v + (s3 - s2) * h * B.dl;
}

export function evalKeys(keys, x) {
  if (x <= keys[0].t) return keys[0].v;
  const L = keys[keys.length - 1];
  if (x >= L.t) return L.v;
  let j = 0;
  while (keys[j + 1].t < x) j++;
  return hermite(keys[j], keys[j + 1], x);
}

export function keyReducer(values, o = {}) {
  const v = values, n = v.length, t = o.times ? i => o.times[i] : i => i, prec = o.precision ?? 1, maxDepth = o.maxDepth ?? 10;
  const mode = o.tangents ?? 'auto';
  const dl = new Float64Array(n), dr = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const back = i > 0 ? (v[i] - v[i - 1]) / (t(i) - t(i - 1)) : null, fwd = i < n - 1 ? (v[i + 1] - v[i]) / (t(i + 1) - t(i)) : null;
    if (o.dl && o.dr) { dl[i] = o.dl[i]; dr[i] = o.dr[i]; }
    else if (mode === 'linear') { dl[i] = back ?? fwd ?? 0; dr[i] = fwd ?? back ?? 0; }
    else { const c = back === null ? fwd ?? 0 : fwd === null ? back : (v[i + 1] - v[i - 1]) / (t(i + 1) - t(i - 1)); dl[i] = c; dr[i] = c; }
  }
  const key = i => ({ i, t: t(i), v: v[i], dl: dl[i], dr: dr[i], smooth: dl[i] === dr[i] });
  if (n <= 2) return Array.from({ length: n }, (_, i) => key(i));
  const keys = new Map([[0, key(0)], [n - 1, key(n - 1)]]);
  const sub = (a, b, depth) => {
    if (b - a < 2) return;
    const A = keys.get(a), B = keys.get(b);
    let best = -1, be = -1;
    for (let k = a + 1; k < b; k++) { const e = (hermite(A, B, t(k)) - v[k]) ** 2; if (e > be) { be = e; best = k; } }
    if (be <= prec * prec || depth >= maxDepth) return;
    keys.set(best, key(best));
    sub(a, best, depth + 1);
    sub(best, b, depth + 1);
  };
  sub(0, n - 1, 0);
  return [...keys.values()].sort((x, y) => x.t - y.t);
}

export function gaussianKernel(window = 5, sigma) {
  const h = window >> 1, s = sigma ?? Math.max(1, h / 2), g = [];
  for (let k = -h; k <= h; k++) g.push(Math.exp(-(k * k) / (2 * s * s)));
  return g;
}

export function adaptiveGaussian(samples, dim, o = {}) {
  const n = samples.length / dim, N = (o.window ?? 5) | 1, h = N >> 1, g = o.kernel ?? gaussianKernel(N, o.sigma);
  const a = Math.max(0, o.a ?? 0), b = Math.min(n - 1, o.b ?? n - 1), med = o.median ?? 'norm', sc = o.scale ?? 1, out = Float64Array.from(samples);
  const mirror = i => clamp(i < 0 ? -i : i >= n ? 2 * n - i - 2 : i, 0, n - 1);
  const W = [];
  for (let f = a; f <= b; f++) {
    W.length = 0;
    for (let k = -h; k <= h; k++) { const j = mirror(f + k); W.push(Array.from(samples.subarray ? samples.subarray(j * dim, j * dim + dim) : samples.slice(j * dim, j * dim + dim))); }
    let m;
    if (med === 'medoid') {
      let bs = Infinity;
      for (const x of W) { let s = 0; for (const y of W) { let d = 0; for (let c = 0; c < dim; c++) d += (x[c] - y[c]) ** 2; s += Math.sqrt(d); } if (s < bs) { bs = s; m = x; } }
    } else {
      const srt = W.map((x, k) => [x.reduce((s, c) => s + c * c, 0), k]).sort((x, y) => x[0] - y[0]);
      m = W[srt[h][1]];
    }
    const w = W.map((x, k) => { let d = 0; for (let c = 0; c < dim; c++) d += (x[c] - m[c]) ** 2; return g[k] / (Math.sqrt(d) * sc + 1); });
    const sw = w.reduce((s, x) => s + x, 0);
    for (let c = 0; c < dim; c++) { let s = 0; for (let k = 0; k < N; k++) s += w[k] * W[k][c]; out[f * dim + c] = s / sw; }
  }
  return out;
}

export function adaptiveGaussianQuat(track, o = {}) {
  const n = track.length / 4, a = Math.max(0, o.a ?? 0), b = Math.min(n - 1, o.b ?? n - 1);
  const ref = o.ref ?? getQ(track, (a + b) >> 1), { v, ref: r } = relLogs(track, 0, n - 1, ref);
  const fv = adaptiveGaussian(v, 3, { median: 'medoid', scale: 360 / Math.PI, ...o, a, b }), out = Float64Array.from(track);
  for (let f = a; f <= b; f++) {
    let q = qmul(r, qexp([fv[f * 3], fv[f * 3 + 1], fv[f * 3 + 2]]));
    if (qdot(q, getQ(track, f)) < 0) q = q.map(x => -x);
    setQ(out, f, q);
  }
  return out;
}
