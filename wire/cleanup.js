import {
  readTrack, butterworthQuat, butterworthVec, peakRemovalQuat, peakRemovalVec,
  adaptiveGaussianQuat, adaptiveGaussian, quatContinuity, qlog, keyReducer,
} from '../filters.js';

const ROT_EPS = 1e-3 * Math.PI / 180;
const POS_EPS = 1e-4;
const MINOR = /twist|corrective|cor_|cor\d|_fwd|_bck|_in_|_in$|_out|knee|bicep|tricep|pec|scap|ankle_|wrist_|metacarpal|weapon|ik_|_ik|prop|attach|socket/i;

const qdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qangle = (a, b) => { const d = qmul([-a[0], -a[1], -a[2], a[3]], b); return 2 * Math.atan2(Math.hypot(d[0], d[1], d[2]), Math.abs(d[3])); };
const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
const smooth = t => t * t * (3 - 2 * t);

function slerp(a, b, t) {
  let d = qdot(a, b), c = b;
  if (d < 0) { d = -d; c = b.map(x => -x); }
  if (d > 0.9995) return qnorm(a.map((x, k) => x + (c[k] - x) * t));
  const th = Math.acos(d), s = Math.sin(th), wa = Math.sin((1 - t) * th) / s, wb = Math.sin(t * th) / s;
  return a.map((x, k) => x * wa + c[k] * wb);
}

function boneSet(sel, nb, names) {
  if (Array.isArray(sel)) return sel.filter(i => Number.isInteger(i) && i >= 0 && i < nb);
  const all = Array.from({ length: nb }, (_, i) => i);
  if (sel === 'major' && names && names.length === nb) return all.filter(i => !MINOR.test(names[i]));
  return all;
}

function movingPosBones(p, nb, n, a, b, bones) {
  return bones.filter(i => {
    for (let k = 0; k < 3; k++) {
      let lo = Infinity, hi = -Infinity;
      for (let f = a; f <= b; f++) { const v = p[(f * nb + i) * 3 + k]; if (v < lo) lo = v; if (v > hi) hi = v; }
      if (hi - lo > POS_EPS * 10) return true;
    }
    return false;
  });
}

function easeWeight(f, a, b, n, e) {
  if (e <= 0) return 1;
  const left = a <= 0 ? 1 : (f - a) / e, right = b >= n - 1 ? 1 : (b - f) / e;
  return smooth(Math.max(0, Math.min(1, left, right)));
}

function commitQuat(out, nb, n, i, a, b, next, weight, cell) {
  let changed = 0, maxRad = 0;
  for (let f = a; f <= b; f++) {
    const o = (f * nb + i) * 4, old = [out[o], out[o + 1], out[o + 2], out[o + 3]];
    let q = [next[f * 4], next[f * 4 + 1], next[f * 4 + 2], next[f * 4 + 3]];
    if (!q.every(Number.isFinite)) continue;
    q = qnorm(q);
    const w = weight ? weight(f) : 1;
    if (w <= 0) continue;
    if (w < 1) q = slerp(old, q, w);
    const ang = qangle(old, q);
    if (!(ang > ROT_EPS)) continue;
    const ref = f > 0 ? [out[o - nb * 4], out[o - nb * 4 + 1], out[o - nb * 4 + 2], out[o - nb * 4 + 3]] : old;
    if (qdot(q, ref) < 0) q = q.map(x => -x);
    out[o] = q[0]; out[o + 1] = q[1]; out[o + 2] = q[2]; out[o + 3] = q[3];
    cell(f);
    changed++;
    if (ang > maxRad) maxRad = ang;
  }
  return { changed, maxRad };
}

function commitVec(out, nb, i, a, b, next, weight, cell) {
  let changed = 0, maxD = 0;
  for (let f = a; f <= b; f++) {
    const o = (f * nb + i) * 3, w = weight ? weight(f) : 1, v = [0, 1, 2].map(k => out[o + k] + (next[f * 3 + k] - out[o + k]) * w);
    if (!v.every(Number.isFinite)) continue;
    const d = Math.hypot(v[0] - out[o], v[1] - out[o + 1], v[2] - out[o + 2]);
    if (!(d > POS_EPS)) continue;
    out[o] = v[0]; out[o + 1] = v[1]; out[o + 2] = v[2];
    cell(f);
    changed++;
    if (d > maxD) maxD = d;
  }
  return { changed, maxD };
}

function vecSeries(tr, comps, from, to, refIdx) {
  const out = [];
  if (comps === 3) { for (let k = from; k <= to; k++) out.push([tr[k * 3], tr[k * 3 + 1], tr[k * 3 + 2]]); return out; }
  const r = [tr[refIdx * 4], tr[refIdx * 4 + 1], tr[refIdx * 4 + 2], tr[refIdx * 4 + 3]], ir = [-r[0], -r[1], -r[2], r[3]];
  for (let k = from; k <= to; k++) {
    let q = [tr[k * 4], tr[k * 4 + 1], tr[k * 4 + 2], tr[k * 4 + 3]];
    if (qdot(q, r) < 0) q = q.map(x => -x);
    out.push(qlog(qmul(ir, q)).map(x => 2 * x));
  }
  return out;
}

const vsub = (u, v) => [u[0] - v[0], u[1] - v[1], u[2] - v[2]];
const vlen = u => Math.hypot(u[0], u[1], u[2]);
const vdot = (u, v) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
const d2 = (V, k) => [0, 1, 2].map(c => V[k - 1][c] - 2 * V[k][c] + V[k + 1][c]);

function bridge(out, orig, comps, L, R) {
  if (comps === 3) {
    for (let k = L + 1; k < R; k++) { const t = (k - L) / (R - L); for (let c = 0; c < 3; c++) out[k * 3 + c] = orig[L * 3 + c] + (orig[R * 3 + c] - orig[L * 3 + c]) * t; }
    return;
  }
  const qL = [orig[L * 4], orig[L * 4 + 1], orig[L * 4 + 2], orig[L * 4 + 3]], qR = [orig[R * 4], orig[R * 4 + 1], orig[R * 4 + 2], orig[R * 4 + 3]];
  for (let k = L + 1; k < R; k++) out.set(slerp(qL, qR, (k - L) / (R - L)), k * 4);
}

function gateSpikes(orig, cand, comps, n, g) {
  const out = Float64Array.from(orig), stats = { accepted: 0, rejected: 0 }, minMag = comps === 3 ? g.minPos : g.minRot;
  const disp = new Float64Array(n);
  for (let f = 0; f < n; f++) {
    disp[f] = comps === 3
      ? Math.hypot(orig[f * 3] - cand[f * 3], orig[f * 3 + 1] - cand[f * 3 + 1], orig[f * 3 + 2] - cand[f * 3 + 2])
      : qangle([orig[f * 4], orig[f * 4 + 1], orig[f * 4 + 2], orig[f * 4 + 3]], [cand[f * 4], cand[f * 4 + 1], cand[f * 4 + 2], cand[f * 4 + 3]]);
  }
  let f = 0;
  while (f < n) {
    if (!(disp[f] > minMag)) { f++; continue; }
    let e = f, pk = f;
    while (e + 1 < n && disp[e + 1] > minMag) { e++; if (disp[e] > disp[pk]) pk = e; }
    f = e + 1;
    let cs = pk, ce = pk;
    while (cs - 1 >= 0 && disp[cs - 1] >= 0.25 * disp[pk]) cs--;
    while (ce + 1 < n && disp[ce + 1] >= 0.25 * disp[pk]) ce++;
    const L = cs - 1, R = ce + 1;
    f = Math.max(f, R);
    let ok = ce - cs + 1 <= g.maxWidth && L >= 0 && R < n;
    if (ok) {
      const lo = Math.max(0, L - 2), hi = Math.min(n - 1, R + 2), V = vecSeries(orig, comps, lo, hi, L), at = k => k - lo;
      const lin = k => V[at(L)].map((x, c) => x + (V[at(R)][c] - x) * (k - L) / (R - L));
      const dS = vsub(V[at(cs)], lin(cs)), dE = vsub(V[at(ce)], lin(ce)), has = k => k - 1 >= lo && k + 1 <= hi;
      let m = 0;
      for (let k = cs; k <= ce; k++) m = Math.max(m, vlen(vsub(V[at(k)], lin(k))));
      let c = 0;
      if (has(L - 1)) c = Math.max(c, vlen(d2(V, at(L - 1))));
      if (has(R + 1)) c = Math.max(c, vlen(d2(V, at(R + 1))));
      const eL = has(L) ? d2(V, at(L)) : null, eR = has(R) ? d2(V, at(R)) : null;
      ok = Boolean(eL || eR) && m > minMag && m > g.ratio * c
        && (!eL || (vdot(eL, dS) > 0 && vlen(eL) > g.ratio * c))
        && (!eR || (vdot(eR, dE) > 0 && vlen(eR) > g.ratio * c));
    }
    if (ok) { bridge(out, orig, comps, L, R); stats.accepted++; }
    else stats.rejected++;
  }
  return { track: out, stats };
}

function runFix(fx, ix, Q, P, nb, n, fps, names) {
  const a = Math.max(0, Math.min(n - 1, Math.round(+fx.a || 0))), b = Math.max(0, Math.min(n - 1, Math.round(fx.b ?? n - 1)));
  const rep = { fix: ix, type: fx.type, a, b, frames: 0, bones: 0, rotCells: 0, posCells: 0, maxDeg: 0, maxPos: 0, spikes: 0 };
  if (b - a < 2) return rep;
  const bones = boneSet(fx.bones ?? 'all', nb, names), posBones = movingPosBones(P, nb, n, a, b, bones);
  const frameHit = new Uint8Array(n), boneHit = new Uint8Array(nb);
  let weight = null, rotOf, posOf;
  if (fx.type === 'despike') {
    const o = { a, b: b + 1, timeTolerance: fx.timeTolerance ?? 3, valueTolerance: fx.valueTolerance ?? 3 };
    const g = { maxWidth: fx.maxWidth ?? 3, ratio: fx.ratio ?? 2, minRot: (fx.minDeg ?? 0.5) * Math.PI / 180, minPos: fx.minPos ?? 0.5 };
    rep.rejected = 0;
    const tally = s => { rep.spikes += s.accepted; rep.rejected += s.rejected; };
    rotOf = tr => { const r = gateSpikes(tr, peakRemovalQuat(tr, o), 4, n, g); tally(r.stats); return r.track; };
    posOf = tr => { const r = gateSpikes(tr, peakRemovalVec(tr, 3, o), 3, n, g); tally(r.stats); return r.track; };
  } else if (fx.type === 'filter') {
    const fc = +fx.fc > 0 ? +fx.fc : 7, o = { a, b, fc, order: fx.order ?? 4 }, e = Math.min(fx.ease ?? 4, Math.floor((b - a) / 2));
    weight = f => easeWeight(f, a, b, n, e);
    rotOf = tr => butterworthQuat(tr, fps, o);
    posOf = tr => butterworthVec(tr, 3, fps, o);
  } else if (fx.type === 'median') {
    const o = { a, b, window: Math.max(3, Math.round(fx.window ?? 5)) | 1 };
    rotOf = tr => adaptiveGaussianQuat(tr, o);
    posOf = tr => adaptiveGaussian(tr, 3, { median: 'medoid', ...o });
  } else return rep;
  for (const i of bones) {
    const r = commitQuat(Q, nb, n, i, a, b, rotOf(readTrack(Q, nb, i, 4)), weight, f => { frameHit[f] = 1; boneHit[i] = 1; rep.rotCells++; });
    rep.maxDeg = Math.max(rep.maxDeg, r.maxRad * 180 / Math.PI);
  }
  for (const i of posBones) {
    const r = commitVec(P, nb, i, a, b, posOf(readTrack(P, nb, i, 3)), weight, f => { frameHit[f] = 1; boneHit[i] = 1; rep.posCells++; });
    rep.maxPos = Math.max(rep.maxPos, r.maxD);
  }
  rep.frames = frameHit.reduce((s, x) => s + x, 0);
  rep.bones = boneHit.reduce((s, x) => s + x, 0);
  return rep;
}

export function applyCleanup({ q, p, nb, n, fps, fixes, names }) {
  const Q = new q.constructor(q), P = new p.constructor(p);
  const N = n ?? q.length / (nb * 4), report = [];
  (fixes || []).forEach((fx, ix) => {
    if (!fx || !['despike', 'filter', 'median'].includes(fx.type)) return;
    report.push(runFix(fx, ix, Q, P, nb, N, fps || 30, names));
  });
  return { q: Q, p: P, report };
}

function matQuat(W, o) {
  const c = k => Math.hypot(W[o + k * 4], W[o + k * 4 + 1], W[o + k * 4 + 2]) || 1;
  const sx = c(0), sy = c(1), sz = c(2);
  const m00 = W[o] / sx, m10 = W[o + 1] / sx, m20 = W[o + 2] / sx;
  const m01 = W[o + 4] / sy, m11 = W[o + 5] / sy, m21 = W[o + 6] / sy;
  const m02 = W[o + 8] / sz, m12 = W[o + 9] / sz, m22 = W[o + 10] / sz;
  const tr = m00 + m11 + m22;
  let x, y, z, w;
  if (tr > 0) { const s = 0.5 / Math.sqrt(tr + 1); w = 0.25 / s; x = (m21 - m12) * s; y = (m02 - m20) * s; z = (m10 - m01) * s; }
  else if (m00 > m11 && m00 > m22) { const s = 2 * Math.sqrt(1 + m00 - m11 - m22); w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s; }
  else if (m11 > m22) { const s = 2 * Math.sqrt(1 + m11 - m00 - m22); w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s; }
  else { const s = 2 * Math.sqrt(1 + m22 - m00 - m11); w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s; }
  return qnorm([x, y, z, w]);
}

function keptFrames(values, prec, n) {
  return keyReducer(values, { precision: prec, maxDepth: n + 2 }).map(k => k.i);
}

export function reduceKeysReport({ W, q, p, nb, n, fps, precisionDeg = 0.1, precisionCm = 0.05, upm = 100 }) {
  const N = n ?? (W ? W.length / (nb * 16) : q.length / (nb * 4));
  const rotPrec = precisionDeg * Math.PI / 180 / 2, posPrec = precisionCm * upm / 100;
  const keptPerBone = new Array(nb).fill(0), ch = new Float64Array(N);
  for (let i = 0; i < nb; i++) {
    const keep = new Uint8Array(N), qt = new Float64Array(N * 4), pt = new Float64Array(N * 3);
    for (let f = 0; f < N; f++) {
      if (W) {
        const o = (f * nb + i) * 16;
        qt.set(matQuat(W, o), f * 4);
        pt[f * 3] = W[o + 12]; pt[f * 3 + 1] = W[o + 13]; pt[f * 3 + 2] = W[o + 14];
      } else {
        for (let k = 0; k < 4; k++) qt[f * 4 + k] = q[(f * nb + i) * 4 + k];
        for (let k = 0; k < 3; k++) pt[f * 3 + k] = p ? p[(f * nb + i) * 3 + k] : 0;
      }
    }
    const qc = quatContinuity(qt), mid = N >> 1, ir = [-qc[mid * 4], -qc[mid * 4 + 1], -qc[mid * 4 + 2], qc[mid * 4 + 3]];
    const lg = new Float64Array(N * 3);
    for (let f = 0; f < N; f++) lg.set(qlog(qmul(ir, [qc[f * 4], qc[f * 4 + 1], qc[f * 4 + 2], qc[f * 4 + 3]])), f * 3);
    for (const [src, prec] of [[lg, rotPrec], [pt, posPrec]]) {
      for (let k = 0; k < 3; k++) {
        for (let f = 0; f < N; f++) ch[f] = src[f * 3 + k];
        for (const f of keptFrames(ch, prec, N)) keep[f] = 1;
      }
    }
    keptPerBone[i] = keep.reduce((s, x) => s + x, 0);
  }
  const totalKept = keptPerBone.reduce((s, x) => s + x, 0), totalFrames = N * nb;
  return { keptPerBone, totalKept, totalFrames, ratio: totalFrames ? totalKept / totalFrames : 0, fps, precisionDeg, precisionCm };
}
