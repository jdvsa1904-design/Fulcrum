const EPS_BEZ = 1e-10;
const D2R = Math.PI / 180;

export const TANGENT_TYPES = ['fixed', 'linear', 'flat', 'step', 'stepnext', 'spline', 'clamped', 'plateau', 'auto', 'autoease', 'automix', 'autocustom'];

export function hermiteCoeffs(x0, y0, m1, m2, x3, y3) {
  const dx = x3 - x0, dy = y3 - y0;
  const tol1 = Math.abs(m1 * dx * 2e-7), tol2 = Math.abs(m2 * dx * 2e-7);
  if (Math.abs(m1 * dx - dy) <= tol1 && Math.abs(m2 * dx - dy) <= tol2) return { x0, a: 0, b: 0, c: dy / dx, d: y0 };
  const d1 = dx * m1, d2 = dx * m2, inv = 1 / (dx * dx);
  return { x0, a: (d1 + d2 - dy - dy) * inv / dx, b: (dy + dy + dy - d1 - d1 - d2) * inv, c: m1, d: y0 };
}

export function hermiteEval(h, t) {
  const s = t - h.x0;
  return ((h.a * s + h.b) * s + h.c) * s + h.d;
}

function constrainInsideBounds(a, b) {
  if (a + EPS_BEZ < 4 / 3) {
    const B = a - 2, disc = B * B - 4 * (a - 1) * (a - 1), r = Math.sqrt(disc);
    const hi = (r - B) * 0.5;
    if (b + EPS_BEZ > hi) b = hi - EPS_BEZ;
    else {
      const lo = (-B - r) * 0.5 + EPS_BEZ;
      if (lo > b) b = lo;
    }
    return [a, b];
  }
  return [1.3333333333256, 0.33333333332];
}

export function checkMonotonic(u1, u2, flag) {
  let v = 1 - u2;
  if (u1 < 0) u1 = 0;
  if (v < 0) v = 0;
  if (u1 > 1 || v > 1) {
    if ((u1 - 2 + v) * u1 + (v - 2) * v + 1 + EPS_BEZ > 0) {
      if (flag) [u1, v] = constrainInsideBounds(u1, v);
      else [v, u1] = constrainInsideBounds(v, u1);
    }
  }
  return [u1, 1 - v];
}

function toPower(p0, p1, p2, p3) {
  const a = p1 - p0, b = p2 - p1, c = p3 - p2, d = b - a;
  return [p3 - p0 - 3 * b, 3 * d, 3 * a, p0];
}

export function bezierSegment(x0, y0, x1, y1, x2, y2, x3, y3, flag = true) {
  const dx = x3 - x0;
  const u1o = (x1 - x0) / dx, u2o = (x2 - x0) / dx;
  const linearTime = Math.abs(u1o - 1 / 3) <= 1e-6 && Math.abs(u2o - 2 / 3) <= 1e-6;
  let u1 = u1o < 0 ? 0 : u1o, u2 = u2o > 1 ? 1 : u2o;
  if (u1 > 1 || u2 < 0) [u1, u2] = checkMonotonic(u1, u2, flag);
  if (u1 !== u1o && Math.abs(u1o) > EPS_BEZ) y1 = y0 + (y1 - y0) * u1 / u1o;
  if (u2 !== u2o && Math.abs(1 - u2o) > EPS_BEZ) y2 = y3 - (y3 - y2) * (1 - u2) / (1 - u2o);
  return { x0, x3, linearTime, tx: toPower(0, u1, u2, 1), ty: toPower(y0, y1, y2, y3) };
}

function poly(c, u) { return ((c[0] * u + c[1]) * u + c[2]) * u + c[3]; }
function dpoly(c, u) { return (3 * c[0] * u + 2 * c[1]) * u + c[2]; }

function solveUnit(c, s) {
  let lo = 0, hi = 1, u = s;
  for (let i = 0; i < 100; i++) {
    const f = poly(c, u) - s;
    if (Math.abs(f) < 1e-15) return u;
    if (f > 0) hi = u; else lo = u;
    const d = dpoly(c, u);
    let n = d !== 0 ? u - f / d : NaN;
    if (!(n > lo && n < hi)) n = (lo + hi) / 2;
    if (Math.abs(n - u) < 1e-16) return n;
    u = n;
  }
  return u;
}

export function bezierEval(b, t) {
  let s;
  if (Math.abs(t - b.x0) <= EPS_BEZ) s = 0;
  else if (Math.abs(t - b.x3) <= EPS_BEZ) s = 1;
  else s = (t - b.x0) / (b.x3 - b.x0);
  const u = b.linearTime ? s : solveUnit(b.tx, s);
  return poly(b.ty, u);
}

function seg(curve, i) {
  const k0 = curve.keys[i], k1 = curve.keys[i + 1];
  if (curve.weighted) {
    const oa = k0.outAngle * D2R, ia = k1.inAngle * D2R;
    return {
      kind: 'b', b: bezierSegment(k0.t, k0.v, k0.t + k0.outWeight * Math.cos(oa), k0.v + k0.outWeight * Math.sin(oa),
        k1.t - k1.inWeight * Math.cos(ia), k1.v - k1.inWeight * Math.sin(ia), k1.t, k1.v, curve.monotonicFlag ?? true),
    };
  }
  return { kind: 'h', h: hermiteCoeffs(k0.t, k0.v, Math.tan(k0.outAngle * D2R), Math.tan(k1.inAngle * D2R), k1.t, k1.v) };
}

function findSegment(keys, t) {
  let lo = 0, hi = keys.length - 1;
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1;
    if (keys[m].t <= t) lo = m; else hi = m;
  }
  return lo;
}

export function evaluateInRange(curve, t) {
  const keys = curve.keys, n = keys.length;
  if (t <= keys[0].t) return keys[0].v;
  if (t >= keys[n - 1].t) return keys[n - 1].v;
  const i = findSegment(keys, t);
  if (t === keys[i].t) return keys[i].v;
  const k0 = keys[i];
  if (k0.outType === 'step') return k0.v;
  if (k0.outType === 'stepnext') return keys[i + 1].v;
  curve._cache ??= new Map();
  let s = curve._cache.get(i);
  if (!s) { s = seg(curve, i); curve._cache.set(i, s); }
  return s.kind === 'h' ? hermiteEval(s.h, t) : bezierEval(s.b, t);
}

function endSlope(key, isIn) {
  const a = (isIn ? key.inAngle : key.outAngle) * D2R;
  return Math.tan(a);
}

export function evaluate(curve, t) {
  const keys = curve.keys, n = keys.length;
  if (!n) return 0;
  if (n === 1) {
    const k = keys[0];
    if (t < k.t && curve.preInfinity === 'linear') return k.v + (t - k.t) * endSlope(k, true);
    if (t > k.t && curve.postInfinity === 'linear') return k.v + (t - k.t) * endSlope(k, false);
    return k.v;
  }
  const t0 = keys[0].t, t1 = keys[n - 1].t, span = t1 - t0, dv = keys[n - 1].v - keys[0].v;
  if (t < t0) {
    const mode = curve.preInfinity ?? 'constant';
    if (mode === 'constant') return keys[0].v;
    if (mode === 'linear') {
      const k = keys[0];
      return k.v + (t - t0) * endSlope(k, true);
    }
    return wrapEval(curve, t, mode, t0, span, dv);
  }
  if (t > t1) {
    const mode = curve.postInfinity ?? 'constant';
    if (mode === 'constant') return keys[n - 1].v;
    if (mode === 'linear') {
      const k = keys[n - 1];
      return k.v + (t - t1) * endSlope(k, false);
    }
    return wrapEval(curve, t, mode, t0, span, dv);
  }
  return evaluateInRange(curve, t);
}

function wrapEval(curve, t, mode, t0, span, dv) {
  const c = Math.floor((t - t0) / span);
  let local = t - c * span;
  if (mode === 'cycle') return evaluateInRange(curve, local);
  if (mode === 'cycleRelative') return evaluateInRange(curve, local) + c * dv;
  if (mode === 'oscillate') {
    if (c % 2 !== 0) local = t0 + span - (local - t0);
    return evaluateInRange(curve, local);
  }
  return evaluateInRange(curve, local);
}

function vecToKey(key, side, x, y, weighted) {
  const ang = Math.atan2(y, x) / D2R;
  if (side === 'in') { key.inAngle = ang; if (weighted) key.inWeight = Math.hypot(x, y) / 3; }
  else { key.outAngle = ang; if (weighted) key.outWeight = Math.hypot(x, y) / 3; }
}

function keyVec(key, side, weighted) {
  const a = (side === 'in' ? key.inAngle : key.outAngle) * D2R;
  const w = weighted ? (side === 'in' ? key.inWeight : key.outWeight) * 3 : 1;
  return [w * Math.cos(a), w * Math.sin(a)];
}

function smoothVec(p, k, n, side, weighted, fps) {
  if (!p && !n) return [fps, 0];
  if (!p) return [n.t - k.t, n.v - k.v];
  if (!n) return [k.t - p.t, k.v - p.v];
  const s = (n.v - p.v) / (n.t - p.t);
  if (!weighted) return [1, s];
  const x = side === 'in' ? k.t - p.t : n.t - k.t;
  return [x, x * s];
}

function linearVec(p, k, n, side, fps) {
  if (side === 'in') return p ? [k.t - p.t, k.v - p.v] : [fps, 0];
  return n ? [n.t - k.t, n.v - k.v] : [fps, 0];
}

function flatVec(p, k, n, side) {
  let a = p ? k.t - p.t : 0, b = n ? n.t - k.t : 0;
  if (!p) a = b;
  if (!n) b = a;
  return [side === 'in' ? a : b, 0];
}

export const LEGACY_SLOPE = { fast: 1.7319999933242798, slow: 0.5771999955177307 };

function legacyVec(p, k, n, side, ty, fps, unitScale) {
  const m = LEGACY_SLOPE[ty] * unitScale / fps;
  let x = 0;
  if (side === 'in') x = p ? k.t - p.t : n ? n.t - k.t : 0;
  else x = n ? n.t - k.t : p ? k.t - p.t : 0;
  return [x, x * m];
}

function oldSmoothVec(p, k, n, side, weighted, fps, unitScale) {
  if (!weighted || !p || !n) return smoothVec(p, k, n, side, weighted, fps);
  const tp = (k.t - p.t) / fps, tn = (n.t - k.t) / fps;
  const dp = (k.v - p.v) / unitScale, dn = (n.v - k.v) / unitScale;
  const sp = tp > 0.01 ? dp / tp : 0, sn = tn > 0.01 ? dn / tn : 0;
  const lp = Math.sqrt(dp * dp + tp * tp), ln = Math.sqrt(dn * dn + tn * tn);
  const s = (lp * sn + ln * sp) / (ln + lp);
  const usePrev = (s * tn) * (s * tn) + tn * tn > tp * tp + (s * tp) * (s * tp);
  const x = usePrev ? k.t - p.t : n.t - k.t;
  return [x, x * s * unitScale / fps];
}

const AUTO_MODE = { auto: 18, automix: 27, autoease: 28, autocustom: 29 };

export function autoSlope(p, k, n, type, coefficient = -0.5) {
  const h1 = k.t - p.t, h2 = n.t - k.t, H = n.t - p.t;
  const d1 = k.v - p.v, d2 = n.v - k.v;
  const lim1 = 3 * d1 / h1, lim2 = 3 * d2 / h2;
  let s;
  if (type === 'auto') s = (n.v - p.v) / H;
  else {
    const a = h1 / H, m1 = d1 / h1, m2 = d2 / h2;
    if (type === 'automix') s = (1 - a) * m1 + a * m2;
    else {
      const c = Math.fround(type === 'autocustom' ? coefficient : -0.5);
      const b = a - 0.5;
      const w = (b * b * Math.fround(4 * c) + Math.fround(1 - c)) * b + 0.5;
      s = (1 - w) * m1 + w * m2;
    }
  }
  const sg = x => (x < 0 ? -1 : 1);
  if (sg(lim1) !== sg(lim2) || sg(s) !== sg(lim2)) return 0;
  if (lim2 >= 0) return Math.min(s, lim1, lim2);
  return Math.max(s, lim1, lim2);
}

function hullOverlaps(a, aOutVec, b, bInVec, weighted) {
  let y1, y2;
  if (weighted) {
    y1 = a.v + aOutVec[1] / 3;
    y2 = b.v - bInVec[1] / 3;
  } else {
    const dt3 = (b.t - a.t) / 3;
    y1 = a.v + (aOutVec[0] !== 0 ? aOutVec[1] / aOutVec[0] * dt3 : aOutVec[1]);
    y2 = b.v - (bInVec[0] !== 0 ? bInVec[1] / bInVec[0] * dt3 : bInVec[1]);
  }
  const mn = Math.min(a.v, b.v, y1, y2), mx = Math.max(a.v, b.v, y1, y2);
  return { mn, mx };
}

function overshoots(v, vo, box) {
  const e = 1e-10;
  if (v >= box.mn + e && vo >= box.mn + e) return true;
  if (box.mx - e >= v && box.mx - e >= vo) return true;
  return false;
}

export function computeTangents(curve, opts = {}) {
  const fps = opts.fps ?? 24, normScale = opts.normScale ?? 1, coefficient = opts.autoCustomCoefficient ?? -0.5;
  const weighted = !!curve.weighted;
  const unitScale = opts.unitScale ?? curve.unitScale ?? 1;
  const keys = curve.keys.map(k => ({ ...k }));
  const passes = opts.passes ?? 2;
  for (let pass = 0; pass < passes; pass++) {
    for (let i = 0; i < keys.length; i++) {
      const p = keys[i - 1], k = keys[i], n = keys[i + 1];
      const set = (side, vec) => vecToKey(k, side, vec[0], vec[1], weighted);
      const types = { in: k.inType, out: k.outType };
      const done = {};
      for (const side of ['in', 'out']) {
        const ty = types[side];
        if (ty === 'linear') { set(side, linearVec(p, k, n, side, fps)); done[side] = 1; }
        else if (ty === 'flat') { set(side, flatVec(p, k, n, side)); done[side] = 1; }
        else if (ty === 'spline') { set(side, smoothVec(p, k, n, side, weighted, fps)); done[side] = 1; }
        else if (ty === 'step' || ty === 'stepnext') { if (weighted) { if (side === 'in') { k.inAngle = 0; k.inWeight = 0; } else { k.outAngle = 0; k.outWeight = 0; } } else if (side === 'in') k.inAngle = 0; else k.outAngle = 0; done[side] = 1; }
        else if (ty === 'fast' || ty === 'slow') { set(side, legacyVec(p, k, n, side, ty, fps, unitScale)); done[side] = 1; }
        else if (ty === 'oldSmooth') { set(side, oldSmoothVec(p, k, n, side, weighted, fps, unitScale)); done[side] = 1; }
        else if (ty === 'oldClamped') {
          const tol = normScale > 1e-10 ? 0.05 / normScale : 0.05;
          const o = side === 'in' ? p : n;
          if (o) set(side, Math.abs(o.v - k.v) / unitScale <= tol ? flatVec(p, k, n, side) : oldSmoothVec(p, k, n, side, weighted, fps, unitScale));
          done[side] = 1;
        }
        else if (ty in AUTO_MODE) {
          if (!p || !n) set(side, flatVec(p, k, n, side));
          else {
            const s = autoSlope(p, k, n, ty, coefficient);
            const x = side === 'in' ? k.t - p.t : n.t - k.t;
            set(side, [x, x * s]);
          }
          done[side] = 1;
        }
      }
      if (!p && types.in === 'oldClamped') { k.inAngle = k.outAngle; if (weighted) k.inWeight = k.outWeight; }
      if (!n && types.out === 'oldClamped') { k.outAngle = k.inAngle; if (weighted) k.outWeight = k.inWeight; }
      if (types.in === 'clamped' || types.out === 'clamped') {
        const tol = normScale > 1e-10 ? 0.05 / normScale : 0.05;
        const near = (vp, vn) => Math.abs(vp - k.v) <= tol || Math.abs(vn - k.v) <= tol;
        if (p && types.in === 'clamped') {
          const vp = p.v, vn = n ? n.v : p.v;
          if (near(vp, vn)) { set('in', flatVec(p, k, n, 'in')); if (types.out === 'clamped') set('out', flatVec(p, k, n, 'out')); }
          else set('in', smoothVec(p, k, n, 'in', weighted, fps));
        }
        if (n && types.out === 'clamped') {
          const vn = n.v, vp = p ? p.v : n.v;
          if (near(vp, vn)) { set('out', flatVec(p, k, n, 'out')); if (types.in === 'clamped' && p) set('in', flatVec(p, k, n, 'in')); }
          else set('out', smoothVec(p, k, n, 'out', weighted, fps));
        }
        if (!p && types.in === 'clamped') { k.inAngle = k.outAngle; if (weighted) k.inWeight = k.outWeight; }
        if (!n && types.out === 'clamped') { k.outAngle = k.inAngle; if (weighted) k.outWeight = k.inWeight; }
      }
      if (types.in === 'plateau' || types.out === 'plateau') {
        const e = 1e-10, BIG = Number.MAX_VALUE;
        const vp = p ? p.v : BIG, vn = n ? n.v : BIG, v = k.v;
        const trialIn = smoothVec(p, k, n, 'in', weighted, fps), trialOut = smoothVec(p, k, n, 'out', weighted, fps);
        let flat = !p || !n || (v - e >= vp && v - e >= vn) || (vp >= v + e && vn >= v + e) || Math.abs(vp - v) <= e || Math.abs(vn - v) <= e;
        if (p && overshoots(v, vp, hullOverlaps(p, keyVec(p, 'out', weighted), k, trialIn, weighted))) flat = true;
        if (n && overshoots(v, vn, hullOverlaps(k, trialOut, n, keyVec(n, 'in', weighted), weighted))) flat = true;
        for (const side of ['in', 'out']) {
          if (types[side] !== 'plateau') continue;
          set(side, flat ? flatVec(p, k, n, side) : side === 'in' ? trialIn : trialOut);
        }
        if (!p && types.in === 'plateau') { k.inAngle = k.outAngle; if (weighted) k.inWeight = k.outWeight; }
        if (!n && types.out === 'plateau') { k.outAngle = k.inAngle; if (weighted) k.outWeight = k.inWeight; }
      }
    }
  }
  return { ...curve, keys, _cache: undefined };
}

export function makeCurve(keys, opts = {}) {
  const curve = {
    weighted: !!opts.weighted, preInfinity: opts.preInfinity ?? 'constant', postInfinity: opts.postInfinity ?? 'constant',
    keys: keys.map(k => ({ inType: 'auto', outType: 'auto', inAngle: 0, outAngle: 0, inWeight: 1, outWeight: 1, ...k })),
  };
  return computeTangents(curve, opts);
}

export function sampleCurve(curve, t0, t1, step = 1) {
  const out = [];
  for (let t = t0; t <= t1 + 1e-9; t += step) out.push(evaluate(curve, t));
  return out;
}
