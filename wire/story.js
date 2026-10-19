import { BLEND, transition, matchPose, MATCH_T, MATCH_R } from '../solvers/mobu/story.js';

export const CURVES = Object.freeze({ linear: BLEND.LINEAR, ease: BLEND.EASE, slow_in: BLEND.SLOW_IN, fast_in: BLEND.FAST_IN });

export function storyWindow(fx, n) {
  const pre = Math.max(0, fx.pre ?? fx.blend ?? 6), post = Math.max(1, fx.post ?? fx.blend ?? 6);
  return { s: Math.max(0, fx.at - pre), e: Math.min(n - 1, fx.at + post - 1) };
}

export function storyWeight(fx, f, n) {
  const { s, e } = storyWindow(fx, n);
  return transition(f, { stop: e + 1 }, { start: s, inType: CURVES[fx.curve ?? 'ease'] ?? BLEND.EASE }).wIn;
}

function slerpInto(dst, o, a, ao, b, bo, t) {
  let ax = a[ao], ay = a[ao + 1], az = a[ao + 2], aw = a[ao + 3], bx = b[bo], by = b[bo + 1], bz = b[bo + 2], bw = b[bo + 3];
  let d = ax * bx + ay * by + az * bz + aw * bw;
  if (d < 0) { bx = -bx; by = -by; bz = -bz; bw = -bw; d = -d; }
  let k0 = 1 - t, k1 = t;
  if (d < 0.9995) { const th = Math.acos(d), s = Math.sin(th); k0 = Math.sin((1 - t) * th) / s; k1 = Math.sin(t * th) / s; }
  const x = ax * k0 + bx * k1, y = ay * k0 + by * k1, z = az * k0 + bz * k1, w = aw * k0 + bw * k1, l = Math.hypot(x, y, z, w) || 1;
  dst[o] = x / l; dst[o + 1] = y / l; dst[o + 2] = z / l; dst[o + 3] = w / l;
}

export function applyStoryBlends(o) {
  const { q, p, nb, n, src, srcIdx, fixes, root, up = [0, 1, 0] } = o;
  const report = [];
  if (!src || !srcIdx) return report;
  const clampS = k => Math.max(0, Math.min(src.n - 1, k));
  const horiz = v => { const d = v[0] * up[0] + v[1] * up[1] + v[2] * up[2]; return [v[0] - up[0] * d, v[1] - up[1] * d, v[2] - up[2] * d]; };
  for (const fx of [...fixes].sort((u, v) => u.at - v.at)) {
    const at = fx.at;
    if (at < 1 || at >= n) continue;
    const { s, e } = storyWindow(fx, n);
    const o0 = srcIdx[at - 1], i0 = srcIdx[at];
    const shift = (f, k) => { const a = (f * nb + root) * 3, b = k * nb * 3 + root * 3; return [p[a] - src.p[b], p[a + 1] - src.p[b + 1], p[a + 2] - src.p[b + 2]]; };
    const Dout = root != null ? shift(at - 1, o0) : [0, 0, 0], Din = root != null ? shift(at, i0) : [0, 0, 0];
    const outQ = new Float32Array((e - s + 1) * nb * 4), outP = new Float32Array((e - s + 1) * nb * 3), inQ = new Float32Array(outQ.length), inP = new Float32Array(outP.length);
    for (let f = s; f <= e; f++) {
      const r = f - s;
      if (f < at) { outQ.set(q.subarray(f * nb * 4, (f + 1) * nb * 4), r * nb * 4); outP.set(p.subarray(f * nb * 3, (f + 1) * nb * 3), r * nb * 3); }
      else {
        const k = clampS(o0 + 1 + (f - at));
        outQ.set(src.q.subarray(k * nb * 4, (k + 1) * nb * 4), r * nb * 4); outP.set(src.p.subarray(k * nb * 3, (k + 1) * nb * 3), r * nb * 3);
        if (root != null) for (let c = 0; c < 3; c++) outP[(r * nb + root) * 3 + c] += Dout[c];
      }
      if (f >= at) { inQ.set(q.subarray(f * nb * 4, (f + 1) * nb * 4), r * nb * 4); inP.set(p.subarray(f * nb * 3, (f + 1) * nb * 3), r * nb * 3); }
      else {
        const k = clampS(i0 + (f - at));
        inQ.set(src.q.subarray(k * nb * 4, (k + 1) * nb * 4), r * nb * 4); inP.set(src.p.subarray(k * nb * 3, (k + 1) * nb * 3), r * nb * 3);
        if (root != null) for (let c = 0; c < 3; c++) inP[(r * nb + root) * 3 + c] += Din[c];
      }
    }
    let d = [0, 0, 0];
    if (root != null && fx.match !== 'none') {
      const r = at - s, A = { t: Array.from(outP.subarray((r * nb + root) * 3, (r * nb + root) * 3 + 3)), q: [0, 0, 0, 1] }, B = { t: Array.from(inP.subarray((r * nb + root) * 3, (r * nb + root) * 3 + 3)), q: [0, 0, 0, 1] };
      const M = matchPose(A, B, fx.match === 'direct' ? MATCH_T.DIRECT : MATCH_T.GRAVITY, MATCH_R.NONE);
      d = [B.t[0] - M.t[0], B.t[1] - M.t[1], B.t[2] - M.t[2]];
      if (fx.match !== 'direct') d = horiz(d);
    }
    let maxW = 0;
    for (let f = s; f <= e; f++) {
      const r = f - s, w = storyWeight(fx, f, n);
      maxW = Math.max(maxW, w);
      for (let i = 0; i < nb; i++) {
        slerpInto(q, (f * nb + i) * 4, outQ, (r * nb + i) * 4, inQ, (r * nb + i) * 4, w);
        for (let c = 0; c < 3; c++) {
          const ip = inP[(r * nb + i) * 3 + c] - (i === root ? d[c] : 0);
          p[(f * nb + i) * 3 + c] = outP[(r * nb + i) * 3 + c] * (1 - w) + ip * w;
        }
      }
    }
    if (root != null) for (let f = e + 1; f < n; f++) for (let c = 0; c < 3; c++) p[(f * nb + root) * 3 + c] -= d[c];
    report.push({ at, s, e, outgoingFrom: o0 + 1, incomingFrom: i0, matchOffset: d });
  }
  return report;
}

export function cutIndex(srcIdx, a, b) {
  const out = new Int32Array(srcIdx.length - (b - a + 1));
  out.set(srcIdx.subarray(0, a)); out.set(srcIdx.subarray(b + 1), a);
  return out;
}

export function insertIndex(srcIdx, at, a, b) {
  const k = b - a + 1, out = new Int32Array(srcIdx.length + k);
  out.set(srcIdx.subarray(0, at)); out.set(srcIdx.subarray(a, b + 1), at); out.set(srcIdx.subarray(at), at + k);
  return out;
}
