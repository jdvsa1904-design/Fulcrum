const P = (W, nb, f, i, k) => W[(f * nb + i) * 16 + 12 + k];
const smooth = t => t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t);

function hSpeed(W, nb, n, i, f) {
  const f0 = Math.max(0, f - 1), f1 = Math.min(n - 1, f + 1);
  if (f1 === f0) return 0;
  return Math.hypot(P(W, nb, f1, i, 0) - P(W, nb, f0, i, 0), P(W, nb, f1, i, 2) - P(W, nb, f0, i, 2)) / (f1 - f0);
}

export function fulcrumOffsets(W, nb, pivotIdx, a, b, anchor = 'auto', blend = 3) {
  const n = W.length / (nb * 16), out = new Array(n).fill(null);
  a = Math.max(0, a); b = Math.min(n - 1, b);
  if (b < a) return out;
  let af = anchor;
  if (af === 'auto') {
    let bs = Infinity;
    for (let f = a; f <= b; f++) { const s = hSpeed(W, nb, n, pivotIdx, f); if (s < bs) { bs = s; af = f; } }
  }
  af = Math.min(b, Math.max(a, af | 0));
  const t = [0, 1, 2].map(k => P(W, nb, af, pivotIdx, k));
  const d = f => t.map((v, k) => v - P(W, nb, f, pivotIdx, k));
  for (let f = a; f <= b; f++) out[f] = d(f);
  const da = d(a), db = d(b);
  for (let j = 1; j <= blend; j++) {
    const w = smooth(1 - j / (blend + 1));
    if (a - j >= 0) out[a - j] = da.map(v => v * w);
    if (b + j < n) out[b + j] = db.map(v => v * w);
  }
  return out;
}

export function combineOffsets(...arrays) {
  const L = arrays.filter(Boolean), n = Math.max(0, ...L.map(x => x.length));
  return Array.from({ length: n }, (_, f) => {
    let o = null;
    for (const x of L) { const v = x[f]; if (v) o = o ? [o[0] + v[0], o[1] + v[1], o[2] + v[2]] : [v[0], v[1], v[2]]; }
    return o;
  });
}

export function detectFulcrums(W, nb, n, fps, upm, candidates, ground, maxSpeed = 0.25, maxHeight = 0.08, minFrames = 4) {
  const res = [], vMax = maxSpeed * upm / fps, hMax = maxHeight * upm;
  for (const { name, idx } of candidates) {
    if (idx === undefined) continue;
    let s = -1;
    for (let f = 0; f <= n; f++) {
      const ok = f < n && P(W, nb, f, idx, 1) - ground <= hMax && hSpeed(W, nb, n, idx, f) <= vMax;
      if (ok && s < 0) s = f;
      else if (!ok && s >= 0) { if (f - s >= minFrames) res.push({ pivot: name, idx, a: s, b: f - 1 }); s = -1; }
    }
  }
  return res.sort((x, y) => x.a - y.a || x.idx - y.idx);
}

export function pivotSlip(W, nb, idx, a, b, upm) {
  let m = 0;
  for (let f = a; f <= b; f++) for (let g = f; g <= b; g++)
    m = Math.max(m, Math.hypot(P(W, nb, g, idx, 0) - P(W, nb, f, idx, 0), P(W, nb, g, idx, 2) - P(W, nb, f, idx, 2)));
  return m * 100 / upm;
}
