export function mulberry32(seed) {
  let s = seed >>> 0;
  return () => { s = (s + 0x6D2B79F5) >>> 0; let t = s; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}

const LC = [76.18009172947146, -86.50532032941677, 24.01409824083091, -1.231739572450155, 0.1208650973866179e-2, -0.5395239384953e-5];
export function lgamma(x) {
  let y = x, t = x + 5.5, s = 1.000000000190015;
  t -= (x + 0.5) * Math.log(t);
  for (const c of LC) s += c / ++y;
  return -t + Math.log(2.5066282746310005 * s / x);
}

export const lse = (a, b) => a === -Infinity ? b : b === -Infinity ? a : a > b ? a + Math.log1p(Math.exp(b - a)) : b + Math.log1p(Math.exp(a - b));

export function binomCdf(k, n, p) {
  if (k < 0) return 0;
  if (k >= n || p <= 0) return 1;
  if (p >= 1) return 0;
  const lr = Math.log(p) - Math.log1p(-p);
  let lt = n * Math.log1p(-p), s = lt;
  for (let i = 1; i <= k; i++) { lt += Math.log((n - i + 1) / i) + lr; s = lse(s, lt); }
  return Math.min(1, Math.exp(s));
}

export function cpUpper(k, n, conf = 0.95) {
  if (k >= n) return 1;
  let lo = k / n, hi = 1;
  for (let it = 0; it < 100; it++) { const m = (lo + hi) / 2; if (binomCdf(k, n, m) > 1 - conf) lo = m; else hi = m; }
  return hi;
}

export const bentkusP = (rhat, n, alpha) => Math.min(1, Math.E * binomCdf(Math.ceil(n * rhat - 1e-9), n, alpha));

export function lttFixedSequence(lambdas, risk, n, alpha, delta) {
  let index = -1, p = 1;
  for (let j = 0; j < lambdas.length; j++) {
    const pj = Number.isFinite(risk[j]) ? bentkusP(risk[j], n, alpha) : 1;
    if (pj > delta) { if (index < 0) p = pj; break; }
    index = j; p = pj;
  }
  return { index, lambda: index < 0 ? Infinity : lambdas[index], p };
}

export function conformalRank(n, a) { const k = Math.ceil((n + 1) * (1 - a) - 1e-9); return k > n ? Infinity : Math.max(1, k); }

export function conformalQuantile(scores, a) {
  const v = Float64Array.from(scores).sort(), n = v.length, k = conformalRank(n, a);
  return { q: k === Infinity ? Infinity : v[k - 1], k, n };
}

export function clusterBootstrap(num, den, B, rand) {
  const m = num.length, rat = (sn, sd) => sd > 0 ? sn / sd : 0, rs = new Float64Array(B);
  let sn = 0, sd = 0;
  for (let i = 0; i < m; i++) { sn += num[i]; sd += den[i]; }
  for (let b = 0; b < B; b++) {
    let a = 0, c = 0;
    for (let i = 0; i < m; i++) { const k = Math.floor(rand() * m); a += num[k]; c += den[k]; }
    rs[b] = rat(a, c);
  }
  rs.sort();
  const q = p => rs[Math.min(B - 1, Math.max(0, Math.round(p * (B - 1))))];
  return { ratio: rat(sn, sd), lo: B ? q(0.025) : NaN, hi: B ? q(0.975) : NaN };
}

export function autocorrTime(x) {
  const n = x.length;
  if (n < 4) return 1;
  let mu = 0;
  for (let i = 0; i < n; i++) mu += x[i];
  mu /= n;
  const g = k => { let s = 0; for (let i = 0; i + k < n; i++) s += (x[i] - mu) * (x[i + k] - mu); return s / n; };
  const g0 = g(0);
  if (!(g0 > 0)) return 1;
  let tau = -1;
  for (let m = 0; 2 * m + 1 < n; m++) { const G = g(2 * m) + g(2 * m + 1); if (!(G > 0)) break; tau += 2 * G / g0; }
  return Math.max(1, tau);
}
