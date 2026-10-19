const smooth = t => t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t);

function mk(a, b, fps, g, p0, v0) {
  const T = (b - a) / fps, apexT = Math.min(T, Math.max(0, v0[1] / g));
  const at = f => { const t = (f - a) / fps; return [p0[0] + v0[0] * t, p0[1] + v0[1] * t - g * t * t / 2, p0[2] + v0[2] * t]; };
  return { a, b, T, p0, v0, apexY: p0[1] + v0[1] * apexT - g * apexT * apexT / 2, apexT, at };
}

export function detectFlights(any, minFrames = 3) {
  const out = [], n = any.length;
  for (let f = 1; f < n; f++) {
    if (any[f] || !any[f - 1]) continue;
    let e = f;
    while (e < n && !any[e]) e++;
    if (e < n && e - f >= minFrames) out.push({ a: f - 1, b: e });
    f = e;
  }
  return out;
}

export function fitBallistic(com, a, b, fps, g) {
  const T = (b - a) / fps, A = a * 3, B = b * 3;
  const p0 = [com[A], com[A + 1], com[A + 2]];
  const v0 = [(com[B] - p0[0]) / T, (com[B + 1] - p0[1] + g * T * T / 2) / T, (com[B + 2] - p0[2]) / T];
  return mk(a, b, fps, g, p0, v0);
}

export function fitBallisticLeastSquares(com, a, b, fps, g) {
  const m = b - a + 1;
  let st = 0, stt = 0;
  const sy = [0, 0, 0], sty = [0, 0, 0];
  for (let f = a; f <= b; f++) {
    const t = (f - a) / fps, o = f * 3;
    st += t; stt += t * t;
    for (let k = 0; k < 3; k++) { const y = com[o + k] + (k === 1 ? g * t * t / 2 : 0); sy[k] += y; sty[k] += t * y; }
  }
  const d = m * stt - st * st, p0 = [], v0 = [];
  for (let k = 0; k < 3; k++) { v0.push((m * sty[k] - st * sy[k]) / d); p0.push((sy[k] - v0[k] * st) / m); }
  return mk(a, b, fps, g, p0, v0);
}

export function ballisticOffsets(com, flights, fps, g, mode = 'endpoints') {
  const n = com.length / 3, off = new Array(n).fill(null);
  for (const { a, b } of flights) {
    if (a < 0 || b >= n || b - a < 2) continue;
    const fit = (mode === 'fit' ? fitBallisticLeastSquares : fitBallistic)(com, a, b, fps, g);
    for (let f = a + 1; f < b; f++) {
      const w = smooth(Math.min(f - a, b - f) / 2), p = fit.at(f), o = f * 3;
      off[f] = [(p[0] - com[o]) * w, (p[1] - com[o + 1]) * w, (p[2] - com[o + 2]) * w];
    }
  }
  return off;
}

export function flightReport(fit, upm) {
  const [vx, vy, vz] = fit.v0, h = Math.hypot(vx, vz), e = fit.at(fit.b);
  return {
    airtime_s: fit.T,
    apex_m_above_takeoff: (fit.apexY - fit.p0[1]) / upm,
    takeoff_speed_ms: Math.hypot(h, vy) / upm,
    takeoff_angle_deg: Math.atan2(vy, h) * 180 / Math.PI,
    distance_m: Math.hypot(e[0] - fit.p0[0], e[2] - fit.p0[2]) / upm
  };
}
