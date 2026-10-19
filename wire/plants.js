import { detectPlants } from '../contacts.js';

const SIDES = ['l', 'r'];

function track(W, nb, n, bone, ground) {
  const t = new Float64Array(n * 3);
  for (let f = 0; f < n; f++) {
    const o = (f * nb + bone) * 16;
    t[f * 3] = W[o + 12];
    t[f * 3 + 1] = W[o + 13] - ground;
    t[f * 3 + 2] = W[o + 14];
  }
  return t;
}

function level(t, n, pct) {
  const h = new Float64Array(n);
  for (let f = 0; f < n; f++) h[f] = t[f * 3 + 1];
  h.sort();
  return h[Math.min(n - 1, Math.floor(n * pct))];
}

function fillGaps(m, maxGap) {
  let last = -1;
  for (let f = 0; f < m.length; f++) {
    if (!m[f]) continue;
    if (last >= 0 && f - last - 1 <= maxGap) m.fill(1, last + 1, f);
    last = f;
  }
}

function runsOf(m) {
  const out = [];
  let s = -1;
  for (let f = 0; f <= m.length; f++) {
    const on = f < m.length && m[f];
    if (on && s < 0) s = f;
    if (!on && s >= 0) { out.push([s, f - 1]); s = -1; }
  }
  return out;
}

export function detectFootPlants({ W, nb, n, fps, upm, ground, legs, legLength_cm, footLength_cm, minFrames = 4, tol = 0.07, heightTol_cm = 3, levelPct = 0.1, maxGap = 2, slidingDist = 25, slowSpeed = 0.5, minTouch = 2, maxAnkle = 0.2, maxBall = 0.06 }) {
  const out = [];
  if (!W || !(n > 0) || !(nb > 0) || !(fps > 0) || !(upm > 0) || !(legLength_cm > 0)) return out;
  const S = legLength_cm / 100 * upm, footLen = (footLength_cm > 0 ? footLength_cm : 0.3 * legLength_cm) / 100 * upm;
  const stepTol = tol * 30 / fps, hTol = heightTol_cm / 100 * upm, slideLim = footLen * slidingDist / 50;
  for (const side of SIDES) {
    const leg = legs?.[side];
    if (!leg || leg.foot == null || leg.foot < 0 || leg.foot >= nb) continue;
    const hasBall = leg.ball != null && leg.ball >= 0 && leg.ball < nb && leg.ball !== leg.foot;
    const pts = [leg.foot, hasBall ? leg.ball : leg.foot].map(b => track(W, nb, n, b, ground ?? 0));
    const lv = pts.map((t, k) => Math.min(level(t, n, levelPct), (k ? maxBall : maxAnkle) * S));
    const masks = pts.map((t, k) => detectPlants(t, { legLength: S, tol: stepTol, floor: lv[k], verticalTol: hTol / S, minRun: 0 }).planted);
    const m = new Uint8Array(n);
    for (let f = 0; f < n; f++) m[f] = masks[0][f] | masks[1][f];
    fillGaps(m, maxGap);
    const sp = new Float64Array(n), pick = new Uint8Array(n);
    for (let f = 0; f < n; f++) {
      const f0 = Math.max(0, f - 1), f1 = Math.min(n - 1, f + 1);
      const step = (t, i, j) => Math.hypot(t[j * 3] - t[i * 3], t[j * 3 + 2] - t[i * 3 + 2]) * fps / upm;
      const v = pts.map(t => f0 === f ? step(t, f, f1) : f1 === f ? step(t, f0, f) : Math.min(step(t, f0, f), step(t, f, f1)));
      const low = pts[1][f * 3 + 1] - lv[1] <= pts[0][f * 3 + 1] - lv[0];
      pick[f] = !hasBall ? 0 : masks[1][f] || masks[0][f] ? (masks[1][f] && (!masks[0][f] || v[1] <= v[0]) ? 1 : 0) : low ? 1 : 0;
      sp[f] = v[pick[f]];
    }
    const measure = (a, b) => {
      let anchor = a, best = Infinity, drift = 0, nBall = 0;
      for (let f = a; f <= b; f++) if (sp[f] < best) { best = sp[f]; anchor = f; }
      for (let f = a; f <= b; f++) {
        const t = pts[pick[f]];
        nBall += pick[f];
        drift = Math.max(drift, Math.hypot(t[f * 3] - t[anchor * 3], t[f * 3 + 2] - t[anchor * 3 + 2]));
      }
      const speed = Array.from(sp.subarray(a, b + 1)).sort((x, y) => x - y)[(b - a + 1) >> 1];
      return { anchor, drift, speed, point: nBall * 2 > b - a + 1 ? 'ball' : 'foot' };
    };
    const kept = new Uint8Array(n);
    for (let [a, b] of runsOf(m)) {
      if (b - a + 1 < minFrames) continue;
      let ca = a, cb = b;
      while (ca <= cb && sp[ca] > slowSpeed) ca++;
      while (cb >= ca && sp[cb] > slowSpeed) cb--;
      if (cb - ca + 1 >= minFrames) { a = ca; b = cb; }
      const r = measure(a, b);
      if (!Number.isFinite(r.drift) || !Number.isFinite(r.speed)) continue;
      kept.fill(1, a, b + 1);
      out.push({ side, a, b, sliding: r.drift > slideLim && r.speed > slowSpeed, anchor: r.anchor, point: r.point, touch: false, drift_cm: r.drift / upm * 100, speed: r.speed });
    }
    if (minTouch > 0) {
      const h = new Uint8Array(n);
      for (let f = 0; f < n; f++) if (!kept[f] && (pts[0][f * 3 + 1] - lv[0] <= hTol || pts[1][f * 3 + 1] - lv[1] <= hTol)) h[f] = 1;
      for (const [a, b] of runsOf(h)) {
        if (b - a + 1 < minTouch || (a > 0 && kept[a - 1]) || (b < n - 1 && kept[b + 1])) continue;
        const r = measure(a, b);
        if (!Number.isFinite(r.drift) || !Number.isFinite(r.speed)) continue;
        out.push({ side, a, b, sliding: true, anchor: r.anchor, point: r.point, touch: true, drift_cm: r.drift / upm * 100, speed: r.speed });
      }
    }
  }
  return out.sort((x, y) => x.a - y.a || (x.side < y.side ? -1 : 1));
}
