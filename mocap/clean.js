import { peakRemovalVec, butterworth } from '../filters.js';
import { NLM, LM } from './skeleton.js';

function lineFit(v, a, b) {
  let sx = 0, sy = 0, sxx = 0, sxy = 0;
  const m = b - a;
  for (let i = a; i < b; i++) { const x = i - a; sx += x; sy += v[i]; sxx += x * x; sxy += x * v[i]; }
  const den = m * sxx - sx * sx, k = Math.abs(den) > 1e-12 ? (m * sxy - sx * sy) / den : 0;
  return { k, c: (sy - k * sx) / m };
}

export function lowpass(values, fps, fc) {
  const n = values.length;
  if (!(fc > 0) || n < 4 || fc >= fps / 2) return Float64Array.from(values);
  const m = Math.max(3, Math.min(n, Math.round(fps / fc))), pad = 2 * m;
  const head = lineFit(values, 0, m), tail = lineFit(values, n - m, n);
  const ext = new Float64Array(n + 2 * pad);
  for (let i = 0; i < pad; i++) ext[i] = head.c + head.k * (i - pad);
  ext.set(values, pad);
  for (let i = 0; i < pad; i++) ext[pad + n + i] = tail.c + tail.k * (m + i);
  const r = butterworth(ext, fps, { fc: Math.min(fc, 0.45 * fps) });
  return r.slice(pad, pad + n);
}

export function lowpassVec(track, comps, fps, fc) {
  const n = track.length / comps, out = new Float64Array(track.length), ch = new Float64Array(n);
  for (let k = 0; k < comps; k++) {
    for (let i = 0; i < n; i++) ch[i] = track[i * comps + k];
    const r = lowpass(ch, fps, fc);
    for (let i = 0; i < n; i++) out[i * comps + k] = r[i];
  }
  return out;
}

export function rootTrack(P, n) {
  const R = new Float64Array(n * 3);
  for (let i = 0; i < n; i++) for (let k = 0; k < 3; k++) R[i * 3 + k] = (P[(i * NLM + LM.lHip) * 3 + k] + P[(i * NLM + LM.rHip) * 3 + k]) / 2;
  return R;
}

export function cleanPoints(P, n, fps, o = {}) {
  const fc = o.fc ?? 6, fcRoot = o.fcRoot ?? 4, fcDepth = o.fcDepth ?? 1.5, out = new Float64Array(P.length);
  const R = rootTrack(P, n), rel = new Float64Array(n * 3);
  let spikes = 0;
  for (let j = 0; j < NLM; j++) {
    for (let i = 0; i < n; i++) for (let k = 0; k < 3; k++) rel[i * 3 + k] = P[(i * NLM + j) * 3 + k] - R[i * 3 + k];
    let t = rel;
    if (o.despike !== false && n > 6) { t = peakRemovalVec(rel, 3, { timeTolerance: 3, valueTolerance: o.spikeTolerance ?? 4 }); spikes += t.removed.length; }
    t = lowpassVec(t, 3, fps, fc);
    for (let i = 0; i < n; i++) for (let k = 0; k < 3; k++) out[(i * NLM + j) * 3 + k] = t[i * 3 + k];
  }
  const root = new Float64Array(n * 3), Rs = o.despike !== false && n > 6 ? peakRemovalVec(R, 3, { timeTolerance: 3, valueTolerance: o.spikeTolerance ?? 4 }) : R, ch = new Float64Array(n);
  for (let k = 0; k < 3; k++) {
    for (let i = 0; i < n; i++) ch[i] = Rs[i * 3 + k];
    const r = lowpass(ch, fps, k === 2 ? fcDepth : fcRoot);
    for (let i = 0; i < n; i++) root[i * 3 + k] = r[i];
  }
  for (let i = 0; i < n; i++) for (let j = 0; j < NLM; j++) for (let k = 0; k < 3; k++) out[(i * NLM + j) * 3 + k] += root[i * 3 + k];
  return { P: out, root, spikes };
}
