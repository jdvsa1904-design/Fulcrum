import { solve3, clamp, mid, median } from './math.js';
import { NLM, LM } from './skeleton.js';

export const IMG_STRIDE = 5;

export function focalPx(width, hfovDeg) {
  return width / 2 / Math.tan(clamp(hfovDeg, 10, 170) * Math.PI / 360);
}

function poseCenter(img, aspect) {
  const v = i => img[i * IMG_STRIDE + 3];
  const pick = v(LM.lHip) > 0.2 || v(LM.rHip) > 0.2 ? [LM.lHip, LM.rHip] : [LM.lSh, LM.rSh];
  return [(img[pick[0] * IMG_STRIDE] + img[pick[1] * IMG_STRIDE]) / 2 * aspect, (img[pick[0] * IMG_STRIDE + 1] + img[pick[1] * IMG_STRIDE + 1]) / 2];
}

export function poseBox(img) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (let i = 0; i < NLM; i++) {
    const x = img[i * IMG_STRIDE], y = img[i * IMG_STRIDE + 1];
    if (x < x0) x0 = x;
    if (x > x1) x1 = x;
    if (y < y0) y0 = y;
    if (y > y1) y1 = y;
  }
  return [x0, y0, x1, y1];
}

function poseVis(img) {
  let s = 0;
  for (let i = 11; i < NLM; i++) s += img[i * IMG_STRIDE + 3];
  return s / (NLM - 11);
}

export function buildTracks(capture, o = {}) {
  const aspect = (capture.video.width || 16) / (capture.video.height || 9), fps = capture.video.fps || 30;
  const maxGap = Math.max(2, Math.round((o.maxGapSec ?? 0.6) * fps)), tracks = [];
  capture.frames.forEach((fr, f) => {
    const dets = (fr.poses || []).map((p, k) => {
      const box = poseBox(p.img), c = poseCenter(p.img, aspect);
      return { k, c, size: Math.max(0.02, Math.hypot((box[2] - box[0]) * aspect, box[3] - box[1])), vis: poseVis(p.img), box };
    });
    const live = tracks.filter(t => f - t.last <= maxGap);
    const pairs = [];
    for (const t of live) for (const d of dets) {
      const gap = f - t.last, pred = [t.c[0] + t.v[0] * gap, t.c[1] + t.v[1] * gap];
      const cost = Math.hypot(d.c[0] - pred[0], d.c[1] - pred[1]) / Math.max(t.size, d.size) + Math.abs(Math.log(d.size / t.size)) * 0.5;
      if (cost < (o.maxCost ?? 0.6)) pairs.push({ t, d, cost });
    }
    pairs.sort((a, b) => a.cost - b.cost);
    const usedT = new Set(), usedD = new Set();
    for (const { t, d } of pairs) {
      if (usedT.has(t) || usedD.has(d)) continue;
      usedT.add(t); usedD.add(d);
      const gap = f - t.last;
      t.v = [(d.c[0] - t.c[0]) / gap * 0.6 + t.v[0] * 0.4, (d.c[1] - t.c[1]) / gap * 0.6 + t.v[1] * 0.4];
      t.c = d.c; t.size = d.size * 0.5 + t.size * 0.5; t.last = f;
      t.frames.set(f, d.k);
      t.score += d.size * d.vis;
    }
    for (const d of dets) {
      if (usedD.has(d)) continue;
      tracks.push({ id: tracks.length, c: d.c, v: [0, 0], size: d.size, first: f, last: f, frames: new Map([[f, d.k]]), score: d.size * d.vis });
    }
  });
  return tracks.map(t => ({ id: t.id, first: t.first, last: t.last, frames: t.frames, score: t.score, count: t.frames.size }));
}

export function pickTrack(tracks, capture, o = {}) {
  if (!tracks.length) return null;
  if (o.point && o.frame != null) {
    const aspect = (capture.video.width || 16) / (capture.video.height || 9);
    let best = null, bestD = Infinity;
    for (const t of tracks) {
      let f = o.frame;
      if (!t.frames.has(f)) {
        let near = null;
        for (const g of t.frames.keys()) if (near === null || Math.abs(g - o.frame) < Math.abs(near - o.frame)) near = g;
        if (near === null || Math.abs(near - o.frame) > 15) continue;
        f = near;
      }
      const img = capture.frames[f].poses[t.frames.get(f)].img, box = poseBox(img);
      const cx = clamp(o.point[0], box[0], box[2]), cy = clamp(o.point[1], box[1], box[3]);
      const d = Math.hypot((o.point[0] - cx) * aspect, o.point[1] - cy) + Math.abs(f - o.frame) * 0.01;
      if (d < bestD) { bestD = d; best = t; }
    }
    if (best) return best;
  }
  return tracks.reduce((a, b) => b.score > a.score ? b : a);
}

export function trackSeries(capture, track, o = {}) {
  const first = Math.max(track.first, o.trim?.[0] ?? 0), last = Math.min(track.last, o.trim?.[1] ?? Infinity);
  const n = Math.max(0, last - first + 1), img = new Float64Array(n * NLM * IMG_STRIDE), world = new Float64Array(n * NLM * 3), valid = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const f = first + i, k = track.frames.get(f);
    if (k === undefined) continue;
    const p = capture.frames[f].poses[k];
    img.set(p.img, i * NLM * IMG_STRIDE);
    world.set(p.world, i * NLM * 3);
    valid[i] = 1;
  }
  fillGaps(img, valid, NLM * IMG_STRIDE);
  fillGaps(world, valid, NLM * 3);
  return { first, last, n, img, world, valid, fps: capture.video.fps, width: capture.video.width, height: capture.video.height };
}

export function fillGaps(arr, valid, stride) {
  const n = valid.length;
  let prev = -1;
  for (let i = 0; i <= n; i++) {
    if (i < n && !valid[i]) continue;
    if (i - prev > 1) {
      for (let j = prev + 1; j < i; j++) {
        const a = prev >= 0 ? prev : i, b = i < n ? i : prev, u = a === b ? 0 : (j - a) / (b - a);
        if (a < 0 || b < 0 || a >= n || b >= n) continue;
        for (let k = 0; k < stride; k++) arr[j * stride + k] = arr[a * stride + k] + (arr[b * stride + k] - arr[a * stride + k]) * u;
      }
    }
    prev = i;
  }
  return arr;
}

export function solveTranslation(img, world, width, height, f, o = {}) {
  const AtA = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], Atb = [0, 0, 0];
  const minVis = o.minVis ?? 0.3;
  let used = 0;
  for (let i = 0; i < NLM; i++) {
    const vis = img[i * IMG_STRIDE + 3];
    if (!(vis >= minVis) && o.allowLow !== true) continue;
    const w = clamp(vis, 0.05, 1) ** 2;
    const a = (img[i * IMG_STRIDE] * width - width / 2) / f, b = (img[i * IMG_STRIDE + 1] * height - height / 2) / f;
    const X = world[i * 3], Y = world[i * 3 + 1], Z = world[i * 3 + 2];
    const rows = [[[1, 0, -a], a * Z - X], [[0, 1, -b], b * Z - Y]];
    for (const [r, rhs] of rows) {
      for (let p = 0; p < 3; p++) {
        Atb[p] += w * r[p] * rhs;
        for (let q = 0; q < 3; q++) AtA[p][q] += w * r[p] * r[q];
      }
    }
    used++;
  }
  if (used < 4) return o.allowLow === true ? null : solveTranslation(img, world, width, height, f, { ...o, allowLow: true });
  const t = solve3(AtA, Atb);
  if (!t || !(t[2] > (o.minDepth ?? 0.3))) return null;
  return t;
}

export function cameraPoints(series, o = {}) {
  const { n, img, world, width, height } = series, f = focalPx(width, o.hfov ?? 60), fit = clamp(o.imageFit ?? 0.5, 0, 1);
  const P = new Float64Array(n * NLM * 3), T = new Float64Array(n * 3), vis = new Float64Array(n * NLM), ok = new Uint8Array(n);
  let last = null;
  for (let i = 0; i < n; i++) {
    const im = img.subarray(i * NLM * IMG_STRIDE, (i + 1) * NLM * IMG_STRIDE), wd = world.subarray(i * NLM * 3, (i + 1) * NLM * 3);
    let t = solveTranslation(im, wd, width, height, f, o);
    if (t) { ok[i] = 1; last = t; } else t = last;
    if (t) T.set(t, i * 3);
  }
  const firstOk = ok.indexOf(1);
  if (firstOk < 0) return null;
  fillGaps(T, ok, 3);
  for (let i = 0; i < n; i++) {
    const t = [T[i * 3], T[i * 3 + 1], T[i * 3 + 2]];
    for (let j = 0; j < NLM; j++) {
      const o3 = (i * NLM + j) * 3, oi = (i * NLM + j) * IMG_STRIDE;
      let X = world[o3] + t[0], Y = world[o3 + 1] + t[1], Z = world[o3 + 2] + t[2];
      const v = img[oi + 3];
      vis[i * NLM + j] = v;
      if (fit > 0 && Z > 0.1) {
        const w = fit * clamp((v - 0.3) / 0.5, 0, 1);
        const a = (img[oi] * width - width / 2) / f, b = (img[oi + 1] * height - height / 2) / f;
        X += (a * Z - X) * w;
        Y += (b * Z - Y) * w;
      }
      P[o3] = X; P[o3 + 1] = -Y; P[o3 + 2] = -Z;
    }
  }
  return { n, P, T, vis, ok, focal: f, width, height };
}

export function projectPoint(p, cam) {
  const X = p[0], Y = -p[1], Z = -p[2];
  if (Z <= 1e-6) return null;
  return [(X / Z * cam.focal + cam.width / 2) / cam.width, (Y / Z * cam.focal + cam.height / 2) / cam.height];
}

export function seriesStats(series) {
  const hips = [];
  for (let i = 0; i < series.n; i++) {
    const o = i * NLM * IMG_STRIDE;
    hips.push(mid([series.img[o + LM.lHip * IMG_STRIDE], series.img[o + LM.lHip * IMG_STRIDE + 1], 0], [series.img[o + LM.rHip * IMG_STRIDE], series.img[o + LM.rHip * IMG_STRIDE + 1], 0]));
  }
  return { frames: series.n, coverage: series.valid.reduce((s, v) => s + v, 0) / Math.max(1, series.n), hipX: median(hips.map(h => h[0])) };
}
