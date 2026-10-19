import { IMG_STRIDE } from './landmarks.js';
import { NLM } from './skeleton.js';

const BASE = new URL('../vendor/mediapipe/', import.meta.url).href;
export const MODELS = { lite: 'Fast', full: 'Balanced', heavy: 'Best' };
let visionModule = null;

async function vision() {
  if (!visionModule) visionModule = await import(BASE + 'tasks-vision/vision_bundle.mjs');
  return visionModule;
}

export async function createLandmarker(model = 'full', o = {}) {
  const V = await vision(), fileset = await V.FilesetResolver.forVisionTasks(BASE + 'tasks-vision/wasm');
  let last = null;
  for (const delegate of o.delegates ?? ['GPU', 'CPU']) {
    try {
      const lm = await V.PoseLandmarker.createFromOptions(fileset, {
        baseOptions: { modelAssetPath: BASE + `models/pose_landmarker_${model}.task`, delegate },
        runningMode: 'VIDEO', numPoses: o.numPoses ?? 2,
        minPoseDetectionConfidence: o.detect ?? 0.5, minPosePresenceConfidence: o.presence ?? 0.5, minTrackingConfidence: o.track ?? 0.5,
      });
      return { lm, delegate, model };
    } catch (e) { last = e; }
  }
  throw last ?? new Error('Could not start the pose tracker.');
}

export function seek(video, t) {
  return new Promise(resolve => {
    if (Math.abs(video.currentTime - t) < 1e-4 && video.readyState >= 2) return resolve();
    const done = () => { video.removeEventListener('seeked', done); resolve(); };
    video.addEventListener('seeked', done);
    video.currentTime = t;
  });
}

export function packResult(res) {
  return (res.landmarks ?? []).map((lms, k) => {
    const img = new Float32Array(NLM * IMG_STRIDE), world = new Float32Array(NLM * 3), wl = res.worldLandmarks?.[k] ?? [];
    for (let i = 0; i < NLM; i++) {
      const p = lms[i] ?? {}, w = wl[i] ?? {};
      img.set([p.x ?? 0, p.y ?? 0, p.z ?? 0, p.visibility ?? 1, p.presence ?? 1], i * IMG_STRIDE);
      world.set([w.x ?? 0, w.y ?? 0, w.z ?? 0], i * 3);
    }
    return { img, world };
  });
}

function detect(tracker, canvas, ts) {
  const t = performance.now(), res = tracker.lm.detectForVideo(canvas, ts);
  return { res, ms: performance.now() - t };
}

export async function captureVideo(video, o = {}) {
  const fps = o.fps || 30, first = o.first ?? 0, last = o.last ?? Math.max(0, Math.floor(video.duration * fps) - 1);
  const scale = Math.min(1, (o.maxWidth ?? 960) / video.videoWidth);
  const canvas = document.createElement('canvas');
  canvas.width = Math.max(2, Math.round(video.videoWidth * scale));
  canvas.height = Math.max(2, Math.round(video.videoHeight * scale));
  const ctx = canvas.getContext('2d', { willReadFrequently: false });
  const model = o.model ?? 'full';
  let tracker = o.tracker ?? await createLandmarker(model, o), probe = !o.tracker && !o.delegates && tracker.delegate === 'GPU' ? [] : null;
  const frames = new Array(last + 1).fill(null).map((_, i) => ({ t: i / fps, poses: [] }));
  const wasPaused = video.paused;
  video.pause();
  const t0 = performance.now(), at = f => Math.min(video.duration - 1e-3, (f + 0.5) / fps);
  let next = seek(video, at(first));
  for (let f = first; f <= last; f++) {
    if (o.signal?.aborted) break;
    await next;
    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
    next = f < last ? seek(video, at(f + 1)) : Promise.resolve();
    const ts = Math.round(f * 1000 / fps) + 1;
    let out;
    try {
      out = detect(tracker, canvas, ts);
    } catch (e) {
      if (tracker.delegate !== 'GPU') throw e;
      tracker.lm.close?.();
      tracker = await createLandmarker(model, { ...o, delegates: ['CPU'] });
      probe = null;
      out = detect(tracker, canvas, ts);
    }
    if (probe) {
      probe.push(out.ms);
      if (probe.length === 3) {
        const gpu = (probe[1] + probe[2]) / 2;
        if (gpu > (o.slowMs ?? 90)) {
          const cpu = await createLandmarker(model, { ...o, delegates: ['CPU'] });
          detect(cpu, canvas, ts);
          const c = detect(cpu, canvas, ts + 1);
          if (c.ms < gpu * 0.8) { tracker.lm.close?.(); tracker = cpu; out = c; }
          else cpu.lm.close?.();
        }
        probe = null;
      }
    }
    frames[f] = { t: f / fps, poses: packResult(out.res) };
    o.onFrame?.(f, frames[f], { done: f - first + 1, total: last - first + 1, ms: performance.now() - t0, delegate: tracker.delegate });
    if ((f - first) % 4 === 3) await new Promise(r => setTimeout(r, 0));
  }
  await next;
  if (!o.tracker) tracker.lm.close?.();
  if (!wasPaused) video.play().catch(() => {});
  return {
    version: 1,
    video: { name: o.name ?? 'video', width: video.videoWidth, height: video.videoHeight, fps, duration: video.duration, frames: last + 1 },
    model: tracker.model, delegate: tracker.delegate, aborted: !!o.signal?.aborted, range: [first, last], frames,
  };
}

export function captureToJSON(capture) {
  const round = a => Array.from(a, x => Math.round(x * 1e5) / 1e5);
  return { ...capture, frames: capture.frames.map(f => f && { t: f.t, poses: f.poses.map(p => ({ img: round(p.img), world: round(p.world) })) }) };
}

export function captureFromJSON(data) {
  return { ...data, frames: data.frames.map((f, i) => f ? { t: f.t, poses: f.poses.map(p => ({ img: Float32Array.from(p.img), world: Float32Array.from(p.world) })) } : { t: i / data.video.fps, poses: [] }) };
}
