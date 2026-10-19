const RATES = [23.976, 24, 25, 29.97, 30, 48, 50, 59.94, 60, 90, 100, 119.88, 120, 240];
const CONTAINERS = new Set(['moov', 'trak', 'mdia', 'minf', 'stbl', 'edts']);

const fourcc = (v, o) => String.fromCharCode(v.getUint8(o), v.getUint8(o + 1), v.getUint8(o + 2), v.getUint8(o + 3));

function* boxes(view, start, end) {
  let p = start;
  while (p + 8 <= end) {
    let size = view.getUint32(p), head = 8;
    const type = fourcc(view, p + 4);
    if (size === 1) { size = Number(view.getBigUint64(p + 8)); head = 16; }
    else if (size === 0) size = end - p;
    if (size < head || p + size > end) return;
    yield { type, start: p, body: p + head, end: p + size };
    p += size;
  }
}

function find(view, box, path) {
  let cur = [box];
  for (const name of path) {
    const next = [];
    for (const b of cur) for (const c of boxes(view, b.body, b.end)) if (c.type === name) next.push(c);
    cur = next;
  }
  return cur;
}

export function snapRate(fps) {
  const near = RATES.reduce((a, b) => Math.abs(b - fps) < Math.abs(a - fps) ? b : a, RATES[0]);
  return Math.abs(near - fps) < 0.02 * fps ? near : Math.round(fps * 1000) / 1000;
}

export function parseMoov(buffer) {
  const view = new DataView(buffer), root = { body: 0, end: buffer.byteLength };
  const moov = [...boxes(view, 0, buffer.byteLength)].find(b => b.type === 'moov') ?? (fourcc(view, 4) === 'moov' ? { body: 8, end: buffer.byteLength } : null);
  if (!moov) return null;
  for (const trak of find(view, moov, ['trak'])) {
    const hdlr = find(view, trak, ['mdia', 'hdlr'])[0];
    if (!hdlr || fourcc(view, hdlr.body + 8) !== 'vide') continue;
    const mdhd = find(view, trak, ['mdia', 'mdhd'])[0], v = view.getUint8(mdhd.body);
    const timescale = view.getUint32(mdhd.body + (v === 1 ? 20 : 12));
    const duration = v === 1 ? Number(view.getBigUint64(mdhd.body + 24)) : view.getUint32(mdhd.body + 16);
    const stts = find(view, trak, ['mdia', 'minf', 'stbl', 'stts'])[0];
    let frames = 0, best = 0, bestCount = -1;
    if (stts) {
      const n = view.getUint32(stts.body + 4);
      for (let i = 0; i < n; i++) {
        const count = view.getUint32(stts.body + 8 + i * 8), delta = view.getUint32(stts.body + 12 + i * 8);
        frames += count;
        if (count > bestCount) { bestCount = count; best = delta; }
      }
    }
    const tkhd = find(view, trak, ['tkhd'])[0];
    let width = 0, height = 0, rotation = 0;
    if (tkhd) {
      const mat = tkhd.body + (view.getUint8(tkhd.body) === 1 ? 52 : 40);
      rotation = Math.round(Math.atan2(view.getInt32(mat + 4) / 65536, view.getInt32(mat) / 65536) * 180 / Math.PI);
      width = view.getUint32(mat + 36) / 65536;
      height = view.getUint32(mat + 40) / 65536;
    }
    const fps = best > 0 ? snapRate(timescale / best) : frames && duration ? snapRate(frames * timescale / duration) : 0;
    return { fps, frames, duration: timescale ? duration / timescale : 0, width, height, rotation, timescale };
  }
  return null;
}

export async function probeMp4(file) {
  const size = file.size, read = async (a, b) => file.slice(a, b).arrayBuffer();
  let p = 0;
  for (let guard = 0; guard < 64 && p + 8 <= size; guard++) {
    const h = new DataView(await read(p, Math.min(size, p + 16)));
    let box = h.getUint32(0);
    const type = fourcc(h, 4);
    if (box === 1 && h.byteLength >= 16) box = Number(h.getBigUint64(8));
    else if (box === 0) box = size - p;
    if (box < 8) return null;
    if (type === 'moov') return parseMoov(await read(p, p + box));
    p += box;
  }
  return null;
}

export function estimateFps(video, ms = 1200) {
  return new Promise(resolve => {
    if (!('requestVideoFrameCallback' in HTMLVideoElement.prototype)) return resolve(0);
    const times = [], t0 = performance.now(), was = video.currentTime, muted = video.muted;
    video.muted = true;
    const tick = (now, meta) => {
      times.push(meta.mediaTime);
      if (performance.now() - t0 < ms && times.length < 90) video.requestVideoFrameCallback(tick);
      else finish();
    };
    const finish = () => {
      video.pause();
      video.muted = muted;
      video.currentTime = was;
      const d = [];
      for (let i = 1; i < times.length; i++) if (times[i] > times[i - 1]) d.push(times[i] - times[i - 1]);
      d.sort((a, b) => a - b);
      resolve(d.length >= 5 ? snapRate(1 / d[d.length >> 2]) : 0);
    };
    video.requestVideoFrameCallback(tick);
    video.play().catch(() => resolve(0));
    setTimeout(() => { if (!video.paused) finish(); }, ms + 800);
  });
}
