import { eulerFromQuat, eulerFilter } from '../filters.js';
import { BONES, PARENT, NB, ENDS, getQ } from './skeleton.js';

const R2D = 180 / Math.PI;
const fmt = x => (Math.abs(x) < 5e-7 ? 0 : x).toFixed(4);

export function toBVH(skel, clip, o = {}) {
  const s = o.scale ?? 100, kids = BONES.map((_, i) => BONES.map((__, j) => j).filter(j => PARENT[j] === i));
  const lines = ['HIERARCHY'];
  const emit = (i, depth) => {
    const pad = '\t'.repeat(depth), off = PARENT[i] < 0 ? [0, 0, 0] : skel.offsets[i];
    lines.push(`${pad}${PARENT[i] < 0 ? 'ROOT' : 'JOINT'} ${BONES[i]}`, `${pad}{`, `${pad}\tOFFSET ${off.map(v => fmt(v * s)).join(' ')}`);
    lines.push(`${pad}\tCHANNELS ${PARENT[i] < 0 ? '6 Xposition Yposition Zposition ' : '3 '}Zrotation Xrotation Yrotation`);
    for (const k of kids[i]) emit(k, depth + 1);
    if (ENDS[BONES[i]]) lines.push(`${pad}\tEnd Site`, `${pad}\t{`, `${pad}\t\tOFFSET ${skel.ends[BONES[i]].map(v => fmt(v * s)).join(' ')}`, `${pad}\t}`);
    lines.push(`${pad}}`);
  };
  emit(0, 0);
  const n = clip.n, order = [], tracks = [];
  const walk = i => { order.push(i); for (const k of kids[i]) walk(k); };
  walk(0);
  for (const b of order) {
    const e = new Float64Array(n * 3);
    for (let f = 0; f < n; f++) e.set(eulerFromQuat(getQ(clip.rot, f * NB + b), 'yxz'), f * 3);
    tracks[b] = eulerFilter(e, { order: 'yxz' });
  }
  lines.push('MOTION', `Frames: ${n}`, `Frame Time: ${(1 / clip.fps).toFixed(7)}`);
  for (let f = 0; f < n; f++) {
    const row = [clip.root[f * 3] * s, clip.root[f * 3 + 1] * s, clip.root[f * 3 + 2] * s];
    for (const b of order) { const e = tracks[b]; row.push(e[f * 3 + 2] * R2D, e[f * 3] * R2D, e[f * 3 + 1] * R2D); }
    lines.push(row.map(fmt).join(' '));
  }
  return lines.join('\n') + '\n';
}

export function parseBVH(text) {
  const tok = text.split(/\s+/).filter(Boolean), joints = [], stack = [];
  let i = 0, last = null;
  while (i < tok.length && tok[i] !== 'MOTION') {
    const t = tok[i++];
    if (t === 'ROOT' || t === 'JOINT') { last = { name: tok[i++], parent: stack.length ? stack[stack.length - 1] : -1, offset: null, channels: [] }; joints.push(last); }
    else if (t === 'End') { i++; last = { end: true }; }
    else if (t === '{') stack.push(last && !last.end ? joints.indexOf(last) : -2);
    else if (t === '}') stack.pop();
    else if (t === 'OFFSET') { const v = [+tok[i++], +tok[i++], +tok[i++]]; if (last && !last.end) last.offset = v; }
    else if (t === 'CHANNELS') { const c = +tok[i++]; last.channels = tok.slice(i, i + c); i += c; }
  }
  for (const j of joints) while (j.parent === -2) j.parent = -1;
  i++;
  const frames = +tok[i + 1], frameTime = +tok[i + 4];
  i += 5;
  const width = joints.reduce((a, j) => a + j.channels.length, 0), values = [];
  for (let f = 0; f < frames; f++) { values.push(tok.slice(i, i + width).map(Number)); i += width; }
  return { joints, frames, frameTime, values };
}
