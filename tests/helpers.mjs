import { readFileSync, existsSync } from 'node:fs';
import assert from 'node:assert/strict';

export const json = p => JSON.parse(readFileSync(p, 'utf8'));
export const exists = existsSync;

export function csv(p) {
  const lines = readFileSync(p, 'utf8').trim().split(/\r?\n/);
  const split = l => { const out = []; let cur = '', q = false; for (const c of l) { if (c === '"') q = !q; else if (c === ',' && !q) { out.push(cur); cur = ''; } else cur += c; } out.push(cur); return out; };
  const head = split(lines[0]);
  return lines.slice(1).map(l => { const v = split(l), o = {}; head.forEach((h, i) => { const x = v[i]; o[h] = x === 'True' ? true : x === 'False' ? false : x !== '' && !isNaN(+x) ? +x : x; }); return o; });
}

export function near(a, b, tol, msg) {
  assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} |${a} - ${b}| = ${Math.abs(a - b)} > ${tol}`);
}

export function nearVec(a, b, tol, msg) {
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i] - b[i]));
  assert.ok(m <= tol, `${msg ?? ''} max diff ${m} > ${tol} (${Array.from(a).map(x => +x.toFixed(6))} vs ${Array.from(b).map(x => +x.toFixed(6))})`);
}

export const v3 = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  len: a => Math.hypot(a[0], a[1], a[2]),
  unit: a => { const l = Math.hypot(a[0], a[1], a[2]); return [a[0] / l, a[1] / l, a[2] / l]; },
  dist: (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]),
};

export const q4 = {
  mul: (a, b) => [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]],
  conj: q => [-q[0], -q[1], -q[2], q[3]],
  axis: (k, a) => { const l = Math.hypot(k[0], k[1], k[2]), s = Math.sin(a / 2) / l; return [k[0] * s, k[1] * s, k[2] * s, Math.cos(a / 2)]; },
  rot: (q, v) => {
    const [x, y, z, w] = q, ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
    return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x];
  },
  angle: (a, b) => {
    const d = [
      a[3] * b[0] - a[0] * b[3] - a[1] * b[2] + a[2] * b[1],
      a[3] * b[1] + a[0] * b[2] - a[1] * b[3] - a[2] * b[0],
      a[3] * b[2] - a[0] * b[1] + a[1] * b[0] - a[2] * b[3],
      a[3] * b[3] + a[0] * b[0] + a[1] * b[1] + a[2] * b[2]];
    return 2 * Math.atan2(Math.hypot(d[0], d[1], d[2]), Math.abs(d[3]));
  },
};

export const deg = r => r * 180 / Math.PI;
export const rad = d => d * Math.PI / 180;

export function rng(seed) {
  let s = seed >>> 0;
  return () => { s = (s + 0x6D2B79F5) >>> 0; let t = s; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}
