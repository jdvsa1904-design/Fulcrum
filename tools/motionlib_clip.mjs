// Loads one motion library BVH into the studio's motion layout: world matrices W (n*nb*16, column-major,
// Y up, centimetres, upm = 100), the canonical rig (wire/rigmap.js names -> bone index) and a standing rest
// pose built from the clip's own bone lengths (the library rigs have zero offsets, so their bind pose is
// degenerate). Shared by tools/motionlib_prior.mjs and tests/phases_library.test.mjs. Node only (reads files).
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { parseBVH } from '../mocap/bvh.js';
import { buildRigMap } from '../wire/rigmap.js';
import { rigIndices, legsOf } from '../wire/facts.js';

export const UPM = 100;
const D = Math.PI / 180;
const qmul = (a, b) => [a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qaxis = (ax, deg) => { const h = deg * D / 2, s = Math.sin(h), c = Math.cos(h); return ax === 'X' ? [s, 0, 0, c] : ax === 'Y' ? [0, s, 0, c] : [0, 0, s, c]; };
const qrot = (q, v) => { const [x, y, z, w] = q, ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x]; };
function compose(q, p, out, o) {
  const [x, y, z, w] = q, x2 = x + x, y2 = y + y, z2 = z + z, xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  out[o] = 1 - (yy + zz); out[o + 1] = xy + wz; out[o + 2] = xz - wy; out[o + 3] = 0;
  out[o + 4] = xy - wz; out[o + 5] = 1 - (xx + zz); out[o + 6] = yz + wx; out[o + 7] = 0;
  out[o + 8] = xz + wy; out[o + 9] = yz - wx; out[o + 10] = 1 - (xx + yy); out[o + 11] = 0;
  out[o + 12] = p[0]; out[o + 13] = p[1]; out[o + 14] = p[2]; out[o + 15] = 1;
}

// FK of every frame into W. unit converts file units to centimetres.
function forward(bvh, unit) {
  const J = bvh.joints, nb = J.length, n = bvh.values.length, W = new Float32Array(n * nb * 16), cols = [];
  let c = 0;
  for (const j of J) { cols.push(c); c += j.channels.length; }
  const P = new Float64Array(nb * 3), Q = new Float64Array(nb * 4);
  for (let f = 0; f < n; f++) {
    const row = bvh.values[f];
    for (let i = 0; i < nb; i++) {
      const j = J[i], t = [...(j.offset ?? [0, 0, 0])];
      let q = [0, 0, 0, 1];
      j.channels.forEach((ch, k) => { const v = row[cols[i] + k]; if (ch.endsWith('position')) t['XYZ'.indexOf(ch[0])] += v; else q = qmul(q, qaxis(ch[0], v)); });
      for (let k = 0; k < 3; k++) t[k] *= unit;
      let wq = q, wp = t;
      if (j.parent >= 0) { const pq = Array.from(Q.subarray(j.parent * 4, j.parent * 4 + 4)), r = qrot(pq, t); wq = qmul(pq, q); wp = [P[j.parent * 3] + r[0], P[j.parent * 3 + 1] + r[1], P[j.parent * 3 + 2] + r[2]]; }
      Q.set(wq, i * 4); P.set(wp, i * 3);
      compose(wq, wp, W, (f * nb + i) * 16);
    }
  }
  return { W, n, nb };
}

const pos = (W, nb, f, i) => { const o = (f * nb + i) * 16 + 12; return [W[o], W[o + 1], W[o + 2]]; };
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

// A standing rest pose: frame-0 rotations, legs straight down under the hips with the clip's own segment lengths.
function standingRest(W, nb, rig) {
  const R = Float32Array.from(W.subarray(0, nb * 16)), legs = legsOf(rig), set = (i, p) => { if (i !== undefined) R.set(p, i * 16 + 12); };
  const lens = {};
  for (const s of ['l', 'r']) { const L = legs[s]; lens[s] = { th: dist(pos(W, nb, 0, L.thigh), pos(W, nb, 0, L.calf)), sh: dist(pos(W, nb, 0, L.calf), pos(W, nb, 0, L.foot)), ft: L.ball !== L.foot ? dist(pos(W, nb, 0, L.foot), pos(W, nb, 0, L.ball)) : 0 }; }
  const P0 = pos(W, nb, 0, rig.pelvis), ankle = 8, hipDrop = 8, top = Math.max(...['l', 'r'].map(s => lens[s].th + lens[s].sh)) + ankle;
  const lift = top + hipDrop - P0[1];
  for (let i = 0; i < nb; i++) { const p = pos(W, nb, 0, i); set(i, [p[0] - P0[0], p[1] + lift, p[2] - P0[2]]); }
  for (const [s, x] of [['l', 1], ['r', -1]]) {
    const L = legs[s], hx = 10 * x;
    set(L.thigh, [hx, top, 0]); set(L.calf, [hx, top - lens[s].th, 0]); set(L.foot, [hx, ankle, 0]);
    if (L.ball !== L.foot) set(L.ball, [hx, 1, Math.sqrt(Math.max(0, lens[s].ft ** 2 - (ankle - 1) ** 2))]);
  }
  set(rig.pelvis, [0, top + hipDrop, 0]);
  return R;
}

export function loadClipText(text) {
  const bvh = parseBVH(text);
  const fps = 1 / bvh.frameTime, names = bvh.joints.map(j => j.name);
  const map = buildRigMap(names.map((name, i) => ({ name, parent: bvh.joints[i].parent, pos: [0, 0, 0] })));
  const byName = new Map(names.map((nm, i) => [nm, i]));
  const rig = rigIndices(k => byName.get(map.fromCanonical.get(k)));
  const one = forward({ ...bvh, values: bvh.values.slice(0, 1) }, 1), legs = legsOf(rig);
  let unit = 100;
  if (legs.l.calf !== undefined && legs.l.foot !== undefined && legs.l.thigh !== undefined) {
    const raw = dist(pos(one.W, one.nb, 0, legs.l.thigh), pos(one.W, one.nb, 0, legs.l.calf)) + dist(pos(one.W, one.nb, 0, legs.l.calf), pos(one.W, one.nb, 0, legs.l.foot));
    for (const u of [100, 1, 10, 0.1, 10000]) if (raw * u > 45 && raw * u < 140) { unit = u; break; }
  }
  const { W, n, nb } = forward(bvh, unit);
  const ok = ['pelvis', 'thigh_l', 'calf_l', 'foot_l', 'thigh_r', 'calf_r', 'foot_r'].every(k => rig[k] !== undefined);
  return { W, n, nb, fps, upm: UPM, rig, names, parents: bvh.joints.map(j => j.parent), restW: ok && n ? standingRest(W, nb, rig) : null, rigmap: { confidence: map.confidence, missing: map.missing }, ok };
}

export const loadClip = (dir, file) => loadClipText(readFileSync(join(dir, file), 'utf8'));
