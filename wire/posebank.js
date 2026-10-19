// Motion-library pose bank: rig-independent canonical bone frames, the compact bank codec, nearest-pose search and
// library-backed inbetween. Quaternions are [x, y, z, w]. Plain ES module, no dependencies (browser and node).
//
// Canonical frame of a bone, carried rigidly by the bone and found without a rest pose (library BVHs bake their rest
// into the channels): y along the bone (to its first canonical descendant), x from a rigid anatomical vector — hip line
// (pelvis), shoulder line (spine_03), knee / elbow / ankle hinge axis averaged over bent frames (legs, arms, feet),
// index-to-pinky line (hands) — or, for the rest, the ancestor's x carried over; z = x × y. Calibrated per skeleton
// from its own motion (rigCalibration). World is Y up; the pelvis frame faces +Z with the body's left at +X.
// Bank poses store canonical local rotations (C_k = F_parent⁻¹ F_k) with the pelvis heading removed, so UE, Mixamo,
// Auto-Rig Pro and studio rigs compare whatever their bone axes and rest pose.

export const BANK_FORMAT = 'fulcrum-pose-bank-v1';
const S = ['l', 'r'];
export const BANK_BONES = ['pelvis', 'spine_01', 'spine_02', 'spine_03', 'neck_01', 'head',
  ...S.flatMap(s => ['clavicle', 'upperarm', 'lowerarm', 'hand'].map(b => `${b}_${s}`)), ...S.flatMap(s => ['thigh', 'calf', 'foot', 'ball'].map(b => `${b}_${s}`))];
const PARENT_NAME = { spine_01: 'pelvis', spine_02: 'spine_01', spine_03: 'spine_02', neck_01: 'spine_03', head: 'neck_01',
  ...Object.fromEntries(S.flatMap(s => [[`clavicle_${s}`, 'spine_03'], [`upperarm_${s}`, `clavicle_${s}`], [`lowerarm_${s}`, `upperarm_${s}`], [`hand_${s}`, `lowerarm_${s}`],
    [`thigh_${s}`, 'pelvis'], [`calf_${s}`, `thigh_${s}`], [`foot_${s}`, `calf_${s}`], [`ball_${s}`, `foot_${s}`]])) };
export const BANK_PARENT = BANK_BONES.map(b => b === 'pelvis' ? -1 : BANK_BONES.indexOf(PARENT_NAME[b]));
export const NB = BANK_BONES.length;
const DIR = { pelvis: ['spine_01', 'spine_02', 'spine_03', 'neck_01'], spine_01: ['spine_02', 'spine_03', 'neck_01'], spine_02: ['spine_03', 'neck_01'], spine_03: ['neck_01', 'head'], neck_01: ['head'], head: [],
  ...Object.fromEntries(S.flatMap(s => [[`clavicle_${s}`, [`upperarm_${s}`]], [`upperarm_${s}`, [`lowerarm_${s}`]], [`lowerarm_${s}`, [`hand_${s}`]], [`hand_${s}`, [`middle_01_${s}`, `index_01_${s}`]],
    [`thigh_${s}`, [`calf_${s}`]], [`calf_${s}`, [`foot_${s}`]], [`foot_${s}`, [`ball_${s}`]], [`ball_${s}`, []]])) };
// x reference: ['line', a, b] = P_a - P_b (rigid children); ['hinge', a, b, c] = (P_b - P_a) × (P_c - P_b) on bent frames
const XREF = { pelvis: [['line', 'thigh_l', 'thigh_r']], spine_03: [['line', 'clavicle_l', 'clavicle_r'], ['line', 'upperarm_l', 'upperarm_r']],
  ...Object.fromEntries(S.flatMap(s => [[`upperarm_${s}`, [['hinge', `upperarm_${s}`, `lowerarm_${s}`, `hand_${s}`]]], [`lowerarm_${s}`, [['hinge', `upperarm_${s}`, `lowerarm_${s}`, `hand_${s}`]]],
    [`hand_${s}`, [['line', `index_01_${s}`, `pinky_01_${s}`]]], [`thigh_${s}`, [['hinge', `thigh_${s}`, `calf_${s}`, `foot_${s}`]]], [`calf_${s}`, [['hinge', `thigh_${s}`, `calf_${s}`, `foot_${s}`]]],
    [`foot_${s}`, [['hinge', `calf_${s}`, `foot_${s}`, `ball_${s}`]]]])) };

// ---------- quaternion / vector math ----------
export const qmul = (a, b) => [a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
export const qconj = a => [-a[0], -a[1], -a[2], a[3]];
export const qnorm = a => { const l = Math.hypot(a[0], a[1], a[2], a[3]) || 1; return [a[0] / l, a[1] / l, a[2] / l, a[3] / l]; };
const qdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
export const qangle = (a, b) => 2 * Math.acos(Math.min(1, Math.abs(qdot(a, b))));
export const qrot = (q, v) => { const [x, y, z, w] = q, ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x]; };
export function qslerp(a, b, t) {
  let d = qdot(a, b), bb = b;
  if (d < 0) { d = -d; bb = [-b[0], -b[1], -b[2], -b[3]]; }
  if (d > 0.9995) return qnorm(a.map((x, i) => x + (bb[i] - x) * t));
  const th = Math.acos(d), s = Math.sin(th), u = Math.sin((1 - t) * th) / s, v = Math.sin(t * th) / s;
  return [a[0] * u + bb[0] * v, a[1] * u + bb[1] * v, a[2] * u + bb[2] * v, a[3] * u + bb[3] * v];
}
export const qyaw = a => [0, Math.sin(a / 2), 0, Math.cos(a / 2)];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = a => { const l = Math.hypot(a[0], a[1], a[2]); return l > 1e-12 ? [a[0] / l, a[1] / l, a[2] / l] : null; };
// rotation whose matrix columns are x, y, z
function qbasis(x, y, z) {
  const m00 = x[0], m11 = y[1], m22 = z[2], tr = m00 + m11 + m22;
  let q;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; q = [(y[2] - z[1]) / s, (z[0] - x[2]) / s, (x[1] - y[0]) / s, s / 4]; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; q = [s / 4, (y[0] + x[1]) / s, (z[0] + x[2]) / s, (y[2] - z[1]) / s]; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; q = [(y[0] + x[1]) / s, s / 4, (z[1] + y[2]) / s, (z[0] - x[2]) / s]; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; q = [(z[0] + x[2]) / s, (z[1] + y[2]) / s, s / 4, (x[1] - y[0]) / s]; }
  return qnorm(q);
}
const get4 = (A, o) => [A[o], A[o + 1], A[o + 2], A[o + 3]];
const wrapPi = a => a - 2 * Math.PI * Math.round(a / (2 * Math.PI));

// Heading of a canonical pelvis frame: blend of its forward axis and (lateral × up), robust when lying down.
export function headingOf(Fp) {
  const z = qrot(Fp, [0, 0, 1]), x = qrot(Fp, [1, 0, 0]);
  return Math.atan2(z[0] - x[2], z[2] + x[0]);
}

// ---------- rigs ----------
// lookup(canonicalName incl. fingers) -> rig bone index or undefined. add(worldQ, worldP) once per sampled frame
// (functions of a rig bone index, Y-up world); finish() -> rig { idx, anc, G, O[k] (bone-local canonical frame), scale }.
export function rigCalibration(lookup) {
  const at = k => { const i = lookup(k); return i === undefined || i === null || i < 0 ? -1 : i; };
  const idx = BANK_BONES.map(at);
  if (idx[0] < 0) throw new Error('Pose bank needs a pelvis bone.');
  const has = k => at(k) >= 0, anc = new Int32Array(NB).fill(-1);
  BANK_BONES.forEach((b, k) => { let a = BANK_PARENT[k]; while (a >= 0 && idx[a] < 0) a = BANK_PARENT[a]; anc[k] = a; });
  const ySum = BANK_BONES.map(() => [0, 0, 0]), xSum = BANK_BONES.map(() => [0, 0, 0]), xW = new Float64Array(NB), kept = [];
  let child = null, scale = 0, frames = 0;
  const add3 = (A, v, w = 1) => { A[0] += v[0] * w; A[1] += v[1] * w; A[2] += v[2] * w; };
  return {
    add(worldQ, worldP) {
      const P = k => worldP(at(k));
      if (!child) {
        const top = ['neck_01', 'head', 'spine_03', 'spine_02', 'spine_01'].find(has);
        if (!top || !(has('thigh_l') && has('thigh_r'))) throw new Error('Pose bank needs both hips and a spine or head bone.');
        scale = Math.hypot(...sub(P(top), P('pelvis'))) || 1;
        child = BANK_BONES.map(b => DIR[b].find(n => has(n) && Math.hypot(...sub(P(n), P(b))) > 0.03 * scale) ?? null);
      }
      const W = idx.map(i => i >= 0 ? qnorm(worldQ(i)) : null);
      BANK_BONES.forEach((b, k) => {
        if (idx[k] < 0) return;
        const inv = qconj(W[k]), d = child[k] ? unit(sub(P(child[k]), P(b))) : anc[k] >= 0 ? unit(sub(P(b), P(BANK_BONES[anc[k]]))) : [0, 1, 0];
        if (d) add3(ySum[k], qrot(inv, d));
        for (const r of XREF[b] ?? []) {
          if (!r.slice(1).every(has)) continue;
          if (r[0] === 'line') { const v = unit(sub(P(r[1]), P(r[2]))); if (v) { add3(xSum[k], qrot(inv, v)); xW[k]++; } break; }
          const u = unit(sub(P(r[2]), P(r[1]))), v = unit(sub(P(r[3]), P(r[2])));
          if (u && v) { const c = cross(u, v), sn = Math.hypot(...c); if (sn > 0.25) { add3(xSum[k], qrot(inv, c), 1 / sn); xW[k]++; } }
          break;
        }
      });
      kept.push(W); frames++;
    },
    finish() {
      if (!frames) throw new Error('Pose bank calibration needs at least one frame.');
      const O = new Array(NB).fill(null);
      BANK_BONES.forEach((b, k) => {
        if (idx[k] < 0) return;
        const y = unit(ySum[k]) ?? [0, 1, 0];
        let x = xW[k] >= 3 ? unit(xSum[k]) : null;
        if (!x) {
          // carry the ancestor's lateral axis over; clavicles lie along it, so they carry its forward axis instead
          const acc = [0, 0, 0], a = anc[k], ax = b.startsWith('clavicle') ? [0, 0, 1] : [1, 0, 0];
          for (const W of kept) add3(acc, qrot(qconj(W[k]), a >= 0 ? qrot(qmul(W[a], O[a]), ax) : ax));
          x = ax[2] ? unit(cross(y, acc)) : unit(acc);
        }
        x = x && unit(sub(x, y.map(v => v * dot(x, y))));
        if (!x) x = unit(Math.abs(y[0]) < 0.9 ? cross(y, [0, 0, 1]) : cross(y, [0, 1, 0]));
        O[k] = qbasis(x, y, cross(x, y));
      });
      return { idx, anc, G: [0, 0, 0, 1], O, scale, frames };
    },
  };
}

// a rig from a stored calibration (bone-local frames O per bank bone, as finish() returns them)
export function rigFromCalibration(lookup, O, scale = 1) {
  const idx = BANK_BONES.map(k => { const i = lookup(k); return i === undefined || i === null || i < 0 ? -1 : i; }), anc = new Int32Array(NB).fill(-1);
  BANK_BONES.forEach((b, k) => { let a = BANK_PARENT[k]; while (a >= 0 && idx[a] < 0) a = BANK_PARENT[a]; anc[k] = a; });
  return { idx, anc, G: [0, 0, 0, 1], O: O.map(q => q && qnorm(q)), scale };
}

// Canonical world frames F (NB*4) from a function giving each rig bone's world rotation. Missing bones copy their ancestor.
export function canonFrames(rig, worldQ) {
  const F = new Float64Array(NB * 4);
  for (let k = 0; k < NB; k++) {
    const q = rig.idx[k] >= 0 ? qmul(qmul(rig.G, worldQ(rig.idx[k])), rig.O[k]) : rig.anc[k] >= 0 ? get4(F, rig.anc[k] * 4) : [0, 0, 0, 1];
    F.set(qnorm(q), k * 4);
  }
  return F;
}
// heading-free frames F' and anc-relative local rotations L (NB*4), plus yaw
export function canonPose(F, anc = BANK_PARENT) {
  const yaw = headingOf(get4(F, 0)), R = qyaw(-yaw), Fh = new Float64Array(NB * 4), L = new Float64Array(NB * 4);
  for (let k = 0; k < NB; k++) Fh.set(qnorm(qmul(R, get4(F, k * 4))), k * 4);
  for (let k = 0; k < NB; k++) L.set(anc[k] < 0 ? get4(Fh, k * 4) : qnorm(qmul(qconj(get4(Fh, anc[k] * 4)), get4(Fh, k * 4))), k * 4);
  return { yaw, Fh, L };
}
// chain anc-relative locals back to heading-free frames
export function chainPose(L, anc = BANK_PARENT) {
  const F = new Float64Array(NB * 4);
  for (let k = 0; k < NB; k++) F.set(anc[k] < 0 ? get4(L, k * 4) : qnorm(qmul(get4(F, anc[k] * 4), get4(L, k * 4))), k * 4);
  return F;
}

// Studio skeletons: local rotations q (n*nb*4), parents, and the world rotation above each root bone.
export function studioWorld(q, nb, f, parents, rootParentQ) {
  const W = new Array(nb), go = i => { if (W[i]) return W[i]; const p = parents[i], l = get4(q, (f * nb + i) * 4); return (W[i] = qnorm(qmul(p >= 0 ? go(p) : rootParentQ(i), l))); };
  for (let i = 0; i < nb; i++) go(i);
  return W;
}
// Calibrate a studio rig from its take: local rotations q and world matrices W (n*nb*16), at most `samples` frames.
export function calibrateTakeRig({ lookup, q, W, nb, n, parents, rootParentQ, samples = 240 }) {
  const cal = rigCalibration(lookup), step = Math.max(1, Math.floor(n / samples)), pos = matrixPositions(W, nb);
  for (let t = 0; t < n; t += step) { const Q = studioWorld(q, nb, t, parents, rootParentQ); cal.add(i => Q[i], i => pos(t, i)); }
  return cal.finish();
}
// Target canonical world frames F -> local rotations for every studio bone; non-canonical bones keep `q` at frame f.
export function studioLocal(rig, F, q, nb, f, parents, rootParentQ) {
  const target = new Map(), Gi = qconj(rig.G);
  rig.idx.forEach((i, k) => { if (i >= 0) target.set(i, qmul(qmul(Gi, get4(F, k * 4)), qconj(rig.O[k]))); });
  const W = new Array(nb), out = new Float32Array(nb * 4);
  const go = i => {
    if (W[i]) return W[i];
    const p = parents[i], pw = p >= 0 ? go(p) : rootParentQ(i);
    if (target.has(i)) { W[i] = qnorm(target.get(i)); let l = qnorm(qmul(qconj(pw), W[i])); const c = get4(q, (f * nb + i) * 4); if (qdot(l, c) < 0) l = l.map(v => -v); out.set(l, i * 4); }
    else { const l = get4(q, (f * nb + i) * 4); out.set(l, i * 4); W[i] = qnorm(qmul(pw, l)); }
    return W[i];
  };
  for (let i = 0; i < nb; i++) go(i);
  return out;
}

// Root and contact context of frame f: pelvis velocity (m/s, heading frame), foot contacts, pelvis height over the floor.
// pos(t, i) gives rig bone i's world position in metres; floor comes from contextFloor (5th percentile of the low foot).
const lowFoot = (pos, rig, t, s) => { let best = null; for (const b of ['foot_', 'ball_']) { const i = rig.idx[BANK_BONES.indexOf(b + s)]; if (i < 0) continue; const y = qrot(rig.G, pos(t, i))[1]; if (!best || y < best.y) best = { i, y }; } return best; };
export function contextFloor({ pos, n, rig }) {
  const ys = [], step = Math.max(1, Math.floor(n / 400));
  for (let t = 0; t < n; t += step) for (const s of S) { const l = lowFoot(pos, rig, t, s); if (l) ys.push(l.y); }
  ys.sort((x, y) => x - y);
  return ys.length ? ys[Math.floor(ys.length * 0.05)] : 0;
}
export function frameContext({ pos, n, fps, rig, f, yaw, floor }) {
  const pel = rig.idx[0], a = Math.max(0, f - 1), b = Math.min(n - 1, f + 1), dt = Math.max(1, b - a) / fps;
  const toLocal = v => qrot(qyaw(-yaw), qrot(rig.G, v));
  const vel = toLocal(sub(pos(b, pel), pos(a, pel)).map(v => v / dt));
  const contact = S.map(s => { const l = lowFoot(pos, rig, f, s); if (!l) return null; const v = toLocal(sub(pos(b, l.i), pos(a, l.i))).map(x => x / dt); return l.y - floor < 0.04 && Math.hypot(v[0], v[2]) < 0.5 ? 1 : 0; });
  return { vel, contact, height: qrot(rig.G, pos(f, pel))[1] - floor };
}
// studio world matrices W (n*nb*16, translation at 12..14), `upm` units per metre
export const matrixPositions = (W, nb, upm = 1) => (t, i) => [W[(t * nb + i) * 16 + 12] / upm, W[(t * nb + i) * 16 + 13] / upm, W[(t * nb + i) * 16 + 14] / upm];

// ---------- bank codec ----------
const Q16 = 32767;
// frames: array of { L: Float64Array(NB*4) heading-free canonical locals (BANK_PARENT), vel, yaw, height, contact:[l,r], phase }
export function encodeBank(frames) {
  const N = frames.length, q = new Int16Array(N * NB * 4), root = new Int16Array(N * 5), flags = new Uint8Array(N), phase = new Uint8Array(N);
  const c16 = v => Math.max(-32768, Math.min(32767, Math.round(v)));
  frames.forEach((fr, t) => {
    for (let k = 0; k < NB; k++) { const o = k * 4, s = fr.L[o + 3] < 0 ? -1 : 1; for (let c = 0; c < 4; c++) q[(t * NB + k) * 4 + c] = c16(fr.L[o + c] * s * Q16); }
    root.set([c16(fr.vel[0] * 1000), c16(fr.vel[1] * 1000), c16(fr.vel[2] * 1000), c16(wrapPi(fr.yaw) / Math.PI * Q16), c16(fr.height * 1000)], t * 5);
    flags[t] = (fr.contact[0] ? 1 : 0) | (fr.contact[1] ? 2 : 0); phase[t] = fr.phase ?? 255;
  });
  const parts = [q, root, flags, phase], layout = {}, names = ['quats', 'root', 'flags', 'phase'];
  let off = 0;
  parts.forEach((p, i) => { layout[names[i]] = { offset: off, bytes: p.byteLength, type: p.constructor.name.replace('Array', '').toLowerCase() }; off += p.byteLength; off = (off + 3) & ~3; });
  const buf = new Uint8Array(off);
  parts.forEach((p, i) => buf.set(new Uint8Array(p.buffer, p.byteOffset, p.byteLength), layout[names[i]].offset));
  layout.quats.scale = Q16; layout.root.fields = ['vx_mm_s', 'vy_mm_s', 'vz_mm_s', 'yaw_pi_q15', 'pelvis_height_mm']; layout.flags.bits = ['contact_l', 'contact_r'];
  return { bytes: buf, layout };
}

export function decodeBank(meta, buffer) {
  if (meta.format !== BANK_FORMAT) throw new Error('Unsupported pose bank format: ' + meta.format);
  if (meta.bones.join() !== BANK_BONES.join()) throw new Error('Pose bank bones do not match this build.');
  const u8 = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer), N = meta.frames, L = meta.layout;
  const view = (k, T) => new T(u8.buffer.slice(u8.byteOffset + L[k].offset, u8.byteOffset + L[k].offset + L[k].bytes));
  const q16 = view('quats', Int16Array), r16 = view('root', Int16Array), flags = view('flags', Uint8Array), phase = view('phase', Uint8Array);
  if (q16.length !== N * NB * 4 || r16.length !== N * 5) throw new Error('Pose bank binary does not match its catalogue.');
  const C = new Float32Array(N * NB * 4), F = new Float32Array(N * NB * 4), vel = new Float32Array(N * 3), yaw = new Float32Array(N), height = new Float32Array(N), clipOf = new Int32Array(N);
  for (let t = 0; t < N; t++) {
    for (let k = 0; k < NB; k++) {
      const o = (t * NB + k) * 4, c = qnorm([q16[o] / Q16, q16[o + 1] / Q16, q16[o + 2] / Q16, q16[o + 3] / Q16]);
      C.set(c, o);
      const p = BANK_PARENT[k];
      F.set(p < 0 ? c : qnorm(qmul(get4(F, (t * NB + p) * 4), c)), o);
    }
    vel.set([r16[t * 5] / 1000, r16[t * 5 + 1] / 1000, r16[t * 5 + 2] / 1000], t * 3); yaw[t] = r16[t * 5 + 3] / Q16 * Math.PI; height[t] = r16[t * 5 + 4] / 1000;
  }
  const yawU = new Float64Array(N);
  meta.clips.forEach((c, ci) => { clipOf.fill(ci, c.start, c.start + c.n); for (let t = c.start + 1; t < c.start + c.n; t++) yawU[t] = yawU[t - 1] + wrapPi(yaw[t] - yaw[t - 1]); });
  return { meta, N, fps: meta.fps, clips: meta.clips, C, F, vel, yaw, yawU, height, contact: flags, phase, clipOf };
}

// bank local of bone k at frame t, relative to `a` (an ancestor in canonical order; -1 = heading-free world)
function bankLocal(bank, t, k, a) {
  if (a === BANK_PARENT[k]) return get4(bank.C, (t * NB + k) * 4);
  const f = get4(bank.F, (t * NB + k) * 4);
  return a < 0 ? f : qnorm(qmul(qconj(get4(bank.F, (t * NB + a) * 4)), f));
}
const ctxCost = (bank, t, c, w) => {
  if (!c) return 0;
  let s = 0;
  if (c.vel) { const dx = bank.vel[t * 3] - c.vel[0], dy = bank.vel[t * 3 + 1] - c.vel[1], dz = bank.vel[t * 3 + 2] - c.vel[2]; s += w.vel * (dx * dx + dy * dy + dz * dz); }
  if (c.contact) for (let k = 0; k < 2; k++) if (c.contact[k] != null && ((bank.contact[t] >> k) & 1) !== c.contact[k]) s += w.contact;
  if (Number.isFinite(c.height)) { const d = bank.height[t] - c.height; s += w.height * d * d; }
  return s;
};
export const CONTEXT_WEIGHTS = Object.freeze({ vel: 0.01, contact: 0.01, height: 1 });
const excluded = (bank, ex) => { const s = new Set(); for (const e of ex ?? []) bank.clips.forEach((c, i) => { if (i === e || c.file === e || c.name === e) s.add(i); }); return s; };

// Nearest bank poses on guided bones (bank bone indices) of a target pose given as anc-relative locals L (canonPose).
export function searchBank(bank, { L, anc = BANK_PARENT, guides, context = null, k = 3, exclude = [], weights = CONTEXT_WEIGHTS, perClip = 1 }) {
  if (!guides.length) return [];
  const ex = excluded(bank, exclude), T = guides.map(g => get4(L, g * 4)), rows = [];
  for (let t = 0; t < bank.N; t++) {
    if (ex.has(bank.clipOf[t])) continue;
    let e = 0;
    for (let j = 0; j < guides.length; j++) { const d = qangle(T[j], bankLocal(bank, t, guides[j], anc[guides[j]])); e += d * d; }
    e /= guides.length;
    const cx = ctxCost(bank, t, context, weights);
    rows.push({ t, error: Math.sqrt(e), context: cx, score: e + cx });
  }
  rows.sort((a, b) => a.score - b.score);
  const out = [], used = new Map();
  for (const r of rows) {
    const c = bank.clipOf[r.t], n = used.get(c) ?? 0;
    if (n >= perClip) continue;
    used.set(c, n + 1); out.push(describe(bank, r)); if (out.length >= k) break;
  }
  return out;
}
function describe(bank, r) {
  const c = bank.clips[bank.clipOf[r.t]], local = r.t - c.start;
  return { ...r, clip: bank.clipOf[r.t], clipName: c.name, clipFile: c.file, category: c.category, clipFrame: Math.round(local * c.src_fps / bank.fps), phase: bank.meta.phases?.[bank.phase[r.t]] ?? null };
}
// Full heading-free pose of bank frame t as anc-relative locals; bones in `keep` take values from L.
export function bankPose(bank, t, anc = BANK_PARENT, keep = [], L = null) {
  const out = new Float64Array(NB * 4), K = new Set(keep);
  for (let k = 0; k < NB; k++) out.set(K.has(k) && L ? get4(L, k * 4) : bankLocal(bank, t, k, anc[k]), k * 4);
  return out;
}

// Library inbetween: windowed match of the two key poses (anc-relative locals LA, LB with yaws) against bank segments
// whose length is the gap time-warped by `warp`; returns ranked segments and, for the best, `frames(m)` = heading-applied
// canonical world frames for m evenly spaced inner frames, displacement-warped so they meet both keys exactly.
export function bankInbetween(bank, { LA, LB, yawA = 0, yawB = 0, anc = BANK_PARENT, ctxA = null, ctxB = null, seconds, warp = [0.7, 1.45], k = 3, exclude = [], weights = CONTEXT_WEIGHTS, present = null, yawWeight = 0.05 }) {
  const ex = excluded(bank, exclude), use = BANK_BONES.map((_, i) => i).filter(b => !present || present[b]);
  const A = use.map(b => get4(LA, b * 4)), B = use.map(b => get4(LB, b * 4)), pA = new Float64Array(bank.N), pB = new Float64Array(bank.N), dA = new Float64Array(bank.N), dB = new Float64Array(bank.N);
  for (let t = 0; t < bank.N; t++) {
    if (ex.has(bank.clipOf[t])) continue;
    let a = 0, b = 0;
    for (let j = 0; j < use.length; j++) { const q = bankLocal(bank, t, use[j], anc[use[j]]), x = qangle(A[j], q), y = qangle(B[j], q); a += x * x; b += y * y; }
    pA[t] = a / use.length; pB[t] = b / use.length; dA[t] = pA[t] + ctxCost(bank, t, ctxA, weights); dB[t] = pB[t] + ctxCost(bank, t, ctxB, weights);
  }
  const span = Math.max(1, seconds * bank.fps), lo = Math.max(1, Math.ceil(span * warp[0] - 1e-9)), hi = Math.max(lo, Math.floor(span * warp[1] + 1e-9)), dYaw = wrapPi(yawB - yawA), rows = [];
  bank.clips.forEach((c, ci) => {
    if (ex.has(ci)) return;
    let best = null;
    for (let s = c.start; s < c.start + c.n; s++) for (let L = lo; L <= hi && s + L < c.start + c.n; L++) {
      const e = s + L, dy = wrapPi(dYaw - unwrapYaw(bank, s, e)), score = dA[s] + dB[e] + yawWeight * dy * dy + 0.002 * Math.log(L / span) ** 2;
      if (!best || score < best.score) best = { s, e, score };
    }
    if (best) rows.push(best);
  });
  rows.sort((a, b) => a.score - b.score);
  const out = rows.slice(0, k).map(r => { const ci = bank.clipOf[r.s], c = bank.clips[ci];
    return { ...r, clip: ci, clipName: c.name, clipFile: c.file, category: c.category, startFrame: Math.round((r.s - c.start) * c.src_fps / bank.fps), endFrame: Math.round((r.e - c.start) * c.src_fps / bank.fps), warp: (r.e - r.s) / span, poseErrorDeg: Math.sqrt((pA[r.s] + pB[r.e]) / 2) * 180 / Math.PI }; });
  const frames = (seg, m) => {
    const res = [], dSA = [], dSB = [];
    for (let k2 = 0; k2 < NB; k2++) { const a = get4(LA, k2 * 4), b = get4(LB, k2 * 4); dSA.push(qmul(a, qconj(bankLocal(bank, seg.s, k2, anc[k2])))); dSB.push(qmul(b, qconj(bankLocal(bank, seg.e, k2, anc[k2])))); }
    const yS = unwrapYaw(bank, seg.s, seg.e), yawFix = wrapPi(dYaw - yS);
    for (let j = 1; j <= m; j++) {
      const u = j / (m + 1), tau = seg.s + u * (seg.e - seg.s), t0 = Math.floor(tau), t1 = Math.min(seg.e, t0 + 1), w = tau - t0, Lu = new Float64Array(NB * 4);
      for (let k2 = 0; k2 < NB; k2++) Lu.set(qnorm(qmul(qslerp(dSA[k2], dSB[k2], u), qslerp(bankLocal(bank, t0, k2, anc[k2]), bankLocal(bank, t1, k2, anc[k2]), w))), k2 * 4);
      const Fh = chainPose(Lu, anc), yaw = yawA + unwrapYaw(bank, seg.s, t0) + w * unwrapYaw(bank, t0, t1) + u * yawFix, R = qyaw(yaw), F = new Float64Array(NB * 4);
      for (let k2 = 0; k2 < NB; k2++) F.set(qnorm(qmul(R, get4(Fh, k2 * 4))), k2 * 4);
      res.push({ F, L: Lu, yaw });
    }
    return res;
  };
  return { segments: out, frames: (m, seg = out[0]) => seg ? frames(seg, m) : [] };
}
const unwrapYaw = (bank, s, e) => bank.yawU[e] - bank.yawU[s];
