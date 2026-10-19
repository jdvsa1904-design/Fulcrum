import { sub, qmul, qconj, qrot, qyaw, qlog, qexp, qalign, scale, wrapAngle, mean } from './math.js';
import { B, NB, getV, getQ, setQ } from './skeleton.js';
import { takeWorld, worldAt, soleAt } from './ground.js';
import { lowpass } from './clean.js';

const FEAT_JOINTS = [B.LeftFoot, B.RightFoot, B.LeftToeBase, B.RightToeBase, B.LeftLeg, B.RightLeg, B.LeftHand, B.RightHand, B.LeftForeArm, B.RightForeArm, B.Head, B.Spine2];
const VEL_JOINTS = [B.LeftFoot, B.RightFoot, B.LeftHand, B.RightHand];

export function facing(take, fps, fc = 1) {
  const n = take.n, raw = new Float64Array(n);
  let prev = 0;
  for (let i = 0; i < n; i++) {
    const hips = getQ(take.rot, i * NB), chest = qmul(qmul(qmul(hips, getQ(take.rot, i * NB + B.Spine)), getQ(take.rot, i * NB + B.Spine1)), getQ(take.rot, i * NB + B.Spine2));
    const h = qrot(hips, [0, 0, 1]), c = qrot(chest, [0, 0, 1]);
    let a = Math.atan2(h[0] + c[0], h[2] + c[2]);
    if (i > 0) a = prev + wrapAngle(a - prev);
    raw[i] = a; prev = a;
  }
  return fc > 0 ? lowpass(raw, fps, fc) : raw;
}

export function rootVelocity(take, fps, fc = 1) {
  const n = take.n, x = new Float64Array(n), z = new Float64Array(n);
  for (let i = 0; i < n; i++) { x[i] = take.root[i * 3]; z[i] = take.root[i * 3 + 2]; }
  const xs = lowpass(x, fps, fc), zs = lowpass(z, fps, fc), vx = new Float64Array(n), vz = new Float64Array(n), speed = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
    vx[i] = b > a ? (xs[b] - xs[a]) * fps / (b - a) : 0;
    vz[i] = b > a ? (zs[b] - zs[a]) * fps / (b - a) : 0;
    speed[i] = Math.hypot(vx[i], vz[i]);
  }
  return { vx, vz, speed };
}

export function poseFeatures(take, TW, yaw, fps) {
  const n = take.n, D = FEAT_JOINTS.length * 3 + 1 + VEL_JOINTS.length * 3, F = new Float64Array(n * D);
  const rel = (i, j) => qrot(qyaw(-yaw[i]), sub(getV(worldAt(TW, i).pos, j), getV(worldAt(TW, i).pos, B.Hips)));
  for (let i = 0; i < n; i++) {
    let o = i * D;
    for (const j of FEAT_JOINTS) { const r = rel(i, j); F[o++] = r[0]; F[o++] = r[1]; F[o++] = r[2]; }
    F[o++] = take.root[i * 3 + 1];
    const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
    for (const j of VEL_JOINTS) {
      const va = rel(a, j), vb = rel(b, j), k = b > a ? 0.1 * fps / (b - a) : 0;
      F[o++] = (vb[0] - va[0]) * k; F[o++] = (vb[1] - va[1]) * k; F[o++] = (vb[2] - va[2]) * k;
    }
  }
  return { F, D };
}

export const featDist = (feat, i, j) => {
  const { F, D } = feat;
  let s = 0;
  for (let k = 0; k < D; k++) { const d = F[i * D + k] - F[j * D + k]; s += d * d; }
  return Math.sqrt(s / D);
};

export function findLoop(feat, a, b, fps, o = {}) {
  const Lmin = Math.max(4, Math.round((o.minSec ?? 0.6) * fps)), Lmax = Math.min(b - a, Math.round((o.maxSec ?? 2.6) * fps));
  if (Lmax < Lmin) return null;
  const C = [];
  for (let L = Lmin; L <= Lmax; L++) {
    let s = 0, c = 0;
    for (let i = a; i + L <= b; i++) { s += featDist(feat, i, i + L); c++; }
    C.push({ L, cost: c ? s / c : Infinity });
  }
  const best = C.reduce((x, y) => y.cost < x.cost ? y : x);
  let pick = best;
  for (let k = 1; k < C.length - 1; k++) {
    const c = C[k];
    if (c.L >= best.L) break;
    if (c.cost <= C[k - 1].cost && c.cost <= C[k + 1].cost && c.cost <= best.cost * (o.prefer ?? 1.15)) { pick = c; break; }
  }
  const L = pick.L, midSeg = (a + b - L) / 2;
  let start = a, bc = Infinity;
  for (let i = a; i + L <= b; i++) {
    const c = featDist(feat, i, i + L) * (1 + 0.15 * Math.abs(i - midSeg) / Math.max(1, b - a));
    if (c < bc) { bc = c; start = i; }
  }
  let spread = 0;
  const mu = new Float64Array(feat.D);
  for (let i = a; i <= b; i++) for (let k = 0; k < feat.D; k++) mu[k] += feat.F[i * feat.D + k] / (b - a + 1);
  for (let i = a; i <= b; i++) for (let k = 0; k < feat.D; k++) spread += (feat.F[i * feat.D + k] - mu[k]) ** 2;
  spread = Math.sqrt(spread / ((b - a + 1) * feat.D)) || 1e-9;
  return { start, len: L, cost: featDist(feat, start, start + L), meanCost: pick.cost, quality: featDist(feat, start, start + L) / spread, curve: C };
}

function rampLoop(root, rot, L) {
  const dy = root[L * 3 + 1] - root[1], logs = [];
  for (let b = 0; b < NB; b++) logs.push(qlog(qalign(qmul(getQ(rot, L * NB + b), qconj(getQ(rot, b))), [0, 0, 0, 1])));
  for (let k = 0; k <= L; k++) {
    const t = k / L;
    root[k * 3 + 1] -= dy * t;
    for (let b = 0; b < NB; b++) setQ(rot, k * NB + b, qmul(qexp(scale(logs[b], -t)), getQ(rot, k * NB + b)));
  }
}

function contactsIn(plants, a, b) {
  return plants.filter(p => p.b >= a && p.a <= b).map(p => ({ side: p.side, a: Math.max(a, p.a) - a, b: Math.min(b, p.b) - a, sliding: p.sliding }));
}

export function makeLoopClip(skel, take, fps, s, L, plants, o = {}) {
  const yaw = o.yaw ?? facing(take, fps), r0 = getV(take.root, s), r1 = getV(take.root, s + L);
  const dYaw = wrapAngle(yaw[s + L] - yaw[s]), disp = [r1[0] - r0[0], 0, r1[2] - r0[2]], dist = Math.hypot(disp[0], disp[2]);
  const speed = dist * fps / L, midYaw = yaw[s] + dYaw / 2;
  const travelYaw = dist > 0.1 * L / fps ? Math.atan2(disp[0], disp[2]) : midYaw;
  const rel = wrapAngle(travelYaw - midYaw), refYaw = speed > 0.2 ? travelYaw : midYaw;
  const root = new Float64Array((L + 1) * 3), rot = new Float64Array((L + 1) * NB * 4);
  for (let k = 0; k <= L; k++) {
    const t = k / L, R = qyaw(-refYaw - dYaw * (t - 0.5)), src = s + k;
    const p = qrot(R, [take.root[src * 3] - r0[0] - disp[0] * t, 0, take.root[src * 3 + 2] - r0[2] - disp[2] * t]);
    root.set([p[0], take.root[src * 3 + 1], p[2]], k * 3);
    rot.set(take.rot.subarray(src * NB * 4, (src + 1) * NB * 4), k * NB * 4);
    setQ(rot, k * NB, qmul(R, getQ(take.rot, src * NB)));
  }
  rampLoop(root, rot, L);
  let mx = 0, mz = 0;
  for (let k = 0; k < L; k++) { mx += root[k * 3] / L; mz += root[k * 3 + 2] / L; }
  const contacts = contactsIn(plants, s, s + L - 1);
  const left = contacts.filter(c => c.side === 'L' && c.a > 0).sort((x, y) => x.a - y.a)[0] ?? contacts.find(c => c.side === 'L');
  const shift = o.phase0 ?? (left ? left.a % L : 0);
  const outRoot = new Float64Array(L * 3), outRot = new Float64Array(L * NB * 4);
  for (let k = 0; k < L; k++) {
    const src = (k + shift) % L;
    outRoot.set([root[src * 3] - mx, root[src * 3 + 1], root[src * 3 + 2] - mz], k * 3);
    outRot.set(rot.subarray(src * NB * 4, (src + 1) * NB * 4), k * NB * 4);
  }
  for (let k = 1; k < L; k++) for (let b = 0; b < NB; b++) setQ(outRot, k * NB + b, qalign(getQ(outRot, k * NB + b), getQ(outRot, (k - 1) * NB + b)));
  const direction = Math.abs(rel) < Math.PI / 4 || speed < 0.2 ? 'forward' : Math.abs(rel) > 3 * Math.PI / 4 ? 'back' : rel > 0 ? 'left' : 'right';
  const shifted = contacts.map(c => ({ ...c, a: (c.a - shift + L) % L, b: (c.b - shift + L) % L }));
  return { loop: true, fps, n: L, root: outRoot, rot: outRot, speed, cycle: L / fps, direction, turn: dYaw, src: { a: s, b: s + L }, contacts: shifted, phase0: 0 };
}

export function startYaw(yaw, a, b, fps) {
  const e = Math.min(b, a + Math.round(fps * 0.5));
  let sx = 0, sz = 0;
  for (let i = a; i <= e; i++) { sx += Math.sin(yaw[i]); sz += Math.cos(yaw[i]); }
  return yaw[a] + wrapAngle(Math.atan2(sx, sz) - yaw[a]);
}

export function oneShotClip(skel, take, fps, a, b, o = {}) {
  const yaw = o.yaw ?? facing(take, fps), y0 = startYaw(yaw, a, b, fps), r0 = getV(take.root, a), n = b - a + 1, R = qyaw(-y0);
  const root = new Float64Array(n * 3), rot = new Float64Array(n * NB * 4);
  for (let k = 0; k < n; k++) {
    const src = a + k, p = qrot(R, [take.root[src * 3] - r0[0], 0, take.root[src * 3 + 2] - r0[2]]);
    root.set([p[0], take.root[src * 3 + 1], p[2]], k * 3);
    rot.set(take.rot.subarray(src * NB * 4, (src + 1) * NB * 4), k * NB * 4);
    setQ(rot, k * NB, qmul(R, getQ(take.rot, src * NB)));
  }
  const end = [root[(n - 1) * 3], 0, root[(n - 1) * 3 + 2]];
  return { loop: false, fps, n, root, rot, speed: 0, src: { a, b }, rootMotion: { end, turn: wrapAngle(yaw[b] - y0), distance: Math.hypot(end[0], end[2]) } };
}

function runs(mask) {
  const out = [];
  let s = -1;
  for (let i = 0; i <= mask.length; i++) {
    const on = i < mask.length && mask[i];
    if (on && s < 0) s = i;
    if (!on && s >= 0) { out.push([s, i - 1]); s = -1; }
  }
  return out;
}

function closeGaps(mask, gap) {
  for (const [a, b] of runs(mask.map(x => 1 - x))) if (a > 0 && b < mask.length - 1 && b - a + 1 <= gap) for (let i = a; i <= b; i++) mask[i] = 1;
  return mask;
}

export function segmentTake(take, fps, o = {}) {
  const v = rootVelocity(take, fps, o.fc ?? 1), n = take.n, minLen = Math.round(0.4 * fps);
  let moving = Array.from(v.speed, s => s > (o.moveSpeed ?? 0.35) ? 1 : 0);
  moving = closeGaps(moving, Math.round(0.3 * fps));
  const still = closeGaps(moving.map(x => 1 - x), Math.round(0.3 * fps));
  const segs = [];
  for (const [a, b] of runs(moving)) {
    if (b - a + 1 < minLen) continue;
    const band = new Uint8Array(b - a + 1);
    let cur = v.speed[a] > (o.runSpeed ?? 2.0) ? 1 : 0;
    for (let i = a; i <= b; i++) {
      if (cur === 0 && v.speed[i] > (o.runEnter ?? 2.2)) cur = 1;
      else if (cur === 1 && v.speed[i] < (o.runLeave ?? 1.8)) cur = 0;
      band[i - a] = cur;
    }
    const parts = [];
    let s0 = 0;
    for (let i = 1; i <= band.length; i++) if (i === band.length || band[i] !== band[s0]) { parts.push([a + s0, a + i - 1, band[s0]]); s0 = i; }
    const minPart = Math.round((o.minPartSec ?? 0.8) * fps);
    for (let k = 0; k < parts.length; k++) {
      if (parts.length > 1 && parts[k][1] - parts[k][0] + 1 < minPart) {
        const j = k > 0 ? k - 1 : k + 1;
        parts[j] = [Math.min(parts[j][0], parts[k][0]), Math.max(parts[j][1], parts[k][1]), parts[j][2]];
        parts.splice(k, 1);
        k = -1;
      }
    }
    for (const [pa, pb, run] of parts) segs.push({ kind: 'move', a: pa, b: pb, band: run ? 'fast' : 'slow' });
  }
  for (const [a, b] of runs(still)) if (b - a + 1 >= minLen) segs.push({ kind: 'still', a, b });
  segs.sort((x, y) => x.a - y.a);
  for (const s of segs) s.speed = mean(Array.from(v.speed.subarray(s.a, s.b + 1)));
  return { segments: segs, velocity: v, n };
}

export function airFrames(skel, take, TW, o = {}) {
  const n = take.n, air = new Uint8Array(n), lowest = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const W = worldAt(TW, i), l = ['L', 'R'].map(s => { const so = soleAt(skel, W, s); return Math.min(so.heel[1], so.ball[1], so.toe[1]); });
    lowest[i] = Math.min(...l);
    air[i] = lowest[i] > (o.airTol ?? 0.045) ? 1 : 0;
  }
  return { air, lowest };
}

export function motionEnergy(take, TW, a, b, fps, o = {}) {
  const k = Math.max(1, Math.round((o.window ?? 0.25) * fps)), rel = (i, j) => sub(getV(worldAt(TW, i).pos, j), getV(worldAt(TW, i).pos, B.Hips));
  let s = 0, c = 0;
  for (let i = a + k; i <= b; i++) {
    for (const j of [B.LeftHand, B.RightHand, B.LeftFoot, B.RightFoot, B.Head]) {
      const p0 = rel(i - k, j), p1 = rel(i, j);
      s += Math.hypot(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]) * fps / k; c++;
    }
  }
  return c ? s / c : 0;
}

export function findJumps(skel, take, TW, fps, o = {}) {
  const { air, lowest } = airFrames(skel, take, TW, o), out = [];
  for (const [a, b] of runs(Array.from(air))) {
    const dur = (b - a + 1) / fps;
    if (dur < (o.minAir ?? 0.16)) continue;
    const peak = Math.max(...Array.from(lowest.subarray(a, b + 1)));
    if (peak < (o.minHeight ?? 0.1)) continue;
    out.push({ a, b, air: dur, height: peak });
  }
  return out;
}

export function classifyLoop(skel, clip) {
  const TW = takeWorld(skel, clip), { air } = airFrames(skel, clip, TW);
  const flight = air.reduce((s, x) => s + x, 0) / clip.n;
  const kind = clip.speed < 0.3 ? 'idle' : flight > 0.06 || clip.speed > 2.4 ? 'run' : 'walk';
  return { kind, flight };
}

export function buildClips(skel, take, fps, plants, o = {}) {
  const TW = takeWorld(skel, take), yaw = facing(take, fps), feat = poseFeatures(take, TW, yaw, fps);
  const { segments } = segmentTake(take, fps, o), clips = [], used = [];
  const add = (c, extra) => { const clip = { ...c, ...extra }; clips.push(clip); return clip; };
  for (const s of segments) {
    const dur = (s.b - s.a + 1) / fps;
    if (s.kind === 'move' && dur >= (o.minMoveSec ?? 1.0)) {
      const lp = findLoop(feat, s.a, s.b, fps, { minSec: 0.6, maxSec: 2.6 });
      if (lp && lp.quality < (o.maxLoopQuality ?? 0.8)) {
        const clip = makeLoopClip(skel, take, fps, lp.start, lp.len, plants, { yaw });
        const cls = classifyLoop(skel, clip);
        if (clip.direction === 'forward' && cls.kind !== 'idle') {
          add(clip, { kind: cls.kind, flight: cls.flight, quality: lp.quality, segment: s, name: cls.kind === 'run' ? 'Run' : 'Walk' });
          used.push(s);
          continue;
        }
      }
    }
    if (s.kind === 'still' && dur >= (o.minStillSec ?? 1.2)) {
      const energy = motionEnergy(take, TW, s.a, s.b, fps);
      if (energy < (o.idleEnergy ?? 0.35)) {
        const lp = findLoop(feat, s.a, s.b, fps, { minSec: Math.min(1.5, dur * 0.6), maxSec: Math.min(4, dur) });
        if (lp) {
          const clip = makeLoopClip(skel, take, fps, lp.start, lp.len, [], { yaw, phase0: 0 });
          add(clip, { kind: 'idle', name: 'Idle', quality: lp.quality, segment: s, energy, speed: 0 });
          used.push(s);
          continue;
        }
      }
    }
  }
  const jumps = findJumps(skel, take, TW, fps, o);
  for (const j of jumps) {
    const a = Math.max(0, j.a - Math.round(0.4 * fps)), b = Math.min(take.n - 1, j.b + Math.round(0.4 * fps));
    add(oneShotClip(skel, take, fps, a, b, { yaw }), { kind: 'jump', name: 'Jump', segment: { a, b }, air: j.air, height: j.height, takeoff: j.a - a, landing: j.b - a });
  }
  for (const s of segments) {
    if (used.includes(s) || jumps.some(j => j.a <= s.b && j.b >= s.a)) continue;
    const dur = (s.b - s.a + 1) / fps, energy = motionEnergy(take, TW, s.a, s.b, fps);
    if (dur < 0.6 || (s.kind === 'still' && energy < (o.idleEnergy ?? 0.35))) continue;
    const a = Math.max(0, s.a - Math.round(0.15 * fps)), b = Math.min(take.n - 1, s.b + Math.round(0.15 * fps));
    add(oneShotClip(skel, take, fps, a, b, { yaw }), { kind: 'action', name: 'Move ' + (clips.filter(c => c.kind === 'action').length + 1), segment: s, energy });
  }
  add(oneShotClip(skel, take, fps, 0, take.n - 1, { yaw }), { kind: 'take', name: 'Full take', segment: { a: 0, b: take.n - 1 } });
  return { clips, segments, jumps, yaw };
}

export function pickLibrary(clips) {
  const best = kind => clips.filter(c => c.kind === kind).sort((x, y) => (x.quality ?? 1) - (y.quality ?? 1) || (y.segment.b - y.segment.a) - (x.segment.b - x.segment.a))[0] ?? null;
  return { idle: best('idle'), walk: best('walk'), run: best('run'), jump: clips.find(c => c.kind === 'jump') ?? null, actions: clips.filter(c => c.kind === 'action'), take: clips.find(c => c.kind === 'take') ?? null };
}

export function standingFrame(skel, take, fps, o = {}) {
  const TW = takeWorld(skel, take), v = rootVelocity(take, fps), n = take.n;
  let best = -1, bc = Infinity;
  for (let i = 0; i < n; i++) {
    const W = worldAt(TW, i), feet = ['L', 'R'].map(s => { const so = soleAt(skel, W, s); return Math.max(so.heel[1], so.ball[1]); });
    const lean = 1 - qrot(getQ(take.rot, i * NB), [0, 1, 0])[1];
    const c = v.speed[i] * 2 + Math.max(...feet) * 10 + lean * 5;
    if (c < bc) { bc = c; best = i; }
  }
  return best < 0 ? null : { frame: best, cost: bc, still: v.speed[best] < 0.2 };
}
