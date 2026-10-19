import { add, sub, scale, dot, cross, len, norm, qmul, qrot, qaxis, qyaw, qslerp, frameRotation, clamp, lerp, smoothstep } from './math.js';
import { B, NB, SIDES, getQ, setQ, worldToLocal, standLocal } from './skeleton.js';
import { LEG_REST } from './solve.js';
import { lockFeet } from './ground.js';

const DEG = Math.PI / 180;
export const GAIT = Object.freeze({
  walk: { duty: 0.62, hs: 14, to: 62, ff: -0.13, flat: 0.16, heelOff: 0.6, lift: 0.09, bob: 0.022, sway: 0.024, yaw: 5, roll: 3, tilt: 3, lean: 2, chestYaw: 6, arm: 17, abd: 9, elbow: 16, elbowSwing: 14, width: 0.85, bend: 0.015, toeOut: 6 },
  run: { duty: 0.36, hs: 2, to: 60, ff: -0.13, flat: 0.2, heelOff: 0.42, lift: 0.17, bob: 0.035, sway: 0.012, yaw: 7, roll: 4, tilt: 8, lean: 6, chestYaw: 10, arm: 34, abd: 12, elbow: 84, elbowSwing: 22, width: 0.5, bend: 0.055, toeOut: 4 },
});

export const cycleTime = v => clamp(1.25 - 0.13 * v, 0.62, 1.2);
export const defaultSpeed = (skel, kind) => (kind === 'run' ? 3.3 : 1.3) * clamp(skel.dims.legLength / 0.9, 0.7, 1.3);
const mj = s => s * s * s * (10 - 15 * s + 6 * s * s);
const pitchQ = th => qaxis([1, 0, 0], -th);

function stanceAnkle(skel, P, s, v, T) {
  const d = skel.dims, heel = skel.sole.heel, ball = [0, -d.ankleH, d.ballFwd];
  const zH = P.zH0 - v * T * P.duty * s;
  let th, pivot, off;
  if (s < P.flat) {
    th = P.hs * DEG * (1 - smoothstep(0, P.flat, s));
    const forefoot = P.hs < 0;
    pivot = forefoot ? [0, 0, zH + d.heelBack + d.ballFwd] : [0, 0, zH];
    off = forefoot ? ball : heel;
  } else if (s < P.heelOff) {
    th = 0; pivot = [0, 0, zH]; off = heel;
  } else {
    th = -P.to * DEG * smoothstep(P.heelOff, 1, s) ** 1.2;
    pivot = [0, 0, zH + d.heelBack + d.ballFwd];
    off = ball;
  }
  return { ankle: sub(pivot, qrot(pitchQ(th), off)), th, toe: th < 0 ? -th : 0 };
}

export function gaitClip(skel, o = {}) {
  const kind = o.kind ?? 'walk', P0 = { ...GAIT[kind], ...(o.params ?? {}) }, fps = o.fps ?? 30;
  const v = o.speed ?? defaultSpeed(skel, kind), n = Math.max(8, Math.round((o.cycle ?? cycleTime(v)) * fps)), T = n / fps;
  const m = skel.m, d = skel.dims, L = m.thigh + m.shin;
  const P = { ...P0, zH0: 0 };
  const st0 = stanceAnkle(skel, P, 0.5, v, T);
  P.zH0 = -(st0.ankle[2]) + (o.footForward ?? P.ff);
  const footAt = (side, phi) => {
    const S = SIDES[side], psi = ((phi - (side === 'L' ? 0 : 0.5)) % 1 + 1) % 1, x = S.s * m.hipWidth / 2 * P.width;
    if (psi < P.duty) {
      const st = stanceAnkle(skel, P, psi / P.duty, v, T);
      return { ankle: [x, st.ankle[1], st.ankle[2]], th: st.th, toe: st.toe, stance: true };
    }
    const s = (psi - P.duty) / (1 - P.duty), a = stanceAnkle(skel, P, 0.9999, v, T), b = stanceAnkle(skel, P, 0, v, T);
    const kick = kind === 'run' ? Math.sin(Math.PI * Math.pow(s, 0.75)) ** 2 : Math.sin(Math.PI * Math.pow(s, 0.7));
    const y = lerp(a.ankle[1], b.ankle[1], mj(s)) + P.lift * kick;
    const z = lerp(a.ankle[2], b.ankle[2], mj(s)) - (kind === 'run' ? 0.08 * v / 3.3 * Math.sin(Math.PI * s) * (1 - s) : 0);
    const th = lerp(a.th, b.th, smoothstep(0, 1, s)) + (kind === 'run' ? -0.25 * Math.sin(Math.PI * s) : 0.12 * Math.sin(Math.PI * s));
    return { ankle: [x, y, z], th, toe: Math.max(0, a.toe * (1 - smoothstep(0, 0.35, s))), stance: false };
  };
  const bobAt = phi => (kind === 'run' ? -1 : 1) * P.bob * Math.cos(4 * Math.PI * (phi - P.duty / 2));
  const pelvisAt = phi => {
    const c1 = Math.cos(2 * Math.PI * phi), cm = Math.cos(2 * Math.PI * (phi - P.duty / 2));
    return { q: qmul(qyaw(-P.yaw * DEG * c1), qmul(qaxis([0, 0, 1], P.roll * DEG * cm), qaxis([1, 0, 0], P.tilt * DEG))), x: P.sway * cm, z: 0.01 * v * Math.sin(4 * Math.PI * phi) };
  };
  const reachAt = phi => {
    let top = Infinity;
    const pv = pelvisAt(phi);
    for (const side of ['L', 'R']) {
      const f = footAt(side, phi);
      if (!f.stance) continue;
      const off = qrot(pv.q, [SIDES[side].s * m.hipWidth / 2, -d.hipsUp, 0]);
      const dx = f.ankle[0] - pv.x - off[0], dz = f.ankle[2] - pv.z - off[2], r = (o.reach ?? 0.99) * L, h2 = dx * dx + dz * dz;
      top = Math.min(top, (h2 < r * r ? f.ankle[1] + Math.sqrt(r * r - h2) : f.ankle[1]) - off[1] - d.hipsUp);
    }
    return top;
  };
  const N = 96, cap = Array.from({ length: N }, (_, k) => d.hipY - P.bend + bobAt(k / N)), lim = Array.from({ length: N }, (_, k) => reachAt(k / N));
  let prof = cap.map((c, k) => Math.min(c, lim[k]));
  for (let it = 0; it < 6; it++) {
    prof = prof.map((_, k) => 0.25 * prof[(k + N - 1) % N] + 0.5 * prof[k] + 0.25 * prof[(k + 1) % N]);
    prof = prof.map((x, k) => Math.min(x, lim[k]));
  }
  const heightAt = phi => { const x = (phi % 1) * N, i = Math.floor(x), u = x - i; return prof[i % N] * (1 - u) + prof[(i + 1) % N] * u; };
  const root = new Float64Array(n * 3), rot = new Float64Array(n * NB * 4), contacts = [];
  for (let k = 0; k < n; k++) {
    const phi = k / n, c1 = Math.cos(2 * Math.PI * phi), pv = pelvisAt(phi);
    const world = new Float64Array(NB * 4), qHips = pv.q;
    const qChest = qmul(qyaw(P.chestYaw * DEG * c1), qaxis([1, 0, 0], (P.tilt + P.lean) * DEG));
    const qHead = qmul(qyaw(0.3 * P.chestYaw * DEG * c1), qaxis([1, 0, 0], (o.lookDown ?? 3) * DEG));
    setQ(world, B.Hips, qHips);
    setQ(world, B.Spine, qslerp(qHips, qChest, 1 / 3));
    setQ(world, B.Spine1, qslerp(qHips, qChest, 2 / 3));
    setQ(world, B.Spine2, qChest);
    setQ(world, B.Neck, qslerp(qChest, qHead, 0.5));
    setQ(world, B.Head, qHead);
    const rootPos = [pv.x, heightAt(phi) + d.hipsUp, pv.z];
    for (const side of ['L', 'R']) {
      const S = SIDES[side], s = S.s, arm = (side === 'L' ? -1 : 1) * P.arm * DEG * c1;
      setQ(world, S.clav, qChest);
      const qArm = qmul(qChest, qmul(qaxis([1, 0, 0], -arm), qaxis([0, 0, 1], -s * (90 - P.abd) * DEG)));
      const bend = (P.elbow + P.elbowSwing * Math.max(0, arm / (P.arm * DEG))) * DEG;
      const qFore = qmul(qArm, qaxis([0, -s, 0], bend));
      setQ(world, S.arm, qArm);
      setQ(world, S.fore, qFore);
      setQ(world, S.hand, qmul(qFore, qaxis([0, 0, s], -(kind === 'run' ? 10 : 4) * DEG)));
      const f = footAt(side, phi), hipJ = add(rootPos, qrot(qHips, [s * m.hipWidth / 2, -d.hipsUp, 0]));
      const toeOut = qyaw(-s * P.toeOut * DEG), qFoot = qmul(toeOut, pitchQ(f.th));
      const reachMax = (o.swingReach ?? 0.975) * L, d0v = sub(f.ankle, hipJ), d0l = len(d0v);
      const ank = !f.stance && d0l > reachMax ? add(hipJ, scale(d0v, reachMax / d0l)) : f.ankle, dv = sub(ank, hipJ), dl = clamp(len(dv), 1e-6, 0.9999 * L), dir = norm(dv);
      const a = (m.thigh * m.thigh - m.shin * m.shin + dl * dl) / (2 * dl), h = Math.sqrt(Math.max(0, m.thigh * m.thigh - a * a));
      let bendDir = norm(sub(qrot(toeOut, [0, 0, 1]), scale(dir, dot(qrot(toeOut, [0, 0, 1]), dir))));
      const knee = add(add(hipJ, scale(dir, a)), scale(bendDir, h)), ankle = add(hipJ, scale(dir, dl));
      const upper = sub(knee, hipJ), lower = sub(ankle, knee);
      let hinge = cross(upper, lower);
      hinge = len(hinge) > 1e-6 ? norm(hinge) : norm(cross(bendDir, dir));
      const qU = frameRotation(upper, hinge, LEG_REST.d0, LEG_REST.h0), qK = frameRotation(lower, hinge, LEG_REST.d0, LEG_REST.h0);
      setQ(world, S.up, qU);
      setQ(world, S.knee, qK);
      setQ(world, S.foot, qFoot);
      setQ(world, S.toe, qmul(qFoot, pitchQ(f.toe)));
    }
    const local = worldToLocal(world);
    rot.set(local, k * NB * 4);
    root.set(rootPos, k * 3);
  }
  for (const side of ['L', 'R']) {
    const a = side === 'L' ? 0 : Math.round(n / 2), len_ = Math.max(1, Math.round(P.duty * n));
    contacts.push({ side, a, b: a + len_ - 1 });
  }
  return { name: kind === 'run' ? 'Run' : 'Walk', kind, loop: true, procedural: true, fps, n, root, rot, speed: v, cycle: T, contacts, phase0: 0 };
}

export function holdClip(skel, local, rootPos, o = {}) {
  const fps = o.fps ?? 30, n = Math.max(2, Math.round((o.seconds ?? 4) * fps)), root = new Float64Array(n * 3), rot = new Float64Array(n * NB * 4);
  for (let k = 0; k < n; k++) { root.set(rootPos, k * 3); rot.set(local, k * NB * 4); }
  return { fps, n, root, rot };
}

export function idleClip(skel, o = {}) {
  const fps = o.fps ?? 30, seconds = o.seconds ?? 4, base = o.local ?? standLocal();
  const take = holdClip(skel, base, o.root ?? [0, skel.rest[0][1] - 0.005, 0], { fps, seconds });
  const n = take.n;
  for (let k = 0; k < n; k++) {
    const t = k / n, br = Math.sin(2 * Math.PI * t), sw = Math.sin(2 * Math.PI * t * (o.swayCycles ?? 1));
    take.root[k * 3] += (o.sway ?? 0.012) * sw;
    take.root[k * 3 + 1] += 0.003 * Math.sin(4 * Math.PI * t);
    const local = take.rot.subarray(k * NB * 4, (k + 1) * NB * 4);
    const bump = (bone, axis, ang) => setQ(local, bone, qmul(getQ(local, bone), qaxis(axis, ang)));
    bump(B.Hips, [0, 0, 1], -0.012 * sw);
    bump(B.Spine1, [1, 0, 0], -0.012 * br);
    bump(B.Spine2, [1, 0, 0], -0.018 * br);
    bump(B.Head, [0, 1, 0], 0.03 * Math.sin(2 * Math.PI * t + 1));
    bump(B.LeftArm, [0, 0, 1], 0.015 * br);
    bump(B.RightArm, [0, 0, 1], -0.015 * br);
  }
  const plants = ['L', 'R'].map(side => ({ side, a: 0, b: n - 1, anchor: 0, point: 'foot', sliding: false }));
  lockFeet(skel, take, fps, plants, { flattenPitch: 0, flattenRoll: 0, blendSec: 0.01, refine: false });
  return { name: 'Idle', kind: 'idle', loop: true, procedural: true, fps, n, root: take.root, rot: take.rot, speed: 0, cycle: n / fps, contacts: [{ side: 'L', a: 0, b: n - 1 }, { side: 'R', a: 0, b: n - 1 }], phase0: 0 };
}

export function jumpClip(skel, o = {}) {
  const fps = o.fps ?? 30, base = o.local ?? standLocal(), y0 = o.root?.[1] ?? skel.rest[0][1] - 0.005;
  const crouch = 0.24, push = 0.15, g = 9.81, v0 = o.v0 ?? 2.9, air = 2 * v0 / g, land = 0.22, rec = 0.3;
  const total = crouch + push + air + land + rec, n = Math.round(total * fps) + 1;
  const take = { n, root: new Float64Array(n * 3), rot: new Float64Array(n * NB * 4) };
  const depth = o.depth ?? 0.12, t1 = crouch, t2 = crouch + push, t3 = t2 + air, t4 = t3 + land;
  for (let k = 0; k < n; k++) {
    const t = k / fps;
    let dy = 0, arms = 0, tuck = 0;
    if (t < t1) { const s = smoothstep(0, t1, t); dy = -depth * s; arms = -0.6 * s; }
    else if (t < t2) { const s = smoothstep(t1, t2, t); dy = -depth * (1 - s) + 0.06 * s; arms = -0.6 + 1.9 * s; }
    else if (t < t3) { const ta = t - t2; dy = 0.06 + v0 * ta - 0.5 * g * ta * ta; arms = 1.3 - 0.8 * smoothstep(t2, t3, t); tuck = smoothstep(0, 0.4, ta / air) * (1 - smoothstep(0.6, 1, ta / air)); }
    else if (t < t4) { const s = (t - t3) / land; dy = 0.06 * (1 - s) - depth * 0.9 * Math.sin(Math.PI * Math.min(1, s * 0.9)); arms = 0.5 - 0.4 * s; }
    else { const s = smoothstep(t4, total, t); dy = -depth * 0.9 * Math.sin(Math.PI * 0.9) * (1 - s); arms = 0.1 * (1 - s); }
    take.root.set([0, y0 + dy, 0], k * 3);
    take.rot.set(base, k * NB * 4);
    const local = take.rot.subarray(k * NB * 4, (k + 1) * NB * 4);
    const bump = (bone, axis, ang) => setQ(local, bone, qmul(getQ(local, bone), qaxis(axis, ang)));
    const lean = Math.max(0, -dy) * 2.2;
    bump(B.Hips, [1, 0, 0], lean * 0.6);
    bump(B.Spine1, [1, 0, 0], lean * 0.4);
    bump(B.Head, [1, 0, 0], -lean * 0.5);
    bump(B.LeftArm, [1, 0, 0], -arms);
    bump(B.RightArm, [1, 0, 0], -arms);
    if (tuck > 0) {
      for (const side of ['L', 'R']) {
        const S = SIDES[side];
        bump(S.up, [1, 0, 0], -0.5 * tuck);
        bump(S.knee, [1, 0, 0], 0.9 * tuck);
        bump(S.foot, [1, 0, 0], -0.3 * tuck);
      }
    }
  }
  const a = Math.round(t2 * fps), b = Math.round(t3 * fps), plants = [];
  for (const side of ['L', 'R']) {
    plants.push({ side, a: 0, b: Math.max(0, a - 1), anchor: 0, point: 'foot', sliding: false });
    plants.push({ side, a: Math.min(n - 1, b + 1), b: n - 1, anchor: n - 1, point: 'foot', sliding: false });
  }
  lockFeet(skel, take, fps, plants, { flattenPitch: 0, flattenRoll: 0, blendSec: 0.1, refine: false });
  return { name: 'Jump', kind: 'jump', loop: false, procedural: true, fps, n, root: take.root, rot: take.rot, speed: 0, takeoff: a, landing: b, contacts: plants.map(p => ({ side: p.side, a: p.a, b: p.b })) };
}
