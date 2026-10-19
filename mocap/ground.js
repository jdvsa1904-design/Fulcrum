import { add, sub, scale, dot, cross, len, norm, mid, vlerp, qmul, qconj, qrot, qarc, qaxis, smoothstep, clamp, percentile, median } from './math.js';
import { B, NB, NLM, LM, SIDES, fk, getV, getQ, setQ, lmAt } from './skeleton.js';
import { lowpass } from './clean.js';
import { detectFootPlants } from '../wire/plants.js';
import { twoBoneIK } from '../ik.js';

const UP = [0, 1, 0];
const FOOT_LM = { L: [LM.lHeel, LM.lToe], R: [LM.rHeel, LM.rToe] };

export function torsoUp(P, n) {
  let s = [0, 0, 0], c = 0;
  for (let i = 0; i < n; i++) {
    const L = P.subarray(i * NLM * 3, (i + 1) * NLM * 3);
    const u = norm(sub(mid(lmAt(L, LM.lSh), lmAt(L, LM.rSh)), mid(lmAt(L, LM.lHip), lmAt(L, LM.rHip))));
    if (dot(u, UP) > 0.5) { s = add(s, u); c++; }
  }
  return c > n * 0.5 ? norm(s) : null;
}

export function floorFromPoints(P, n, fps, o = {}) {
  const pts = [];
  const maxSpeed = o.maxFootSpeed ?? 0.8;
  const centroid = (i, side) => { const L = P.subarray(i * NLM * 3, (i + 1) * NLM * 3); return mid(lmAt(L, FOOT_LM[side][0]), lmAt(L, FOOT_LM[side][1])); };
  for (let i = 0; i < n; i++) {
    const L = P.subarray(i * NLM * 3, (i + 1) * NLM * 3);
    let best = null;
    for (const side of ['L', 'R']) {
      const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1), sp = b > a ? len(sub(centroid(b, side), centroid(a, side))) * fps / (b - a) : 0;
      const low = FOOT_LM[side].map(k => lmAt(L, k)).reduce((x, y) => y[1] < x[1] ? y : x);
      if (!best || low[1] < best.p[1]) best = { p: low, sp };
    }
    if (best.sp < maxSpeed) pts.push({ p: best.p, w: 1 / (1 + 3 * best.sp) });
  }
  const prior = o.prior ?? torsoUp(P, n) ?? UP;
  if (pts.length < 3) {
    const lows = [];
    for (let i = 0; i < n; i++) for (const side of ['L', 'R']) for (const k of FOOT_LM[side]) lows.push(dot(lmAt(P.subarray(i * NLM * 3), k), prior));
    return { normal: prior, offset: percentile(lows, 0.05), method: 'prior', points: 0, spread: [0, 0], prior };
  }
  const R0 = qarc(prior, UP), q = pts.map(t => ({ p: qrot(R0, t.p), w0: t.w, w: t.w }));
  const lam0 = (o.priorSpread ?? 0.2) ** 2;
  let a = 0, b = 0, c = median(q.map(t => t.p[1]));
  for (let it = 0; it < 6; it++) {
    let S = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], r = [0, 0, 0], sw = 0;
    for (const t of q) {
      const [x, y, z] = t.p, row = [x, z, 1];
      for (let i = 0; i < 3; i++) { r[i] += t.w * row[i] * y; for (let j = 0; j < 3; j++) S[i][j] += t.w * row[i] * row[j]; }
      sw += t.w;
    }
    S[0][0] += lam0 * sw; S[1][1] += lam0 * sw;
    const sol = solveSym3(S, r);
    if (!sol) break;
    [a, b, c] = sol;
    for (const t of q) {
      const res = t.p[1] - (a * t.p[0] + b * t.p[2] + c);
      t.w = t.w0 * (res > 0 ? Math.exp(-((res / (o.upTol ?? 0.05)) ** 2)) : Math.exp(-((res / (o.downTol ?? 0.15)) ** 2)));
    }
  }
  const n0 = norm([-a, 1, -b]), normal = qrot(qconj(R0), n0), offset = c / len([-a, 1, -b]);
  const xs = q.map(t => t.p[0]), zs = q.map(t => t.p[2]);
  return { normal, offset, method: 'fit', points: pts.length, spread: [percentile(xs, 0.9) - percentile(xs, 0.1), percentile(zs, 0.9) - percentile(zs, 0.1)], prior };
}

function solveSym3(A, b) {
  const det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (Math.abs(det) < 1e-18) return null;
  const inv = [
    [A[1][1] * A[2][2] - A[1][2] * A[2][1], A[0][2] * A[2][1] - A[0][1] * A[2][2], A[0][1] * A[1][2] - A[0][2] * A[1][1]],
    [A[1][2] * A[2][0] - A[1][0] * A[2][2], A[0][0] * A[2][2] - A[0][2] * A[2][0], A[0][2] * A[1][0] - A[0][0] * A[1][2]],
    [A[1][0] * A[2][1] - A[1][1] * A[2][0], A[0][1] * A[2][0] - A[0][0] * A[2][1], A[0][0] * A[1][1] - A[0][1] * A[1][0]]];
  return inv.map(row => (row[0] * b[0] + row[1] * b[1] + row[2] * b[2]) / det);
}

export function floorTransform(floor) {
  return { q: qarc(floor.normal, UP), offset: floor.offset };
}

export function applyFloor(P, n, tf) {
  const out = new Float64Array(P.length);
  for (let i = 0; i < n * NLM; i++) {
    const p = qrot(tf.q, [P[i * 3], P[i * 3 + 1], P[i * 3 + 2]]);
    out[i * 3] = p[0]; out[i * 3 + 1] = p[1] - tf.offset; out[i * 3 + 2] = p[2];
  }
  return out;
}

export function takeWorld(skel, take) {
  const n = take.n, pos = new Float64Array(n * NB * 3), q = new Float64Array(n * NB * 4);
  const tmp = { pos: new Float64Array(NB * 3), q: new Float64Array(NB * 4) };
  for (let i = 0; i < n; i++) {
    fk(skel, getV(take.root, i), take.rot.subarray(i * NB * 4, (i + 1) * NB * 4), tmp);
    pos.set(tmp.pos, i * NB * 3);
    q.set(tmp.q, i * NB * 4);
  }
  return { n, pos, q };
}

export const worldAt = (TW, i) => ({ pos: TW.pos.subarray(i * NB * 3, (i + 1) * NB * 3), q: TW.q.subarray(i * NB * 4, (i + 1) * NB * 4) });

export function soleAt(skel, W, side) {
  const S = SIDES[side], fq = getQ(W.q, S.foot), tq = getQ(W.q, S.toe), fp = getV(W.pos, S.foot), tp = getV(W.pos, S.toe);
  return { heel: add(fp, qrot(fq, skel.sole.heel)), ball: add(tp, qrot(tq, skel.sole.ball)), toe: add(tp, qrot(tq, skel.sole.toe)) };
}

export function footSignals(skel, take, fps, TW = takeWorld(skel, take)) {
  const n = take.n, out = {};
  for (const side of ['L', 'R']) {
    const low = new Float64Array(n), speed = new Float64Array(n), c = new Float64Array(n * 3);
    for (let i = 0; i < n; i++) {
      const s = soleAt(skel, worldAt(TW, i), side);
      low[i] = Math.min(s.heel[1], s.ball[1], s.toe[1]);
      c.set(mid(s.heel, s.ball), i * 3);
    }
    for (let i = 0; i < n; i++) {
      const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
      speed[i] = b > a ? Math.hypot(c[b * 3] - c[a * 3], c[b * 3 + 2] - c[a * 3 + 2]) * fps / (b - a) : 0;
    }
    out[side] = { low, speed };
  }
  return out;
}

export function contactMask(sig, o = {}) {
  const n = sig.low.length, base = percentile(sig.low, 0.1), m = new Uint8Array(n);
  for (let i = 0; i < n; i++) m[i] = sig.low[i] - base < (o.heightTol ?? 0.05) && sig.speed[i] < (o.speedTol ?? 0.6) ? 1 : 0;
  return m;
}

export function groundTake(skel, take, fps, o = {}) {
  const n = take.n, sig = footSignals(skel, take, fps), masks = { L: contactMask(sig.L, o), R: contactMask(sig.R, o) };
  const dy = new Float64Array(n), ok = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    let low = Infinity;
    for (const side of ['L', 'R']) if (masks[side][i]) low = Math.min(low, sig[side].low[i]);
    if (low < Infinity) { dy[i] = -low; ok[i] = 1; }
  }
  const count = ok.reduce((s, v) => s + v, 0);
  if (count < (o.minContactShare ?? 0.3) * n) {
    const w = Math.max(2, Math.round((o.windowSec ?? 0.35) * fps)), low = Float64Array.from({ length: n }, (_, i) => Math.min(sig.L.low[i], sig.R.low[i]));
    for (let i = 0; i < n; i++) dy[i] = -percentile(low.subarray(Math.max(0, i - w), Math.min(n, i + w + 1)), o.lowPct ?? 0.25);
  } else {
    let prev = -1;
    for (let i = 0; i <= n; i++) {
      if (i < n && !ok[i]) continue;
      for (let j = prev + 1; j < i; j++) dy[j] = prev < 0 ? dy[i] : i >= n ? dy[prev] : dy[prev] + (dy[i] - dy[prev]) * (j - prev) / (i - prev);
      prev = i;
    }
  }
  const sm = lowpass(dy, fps, o.fcGround ?? 2);
  for (let i = 0; i < n; i++) take.root[i * 3 + 1] += sm[i];
  return { dy: Array.from(sm, x => Math.round(x * 1000) / 1000), contactFrames: count, method: count < (o.minContactShare ?? 0.3) * n ? 'lower-foot' : 'contacts' };
}

export function findPlants(skel, take, fps, o = {}) {
  const n = take.n, TW = takeWorld(skel, take), W = new Float64Array(n * NB * 16);
  for (let i = 0; i < n * NB; i++) { W[i * 16 + 12] = TW.pos[i * 3]; W[i * 16 + 13] = TW.pos[i * 3 + 1]; W[i * 16 + 14] = TW.pos[i * 3 + 2]; W[i * 16 + 15] = 1; }
  return detectFootPlants({
    W, nb: NB, n, fps, upm: 1, ground: 0,
    legs: { l: { foot: B.LeftFoot, ball: B.LeftToeBase }, r: { foot: B.RightFoot, ball: B.RightToeBase } },
    legLength_cm: skel.dims.legLength * 100, footLength_cm: skel.dims.footLength * 100,
    heightTol_cm: o.heightTol_cm ?? 5, minFrames: o.minFrames ?? Math.max(3, Math.round(0.08 * fps)), maxGap: 2,
  }).map(p => ({ ...p, side: p.side === 'l' ? 'L' : 'R' }));
}

function flattenFoot(q, wRoll, wPitch) {
  let out = q;
  const fwd = qrot(out, [0, 0, 1]), horiz = Math.hypot(fwd[0], fwd[2]);
  if (wRoll > 0 && horiz > 0.3) {
    const side = qrot(out, [1, 0, 0]), want = norm(cross(UP, fwd));
    const ang = Math.atan2(dot(cross(side, want), fwd), dot(side, want));
    out = qmul(qaxis(fwd, ang * wRoll * smoothstep(0.3, 0.7, horiz)), out);
  }
  if (wPitch > 0) {
    const f2 = qrot(out, [0, 0, 1]), flat = norm([f2[0], 0, f2[2]]);
    if (len(flat) > 0.5) out = qmul(qaxis(norm(cross(f2, flat)), Math.acos(clamp(dot(f2, flat), -1, 1)) * wPitch), out);
  }
  return out;
}

export function refinePlants(skel, TW, plants, o = {}) {
  const out = [], r = o.perimeter ?? 0.07, rise = o.rise ?? 0.04, minFrames = o.minFrames ?? 3;
  for (const p of plants) {
    if (p.sliding) { out.push(p); continue; }
    const S = SIDES[p.side], bone = p.point === 'ball' ? S.toe : S.foot, A = getV(worldAt(TW, p.anchor).pos, bone);
    const near = f => { const c = getV(worldAt(TW, f).pos, bone); return Math.hypot(c[0] - A[0], c[2] - A[2]) <= r && c[1] - A[1] <= rise; };
    let a = p.anchor, b = p.anchor;
    while (a - 1 >= p.a && near(a - 1)) a--;
    while (b + 1 <= p.b && near(b + 1)) b++;
    if (b - a + 1 >= minFrames) out.push({ ...p, a, b, refined: a !== p.a || b !== p.b });
  }
  return out;
}

export function softReach(hip, goal, L, soft = 0.993) {
  const d = sub(goal, hip), l = len(d), a = soft * L, b = 0.9998 * L;
  if (l <= a || l < 1e-9) return goal;
  return add(hip, scale(d, (a + (b - a) * (1 - Math.exp(-(l - a) / (b - a)))) / l));
}

export function lockFeet(skel, take, fps, plants, o = {}) {
  const n = take.n, blend = Math.max(1, Math.round((o.blendSec ?? 0.12) * fps));
  const TW = takeWorld(skel, take), plan = { L: new Array(n).fill(null), R: new Array(n).fill(null) };
  if (o.refine !== false) plants = refinePlants(skel, TW, plants, o);
  for (const p of plants) {
    if (p.sliding && !o.lockSliding) continue;
    const S = SIDES[p.side], pivotBone = p.point === 'ball' ? S.toe : S.foot, xs = [], zs = [];
    for (let f = p.a; f <= p.b; f++) { const c = getV(worldAt(TW, f).pos, pivotBone); xs.push(c[0]); zs.push(c[2]); }
    const pivot = [median(xs), 0, median(zs)];
    for (let f = Math.max(0, p.a - blend); f <= Math.min(n - 1, p.b + blend); f++) {
      const w = f < p.a ? smoothstep(p.a - blend - 1, p.a, f) : f > p.b ? 1 - smoothstep(p.b, p.b + blend + 1, f) : 1;
      if (plan[p.side][f] && plan[p.side][f].w >= w) continue;
      plan[p.side][f] = { w, pivotBone, pivot };
    }
  }
  const goals = { L: new Array(n).fill(null), R: new Array(n).fill(null) }, drop = new Float64Array(n);
  const Lleg = skel.m.thigh + skel.m.shin, Lmax = Lleg * (o.reach ?? 0.996), vert = new Float64Array(n), vw = new Float64Array(n);
  for (let f = 0; f < n; f++) {
    const W = worldAt(TW, f);
    for (const side of ['L', 'R']) {
      const pl = plan[side][f];
      if (!pl) continue;
      const S = SIDES[side], ank = getV(W.pos, S.foot), fq0 = getQ(W.q, S.foot), sole = soleAt(skel, W, side);
      const low0 = Math.min(sole.heel[1], sole.ball[1], sole.toe[1]), flatW = smoothstep(0.05, 0.02, Math.max(sole.heel[1], sole.ball[1]) - low0);
      const fq = flattenFoot(fq0, pl.w * (o.flattenRoll ?? 1), pl.w * (o.flattenPitch ?? 0.8) * flatW);
      const moved = v => qrot(fq, qrot(qconj(fq0), sub(v, ank)));
      const pivotOff = moved(getV(W.pos, pl.pivotBone)), lowOff = Math.min(...['heel', 'ball', 'toe'].map(k => moved(sole[k])[1]));
      const lockedAnkle = [pl.pivot[0] - pivotOff[0], -lowOff, pl.pivot[2] - pivotOff[2]];
      goals[side][f] = { ank, lockedAnkle, fq, w: pl.w };
      vert[f] += pl.w * (lockedAnkle[1] - ank[1]);
      vw[f] += pl.w;
    }
  }
  const shift = new Float64Array(n), known = new Uint8Array(n);
  if (o.lift !== false) {
    for (let f = 0; f < n; f++) if (vw[f] > 0.5) { shift[f] = vert[f] / vw[f]; known[f] = 1; }
    let prev = -1;
    for (let f = 0; f <= n; f++) {
      if (f < n && !known[f]) continue;
      for (let j = prev + 1; j < f; j++) shift[j] = prev < 0 ? (f < n ? shift[f] : 0) : f >= n ? shift[prev] : shift[prev] + (shift[f] - shift[prev]) * (j - prev) / (f - prev);
      prev = f;
    }
    const sm0 = lowpass(shift, fps, o.fcLift ?? 3);
    for (let f = 0; f < n; f++) { shift[f] = sm0[f]; take.root[f * 3 + 1] += sm0[f]; }
  }
  for (let f = 0; f < n; f++) {
    const W = worldAt(TW, f);
    for (const side of ['L', 'R']) {
      const g = goals[side][f];
      if (!g) continue;
      const ank = [g.ank[0], g.ank[1] + shift[f], g.ank[2]], goal = vlerp(ank, g.lockedAnkle, g.w);
      goals[side][f] = { goal, fq: g.fq, w: g.w };
      const hip0 = getV(W.pos, SIDES[side].up), hip = [hip0[0], hip0[1] + shift[f], hip0[2]], d = sub(goal, hip), h2 = d[0] * d[0] + d[2] * d[2];
      const need = h2 < Lmax * Lmax ? hip[1] - goal[1] - Math.sqrt(Lmax * Lmax - h2) : hip[1] - goal[1];
      drop[f] = Math.max(drop[f], need);
    }
  }
  const held = new Float64Array(n);
  for (let f = 0; f < n; f++) { let m = 0; for (let k = Math.max(0, f - blend); k <= Math.min(n - 1, f + blend); k++) m = Math.max(m, drop[k]); held[f] = m; }
  const sm = lowpass(held, fps, o.fcDrop ?? 3), cap = o.maxDrop ?? 0;
  let maxDrop = 0;
  for (let f = 0; f < n; f++) { const d = Math.min(cap, Math.max(0, sm[f])); take.root[f * 3 + 1] -= d; maxDrop = Math.max(maxDrop, d); }
  const prevBend = { L: null, R: null };
  let locked = 0, lifted = 0;
  for (let f = 0; f < n; f++) {
    const local = take.rot.subarray(f * NB * 4, (f + 1) * NB * 4);
    for (const side of ['L', 'R']) {
      const S = SIDES[side], W = fk(skel, getV(take.root, f), local);
      let g = goals[side][f];
      if (!g && o.noPenetration !== false) {
        const sole = soleAt(skel, W, side), low = Math.min(sole.heel[1], sole.ball[1], sole.toe[1]);
        if (low < -0.005) { g = { goal: add(getV(W.pos, S.foot), [0, -low, 0]), fq: getQ(W.q, S.foot) }; lifted++; }
      }
      if (!g) { prevBend[side] = null; continue; }
      const hip = getV(W.pos, S.up), knee = getV(W.pos, S.knee), ank = getV(W.pos, S.foot), axis = norm(sub(ank, hip));
      const bendV = sub(knee, mid(hip, ank)), bendP = sub(bendV, scale(axis, dot(bendV, axis)));
      const pole = len(bendP) > 0.01 ? bendP : prevBend[side] ?? qrot(g.fq, [0, 0, 1]);
      const r = twoBoneIK(hip, knee, ank, softReach(hip, g.goal, Lleg, o.soft ?? 0.993), { pole, prevBend: prevBend[side], preferredBend: qrot(g.fq, [0, 0, 1]) });
      prevBend[side] = r.bend;
      const qU = qmul(r.rotRoot, getQ(W.q, S.up)), qK = qmul(r.rotMid, qmul(r.rotRoot, getQ(W.q, S.knee)));
      setQ(local, S.up, qmul(qconj(getQ(W.q, B.Hips)), qU));
      setQ(local, S.knee, qmul(qconj(qU), qK));
      setQ(local, S.foot, qmul(qconj(qK), g.fq));
      if (goals[side][f]) locked++;
    }
  }
  return { lockedFrames: locked, liftedFrames: lifted, maxDrop, maxShift: Math.max(0, ...Array.from(shift, Math.abs)), plants };
}

export function footSkate(skel, take, fps, plants) {
  const TW = takeWorld(skel, take), res = [];
  for (const p of plants) {
    if (p.sliding) continue;
    const S = SIDES[p.side], bone = p.point === 'ball' ? S.toe : S.foot, a = getV(worldAt(TW, p.a).pos, bone);
    let worst = 0;
    for (let f = p.a; f <= p.b; f++) { const c = getV(worldAt(TW, f).pos, bone); worst = Math.max(worst, Math.hypot(c[0] - a[0], c[2] - a[2])); }
    res.push({ side: p.side, a: p.a, b: p.b, drift: worst });
  }
  return res;
}
