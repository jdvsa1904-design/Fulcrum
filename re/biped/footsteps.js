export const MAX = {
  ticksPerSecond: 4800,
  legPerHeight: 0.481928,
  widthPerUnit: 0.1,
  standReach: 1.10375,
  heelReach: 1.1185,
  toeReach: 1.155,
  midOffsetTime: 0.9595,
  midOffsetConst: 0.0107,
  gravityPerLeg: 11.7001,
  walkDsGap: 0.93,
  walkDsMin: 0.05,
  walkDsMaxFrames: 5,
  walkDsCycleDiv: 5,
  runContactTravel: 0.6,
  runContactMaxFrac: 0.7,
  zeroSpeed: 1e-4,
  tieEps: 1e-3,
  stanceTangent: 1.2,
  balanceGain: 0.794,
  smoothReach: 0.48,
  smoothCapPerLeg: 0.25,
  lateralPerUnit: 0.1,
  defaults: {
    walk: { stride: 0.75, cycle: 15 },
    run: { stride: 1.5, cycle: 15 },
    jump: { stride: 2.4, cycle: 20 }
  },
  adapt: {
    walk: {
      along: [[-2, -0.0021], [-1, 0.0418], [0, 0.9881], [1, -0.0287]],
      lateral: [[-2, 0.0069], [-1, 0.2317], [0, 0.575], [1, 0.1914], [2, -0.0048]]
    },
    run: {
      along: [[-2, 'lift', 0.0067], [-1, 'touch', 0.0331], [-1, 'lift', 0.2311], [0, 'touch', 0.7989], [0, 'lift', 0.7848], [1, 'touch', 0.1747], [1, 'lift', -0.0228], [2, 'touch', -0.0051]],
      lateral: [[-2, 'lift', 0.0258], [-1, 'touch', 0.1096], [-1, 'lift', 0.2649], [0, 'touch', 0.6541], [0, 'lift', 0.6435], [1, 'touch', 0.2211], [1, 'lift', 0.0654], [2, 'touch', 0.0146]],
      rawLateral: [[-2, 'touch', -0.0002], [-2, 'lift', 0.0261], [-1, 'touch', 0.1092], [-1, 'lift', 0.2623], [0, 'touch', 0.6644], [0, 'lift', 0.6544], [1, 'touch', 0.2180], [1, 'lift', 0.0643], [2, 'touch', 0.0153], [2, 'lift', -0.0021]],
      rawCouple: [[-2, 'touch', 0.017], [-2, 'lift', -0.009], [-1, 'touch', 0.042], [-1, 'lift', -0.051], [0, 'touch', 0.117], [0, 'lift', -0.123], [1, 'touch', 0.044], [1, 'lift', -0.056], [2, 'touch', 0.003], [2, 'lift', -0.007]]
    }
  }
};

const trunc = x => (x < 0 ? Math.ceil(x) : Math.floor(x));
const f32 = Math.fround;

function roundToFrame(x, tpf) {
  const rem = x % tpf;
  let y = x - rem;
  if (rem >= trunc(tpf / 2)) y += tpf;
  return y;
}

export function gaitTiming(gait, stride, S, cycleFrames, fps = 30) {
  const tpf = MAX.ticksPerSecond / fps;
  const cycle = Math.round(cycleFrames * tpf);
  const S32 = f32(S), st = f32(Math.abs(stride));
  const speed = f32(st / cycle);
  let a, b;
  if (gait === 'walk') {
    let gap = f32(S32 * MAX.walkDsGap - st);
    const lo = S32 * MAX.walkDsMin;
    if (lo > gap) gap = f32(lo);
    a = speed < MAX.zeroSpeed ? MAX.walkDsMaxFrames * tpf : trunc(f32(f32(gap / speed) - tpf) + MAX.tieEps);
    if (a > MAX.walkDsMaxFrames * tpf) a = MAX.walkDsMaxFrames * tpf;
  } else {
    const cap = trunc(cycle * MAX.runContactMaxFrac);
    if (speed < MAX.zeroSpeed) a = cap;
    else {
      a = trunc(f32(f32(S32 * MAX.runContactTravel) / speed) + MAX.tieEps);
      if (a >= cap) a = cap;
    }
  }
  a = roundToFrame(a, tpf);
  if (gait === 'walk') {
    const k = trunc(cycle / (tpf * MAX.walkDsCycleDiv));
    if (a > tpf * k) a = tpf * k;
    if (a < 2 * tpf) a = 2 * tpf;
    b = cycle + a;
    if (2 * a >= b) b = 2 * a + tpf;
    return { gait, cycle: cycle / tpf, doubleSupport: a / tpf, duration: b / tpf, swing: (cycle - a) / tpf };
  }
  if (a < 2 * tpf) a = 2 * tpf;
  b = cycle - a;
  return { gait, cycle: cycle / tpf, contact: a / tpf, duration: a / tpf, airborne: b / tpf };
}

export function legScale(height) {
  return height * MAX.legPerHeight;
}

const lerp = (a, b, t) => a + (b - a) * t;

export function generateFootsteps(o = {}) {
  const gait = o.gait ?? 'walk';
  const def = MAX.defaults[gait];
  const S = o.S ?? legScale(o.height ?? 170);
  const fps = o.fps ?? 30;
  const n = o.count ?? 8;
  const L1 = o.stride ?? o.stride1 ?? def.stride, L2 = o.stride2 ?? L1;
  const c1 = o.cycle ?? o.cycle1 ?? def.cycle, c2 = o.cycle2 ?? c1;
  const h1 = o.strideHeight ?? o.strideHeight1 ?? 0, h2 = o.strideHeight2 ?? h1;
  const interp = o.interpolate ?? (L2 !== L1 || c2 !== c1 || h2 !== h1);
  const half = MAX.widthPerUnit * S * (o.width ?? 1);
  const moving = gait === 'jump' ? Math.max(0, Math.floor(n / 2) - 1) : Math.max(0, n - 2);
  const at = i => (interp && moving > 1 ? i / (moving - 1) : 0);
  const steps = [];
  for (let i = 0; i < moving; i++) {
    const u = at(i);
    steps.push({ L: lerp(L1, L2, u) * S, c: lerp(c1, c2, u), h: lerp(h1, h2, u) });
  }
  const T0 = gaitTiming(gait, (steps[0]?.L ?? L1 * S), S, steps[0]?.c ?? c1, fps);
  const fs = [];
  const push = (side, start, dur, x, z, timing) => fs.push({ i: fs.length, side, start, dur, end: start + dur, pos: [x, side === 'left' ? half : -half, z], timing });
  if (gait === 'jump') {
    push('left', 0, T0.contact, 0, 0, T0);
    push('right', 0, T0.contact, 0, 0, T0);
    let x = 0, z = 0, t = 0;
    for (const s of steps) {
      const T = gaitTiming(gait, s.L, S, s.c, fps);
      x += s.L; z += s.h; t += T.cycle;
      push('left', t, T.contact, x, z, T);
      push('right', t, T.contact, x, z, T);
    }
  } else if (gait === 'run') {
    push('left', 0, T0.contact, 0, 0, T0);
    push('right', 0, T0.contact, 0, 0, T0);
    let x = 0, z = 0, t = 0;
    steps.forEach((s, i) => {
      const T = gaitTiming(gait, s.L, S, s.c, fps);
      x += s.L; z += s.h; t += T.cycle;
      push(i % 2 === 0 ? 'left' : 'right', t, T.contact, x, z, T);
    });
  } else {
    push('left', 0, T0.duration, 0, 0, T0);
    push('right', 0, 2 * T0.cycle + T0.doubleSupport + 1, 0, 0, T0);
    let x = 0, z = 0, t = T0.cycle;
    steps.forEach((s, i) => {
      const T = gaitTiming(gait, s.L, S, s.c, fps);
      x += s.L; z += s.h; t += T.cycle;
      push(i % 2 === 0 ? 'left' : 'right', t, T.duration, x, z, T);
    });
  }
  const end = fs.reduce((m, f) => Math.max(m, f.end), 0);
  return { gait, S, fps, half, footsteps: fs, steps, frames: end + 1 };
}

export function toWorld(p, o = {}) {
  const org = o.origin ?? [0, 0, 0];
  const f = o.forward ?? [0, 0, 1];
  const fl = Math.hypot(f[0], f[2]) || 1, fx = f[0] / fl, fz = f[2] / fl;
  const s = o.scale ?? 1;
  return [org[0] + s * (fx * p[0] + fz * p[1]), org[1] + s * p[2], org[2] + s * (fz * p[0] - fx * p[1])];
}

function midOffset(v, S, fps) {
  return MAX.midOffsetTime * v * (30 / fps) + MAX.midOffsetConst * S;
}

const reachZ = (R, dx) => Math.sqrt(Math.max(R * R - dx * dx, 0));

export function gravity(S, o = {}) {
  return o.gravity ?? MAX.gravityPerLeg * S;
}

export function comKeys(plan, o = {}) {
  const { gait, S, fps, footsteps: fs } = plan;
  const R0 = MAX.standReach * S;
  const hk = [{ t: 0, x: 0, y: 0 }], vk = [{ t: 0, z: R0 }];
  const ground = f => f.pos[2];
  if (gait === 'walk') {
    const moving = fs.slice(2);
    moving.forEach((f, i) => {
      const prev = i === 0 ? fs[1] : moving[i - 1];
      const L = f.pos[0] - prev.pos[0] || plan.steps[i]?.L || 0;
      const v = L / f.timing.cycle;
      const m = midOffset(v, S, fps);
      const tm = f.start + f.timing.doubleSupport + f.timing.swing / 2;
      const xm = f.pos[0] + m;
      const xt = xm - v * (tm - f.start);
      const xl = xm + v * (prev.end - tm);
      hk.push({ t: tm, x: xm, y: 0, foot: f.i, kind: 'mid' });
      vk.push({ t: f.start, z: ground(f) + reachZ(MAX.heelReach * S, f.pos[0] - xt), foot: f.i, kind: 'touch' });
      if (prev.end > f.start) vk.push({ t: prev.end, z: ground(prev) + reachZ(MAX.toeReach * S, xl - prev.pos[0]), foot: prev.i, kind: 'lift' });
      vk.push({ t: tm, z: ground(f) + reachZ(R0, m), foot: f.i, kind: 'mid' });
    });
  } else {
    const groups = [];
    for (const f of fs) {
      const g = groups.find(x => x.start === f.start);
      if (g) g.feet.push(f); else groups.push({ start: f.start, end: f.end, feet: [f] });
    }
    groups.forEach((g, i) => {
      if (i === 0) {
        hk.push({ t: g.end, x: 0.5 * S, y: 0, kind: 'start' });
        return;
      }
      const prev = groups[i - 1];
      const x = g.feet.reduce((s, f) => s + f.pos[0], 0) / g.feet.length;
      const px = prev.feet.reduce((s, f) => s + f.pos[0], 0) / prev.feet.length;
      const z = g.feet.reduce((s, f) => s + f.pos[2], 0) / g.feet.length;
      const v = (x - px) / (g.start - prev.start);
      const m = midOffset(v, S, fps);
      const tm = (g.start + g.end) / 2;
      const xt = x + m - v * (tm - g.start), xl = x + m + v * (g.end - tm);
      hk.push({ t: g.start, x: xt, y: 0, foot: g.feet[0].i, kind: 'touch' });
      hk.push({ t: g.end, x: xl, y: 0, foot: g.feet[0].i, kind: 'lift' });
      vk.push({ t: g.start, z: z + reachZ(R0, x - xt), foot: g.feet[0].i, kind: 'touch' });
      vk.push({ t: g.end, z: z + reachZ(R0, xl - x), foot: g.feet[0].i, kind: 'lift' });
    });
  }
  hk.sort((a, b) => a.t - b.t);
  vk.sort((a, b) => a.t - b.t);
  const last = fs.reduce((a, f) => (f.end > a.end ? f : a), fs[0]);
  const fx = fs.filter(f => f.end === last.end || f.start === last.start).reduce((s, f) => s + f.pos[0], 0) / fs.filter(f => f.end === last.end || f.start === last.start).length;
  hk.push({ t: last.end + 1, x: fx, y: 0, kind: 'stand' });
  vk.push({ t: last.end + 1, z: last.pos[2] + R0, kind: 'stand' });
  return { horizontal: hk, vertical: vk };
}

function hermite(p0, p1, m0, m1, h, s) {
  const s2 = s * s, s3 = s2 * s;
  return (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * h * m0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * h * m1;
}

export function spline(keys, field, t) {
  const n = keys.length;
  if (t <= keys[0].t) return keys[0][field];
  if (t >= keys[n - 1].t) return keys[n - 1][field];
  let i = 0;
  while (i < n - 2 && keys[i + 1].t <= t) i++;
  const a = keys[i], b = keys[i + 1];
  const tan = k => {
    const p = keys[k - 1] ?? keys[k], q = keys[k + 1] ?? keys[k];
    return q.t > p.t ? (q[field] - p[field]) / (q.t - p.t) : 0;
  };
  const h = b.t - a.t;
  return hermite(a[field], b[field], tan(i), tan(i + 1), h, (t - a.t) / h);
}

export function flightVelocities(T, dz, g) {
  return { up: g * T / 2 + dz / T, down: g * T / 2 - dz / T };
}

export function sampleCOM(plan, keys, o = {}) {
  const { S, fps } = plan;
  const g = gravity(S, o) / (fps * fps);
  const tau = o.ballisticTension ?? 0.5;
  const out = [];
  const vk = keys.vertical;
  const air = [];
  if (plan.gait !== 'walk') {
    for (let i = 1; i < vk.length - 1; i++) {
      const a = vk[i], b = vk[i + 1];
      if (a.kind === 'lift' && b.kind === 'touch') air.push({ t0: a.t, t1: b.t, z0: a.z, z1: b.z });
    }
  }
  const zAt = t => {
    for (const f of air) {
      if (t >= f.t0 && t <= f.t1) {
        const T = f.t1 - f.t0, u = t - f.t0, v = flightVelocities(T, f.z1 - f.z0, g);
        return f.z0 + v.up * u - 0.5 * g * u * u;
      }
    }
    for (let i = 0; i < air.length - 1; i++) {
      const a = air[i], b = air[i + 1];
      if (t > a.t1 && t < b.t0) {
        const Ta = a.t1 - a.t0, Tb = b.t1 - b.t0;
        const va = flightVelocities(Ta, a.z1 - a.z0, g), vb = flightVelocities(Tb, b.z1 - b.z0, g);
        const k = MAX.stanceTangent * (1 - tau);
        return hermite(a.z1, b.z0, -va.down * k, vb.up * k, b.t0 - a.t1, (t - a.t1) / (b.t0 - a.t1));
      }
    }
    return spline(vk, 'z', t);
  };
  for (let t = 0; t < (o.frames ?? plan.frames); t++) out.push([spline(keys.horizontal, 'x', t), spline(keys.horizontal, 'y', t), zAt(t)]);
  return out;
}

export function balanceOffset(lower, upper, bf = 1, up = 1, k = MAX.balanceGain) {
  const d = [upper[0] - lower[0], upper[1] - lower[1], upper[2] - lower[2]];
  d[up] = 0;
  return d.map(x => -bf * k * x);
}

export function smoothKeys(hk, cap) {
  const n = hk.length, out = hk.map(k => ({ ...k }));
  for (let i = 1; i <= n - 3; i++) {
    const a = hk[i - 1], k = hk[i], b = hk[i + 1];
    let dx = (a.x + b.x) / 2 - k.x, dy = (a.y + b.y) / 2 - k.y;
    const L = Math.hypot(dx, dy);
    if (L > 0) { dx /= L; dy /= L; }
    const ax = k.x - a.x, ay = k.y - a.y, bx = b.x - k.x, by = b.y - k.y;
    const la = Math.hypot(ax, ay), lb = Math.hypot(bx, by);
    const c = la > 0 && lb > 0 ? (ax * bx + ay * by) / (la * lb) : 1;
    const bend = Math.min(Math.acos(Math.max(-1, Math.min(1, c))), Math.PI / 2) / (Math.PI / 2);
    const step = Math.min(MAX.smoothReach * la, MAX.smoothReach * lb, cap) * bend;
    out[i].x = k.x + dx * step;
    out[i].y = k.y + dy * step;
  }
  return out;
}

export function adaptCOM(plan, keys, editIndex, delta) {
  const edits = Array.isArray(editIndex) ? editIndex : [[editIndex, delta]];
  const walk = plan.gait === 'walk';
  const table = MAX.adapt[walk ? 'walk' : 'run'];
  const fs = plan.footsteps;
  const order = walk ? fs.slice(2).map(f => f.i) : [...new Set(fs.slice(2).map(f => f.start))].map(s => fs.find(f => f.start === s).i);
  const hk = keys.horizontal.map(k => ({ ...k }));
  const rows = r => (r.length === 3 ? r : [r[0], 'mid', r[1]]);
  const add = (pos, kernel, axis, amount) => {
    for (const r of kernel) {
      const [off, kind, w] = rows(r);
      const fi = order[pos + off];
      if (fi == null) continue;
      const key = hk.find(k => k.foot === fi && k.kind === kind);
      if (key) key[axis] += w * amount;
    }
  };
  const nominal = MAX.lateralPerUnit * plan.S;
  const dev = new Map();
  for (const f of fs) dev.set(f.i, [0, (f.side === 'left' ? 1 : -1) * (plan.half - nominal)]);
  for (const [i, d] of edits) {
    const f = fs[i];
    const key = plan.gait === 'jump' ? fs.find(g => g.start === f.start).i : i;
    const cur = dev.get(key) ?? [0, 0];
    dev.set(key, [cur[0] + d[0], cur[1] + d[1]]);
  }
  for (const [fi, d] of dev) {
    const pos = order.indexOf(fi);
    if (pos < 0 || (!d[0] && !d[1])) continue;
    add(pos, table.along, 'x', d[0]);
    if (walk) add(pos, table.along, 'y', d[1]);
    else {
      add(pos, table.rawLateral, 'y', d[1]);
      add(pos, table.rawCouple, 'x', Math.abs(d[1]));
    }
  }
  return { ...keys, horizontal: smoothKeys(hk, MAX.smoothCapPerLeg * plan.S) };
}

export function sprintPlan(o = {}) {
  const fps = o.fps ?? 30;
  const S = o.S ?? legScale(o.height ?? 1.8);
  const stride = o.stride ?? MAX.defaults.run.stride;
  const speed = o.speed ?? 3.5;
  const cycle = Math.max(2, Math.round(stride * S / (speed / fps)));
  return generateFootsteps({ ...o, gait: 'run', S, fps, stride, cycle });
}

export function detectFootstepsMax(pos, o = {}) {
  const n = pos.length / 3, S = o.legLength ?? 1, tol = (o.tol ?? 0.07) * S;
  const up = o.up ?? 1;
  const p = new Uint8Array(n);
  for (let i = 1; i < n; i++) {
    const d = Math.hypot(pos[i * 3] - pos[i * 3 - 3], pos[i * 3 + 1] - pos[i * 3 - 2], pos[i * 3 + 2] - pos[i * 3 - 1]);
    if (tol >= d) { p[i - 1] = 1; p[i] = 1; } else p[i] = 0;
  }
  if (o.verticalTol != null) {
    const z = o.floor ?? 0, vt = o.verticalTol * S;
    for (let i = 1; i < n; i++) {
      const h = pos[i * 3 + up];
      if (h > z + vt || z - vt > h) p[i] = 0;
    }
  }
  for (let i = 1; i < n - 1; i++) if (!p[i] && p[i - 1] && p[i + 1]) p[i] = 1;
  for (let i = 0; i + 3 < n; i++) if (p[i + 3] && p[i]) { p[i + 1] = 1; p[i + 2] = 1; }
  for (let i = 0; i + 4 < n; i++) if (!p[i + 4] && !p[i]) { p[i + 1] = 0; p[i + 2] = 0; p[i + 3] = 0; }
  if (!p[0] && n > 1 && p[1]) p[0] = 1;
  const runs = [];
  for (let i = 0; i < n;) {
    if (!p[i]) { i++; continue; }
    let j = i;
    while (j + 1 < n && p[j + 1]) j++;
    let height = -1e6, sliding = false;
    const foot = o.footLength ?? 1, dist = foot * (o.slidingDist ?? 25) / 50, ang = (o.slidingAngle ?? 0) * Math.PI / 180;
    const wrap = x => Math.atan2(Math.sin(x), Math.cos(x));
    const h = up === 1 ? [0, 2] : [0, 1];
    for (let k = i; k <= j; k++) {
      height = Math.max(height, pos[k * 3 + up]);
      if (k === i) continue;
      if (Math.hypot(pos[k * 3 + h[0]] - pos[i * 3 + h[0]], pos[k * 3 + h[1]] - pos[i * 3 + h[1]]) > dist) sliding = true;
      if (o.heading && Math.abs(wrap(o.heading[k] - o.heading[i])) > ang) sliding = true;
    }
    runs.push({ start: i, end: j, duration: j - i + 1, mid: (i + j) >> 1, height, sliding });
    i = j + 1;
  }
  return { planted: p, runs };
}
