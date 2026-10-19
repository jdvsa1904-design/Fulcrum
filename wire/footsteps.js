import { MAX, gaitTiming, detectFootstepsMax, balanceOffset } from '../solvers/biped/footsteps.js';

const median = a => { const s = [...a].sort((x, y) => x - y); return s.length ? s[s.length >> 1] : NaN; };

function heightMinima(h, a, b, ceiling, win = 3) {
  const out = [];
  for (let f = a; f <= b; f++) {
    if (!(h[f] <= ceiling)) continue;
    let ok = true;
    for (let d = -win; d <= win && ok; d++) { const g = f + d; if (g !== f && g >= 0 && g < h.length && h[g] < h[f]) ok = false; }
    if (ok && !out.some(m => Math.abs(m - f) <= win)) out.push(f);
  }
  return out;
}

export function footstepContacts(o) {
  const { ball, a, b, fps, S, speed, ground = 0, up = 1 } = o;
  const n = ball.l.length / 3, sides = {};
  for (const side of ['l', 'r']) {
    const pos = ball[side], h = Array.from({ length: n }, (_, f) => pos[f * 3 + up]);
    const rule = detectFootstepsMax(pos, { legLength: S, up, floor: ground, verticalTol: o.verticalTol ?? 0.03, tol: o.tol ?? 0.07, footLength: o.footLength ?? 0.3 * S });
    const low = r => { let m = r.start; for (let f = r.start; f <= r.end; f++) if (h[f] < h[m]) m = f; return m; };
    const runs = rule.runs.filter(r => r.mid >= a && r.mid <= b).map(r => ({ m: low(r), from: 'ruleA' }));
    const mins = heightMinima(h, a, b, ground + (o.ceiling ?? 0.3) * S).filter(m => !rule.runs.some(r => m >= r.start && m <= r.end)).map(m => ({ m, from: 'minimum' }));
    sides[side] = [...runs, ...mins].sort((u, v) => u.m - v.m);
  }
  const gaps = [];
  for (const side of ['l', 'r']) for (let i = 1; i < sides[side].length; i++) gaps.push(sides[side][i].m - sides[side][i - 1].m);
  const cycle = Math.max(4, Math.round(o.cycle ?? (gaps.length ? median(gaps) : fps * 0.6)));
  const step = speed * cycle / 2 / fps;
  const timing = gaitTiming('run', step, S, cycle / 2, fps);
  const c = Math.max(2, Math.round(o.contact ?? timing.contact));
  const lo = Math.floor((c - 1) / 2), hi = c - 1 - lo;
  const contacts = [];
  for (const side of ['l', 'r']) for (const k of sides[side]) contacts.push({ side, m: k.m, a: Math.max(a, k.m - lo), b: Math.min(b, k.m + hi), from: k.from });
  contacts.sort((u, v) => u.m - v.m || (u.side < v.side ? -1 : 1));
  return { contacts, cycle, contactFrames: c, timing, stepS: step / S };
}

export function midStanceOffset(vPerFrame, S, fps) {
  return MAX.midOffsetTime * vPerFrame * (30 / fps) + MAX.midOffsetConst * S;
}

export function placeFootsteps(o) {
  const { contacts, com, fwd, right, S, fps, vPerFrame, ground = 0, lift = 0, width = 1 } = o;
  const half = MAX.widthPerUnit * S * width, m = midStanceOffset(vPerFrame, S, fps);
  return contacts.map(k => {
    const c = com(k.m), F = fwd(k.m), R = right(k.m), s = k.side === 'l' ? -1 : 1;
    return { ...k, pos: [c[0] - F[0] * m + R[0] * half * s, ground + lift, c[2] - F[2] * m + R[2] * half * s] };
  });
}

export function balanceShift(lower, upper, bf) {
  return balanceOffset(lower, upper, bf, 1);
}

export function planFootsteps(o) {
  const det = footstepContacts(o);
  const prints = placeFootsteps({ ...o, contacts: det.contacts });
  return { ...det, footprints: prints };
}
