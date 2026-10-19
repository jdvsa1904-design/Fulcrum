const PAD = { ground: 3, despike: 2, smooth: 4, floor: 3 };
const LOCO = new Set(['locomotion', 'walk', 'run', 'sprint', 'travel', 'strafe', 'moving']);

export function sideOf(part) {
  const p = String(part ?? '').toLowerCase();
  if (/\bleft\b|^l$|_l\b/.test(p)) return 'l';
  if (/\bright\b|^r$|_r\b/.test(p)) return 'r';
  return null;
}

const clampRange = (a, b, pad, n) => [Math.max(0, a - pad), Math.min(n - 1, b + pad)];
const overlaps = (x, a, b) => x.a <= b && x.b >= a;
const isLoco = f => LOCO.has(String(f.phase ?? '').toLowerCase());
const keyOf = c => JSON.stringify(c.fix ?? c.lock ?? c.edit ?? null);

function plantFor(plants, side, a, b) {
  return (plants ?? []).find(p => (!side || p.side === side) && p.a <= b && p.b >= a);
}

function changeFor(f, ctx, fixes, locks) {
  const { n, plants } = ctx;
  const a0 = f.frame_start ?? 0, b0 = f.frame_end ?? a0, side = sideOf(f.body_part);
  const tag = `${f.rule} ${f.body_part ?? ''} ${a0}-${b0}`;
  switch (f.rule) {
    case 'ground_float': {
      const [a, b] = clampRange(a0, b0, PAD.ground, n);
      return { finding: f, fix: { type: 'ground', a, b, side, anchor: 'ankle' }, note: `ground ${side ?? 'both'} ${a}-${b} for ${tag}` };
    }
    case 'ground_sink': {
      const [a, b] = clampRange(a0, b0, PAD.floor, n);
      return { finding: f, fix: { type: 'floor', a, b }, note: `floor ${a}-${b} for ${tag}` };
    }
    case 'pop':
    case 'plant_pop': {
      const [a, b] = clampRange(a0, b0, PAD.despike, n);
      return { finding: f, fix: { type: 'despike', a, b }, note: `despike ${a}-${b} for ${tag}` };
    }
    case 'foot_slide': {
      const p = plantFor(plants, side, a0, b0);
      const want = p ? [{ side: p.side, a: p.a, b: p.b }] : (side ? [side] : ['l', 'r']).map(s => ({ side: s, a: a0, b: b0 }));
      const lks = want.filter(lk => !locks.some(l => l.side === lk.side && overlaps(l, lk.a, lk.b)));
      if (!lks.length) return { finding: f, note: `${tag}: lock already covers it` };
      return { finding: f, lock: lks.length === 1 ? lks[0] : lks, note: `lock ${lks.map(l => l.side).join('+')} ${lks[0].a}-${lks[0].b} for ${tag}` };
    }
    case 'planted_toe_out_deg':
    case 'planted_ankle_deg': {
      const ti = fixes.findIndex(x => x.type === 'travel' && overlaps(x, a0, b0));
      if (ti >= 0 && isLoco(f)) {
        const v = f.value ?? 0, [lo, hi] = f.expected_range ?? [null, null];
        const target = hi != null && v > hi ? hi : lo != null && v < lo ? lo : null;
        const patch = f.rule === 'planted_toe_out_deg'
          ? { toe: Math.round((fixes[ti].toe ?? 0) + (target != null ? target - v : 0)) }
          : { pitch: target != null ? Math.round(target) : 0 };
        return { finding: f, edit: { index: ti, patch }, note: `travel #${ti} ${JSON.stringify(patch)} for ${tag}` };
      }
      if (fixes.some(x => x.type === 'level')) return { finding: f, note: `${tag}: level already present` };
      return { finding: f, fix: { type: 'level', side }, note: `level ${side ?? 'both'} for ${tag}` };
    }
    case 'crossing': {
      if (f.severity === 'blocker') {
        const [a, b] = clampRange(a0, b0, PAD.smooth, n);
        return { finding: f, fix: { type: 'smooth', a, b }, note: `smooth ${a}-${b} for ${tag}` };
      }
      return { finding: f, note: `${tag}: note only` };
    }
    default:
      return { finding: f, note: `${tag}: no auto fix` };
  }
}

export function applyChange(state, c) {
  const fixes = state.fixes.map(x => ({ ...x })), locks = state.locks.map(x => ({ ...x }));
  if (c.fix) fixes.push({ ...c.fix });
  if (c.lock) for (const l of [].concat(c.lock)) locks.push({ ...l });
  if (c.edit) Object.assign(fixes[c.edit.index], c.edit.patch);
  return { fixes, locks };
}

export function planFixes(findings, { fixes = [], locks = [], n, fps, plants } = {}) {
  const ctx = { n: n ?? Infinity, fps, plants };
  let state = { fixes: fixes.map(x => ({ ...x })), locks: locks.map(x => ({ ...x })) };
  const notes = [], changes = [], seen = new Set();
  for (const f of findings ?? []) {
    const c = changeFor(f, ctx, state.fixes, state.locks);
    notes.push(c.note);
    if (!c.fix && !c.lock && !c.edit) continue;
    const k = keyOf(c);
    if (seen.has(k)) continue;
    seen.add(k);
    changes.push(c);
    state = applyChange(state, c);
  }
  return { fixes: state.fixes, locks: state.locks, notes, changes };
}

export function score(findings) {
  let blockers = 0, warnings = 0;
  for (const f of findings ?? []) {
    if (f.severity === 'blocker') blockers++;
    else if (f.severity === 'warning') warnings++;
  }
  return { blockers, warnings };
}

const better = (s, t) => s.blockers < t.blockers || (s.blockers === t.blockers && s.warnings < t.warnings);
const asFindings = r => Array.isArray(r) ? r : r?.findings ?? [];

export async function autoFixLoop({ judge, apply, maxRounds = 5, fixes = [], locks = [], n, fps, plants, log = () => {} }) {
  let state = { fixes: fixes.map(x => ({ ...x })), locks: locks.map(x => ({ ...x })) };
  await apply(state);
  let findings = asFindings(await judge(state)), cur = score(findings);
  const start = cur, kept = [], reverted = [], notes = [];
  let rounds = 0, reason = 'max_rounds';
  while (true) {
    if (cur.blockers === 0) { reason = 'clean'; break; }
    if (rounds >= maxRounds) break;
    rounds++;
    const plan = planFixes(findings, { ...state, n, fps, plants });
    notes.push(...plan.notes);
    let progressed = false;
    for (const c of plan.changes) {
      const trial = applyChange(state, c);
      await apply(trial);
      const tf = asFindings(await judge(trial)), ts = score(tf);
      if (better(ts, cur)) {
        state = trial; findings = tf; cur = ts; progressed = true;
        kept.push({ round: rounds, note: c.note, ...ts });
        log(`keep ${c.note}`);
      } else {
        reverted.push({ round: rounds, note: c.note, ...ts });
        log(`revert ${c.note}`);
      }
    }
    await apply(state);
    if (!progressed) { reason = 'no_progress'; break; }
  }
  return { fixes: state.fixes, locks: state.locks, findings, start, end: cur, rounds, kept, reverted, notes, reason };
}
