// One-click FIX loop, judge-agnostic and pure: it only touches the world through the two callbacks.
//   apply(state)            -> motion (rebuild with state.fixes/locks/pins; may be async)
//   evaluate(state, motion) -> { findings, cost, score?, fidelity?, facts? } (judgeMotion's shape)
// Per stage it plans candidates from the current findings, tries each candidate and its parameter
// variants, keeps the best trial only when the score drops without more blockers, more failed
// constraints or a fidelity penalty past the caps, and otherwise reverts. It works blockers first,
// then warnings, and stops before export.
import { STAGES, STAGE_TEXT, NO_AUTO_FIX, planCandidates, applyChange, changeKey, preflight } from './fixplan.js';
import { frameText, sideText } from './facts.js';

// fidelityRise caps the rise of the rotation part of the fidelity penalty per kept change (untouched
// joints); pelvisRiseCm caps how much further one change may move the pelvis from the source (root
// fixes such as seams or ballistic move it on purpose, so that part only enters the score).
export const ENGINE_DEFAULTS = Object.freeze({ maxTrials: 80, minGain: 1e-6, relGain: 0.002, fidelityRise: 15, fidelityWorstDeg: 8, pelvisRiseCm: 40, includeInfo: false });
const HARD = ['lock_residual', 'pin_residual', 'nonfinite', 'bone_stretch'];

const count = (fs, sev) => fs.filter(f => f.severity === sev).length;
const hard = fs => fs.filter(f => HARD.includes(f.rule) && f.severity === 'blocker').length;
export const scoreOf = j => Number.isFinite(j?.score) ? j.score : (j?.cost ?? Infinity) + (j?.fidelity?.penalty ?? 0);
const summary = j => ({ blockers: count(j.findings, 'blocker'), warnings: count(j.findings, 'warning'), infos: count(j.findings, 'info'), cost: +(+j.cost || 0).toFixed(1), score: +scoreOf(j).toFixed(1), fidelityDeg: j.fidelity ? +j.fidelity.worst.toFixed(2) : null, pelvisCm: j.fidelity ? +j.fidelity.pelvisCm.toFixed(1) : null });

export function accept(trial, cur, caps = ENGINE_DEFAULTS) {
  if (!trial || !Array.isArray(trial.findings)) return { ok: false, why: 'the judge returned nothing' };
  const tb = count(trial.findings, 'blocker'), cb = count(cur.findings, 'blocker');
  if (tb > cb) return { ok: false, why: `it raised blockers ${cb} to ${tb}` };
  if (hard(trial.findings) > hard(cur.findings)) return { ok: false, why: 'it broke a lock, pin or bone length' };
  const ts = scoreOf(trial), cs = scoreOf(cur), need = Math.max(caps.minGain ?? 1e-6, (caps.relGain ?? 0) * Math.abs(cs));
  if (!(ts < cs - need)) return { ok: false, why: `it did not help (score ${cs.toFixed(1)} to ${ts.toFixed(1)})` };
  const tf = trial.fidelity, cf = cur.fidelity;
  if (tf && cf && !tf.skipped) {
    const rise = (tf.rotPenalty ?? tf.penalty) - (cf.rotPenalty ?? cf.penalty);
    if (rise > (caps.fidelityRise ?? Infinity)) return { ok: false, why: `it changed untouched joints too much (fidelity penalty +${rise.toFixed(1)})` };
    if ((tf.pelvisCm ?? 0) - (cf.pelvisCm ?? 0) > (caps.pelvisRiseCm ?? Infinity)) return { ok: false, why: `it moved the pelvis ${(tf.pelvisCm - cf.pelvisCm).toFixed(0)} cm further from the take` };
    if (tf.worst > Math.max(caps.fidelityWorstDeg ?? Infinity, cf.worst) + 1e-9) return { ok: false, why: `it moved ${tf.worstBone} ${tf.worst.toFixed(1)} deg, past the fidelity limit` };
  }
  return { ok: true, why: `score ${cs.toFixed(1)} to ${ts.toFixed(1)}` };
}

const FIX_TEXT = {
  despike: x => `despike ${frameText(x.a, x.b)}${Array.isArray(x.bones) ? ` on ${x.bones.length} bones` : ''}`,
  filter: x => `filter ${x.fc} Hz ${frameText(x.a, x.b)}${Array.isArray(x.bones) ? ` on ${x.bones.length} bones` : ''}`,
  median: x => `median smooth ${frameText(x.a, x.b)}`, smooth: x => `smooth ${frameText(x.a, x.b)}`,
  knee: x => `${sideText(x.side)} knee ${x.mode} ${frameText(x.a, x.b)}`, seams: x => `match seams (blend ${x.blend}, ${x.mode})`,
  ballistic: x => `ballistic arc ${frameText(x.a, x.b)}`, momentum: x => `momentum ${frameText(x.a, x.b)}`,
  fulcrum: x => `pivot on ${x.pivot} ${frameText(x.a, x.b)}`, ground: x => `ground ${sideText(x.side)} foot ${frameText(x.a, x.b)}`,
  floor: x => `floor contact ${x.side ? sideText(x.side) + ' ' : ''}${frameText(x.a, x.b)} (pivot ${x.pivot}${x.stiffness ? ', stiffness ' + x.stiffness : ''})`,
  ankleLock: x => `moving foot anchor ${sideText(x.side)} ${frameText(x.a, x.b)} (perimeter ${x.perimeter} cm${x.maxSlide ? ', slide ' + x.maxSlide + ' m/s' : ''})`,
  level: x => `level ${sideText(x.side)} foot`, roll: x => `foot roll ${sideText(x.side)} ${frameText(x.a, x.b)}`,
  pull: x => `pull body to hand (pull ${x.pull})`, travel: x => `footsteps ${x.speed} m/s ${frameText(x.a, x.b)}`,
};
export function describeChange(ch) {
  const parts = [];
  for (const l of ch.dropLocks || []) parts.push(`unlock ${sideText(l.side)} ${frameText(l.a, l.b)}`);
  for (const x of ch.fixes || []) parts.push((FIX_TEXT[x.type] ?? (y => y.type))(x));
  for (const l of ch.locks || []) parts.push(`lock ${sideText(l.side)} foot ${frameText(l.a, l.b)}`);
  for (const p of ch.pins || []) parts.push(`pin ${p.hand === 'r' ? 'right' : 'left'} hand ${frameText(p.a, p.b)}`);
  for (const e of ch.edits || []) parts.push(`edit fix #${e.index} ${JSON.stringify(e.patch)}`);
  return parts.join(', ') || 'no change';
}

export function flaggedRanges(findings, severities = ['blocker', 'warning']) {
  const fs = findings.filter(f => severities.includes(f.severity) && Number.isFinite(f.a)).sort((x, y) => x.a - y.a), out = [];
  for (const f of fs) {
    const last = out[out.length - 1], b = f.b ?? f.a;
    if (last && f.a <= last.b + 1) { last.b = Math.max(last.b, b); if (!last.rules.includes(f.rule)) last.rules.push(f.rule); if (f.severity === 'blocker') last.severity = 'blocker'; }
    else out.push({ a: f.a, b, severity: f.severity, rules: [f.rule] });
  }
  return out;
}

export async function runFix({ state, evaluate, apply, stages = STAGES, maxTrials, log = () => {}, plan = planCandidates, caps = {}, includeInfo } = {}) {
  const C = { ...ENGINE_DEFAULTS, ...caps, ...(maxTrials != null ? { maxTrials } : {}), ...(includeInfo != null ? { includeInfo } : {}) };
  const report = [], kept = [], reverted = [], tried = new Set();
  const say = t => { report.push(t); log(t); };
  const run = async s => { const motion = await apply(s); return { motion, j: await evaluate(s, motion) }; };
  let st = applyChange(state || {}, {}), trials = 0, reason = 'done';
  let { motion, j: cur } = await run(st);
  const start = summary(cur);
  say(`Start: ${start.blockers} blocker(s), ${start.warnings} warning(s).`);
  outer: for (const stage of stages) {
    if (stage === 'stop') { say('Stopped before export: review the result, then export separately.'); break; }
    if (stage === 'preflight') { if (cur.facts) for (const t of preflight(cur.facts)) say('Rig check: ' + t); continue; }
    for (;;) {
      if (trials >= C.maxTrials) { reason = 'max_trials'; say(`Stopped after ${trials} trials (budget).`); break outer; }
      const rank = c => ({ blocker: 0, warning: 1, info: 2 })[c.severity ?? c.finding?.severity] ?? 3;
      const cands = plan({ findings: cur.findings, facts: cur.facts, state: st, stage, motion, includeInfo: C.includeInfo }).map(c => ({ ...c, key: c.key ?? changeKey(c.change) })).filter(c => !tried.has(c.key)).sort((x, y) => rank(x) - rank(y));
      if (!cands.length) break;
      const c = cands[0];
      tried.add(c.key);
      let best = null;
      const fails = [];
      for (const ch of [c.change, ...c.variants]) {
        if (trials >= C.maxTrials) break;
        const k = changeKey(ch);
        if (ch !== c.change && tried.has(k)) continue;
        tried.add(k);
        const tag = { id: `fix-${kept.length + 1}`, stage, rule: c.rule };
        const trial = applyChange(st, ch, tag);
        trials++;
        let r;
        try { r = await run(trial); } catch (e) { fails.push({ change: ch, why: 'it failed: ' + (e?.message ?? e) }); continue; }
        const v = accept(r.j, cur, C);
        if (v.ok && (!best || scoreOf(r.j) < scoreOf(best.r.j))) best = { trial, r, ch, tag, why: v.why };
        else if (!v.ok) fails.push({ change: ch, why: v.why, score: scoreOf(r.j) });
      }
      const where = `${STAGE_TEXT[stage] ?? stage}`, problem = c.finding.text ?? c.finding.rule;
      if (best) {
        const before = summary(cur);
        st = best.trial; cur = best.r.j; motion = best.r.motion;
        const item = { id: best.tag.id, stage, rule: c.rule, label: c.label, change: best.ch, detail: describeChange(best.ch), finding: { rule: c.finding.rule, a: c.finding.a, b: c.finding.b, side: c.finding.side, phase: c.finding.phase, text: problem }, why: best.why, variant: best.ch !== c.change, rejected: fails.map(x => ({ detail: describeChange(x.change), why: x.why })), before, after: summary(cur) };
        kept.push(item);
        say(`${where}: ${problem}. ${c.label} (${item.detail}).`);
      } else {
        reverted.push({ stage, rule: c.rule, label: c.label, finding: { rule: c.finding.rule, a: c.finding.a, b: c.finding.b, side: c.finding.side, phase: c.finding.phase, text: problem }, tried: fails.map(x => ({ detail: describeChange(x.change), why: x.why })) });
        say(`${where}: tried to fix "${problem}" (${c.label.toLowerCase()}) but ${fails[0]?.why ?? 'no option could be tried'}; left as it was.`);
      }
    }
  }
  const final = await run(st);
  motion = final.motion; cur = final.j;
  const end = summary(cur), remaining = cur.findings.filter(f => f.severity !== 'info');
  for (const f of remaining) say(`Still open (${f.severity}): ${f.text ?? f.rule}${NO_AUTO_FIX[f.rule] ? ' - ' + NO_AUTO_FIX[f.rule] : ''}.`);
  say(`Done: blockers ${start.blockers} to ${end.blockers}, warnings ${start.warnings} to ${end.warnings}; kept ${kept.length} change(s), reverted ${reverted.length}, ${trials} trial(s).`);
  return { state: st, kept, reverted, report, findings: cur.findings, flagged: flaggedRanges(cur.findings), start, end, trials, reason, judgement: cur, motion };
}
