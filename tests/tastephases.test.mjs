import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { phasedTasteInput, INTEGRITY_RULES } from '../tastebridge.js';

const TASTE = 'C:\\Projects\\LocomotionTaste';
const SERVER = fileURLToPath(new URL('../server.py', import.meta.url));
const TAKE = fileURLToPath(new URL('../exports/taste_in.json', import.meta.url));
const PY = process.env.PYTHON || 'python';
const ENVELOPE = ['planted_foot_pitch_deg', 'planted_ankle_deg', 'heel_raise_pct', 'planted_toe_out_deg', 'knee_behind_line', 'knee_flex_range_deg', 'knee_flex_max_deg', 'min_stance_width', 'max_stance_width'];
const skip = !existsSync(TASTE + '\\taste\\cli.py') ? 'LocomotionTaste not found' : !existsSync(TAKE) ? 'exports/taste_in.json missing' : false;

function py(args, input) {
  const r = spawnSync(PY, ['-B', ...args], { input, cwd: TASTE, env: { ...process.env, PYTHONDONTWRITEBYTECODE: '1' }, maxBuffer: 1 << 28, encoding: 'utf8' });
  assert.ok(r.stdout && r.stdout.trim().startsWith('{'), 'python failed: ' + (r.stderr || '').slice(-2000));
  return JSON.parse(r.stdout);
}

const overlaps = (f, a, b) => f.frame_start != null && f.frame_start <= b && f.frame_end >= a;

test('per-phase judging of the FRT drill drops drill false blockers, hides no float, keeps integrity findings', { skip }, () => {
  const raw = readFileSync(TAKE, 'utf8'), take = JSON.parse(raw), n = take.B.length;
  const whole = py(['-m', 'taste.cli', 'score', '-'], raw);
  const body = phasedTasteInput(take);
  const res = py([SERVER, '--taste-phases'], JSON.stringify(body));
  const phaseOf = f => res.phases[f.phase_index];

  assert.equal(res.mode, 'phased');
  assert.ok(['pass', 'warn', 'fail'].includes(res.verdict));
  assert.ok(Array.isArray(res.findings) && Array.isArray(res.unmeasurable) && typeof res.metrics === 'object');
  assert.equal(res.phases.length, body.phases.length);

  const flights = res.phases.filter(p => p.phase === 'flight');
  assert.equal(flights.length, 0, 'no physically impossible flight hides ground contact');
  assert.ok(res.findings.some(f => f.rule === 'ground_float' && overlaps(f, 65, 93)), 'sprint float f65-93 is judged');
  assert.ok(res.findings.some(f => f.rule === 'ground_float' && overlaps(f, 273, 294)), 'stop float f273-294 is judged');

  const wholeEnv = whole.findings.filter(f => ENVELOPE.includes(f.rule) && f.frame_start === 0 && f.frame_end === n - 1);
  assert.ok(wholeEnv.some(f => f.rule === 'knee_flex_max_deg'), 'whole-take judge reports the kneel knee depth, so this test discriminates');
  for (const f of res.findings.filter(f => ENVELOPE.includes(f.rule))) {
    const p = phaseOf(f);
    assert.equal(p.phase, 'locomotion', `${f.rule} only on locomotion, got ${p.phase}`);
    assert.ok(p.end - p.start + 1 >= 30, `${f.rule} only on locomotion windows of 30+ frames`);
    assert.ok(f.frame_start >= p.start && f.frame_end <= p.end, `${f.rule} range stays inside its phase`);
  }
  const low = res.phases.filter(p => ['kneel', 'slide', 'rise'].includes(p.phase));
  assert.ok(low.length >= 3);
  assert.ok(!res.findings.some(f => ENVELOPE.includes(f.rule) && low.some(p => overlaps(f, p.start, p.end))), 'no knee-depth or stance-width envelope finding over kneel, slide or rise');
  assert.ok(!res.findings.some(f => f.rule === 'foot_slide' && phaseOf(f)?.phase === 'slide'), 'no foot_slide in a slide phase');

  const integ = whole.findings.filter(f => INTEGRITY_RULES.includes(f.rule));
  for (const f of integ) {
    assert.ok(res.findings.some(g => g.rule === f.rule && g.scope === 'whole take' && Math.abs(g.frame_start - f.frame_start) <= 2), `integrity ${f.rule} at f${f.frame_start} kept`);
  }
  assert.ok(res.findings.some(g => g.rule === 'pop' && Math.abs(g.frame_start - 100) <= 2), 'shot1/shot2 seam pop near f100 kept');
  assert.ok(!res.findings.some(g => g.rule === 'crossing'), 'no shin crossing once the retarget is correct');
  assert.ok(!res.findings.some(g => g.rule === 'pop' && Math.abs(g.frame_start - 263) <= 2), 'no f263 pop once the last-frame wrap is fixed');

  for (const f of res.findings) assert.ok(typeof f.phase === 'string', 'every finding names its phase');
});
