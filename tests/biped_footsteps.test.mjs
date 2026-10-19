import test from 'node:test';
import assert from 'node:assert/strict';
import { json } from './helpers.mjs';
import * as F from '../re/biped/footsteps.js';

const M = json(new URL('./fixtures/biped_max_measured.json', import.meta.url));
const LIVE = json(new URL('./fixtures/biped_max_live.json', import.meta.url));

const paramsOf = c => {
  if (c.params) return { gait: c.gait, count: c.params.numFootsteps, stride: c.params.paramStrideLength1, stride2: c.params.paramStrideLength2, cycle: c.params.cycle1, cycle2: c.params.cycle2, width: c.params.paramStrideWidth, strideHeight: c.params.actualStrideHeight1, strideHeight2: c.params.actualStrideHeight2, height: c.h };
  const cy = +(c.tag.match(/_c(\d+)/)?.[1] ?? 15);
  const sl = c.gait === 'walk' ? 0.75 : 1.5;
  return { gait: c.gait, count: 10, stride: c.tag.includes('halfsl') ? sl / 2 : sl, cycle: cy, height: 170 };
};
const generated = M.cases.filter(c => (c.src === 's1' || c.src === 's3' || c.tag.endsWith('base')) && !(c.params && c.params.autoTiming === false));

function steadyError(plan, frames, gz = 0) {
  const com = F.sampleCOM(plan, F.comKeys(plan), { frames: frames.length });
  const s0 = plan.footsteps[Math.min(4, plan.footsteps.length - 1)].start, s1 = plan.footsteps[plan.footsteps.length - 2].start;
  let ex = 0, ez = 0;
  for (const r of frames) {
    const t = r[0];
    if (t < s0 || t > s1) continue;
    ex = Math.max(ex, Math.abs(com[t][0] - r[1]));
    ez = Math.max(ez, Math.abs(com[t][2] - (r[3] - gz)));
  }
  return { ex: ex / plan.S, ez: ez / plan.S };
}

test('gaitTiming reproduces the decoded 180085330 constants', () => {
  const S = 81.9277;
  assert.deepEqual(F.gaitTiming('walk', 0.75 * S, S, 15), { gait: 'walk', cycle: 15, doubleSupport: 3, duration: 18, swing: 12 });
  assert.equal(F.gaitTiming('walk', 0.75 * S, S, 10).doubleSupport, 2);
  assert.equal(F.gaitTiming('walk', 0.75 * S, S, 30).doubleSupport, 5);
  assert.equal(F.gaitTiming('walk', 1.0 * S, S, 15).duration, 17);
  assert.deepEqual(F.gaitTiming('run', 1.5 * S, S, 15), { gait: 'run', cycle: 15, contact: 6, duration: 6, airborne: 9 });
  assert.equal(F.gaitTiming('jump', 2.4 * S, S, 20).contact, 5);
  assert.equal(F.gaitTiming('jump', 2.4 * S, S, 10).contact, 3);
  assert.equal(F.gaitTiming('run', 0, S, 20).contact, 14);
  assert.equal(F.gaitTiming('run', 4 * S, S, 10).contact, 2);
});

test('generated footsteps match every measured Max case exactly in time and to 0.005 units in place', () => {
  let n = 0, worstPos = 0;
  for (const c of generated) {
    const plan = F.generateFootsteps(paramsOf(c));
    assert.equal(plan.footsteps.length, c.footsteps.length, `${c.src} ${c.tag} ${c.gait} count`);
    c.footsteps.forEach((m, i) => {
      const a = plan.footsteps[i];
      assert.equal(a.side, m.side, `${c.tag} ${c.gait} side ${i}`);
      assert.equal(a.start, m.start, `${c.tag} ${c.gait} start ${i}`);
      assert.equal(a.dur, m.dur, `${c.tag} ${c.gait} dur ${i}`);
      for (let k = 0; k < 3; k++) worstPos = Math.max(worstPos, Math.abs(a.pos[k] - m.pos[k]));
    });
    n++;
  }
  console.log(`${n} measured cases, worst footprint position error ${worstPos.toFixed(5)} units`);
  assert.ok(n >= 50);
  assert.ok(worstPos < 0.005);
});

test('interpolated stride and stride height follow the measured ramp', () => {
  const plan = F.generateFootsteps({ gait: 'walk', count: 6, stride: 1.0, stride2: 0.3, cycle: 20, strideHeight: 10, height: 170 });
  const xs = plan.footsteps.map(f => +f.pos[0].toFixed(3));
  assert.deepEqual(xs, [0, 0, 81.928, 144.739, 188.434, 213.012]);
  assert.deepEqual(plan.footsteps.map(f => +f.pos[2].toFixed(3)), [0, 0, 10, 20, 30, 40]);
});

test('COM model stays within 0.6% S horizontally and 2.5% S vertically in steady walk and run (measured batch cases)', () => {
  for (const c of M.cases.filter(x => x.frames && x.gait !== 'jump')) {
    const plan = F.generateFootsteps(paramsOf(c));
    const e = steadyError(plan, c.frames);
    assert.ok(e.ex < 0.006, `${c.src} ${c.tag} ${c.gait} dx ${e.ex}`);
    assert.ok(e.ez < 0.025, `${c.src} ${c.tag} ${c.gait} dz ${e.ez}`);
  }
});

test('held-out live 3ds Max cases: exact footsteps, COM within 1.5% S for walk/run and 3% S for jump', () => {
  let worst = { ex: 0, ez: 0 };
  for (const c of LIVE) {
    const plan = F.generateFootsteps({ gait: c.gait, count: c.count, stride: c.stride, cycle: c.cycle, height: c.height, width: c.width });
    const g = c.footsteps[0].pos[2];
    c.footsteps.forEach((m, i) => {
      const a = plan.footsteps[i];
      assert.equal(a.side, m.side.replace('#', ''));
      assert.equal(a.start, m.start, `${c.gait} start ${i}`);
      assert.equal(a.dur, m.dur, `${c.gait} dur ${i}`);
      assert.ok(Math.abs(a.pos[0] - m.pos[0]) < 0.005 && Math.abs(a.pos[1] - m.pos[1]) < 0.005);
    });
    const e = steadyError(plan, c.frames, g);
    const lim = c.gait === 'jump' ? [0.03, 0.02] : [0.006, 0.015];
    assert.ok(e.ex < lim[0], `${c.gait} ${c.stride} dx ${e.ex}`);
    assert.ok(e.ez < lim[1], `${c.gait} ${c.stride} dz ${e.ez}`);
    if (c.gait !== 'jump') worst = { ex: Math.max(worst.ex, e.ex), ez: Math.max(worst.ez, e.ez) };
  }
  console.log(`live walk/run worst steady COM error dx ${(worst.ex * 100).toFixed(2)}% S, dz ${(worst.ez * 100).toFixed(2)}% S`);
});

test('run touchdown and liftoff COM sit on the standing-reach circle', () => {
  const plan = F.generateFootsteps({ gait: 'run', count: 10, stride: 1.5, cycle: 15, height: 170 });
  const k = F.comKeys(plan);
  const R0 = F.MAX.standReach * plan.S;
  for (const v of k.vertical.filter(v => v.kind === 'touch' || v.kind === 'lift')) {
    const f = plan.footsteps[v.foot];
    const h = k.horizontal.find(x => x.t === v.t);
    assert.ok(Math.abs(Math.hypot(h.x - f.pos[0], v.z - f.pos[2]) - R0) < 1e-6);
  }
});

test('sprintPlan gives real strides at 3.5 m/s instead of running in place', () => {
  const plan = F.sprintPlan({ speed: 3.5, S: 0.87, fps: 30, count: 12 });
  const steps = plan.footsteps.slice(2);
  const dist = steps.at(-1).pos[0] - steps[0].pos[0];
  const time = (steps.at(-1).start - steps[0].start) / 30;
  const speed = dist / time;
  assert.ok(speed > 3.2 && speed < 3.8, `speed ${speed}`);
  const t = steps[0].timing;
  assert.ok(t.contact / 30 > 0.1 && t.contact / 30 < 0.2, `contact ${t.contact}`);
  assert.ok(t.airborne > 0);
  const com = F.sampleCOM(plan, F.comKeys(plan));
  const a = steps[2].start, b = steps.at(-2).start;
  const comSpeed = (com[b][0] - com[a][0]) / ((b - a) / 30);
  assert.ok(Math.abs(comSpeed - speed) / speed < 0.03, `com ${comSpeed} feet ${speed}`);
  const w = F.toWorld(steps[0].pos, { forward: [0, 0, 1] });
  assert.ok(Math.abs(w[2] - steps[0].pos[0]) < 1e-9 && Math.abs(w[0] - steps[0].pos[1]) < 1e-9);
});
