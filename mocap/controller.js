import { qmul, qslerp, qyaw, qrot, qaxis, qalign, clamp, lerp, smoothstep, wrapAngle } from './math.js';
import { NB, getQ, setQ } from './skeleton.js';

export const KEYS = Object.freeze({
  forward: ['KeyW', 'ArrowUp'], back: ['KeyS', 'ArrowDown'], left: ['KeyA', 'ArrowLeft'], right: ['KeyD', 'ArrowRight'],
  run: ['ShiftLeft', 'ShiftRight'], walk: ['AltLeft', 'AltRight'], jump: ['Space'], action: ['KeyE'], select: ['KeyQ'], replay: ['KeyR'], stop: ['Escape'],
  slots: ['Digit1', 'Digit2', 'Digit3', 'Digit4', 'Digit5', 'Digit6', 'Digit7', 'Digit8', 'Digit9', 'Digit0'],
});

export function sampleClip(clip, t, out = { root: [0, 0, 0], rot: new Float64Array(NB * 4) }) {
  const n = clip.n, f = clip.loop ? (((t * clip.fps) % n) + n) % n : clamp(t * clip.fps, 0, n - 1);
  const i0 = Math.floor(f), i1 = clip.loop ? (i0 + 1) % n : Math.min(n - 1, i0 + 1), u = f - i0;
  for (let k = 0; k < 3; k++) out.root[k] = lerp(clip.root[i0 * 3 + k], clip.root[i1 * 3 + k], u);
  for (let b = 0; b < NB; b++) setQ(out.rot, b, qslerp(getQ(clip.rot, i0 * NB + b), getQ(clip.rot, i1 * NB + b), u));
  return out;
}

export function blendPoses(poses, weights, out = { root: [0, 0, 0], rot: new Float64Array(NB * 4) }) {
  let acc = null, wsum = 0;
  out.root[0] = out.root[1] = out.root[2] = 0;
  for (let k = 0; k < poses.length; k++) {
    const w = weights[k];
    if (!(w > 1e-6)) continue;
    for (let c = 0; c < 3; c++) out.root[c] += poses[k].root[c] * w;
    if (!acc) { acc = Float64Array.from(poses[k].rot); wsum = w; continue; }
    wsum += w;
    const t = w / wsum;
    for (let b = 0; b < NB; b++) setQ(acc, b, qslerp(getQ(acc, b), getQ(poses[k].rot, b), t));
  }
  if (!acc) return out;
  for (let c = 0; c < 3; c++) out.root[c] /= wsum;
  out.rot.set(acc);
  return out;
}

function mixPose(a, b, t) {
  return blendPoses([a, b], [1 - t, t]);
}

export function createController(library, o = {}) {
  const walk = library.walk, run = library.run;
  const walkSpeed = clamp(walk?.speed || 1.3, 0.5, 2.4), runSpeed = clamp(run?.speed || 3.3, walkSpeed * 1.3, 8);
  return {
    lib: library, walkSpeed, runSpeed, pos: [0, 0, 0], yaw: o.yaw ?? 0, speed: 0, phase: 0, idleT: 0, yawRate: 0, lean: 0,
    state: 'move', jump: null, action: null, selected: 0, time: 0, lastPose: null, slowWalk: o.slowWalk ?? 0.55,
    turnWalk: o.turnWalk ?? 7, turnRun: o.turnRun ?? 4.5, accel: o.accel ?? 4, decel: o.decel ?? 6,
  };
}

export function moveIntent(input) {
  const x = (input.right ? 1 : 0) - (input.left ? 1 : 0), y = (input.forward ? 1 : 0) - (input.back ? 1 : 0);
  const mag = Math.hypot(x, y);
  if (mag < 1e-6) return null;
  const cy = input.camYaw ?? 0, fwd = [Math.sin(cy), 0, Math.cos(cy)], right = [-Math.cos(cy), 0, Math.sin(cy)];
  const d = [fwd[0] * y + right[0] * x, 0, fwd[2] * y + right[2] * x], l = Math.hypot(d[0], d[2]);
  return { dir: [d[0] / l, 0, d[2] / l], yaw: Math.atan2(d[0], d[2]), amount: Math.min(1, mag) };
}

function locomotionPose(ctl, dt) {
  const { lib } = ctl, wRun = clamp((ctl.speed - ctl.walkSpeed) / (ctl.runSpeed - ctl.walkSpeed), 0, 1);
  const wMove = clamp(ctl.speed / (ctl.walkSpeed * 0.6), 0, 1);
  const strideW = lib.walk.speed * lib.walk.cycle, strideR = lib.run.speed * lib.run.cycle;
  const stride = lerp(strideW, strideR, wRun), cycle = lerp(lib.walk.cycle, lib.run.cycle, wRun);
  const rate = Math.max(ctl.speed / Math.max(0.05, stride), wMove > 0.01 ? 0.5 / cycle : 0);
  ctl.phase = (ctl.phase + rate * dt) % 1;
  ctl.idleT += dt;
  const poses = [], weights = [];
  if (1 - wMove > 1e-3) { poses.push(sampleClip(lib.idle, ctl.idleT)); weights.push(1 - wMove); }
  if (wMove * (1 - wRun) > 1e-3) { poses.push(sampleClip(lib.walk, ctl.phase * lib.walk.cycle)); weights.push(wMove * (1 - wRun)); }
  if (wMove * wRun > 1e-3) { poses.push(sampleClip(lib.run, ctl.phase * lib.run.cycle)); weights.push(wMove * wRun); }
  const pose = blendPoses(poses, weights);
  return { pose, wRun, wMove };
}

function placePose(pose, pos, yaw, lean = 0) {
  const R = qyaw(yaw), local = Float64Array.from(pose.rot);
  const hips = lean ? qmul(R, qmul(qaxis([0, 0, 1], lean), getQ(pose.rot, 0))) : qmul(R, getQ(pose.rot, 0));
  setQ(local, 0, hips);
  const off = qrot(R, [pose.root[0], 0, pose.root[2]]);
  return { root: [pos[0] + off[0], pose.root[1], pos[2] + off[2]], rot: local };
}

function startAction(ctl, clip, kind) {
  if (!clip) return;
  const dur = clip.loop ? clip.cycle * Math.max(1, Math.ceil(1.5 / clip.cycle)) : (clip.n - 1) / clip.fps;
  ctl.action = { clip, t: 0, kind, startPos: ctl.pos.slice(), startYaw: ctl.yaw, from: ctl.lastPose, dur };
  ctl.state = 'action';
  ctl.speed = 0;
}

export function stepController(ctl, input = {}, dt = 1 / 60) {
  dt = clamp(dt, 0, 0.1);
  ctl.time += dt;
  const lib = ctl.lib;
  if (input.stop && ctl.state === 'action') { ctl.state = 'move'; ctl.action = null; }
  const na = lib.actions?.length ?? 0;
  if (input.select && na) ctl.selected = (((ctl.selected + input.select) % na) + na) % na;
  if (input.slot != null && lib.actions?.[input.slot]) { ctl.selected = input.slot; startAction(ctl, lib.actions[input.slot], 'action'); }
  else if ((input.action || input.click) && (lib.actions?.[ctl.selected] || lib.take)) startAction(ctl, lib.actions?.[ctl.selected] ?? lib.take, 'action');
  else if (input.replay && lib.take) startAction(ctl, lib.take, 'take');
  const intent = moveIntent(input);
  let result;
  if (ctl.state === 'action') {
    const A = ctl.action;
    A.t += dt;
    const clipPose = sampleClip(A.clip, A.t), R = qyaw(A.startYaw);
    const off = qrot(R, [clipPose.root[0], 0, clipPose.root[2]]);
    const world = { root: [A.startPos[0] + off[0], clipPose.root[1], A.startPos[2] + off[2]], rot: Float64Array.from(clipPose.rot) };
    setQ(world.rot, 0, qmul(R, getQ(clipPose.rot, 0)));
    const wIn = A.from ? smoothstep(0, 0.25, A.t) : 1;
    result = wIn < 1 ? mixWorld(A.from, world, wIn) : world;
    const f = qrot(getQ(world.rot, 0), [0, 0, 1]);
    ctl.pos = [world.root[0], 0, world.root[2]];
    ctl.yaw = Math.atan2(f[0], f[2]);
    if (A.t >= A.dur || (intent && A.t > 0.3)) {
      if (A.t >= A.dur) {
        const e = qrot(R, A.clip.rootMotion?.end ?? [0, 0, 0]);
        ctl.pos = [A.startPos[0] + e[0], 0, A.startPos[2] + e[2]];
        ctl.yaw = A.startYaw + (A.clip.rootMotion?.turn ?? 0);
      }
      ctl.state = 'move';
      ctl.action = null;
      ctl.blendFrom = { pose: result, t: 0 };
    }
  } else {
    if (input.jump && ctl.state === 'move' && lib.jump) {
      ctl.jump = { t: 0, clip: lib.jump };
      ctl.state = 'jump';
    }
    const runMode = !!input.run && !input.walk;
    const target = intent ? (runMode ? ctl.runSpeed : input.walk ? ctl.walkSpeed * ctl.slowWalk : ctl.walkSpeed) * intent.amount : 0;
    const J0 = ctl.state === 'jump' ? ctl.jump : null, air = !!J0 && J0.t > (J0.clip.takeoff ?? 0) / J0.clip.fps && J0.t < (J0.clip.landing ?? J0.clip.n) / J0.clip.fps;
    if (!air) ctl.speed += clamp(target - ctl.speed, -ctl.decel * dt, (runMode ? ctl.accel * 1.5 : ctl.accel) * dt);
    const prevYaw = ctl.yaw;
    if (intent && !air) {
      const diff = wrapAngle(intent.yaw - ctl.yaw), wRun = clamp((ctl.speed - ctl.walkSpeed) / (ctl.runSpeed - ctl.walkSpeed), 0, 1);
      const rate = lerp(ctl.turnWalk, ctl.turnRun, wRun) * (Math.abs(diff) > 2 && ctl.speed < ctl.walkSpeed ? 1.6 : 1);
      ctl.yaw += clamp(diff, -rate * dt, rate * dt);
    }
    ctl.yawRate = dt > 0 ? wrapAngle(ctl.yaw - prevYaw) / dt : 0;
    ctl.pos[0] += Math.sin(ctl.yaw) * ctl.speed * dt;
    ctl.pos[2] += Math.cos(ctl.yaw) * ctl.speed * dt;
    const loco = locomotionPose(ctl, dt);
    const leanTarget = clamp(-ctl.yawRate * ctl.speed * 0.035 * loco.wRun, -0.14, 0.14);
    ctl.lean += (leanTarget - ctl.lean) * clamp(dt * 8, 0, 1);
    let pose = loco.pose;
    if (ctl.state === 'jump') {
      const J = ctl.jump;
      J.t += dt;
      const jp = sampleClip(J.clip, J.t), dur = (J.clip.n - 1) / J.clip.fps;
      const w = Math.min(smoothstep(0, 0.12, J.t), 1 - smoothstep(dur - 0.2, dur, J.t));
      const jpl = { root: [pose.root[0], jp.root[1], pose.root[2]], rot: jp.rot };
      pose = mixPose(pose, jpl, w);
      if (J.t >= dur) { ctl.state = 'move'; ctl.jump = null; }
    }
    result = placePose(pose, ctl.pos, ctl.yaw, ctl.lean);
    if (ctl.blendFrom) {
      ctl.blendFrom.t += dt;
      const w = smoothstep(0, 0.3, ctl.blendFrom.t);
      result = mixWorld(ctl.blendFrom.pose, result, w);
      if (w >= 1) ctl.blendFrom = null;
    }
  }
  ctl.lastPose = { root: result.root.slice(), rot: Float64Array.from(result.rot) };
  return { ...result, state: ctl.state, speed: ctl.speed, yaw: ctl.yaw, pos: ctl.pos.slice(), phase: ctl.phase, action: ctl.action?.clip?.name ?? null, selected: lib.actions?.[ctl.selected]?.name ?? null };
}

function mixWorld(a, b, t) {
  if (!a) return b;
  const out = { root: [lerp(a.root[0], b.root[0], t), lerp(a.root[1], b.root[1], t), lerp(a.root[2], b.root[2], t)], rot: new Float64Array(NB * 4) };
  for (let i = 0; i < NB; i++) setQ(out.rot, i, qslerp(getQ(a.rot, i), qalign(getQ(b.rot, i), getQ(a.rot, i)), t));
  return out;
}

export function inputFromKeys(down, pressed = new Set()) {
  const has = list => list.some(k => down.has(k)), hit = list => list.some(k => pressed.has(k));
  const slot = KEYS.slots.findIndex(k => pressed.has(k));
  return {
    forward: has(KEYS.forward), back: has(KEYS.back), left: has(KEYS.left), right: has(KEYS.right),
    run: has(KEYS.run), walk: has(KEYS.walk), jump: hit(KEYS.jump), action: hit(KEYS.action), replay: hit(KEYS.replay), stop: hit(KEYS.stop),
    slot: slot >= 0 ? slot : null, select: hit(KEYS.select) ? (has(KEYS.run) ? -1 : 1) : 0, click: pressed.has('Click'),
  };
}

export function recordTake(ctl, script, fps = 30) {
  const frames = [];
  for (const step of script) {
    const n = Math.round(step.seconds * fps);
    for (let i = 0; i < n; i++) frames.push(stepController(ctl, { ...step.input, jump: step.input.jump && i === 0, action: step.input.action && i === 0, replay: step.input.replay && i === 0, slot: i === 0 ? step.input.slot ?? null : null, select: i === 0 ? step.input.select ?? 0 : 0, click: step.input.click && i === 0 }, 1 / fps));
  }
  const n = frames.length, root = new Float64Array(n * 3), rot = new Float64Array(n * NB * 4);
  frames.forEach((f, i) => {
    root.set(f.root, i * 3);
    for (let b = 0; b < NB; b++) {
      let q = getQ(f.rot, b);
      if (i > 0) q = qalign(q, getQ(rot, (i - 1) * NB + b));
      setQ(rot, i * NB + b, q);
    }
  });
  return { n, fps, root, rot, frames };
}
