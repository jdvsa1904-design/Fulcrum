#!/usr/bin/env node
// Indexes a folder of BVH clips: parse, canonical bone map (wire/rigmap.js), duplicates, category guess,
// motion facts and quality flags. Writes index.json and summary.md. No dependencies.
// node tools/motionlib_index.mjs [library dir with Animations/, default data/MotionLibrary] [out dir, default <library>/index] [--limit N]
// Treats the library as data only: nothing in it is executed.
import { readFileSync, writeFileSync, readdirSync, mkdirSync, existsSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { join, basename, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseBVH } from '../mocap/bvh.js';
import { buildRigMap, CORE, WANTED } from '../wire/rigmap.js';
import { RIG_NAMES, legsOf, estimateGround } from '../wire/facts.js';
import { detectFootPlants } from '../wire/plants.js';
import { segmentPhases, BONES as PHASE_BONES } from '../phases.js';
import { GAIT, cycleTime } from '../mocap/gait.js';
import { LIMITS } from '../mocap/depthsign.js';
import { forward, stripPrefix, arpAlias, rigFamily } from './motionlib_bvh.mjs';

const args = process.argv.slice(2);
const flag = (k, d) => { const i = args.indexOf(k); return i >= 0 ? args[i + 1] : d; };
const pos = args.filter((a, i) => !a.startsWith('--') && !(i > 0 && args[i - 1].startsWith('--')));
const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const LIB = resolve(pos[0] ?? join(ROOT, 'data', 'MotionLibrary')), OUT = resolve(pos[1] ?? join(LIB, 'index'));
if (!existsSync(LIB)) { console.error('usage: node tools/motionlib_index.mjs [library dir] [out dir] [--limit N]; not found: ' + LIB); process.exit(2); }
const LIMIT = +flag('--limit', 0) || Infinity;

const D = 180 / Math.PI, FINGER = /(thumb|index|middle|ring|pinky)_0\d/;
const TH = Object.freeze({
  teleport_m_per_frame: 0.4, joint_speed_max_ms: 25, rapid_step_deg: 45, flip_step_deg: 150, rapid_steps_min: 1,
  drift_warn: 0.02, drift_bad: 0.05, sink_cm: 2.5, skate_frac: 0.15, near_dup_deg: 3, near_dup_travel_deg: 25, static_deg: 1,
});

// ---------- math ----------
const qang = (a, b) => { const d = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]); return 2 * Math.acos(Math.min(1, d)) * D; };
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const len = a => Math.hypot(a[0], a[1], a[2]);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const angAt = (a, b, c) => { const u = sub(a, b), v = sub(c, b), d = len(u) * len(v); return d ? Math.acos(Math.max(-1, Math.min(1, dot(u, v) / d))) * D : NaN; };
const angV = (u, v) => { const d = len(u) * len(v); return d ? Math.acos(Math.max(-1, Math.min(1, dot(u, v) / d))) * D : NaN; };
const pct = (a, p) => { const t = a.filter(Number.isFinite).sort((x, y) => x - y); return t.length ? t[Math.min(t.length - 1, Math.max(0, Math.round((p / 100) * (t.length - 1))))] : NaN; };
const median = a => pct(a, 50);
const mean = a => { const t = a.filter(Number.isFinite); return t.length ? t.reduce((x, y) => x + y, 0) / t.length : NaN; };
const r2 = v => Number.isFinite(v) ? +v.toFixed(2) : null, r3 = v => Number.isFinite(v) ? +v.toFixed(3) : null;
const runsOf = (mask, minLen = 1) => { const out = []; let s = -1; for (let i = 0; i <= mask.length; i++) { if (i < mask.length && mask[i]) { if (s < 0) s = i; } else if (s >= 0) { if (i - s >= minLen) out.push([s, i - 1]); s = -1; } } return out; };
const wrap = a => { while (a > 180) a -= 360; while (a < -180) a += 360; return a; };

const wp = (fk, f, i) => [fk.P[(f * fk.nj + i) * 3], fk.P[(f * fk.nj + i) * 3 + 1], fk.P[(f * fk.nj + i) * 3 + 2]];
const lq = (fk, f, i) => Array.from(fk.L.subarray((f * fk.nj + i) * 4, (f * fk.nj + i) * 4 + 4));
const lt = (fk, f, i) => Array.from(fk.T.subarray((f * fk.nj + i) * 3, (f * fk.nj + i) * 3 + 3));

// ---------- categories ----------
const NAME_RULES = [
  ['reference', /(t_?pose|a_pose)/i],
  ['animal', /(graze|chew|baaa|ramattack)/i],
  ['death', /(death|dying|headshot|_shot|shot_|bleedout|ragd|seizure|throat|electric)/i],
  ['getup', /(get_?up|getup|prone2idle|prone2crouch|crawl_from|crawl_to)/i],
  ['fall', /(fall|falling|ffall)/i],
  ['hit_react', /(hit|gethit)/i],
  ['jump', /(jump|landing)/i],
  ['climb_vault', /(climb|vault)/i],
  ['prone_crawl', /(prone|crawl)/i],
  ['combat', /(attack|melee|kick|slash|grenade|reload|aim_idle|combat|fists)/i],
  ['sit_lie', /(sit|sleep|bed|lay)/i],
  ['hostage_emote', /(hostage|beg_|hands_up|angry|dontknow|looking|staring|holding|lean|planting|patrol_act_idle)/i],
  ['start_stop', /(start|end(run|step)|stop|2idle|2crouch|idle2|combattoidle|idletocombat|gunDown2idle)/i],
  ['turn', /(turn)/i],
  ['strafe', /(strafe|_(45|90|135)[lr]|_(l|r|fl|fr|bl|br)_|_(lt|rt)(45|135)?_|walk(lt|rt)|runltloop|runrtloop|_[lr]_loop|walkingl$|walkingr$|sprint_[lr]$|bkpd)/i],
  ['sprint', /sprint/i],
  ['run', /(run|jog)/i],
  ['walk', /(walk|step|patrol_walk)/i],
  ['crouch', /(crouch|_cr_)/i],
  ['idle', /(idle|breath|loop$)/i],
];
const ORDERED = ['locomotion: idle', 'walk', 'run', 'sprint', 'strafe', 'turn', 'start_stop', 'jump', 'crouch', 'prone_crawl', 'climb_vault', 'combat', 'hit_react', 'death', 'fall', 'getup', 'sit_lie', 'hostage_emote', 'animal', 'reference', 'other'];
function nameGuess(name) {
  for (const [c, re] of NAME_RULES) if (re.test(name)) return c;
  return null;
}
function nameTags(name) {
  const t = [];
  if (/pistol/i.test(name)) t.push('pistol'); if (/rifle|w2_/i.test(name)) t.push('rifle');
  if (/crouch|_cr_/i.test(name)) t.push('crouch'); if (/aim/i.test(name)) t.push('aim');
  if (/_rm$/i.test(name)) t.push('root_motion'); if (/loop/i.test(name)) t.push('loop');
  const dir = name.match(/(135L|135R|45L|45R|90L|90R|_B_?|_F_?|Bwd|Fwd|Lt|Rt|_FL_|_FR_|_BL_|_BR_|_L_|_R_|Left|Right)/i); if (dir) t.push('dir:' + dir[1].replace(/_/g, '').toLowerCase());
  return t;
}

// ---------- per-clip analysis ----------
function analyze(file, text, meta) {
  const sha = createHash('sha256').update(text).digest('hex');
  const motionText = text.slice(text.indexOf('MOTION'));
  const motionSha = createHash('sha256').update(motionText.replace(/^Frame Time:.*$/m, '')).digest('hex');
  const bvh = parseBVH(text);
  const rec = { file, name: meta?.name ?? basename(file, '.bvh'), frames: bvh.frames, frames_parsed: bvh.values.length, fps: r3(1 / bvh.frameTime),
    duration_s: r3(bvh.frames * bvh.frameTime), bones: bvh.joints.length, rig_assignment: meta?.rig_assignment ?? null, sha256: sha, sha256_matches_library: meta?.sha256 ? meta.sha256 === sha : null,
    motion_sha256: motionSha, flags: [], notes: [] };
  if (bvh.values.length !== bvh.frames || bvh.values.some(r => r.length !== bvh.values[0].length || r.some(v => !Number.isFinite(v)))) rec.flags.push('malformed_motion');
  const names = bvh.joints.map(j => stripPrefix(j.name));
  rec.rig_family = rigFamily(names);
  rec.hierarchy = createHash('sha1').update(names.join(' ')).digest('hex').slice(0, 8);

  // unit: leg length should be 0.6-1.3 m
  const fk0 = forward({ ...bvh, values: bvh.values.slice(0, 1) }, 1);
  const bonesFrame0 = names.map((nm, i) => ({ name: nm, parent: bvh.joints[i].parent, pos: wp(fk0, 0, i) }));
  const native = buildRigMap(bonesFrame0);
  const aliased = rec.rig_family === 'autorigpro' ? buildRigMap(bonesFrame0.map(b => ({ ...b, name: arpAlias(b.name) }))) : native;
  const aliasNames = rec.rig_family === 'autorigpro' ? names.map(arpAlias) : names;
  const map = new Map([...aliased.fromCanonical].map(([k, v]) => [k, aliasNames.indexOf(v)]));
  rec.rigmap_native = { confidence: native.confidence, missing: native.missing, mapped: native.fromCanonical.size };
  rec.rigmap = { confidence: aliased.confidence, missing: aliased.missing, mapped: aliased.fromCanonical.size, alias_layer: rec.rig_family === 'autorigpro' ? 'autorigpro' : null,
    canonical: Object.fromEntries([...aliased.fromCanonical].map(([k, v]) => [k, names[aliasNames.indexOf(v)]])) };
  const I = k => map.get(k);
  const legLen0 = ['l', 'r'].map(s => I('calf_' + s) !== undefined && I('foot_' + s) !== undefined ? len(lt(fk0, 0, I('calf_' + s))) + len(lt(fk0, 0, I('foot_' + s))) : NaN);
  const rawLeg = mean(legLen0);
  let unit = 1;
  for (const u of [1, 0.01, 0.1, 0.001, 100]) if (rawLeg * u > 0.45 && rawLeg * u < 1.4) { unit = u; break; }
  rec.unit_to_m = unit;
  const fk = forward(bvh, unit), n = fk.n, fps = 1 / bvh.frameTime;
  if (!fk.finite) rec.flags.push('nonfinite');
  const need = ['pelvis', 'thigh_l', 'calf_l', 'foot_l', 'thigh_r', 'calf_r', 'foot_r'];
  const missingReq = need.filter(k => I(k) === undefined);
  if (missingReq.length) { rec.flags.push('unmapped_legs'); rec.notes.push('missing ' + missingReq.join(',')); return rec; }
  const leg = rawLeg * unit;
  rec.leg_length_m = r3(leg);
  rec.thigh_m = r3(mean(['l', 'r'].map(s => len(lt(fk0, 0, I('calf_' + s))) * unit)));
  if (n < 4) { rec.flags.push('too_short'); rec.notes.push('pose only, ' + n + ' frames'); }

  const pel = I('pelvis'), ft = { l: I('foot_l'), r: I('foot_r') }, ball = { l: I('ball_l') ?? I('foot_l'), r: I('ball_r') ?? I('foot_r') };
  const Y = (f, i) => fk.P[(f * fk.nj + i) * 3 + 1];
  // ground: the world floor (Y=0) when the feet reach it, else the clip's own low-foot level
  const mappedIdx = [...new Set(map.values())];
  const lows = []; for (let f = 0; f < n; f++) lows.push(Math.min(Y(f, ft.l), Y(f, ft.r), Y(f, ball.l), Y(f, ball.r)));
  const lowAll = []; for (let f = 0; f < n; f++) { let m = Infinity; for (const i of mappedIdx) m = Math.min(m, Y(f, i)); lowAll.push(m); }
  const floorP = pct(lows, 2);
  const ground = Math.abs(floorP) < 0.05 ? 0 : floorP;
  rec.ground_y_m = r3(ground); rec.floor_p2_m = r3(floorP);
  if (ground !== 0) rec.flags.push('floor_offset');
  // W in facts.js layout (n*nb*16, translation at 12..14), positions only, metres (upm = 1)
  const W = new Float64Array(n * fk.nj * 16);
  for (let k = 0; k < n * fk.nj; k++) { W[k * 16] = W[k * 16 + 5] = W[k * 16 + 10] = W[k * 16 + 15] = 1; W[k * 16 + 12] = fk.P[k * 3]; W[k * 16 + 13] = fk.P[k * 3 + 1]; W[k * 16 + 14] = fk.P[k * 3 + 2]; }
  const legs = legsOf(Object.fromEntries(map));
  const fg = estimateGround({ W, nb: fk.nj, n, fps, upm: 1, legs });
  rec.fulcrum_ground_m = r3(fg.ground);

  // root / pelvis travel
  let path = 0;
  const hx = f => wp(fk, f, pel);
  for (let f = 1; f < n; f++) { const a = hx(f - 1), b = hx(f); path += Math.hypot(b[0] - a[0], b[2] - a[2]); }
  const p0 = hx(0), p1 = hx(n - 1), net = Math.hypot(p1[0] - p0[0], p1[2] - p0[2]), dur = Math.max(1, n - 1) / fps;
  const pY = Array.from({ length: n }, (_, f) => Y(f, pel) - ground);
  rec.root = { path_m: r3(path), net_m: r3(net), speed_ms: r3(net / dur), path_speed_ms: r3(path / dur),
    pelvis_y_min_m: r3(Math.min(...pY)), pelvis_y_max_m: r3(Math.max(...pY)), vertical_range_m: r3(Math.max(...pY) - Math.min(...pY)), pelvis_height_ratio: r2(median(pY) / (leg + 0.08)) };

  // feet: Fulcrum's own plant detector (wire/plants.js, Biped Rule A), as the judge's foot_slide warnings use it
  const plants = n >= 4 ? detectFootPlants({ W, nb: fk.nj, n, fps, upm: 1, ground: fg.ground, legs, legLength_cm: leg * 100 }) : [];
  const vel = (f, i) => { const a = wp(fk, Math.max(0, f - 1), i), b = wp(fk, Math.min(n - 1, f + 1), i), k = (Math.min(n - 1, f + 1) - Math.max(0, f - 1)) / fps || 1; return [(b[0] - a[0]) / k, (b[1] - a[1]) / k, (b[2] - a[2]) / k]; };
  const hgt = {};
  for (const s of ['l', 'r']) { const h = []; for (let f = 0; f < n; f++) h.push(Math.min(Y(f, ft[s]), Y(f, ball[s])) - ground); const h0 = pct(h, 5); hgt[s] = h.map(v => v - h0); }
  // treadmill: in-place clips move planted feet backward at the locomotion speed
  const plantVel = [];
  for (const p of plants) if (!p.touch) for (let f = p.a; f <= p.b; f++) plantVel.push(vel(f, p.point === 'ball' ? ball[p.side] : ft[p.side]));
  const tread = plantVel.length >= 3 ? [median(plantVel.map(v => v[0])), 0, median(plantVel.map(v => v[2]))] : [0, 0, 0];
  const treadSpeed = Math.hypot(tread[0], tread[2]);
  const inPlace = treadSpeed > 0.25 && rec.root.speed_ms < 0.35 * treadSpeed;
  const stance = plants.filter(p => !p.touch), sliding = stance.filter(p => p.sliding), touches = plants.filter(p => p.touch);
  const stanceFrames = stance.reduce((a, p) => a + p.b - p.a + 1, 0), slideFrames = sliding.reduce((a, p) => a + p.b - p.a + 1, 0);
  const onsets = { l: stance.filter(p => p.side === 'l').map(p => p.a), r: stance.filter(p => p.side === 'r').map(p => p.a) };
  const periods = [];
  for (const s of ['l', 'r']) { const o = onsets[s]; for (let k = 1; k < o.length; k++) periods.push((o[k] - o[k - 1]) / fps); }
  const steps = stance.map(p => [p.a, p.side]).sort((x, y) => x[0] - y[0]), half = [];
  for (let k = 1; k < steps.length; k++) if (steps[k][1] !== steps[k - 1][1]) half.push((steps[k][0] - steps[k - 1][0]) / fps);
  if (!periods.length && half.length) periods.push(2 * median(half));
  // airborne: no plant on either foot, both feet 6 cm above their own lowest level, and no other body part within 10% leg length of the floor
  const planted = new Uint8Array(n); for (const p of plants) planted.fill(1, p.a, p.b + 1);
  const air = Array.from({ length: n }, (_, f) => !planted[f] && hgt.l[f] > 0.06 && hgt.r[f] > 0.06 && lowAll[f] - ground > 0.1 * leg);
  const airRuns = runsOf(air, 2);
  rec.feet = { contacts_l: onsets.l.length, contacts_r: onsets.r.length, contacts_total: onsets.l.length + onsets.r.length, touches: touches.length,
    duty_l: r2(stance.filter(p => p.side === 'l').reduce((a, p) => a + p.b - p.a + 1, 0) / n), duty_r: r2(stance.filter(p => p.side === 'r').reduce((a, p) => a + p.b - p.a + 1, 0) / n),
    step_period_s: r3(median(periods)), airborne_frames: air.filter(Boolean).length, longest_air_frames: airRuns.reduce((m, r) => Math.max(m, r[1] - r[0] + 1), 0),
    treadmill_speed_ms: r3(treadSpeed), in_place: inPlace, sliding_plants: sliding.length, stance_plants: stance.length, skate_frac: r3(stanceFrames ? slideFrames / stanceFrames : 0),
    worst_plant_drift_cm: r2(Math.max(0, ...stance.map(p => p.drift_cm))), median_plant_drift_cm: r2(median(stance.map(p => p.drift_cm))) };
  const locoSpeed = inPlace ? treadSpeed : rec.root.speed_ms;
  rec.locomotion_speed_ms = r3(locoSpeed);

  // facing (feet point forward: ankle -> ball), heading change, travel vs facing
  const facing = f => { let x = 0, z = 0; for (const s of ['l', 'r']) { if (I('ball_' + s) === undefined) continue; const d = sub(wp(fk, f, I('ball_' + s)), wp(fk, f, ft[s])); x += d[0]; z += d[2]; } const sl = I('upperarm_l') ?? I('thigh_l'), sr = I('upperarm_r') ?? I('thigh_r'), h = sub(wp(fk, f, sl), wp(fk, f, sr)); return { foot: Math.atan2(x, z) * D, hip: [h[0], h[2]] }; };
  // body heading from the shoulders (the aim/chest direction; the pelvis turns into strafes), sign chosen to agree with the feet
  const hipYaw = [], footYaw = [];
  for (let f = 0; f < n; f++) { const fc = facing(f); footYaw.push(fc.foot); hipYaw.push(Math.atan2(-fc.hip[1], fc.hip[0]) * D); }
  const agree = mean(hipYaw.map((h, f) => Math.cos((h - footYaw[f]) / D)));
  const yaw = hipYaw.map(h => agree < 0 ? wrap(h + 180) : h);
  let turn = 0; for (let f = 1; f < n; f++) turn += wrap(yaw[f] - yaw[f - 1]);
  const travelDir = inPlace ? Math.atan2(-tread[0], -tread[2]) * D : Math.atan2(p1[0] - p0[0], p1[2] - p0[2]) * D;
  const meanYaw = Math.atan2(mean(yaw.map(a => Math.sin(a / D))), mean(yaw.map(a => Math.cos(a / D)))) * D;
  rec.heading = { turn_deg: r2(turn), travel_vs_facing_deg: locoSpeed > 0.25 ? r2(wrap(travelDir - meanYaw)) : null };

  // angles: knee/elbow flexion, hip flexion envelope; per frame features for near-dup search
  const has = k => I(k) !== undefined;
  const feat = [], knee = [], elbow = [], hipflex = [], trunk = [];
  for (let f = 0; f < n; f++) {
    const P = k => wp(fk, f, I(k)), row = [];
    for (const s of ['l', 'r']) {
      const kf = 180 - angAt(P('thigh_' + s), P('calf_' + s), P('foot_' + s)); knee.push(kf); row.push(kf);
      const ef = has('lowerarm_' + s) && has('hand_' + s) ? 180 - angAt(P('upperarm_' + s), P('lowerarm_' + s), P('hand_' + s)) : NaN; elbow.push(ef); row.push(ef);
      const top = has('neck_01') ? P('neck_01') : has('head') ? P('head') : null;
      const up = top ? sub(top, P('pelvis')) : [0, 1, 0];
      const hf = 180 - angV(up, sub(P('calf_' + s), P('thigh_' + s))); hipflex.push(hf); row.push(hf);
      const sh = has('upperarm_' + s) && has('lowerarm_' + s) ? angV(sub(P('pelvis'), top ?? P('pelvis')), sub(P('lowerarm_' + s), P('upperarm_' + s))) : NaN; row.push(sh);
    }
    const top = has('neck_01') ? P('neck_01') : P('head'); const tilt = angV(sub(top, P('pelvis')), [0, 1, 0]); trunk.push(tilt); row.push(tilt);
    row.push(((Y(f, pel) - ground) / (leg + 0.08)) * 90);
    feat.push(row);
  }
  // quadruped-like stance: both hands near the floor while the pelvis is high (animal clips retargeted to a biped)
  let quad = 0;
  if (has('hand_l') && has('hand_r')) for (let f = 0; f < n; f++) if (Y(f, I('hand_l')) - ground < 0.25 * leg && Y(f, I('hand_r')) - ground < 0.25 * leg && Y(f, pel) - ground > 0.6 * leg) quad++;
  rec.quadruped_frac = r2(quad / n);
  rec.trunk_tilt_median = r2(median(trunk));
  rec.angles = { knee_flex_max: r2(Math.max(...knee.filter(Number.isFinite))), knee_flex_min: r2(Math.min(...knee.filter(Number.isFinite))), elbow_flex_max: r2(Math.max(...elbow.filter(Number.isFinite))),
    hip_flex_max: r2(Math.max(...hipflex.filter(Number.isFinite))), hip_ext_max: r2(-Math.min(...hipflex.filter(Number.isFinite))), trunk_tilt_max: r2(Math.max(...trunk)) };
  rec.depthsign_limit_frames = { knee_hyper: knee.filter(v => v < -LIMITS.kneeFlex).length, elbow_over: elbow.filter(v => v > LIMITS.elbow).length,
    hip_flex_over: hipflex.filter(v => v > LIMITS.hipSag[1]).length, hip_ext_over: hipflex.filter(v => v < LIMITS.hipSag[0]).length };

  // angular speed and rapid rotations on major mapped bones
  const major = [...map].filter(([k]) => !FINGER.test(k) && !/twist|weapon/.test(k));
  let maxW = 0, maxWBone = null, rapid = 0, flips = 0, firstRapid = null, motionRange = 0;
  for (const [k, i] of major) {
    let rng = 0; const q0 = lq(fk, 0, i);
    for (let f = 1; f < n; f++) {
      const st = qang(lq(fk, f - 1, i), lq(fk, f, i)), w = st * fps;
      if (w > maxW) { maxW = w; maxWBone = k; }
      if (st > TH.rapid_step_deg) { rapid++; if (firstRapid === null) firstRapid = { frame: f, bone: k, deg: r2(st) }; }
      if (st > TH.flip_step_deg) flips++;
      rng = Math.max(rng, qang(q0, lq(fk, f, i)));
    }
    motionRange = Math.max(motionRange, rng);
  }
  rec.max_joint_angular_speed_dps = r2(maxW); rec.max_angular_speed_bone = maxWBone;
  rec.rapid_rotation_steps = rapid; rec.flip_steps = flips; rec.first_rapid = firstRapid; rec.pose_range_deg = r2(motionRange);
  rec.sidecar_max_adjacent_world_rotation_deg = meta?.max_adjacent_world_rotation_degrees != null ? r2(meta.max_adjacent_world_rotation_degrees) : null;

  // teleports and joint speed
  let maxStep = 0, maxStepF = null, maxJS = 0;
  for (let f = 1; f < n; f++) {
    const d = len(sub(wp(fk, f, pel), wp(fk, f - 1, pel)));
    if (d > maxStep) { maxStep = d; maxStepF = f; }
    for (const [, i] of major) maxJS = Math.max(maxJS, len(sub(wp(fk, f, i), wp(fk, f - 1, i))) * fps);
  }
  rec.pelvis_max_step_m = r3(maxStep); rec.max_joint_speed_ms = r2(maxJS);

  // bone-length drift (position channels can stretch bones)
  let worstDrift = 0, worstBone = null;
  for (const [k, i] of map) {
    if (k === 'pelvis' || FINGER.test(k)) continue;
    const ls = []; for (let f = 0; f < n; f++) ls.push(len(lt(fk, f, i)));
    const m = mean(ls); if (!(m > 0.03)) continue;
    const d = (Math.max(...ls) - Math.min(...ls)) / m;
    if (d > worstDrift) { worstDrift = d; worstBone = k; }
  }
  rec.bone_length_drift = r3(worstDrift); rec.bone_length_drift_bone = worstBone;

  // penetration
  const sink = lowAll.map(v => ground - v);
  rec.penetration = { frames: sink.filter(v => v > TH.sink_cm / 100).length, max_cm: r2(Math.max(0, ...sink) * 100) };

  // Fulcrum's own phase segmentation on this clip
  try {
    const pmap = { spine_03: I('spine_02') ?? I('spine_01'), spine_05: I('spine_03') ?? I('spine_02') ?? I('neck_01') };
    const idx = PHASE_BONES.map(b => pmap[b] ?? I(b) ?? (b.startsWith('ball') ? I('foot' + b.slice(4)) : b === 'clavicle_l' ? I('upperarm_l') : b === 'clavicle_r' ? I('upperarm_r') : I('neck_01') ?? I('head')));
    if (idx.every(i => i !== undefined) && n >= 6) {
      const B = []; for (let f = 0; f < n; f++) B.push(idx.map(i => { const p = wp(fk, f, i); return [p[0] * 100, -p[2] * 100, (p[1] - ground) * 100]; }));
      const rest = idx.map((_, k) => [0, 0, k === 0 ? (leg + 0.08) * 100 : 0]);
      const segs = segmentPhases({ fps, B, rest, frame_offset: 0 });
      const tally = {}; for (const s of segs) tally[s.phase] = (tally[s.phase] ?? 0) + s.end - s.start + 1;
      rec.fulcrum_phases = segs.map(s => `${s.phase}:${s.start}-${s.end}`);
      rec.fulcrum_phase_major = Object.entries(tally).sort((a, b) => b[1] - a[1])[0]?.[0] ?? null;
    }
  } catch (e) { rec.notes.push('phases failed: ' + e.message); }

  // motion category guess
  const hr = rec.root.pelvis_height_ratio, airF = rec.feet.longest_air_frames, rise = rec.root.pelvis_y_max_m - Math.min(pY[0], pY[n - 1]);
  let mg;
  if (n < 4 || motionRange < TH.static_deg) mg = 'reference';
  else if (hr < 0.4 && trunk.filter(t => t > 55).length > n / 2) mg = (locoSpeed > 0.15 ? 'prone_crawl' : median(pY.slice(-5)) < 0.35 * leg ? 'lying' : 'prone_crawl');
  else if (pY[n - 1] < 0.45 * (leg + 0.08) && pY[0] > 0.8 * (leg + 0.08)) mg = 'fall_or_death';
  else if (pY[0] < 0.5 * (leg + 0.08) && pY[n - 1] > 0.8 * (leg + 0.08)) mg = 'getup';
  else if (airF >= 4 && rise > 0.08) mg = 'jump';
  else if (Math.abs(turn) > 60 && locoSpeed < 0.6) mg = 'turn';
  else if (locoSpeed > 0.25) {
    const tv = Math.abs(rec.heading.travel_vs_facing_deg ?? 0), crouch = hr < 0.82;
    mg = crouch ? 'crouch' : tv > 30 ? 'strafe' : locoSpeed > 4.2 ? 'sprint' : locoSpeed > 2.0 ? 'run' : 'walk';
  } else if (hr < 0.8) mg = 'crouch';
  else mg = 'idle';
  rec.motion_guess = mg;
  const ng = nameGuess(rec.name);
  rec.name_guess = ng;
  rec.category = ng ?? (mg === 'lying' ? 'sit_lie' : mg === 'fall_or_death' ? 'death' : mg);
  rec.tags = nameTags(rec.name);
  if (inPlace) rec.tags.push('in_place'); else if (locoSpeed > 0.25) rec.tags.push('root_travel');
  const gaitName = ['walk', 'run', 'sprint', 'strafe'].includes(ng), gaitMotion = ['walk', 'run', 'sprint', 'strafe', 'crouch'].includes(mg);
  if (ng && gaitName && !gaitMotion) rec.flags.push('name_motion_mismatch');
  if (ng === 'idle' && ['walk', 'run', 'sprint'].includes(mg) && locoSpeed > 0.6) rec.flags.push('name_motion_mismatch');

  // quality flags
  if (maxStep > TH.teleport_m_per_frame || maxJS > TH.joint_speed_max_ms) rec.flags.push('teleport');
  if (flips > 0) rec.flags.push('rotation_flip');
  else if (rapid >= TH.rapid_steps_min) rec.flags.push('rapid_rotation');
  if (worstDrift > TH.drift_bad) rec.flags.push('bone_length_drift');
  else if (worstDrift > TH.drift_warn) rec.flags.push('bone_length_drift_minor');
  if (rec.feet.skate_frac > TH.skate_frac && !['prone_crawl', 'death', 'fall', 'getup', 'sit_lie', 'animal'].includes(rec.category)) rec.flags.push(inPlace ? 'in_place_treadmill' : 'foot_skate');
  if (rec.penetration.frames > 0) rec.flags.push('penetration');
  if (motionRange < TH.static_deg && n >= 4) rec.flags.push('static');
  if (rec.quadruped_frac > 0.5) rec.flags.push('quadruped_pose');
  if (rec.depthsign_limit_frames.knee_hyper > 0) rec.flags.push('knee_hyperextension');
  rec._feat = feat;
  return rec;
}

// ---------- near-duplicates ----------
const MIRROR_IDX = [4, 5, 6, 7, 0, 1, 2, 3, 8, 9];
function featDiff(a, b, mirror = false) {
  let s = 0, c = 0;
  for (let f = 0; f < a.length; f++) for (let k = 0; k < a[f].length; k++) {
    const x = a[f][k], y = b[f][mirror ? MIRROR_IDX[k] : k];
    if (Number.isFinite(x) && Number.isFinite(y)) { s += Math.abs(x - y); c++; }
  }
  return c ? s / c : Infinity;
}

// ---------- main ----------
const animDir = existsSync(join(LIB, 'Animations')) ? join(LIB, 'Animations') : LIB;
const metaIndex = existsSync(join(LIB, 'library.json')) ? JSON.parse(readFileSync(join(LIB, 'library.json'), 'utf8')) : { clips: [] };
const metaBy = new Map((metaIndex.clips ?? []).map(c => [c.file, c]));
const files = readdirSync(animDir).filter(f => f.toLowerCase().endsWith('.bvh')).sort().slice(0, LIMIT);
mkdirSync(OUT, { recursive: true });
const clips = [];
for (const f of files) {
  const text = readFileSync(join(animDir, f), 'utf8');
  let side = null; const sp = join(animDir, f.replace(/\.bvh$/i, '.json'));
  if (existsSync(sp)) { try { side = JSON.parse(readFileSync(sp, 'utf8')); } catch { side = null; } }
  const meta = { ...(metaBy.get(f) ?? {}), ...(side ? { max_adjacent_world_rotation_degrees: side.max_adjacent_world_rotation_degrees, fractional_tail_seconds: side.fractional_tail_seconds } : {}) };
  let rec;
  try { rec = analyze(f, text, meta); } catch (e) { rec = { file: f, name: meta.name ?? f, flags: ['parse_error'], notes: [String(e.message)] }; }
  if (side?.fractional_tail_seconds > 1e-3) rec.notes.push(`partial final interval omitted: ${side.fractional_tail_seconds.toFixed(4)} s`);
  clips.push(rec);
}

// duplicates
const bySha = new Map(), byMotion = new Map();
for (const c of clips) {
  c.duplicate_of = null; c.duplicate_kind = null; c.near_duplicates = [];
  if (!c.sha256) continue;
  if (bySha.has(c.sha256)) { c.duplicate_of = bySha.get(c.sha256).file; c.duplicate_kind = 'identical_file'; continue; }
  bySha.set(c.sha256, c);
  if (byMotion.has(c.motion_sha256)) { c.duplicate_of = byMotion.get(c.motion_sha256).file; c.duplicate_kind = 'identical_motion'; continue; }
  byMotion.set(c.motion_sha256, c);
}
const byLen = new Map();
for (const c of clips) if (c._feat && c.frames >= 4 && !c.duplicate_of) { const k = c.frames; if (!byLen.has(k)) byLen.set(k, []); byLen.get(k).push(c); }
for (const group of byLen.values()) for (let a = 0; a < group.length; a++) for (let b = a + 1; b < group.length; b++) {
  const A = group[a], Bc = group[b];
  const d = featDiff(A._feat, Bc._feat), m = featDiff(A._feat, Bc._feat, true);
  const best = Math.min(d, m);
  const ta = A.heading?.travel_vs_facing_deg, tb = Bc.heading?.travel_vs_facing_deg;
  const sameTravel = (ta == null && tb == null) || (ta != null && tb != null && Math.abs(wrap(ta - tb)) < TH.near_dup_travel_deg);
  if (best < TH.near_dup_deg && !sameTravel && m >= d) { A.same_legs_other_travel = [...(A.same_legs_other_travel ?? []), Bc.file]; Bc.same_legs_other_travel = [...(Bc.same_legs_other_travel ?? []), A.file]; continue; }
  if (best < TH.near_dup_deg) {
    const kind = m < d ? 'mirror' : (Math.abs((A.root?.net_m ?? 0) - (Bc.root?.net_m ?? 0)) > 0.2 ? 'root_motion_variant' : A.rig_family !== Bc.rig_family ? 'cross_rig' : 'same_pose');
    Bc.near_duplicates.push({ of: A.file, kind, mean_angle_diff_deg: r2(best) });
    A.near_duplicates.push({ of: Bc.file, kind, mean_angle_diff_deg: r2(best) });
    if (!Bc.duplicate_of && kind !== 'mirror') { Bc.duplicate_of = A.file; Bc.duplicate_kind = 'near_' + kind; }
  }
}
for (const c of clips) delete c._feat;
for (const c of clips) c.usable = !c.flags.some(f => ['parse_error', 'malformed_motion', 'nonfinite', 'unmapped_legs', 'too_short', 'static', 'teleport', 'rotation_flip'].includes(f)) && c.category !== 'animal' && c.category !== 'reference';

// ---------- summary ----------
const unique = clips.filter(c => !c.duplicate_of);
const cnt = (arr, key) => { const m = {}; for (const x of arr) { const k = typeof key === 'function' ? key(x) : x[key]; m[k] = (m[k] ?? 0) + 1; } return m; };
const lines = [];
const P = s => lines.push(s);
const totalFrames = clips.reduce((a, c) => a + (c.frames ?? 0), 0);
P('# Motion library index');
P('');
P(`Source: ${files.length} BVH files, ${totalFrames} frames. Generated by tools/motionlib_index.mjs.`);
P('');
P('## Overview');
P('');
P(`- Unique clips (no identical file, identical motion or near-duplicate): ${unique.length} of ${clips.length}`);
P(`- Exact duplicates (same file bytes): ${clips.filter(c => c.duplicate_kind === 'identical_file').length}; identical motion with different header: ${clips.filter(c => c.duplicate_kind === 'identical_motion').length}; near duplicates: ${clips.filter(c => c.duplicate_kind?.startsWith('near_')).length}`);
P(`- Usable for motion statistics (unique, mapped, not too short/static/teleporting/flipping, not animal or reference): ${unique.filter(c => c.usable).length} clips, ${unique.filter(c => c.usable).reduce((a, c) => a + c.frames, 0)} frames`);
P(`- Rig families: ${Object.entries(cnt(clips, 'rig_family')).map(([k, v]) => `${k} ${v}`).join(', ')}; rig assignment: ${Object.entries(cnt(clips, 'rig_assignment')).map(([k, v]) => `${k} ${v}`).join(', ')}`);
P(`- FPS: ${Object.entries(cnt(clips, 'fps')).map(([k, v]) => `${k} fps: ${v}`).join(', ')}`);
P(`- Units: ${Object.entries(cnt(clips, 'unit_to_m')).map(([k, v]) => `x${k} -> m: ${v}`).join(', ')}; leg length (m) median ${r3(median(clips.map(c => c.leg_length_m)))}, range ${r3(pct(clips.map(c => c.leg_length_m), 0))}-${r3(pct(clips.map(c => c.leg_length_m), 100))}`);
P('');
P('## Categories (unique clips)');
P('');
P('| category | unique | usable | all files | examples |');
P('|---|---|---|---|---|');
const cats = [...new Set(clips.map(c => c.category))].sort((a, b) => (ORDERED.indexOf(a) + 99) % 99 - (ORDERED.indexOf(b) + 99) % 99);
for (const k of cats) {
  const u = unique.filter(c => c.category === k);
  P(`| ${k} | ${u.length} | ${u.filter(c => c.usable).length} | ${clips.filter(c => c.category === k).length} | ${[...new Set(u.map(c => c.name))].slice(0, 6).join(', ')} |`);
}
P('');
P('Category = name rule when one matches, otherwise the motion guess. Motion guess vs name rule agreement (unique clips with both):');
P('');
const both = unique.filter(c => c.name_guess && c.motion_guess);
const agreeSet = { idle: ['idle', 'turn'], walk: ['walk', 'strafe', 'crouch'], run: ['run', 'sprint', 'strafe', 'walk'], sprint: ['sprint', 'run'], strafe: ['strafe', 'walk', 'run', 'crouch', 'sprint'], crouch: ['crouch'], prone_crawl: ['prone_crawl', 'lying'], death: ['fall_or_death', 'lying', 'prone_crawl'], jump: ['jump'], turn: ['turn', 'idle', 'crouch'] };
P('| name rule | n | motion guesses |');
P('|---|---|---|');
for (const k of Object.keys(agreeSet)) { const g = both.filter(c => c.name_guess === k); if (!g.length) continue; const ok = g.filter(c => agreeSet[k].includes(c.motion_guess)).length; P(`| ${k} | ${g.length} | ${Object.entries(cnt(g, 'motion_guess')).map(([a, b]) => `${a} ${b}`).join(', ')} (agree ${ok}) |`); }
P('');
P('## Locomotion coverage (unique, usable)');
P('');
const U = unique.filter(c => c.usable);
const loco = U.filter(c => ['walk', 'run', 'sprint', 'strafe', 'crouch'].includes(c.category) && c.locomotion_speed_ms > 0.25);
const dirBin = a => a == null ? 'n/a' : Math.abs(a) < 22.5 ? 'F' : Math.abs(a) > 157.5 ? 'B' : a > 0 ? (a < 67.5 ? 'FL45' : a < 112.5 ? 'L90' : 'BL135') : (a > -67.5 ? 'FR45' : a > -112.5 ? 'R90' : 'BR135');
P('Direction is travel relative to facing, measured (sign convention follows the X-reflected basis; L/R may be swapped relative to names).');
P('');
P('| category | clips | in place | root travel | speed m/s (min/median/max) | measured directions |');
P('|---|---|---|---|---|---|');
for (const k of ['walk', 'run', 'sprint', 'strafe', 'crouch']) {
  const g = loco.filter(c => c.category === k); if (!g.length) continue;
  const sp = g.map(c => c.locomotion_speed_ms);
  P(`| ${k} | ${g.length} | ${g.filter(c => c.feet.in_place).length} | ${g.filter(c => !c.feet.in_place).length} | ${r2(Math.min(...sp))} / ${r2(median(sp))} / ${r2(Math.max(...sp))} | ${Object.entries(cnt(g, c => dirBin(c.heading.travel_vs_facing_deg))).map(([a, b]) => `${a} ${b}`).join(', ')} |`);
}
P('');
const have = k => U.filter(c => c.category === k).length;
P(`Coverage check: idle ${have('idle')}, walk ${have('walk')}, run ${have('run')}, sprint ${have('sprint')}, strafe ${have('strafe')}, turn ${have('turn')}, start/stop ${have('start_stop')}, jump ${have('jump')}, crouch ${have('crouch')}, prone/crawl ${have('prone_crawl')}, climb/vault ${have('climb_vault')}, combat ${have('combat')}, hit ${have('hit_react')}, death ${have('death')}, fall ${have('fall')}, get-up ${have('getup')}.`);
P('');
P('### Gait timing vs mocap/gait.js');
P('');
const gaitRows = loco.filter(c => c.feet.step_period_s > 0 && ['walk', 'run', 'sprint'].includes(c.category) && c.heading.travel_vs_facing_deg != null && Math.abs(c.heading.travel_vs_facing_deg) < 30);
for (const k of ['walk', 'run', 'sprint']) {
  const g = gaitRows.filter(c => c.category === k); if (!g.length) continue;
  const duty = median(g.map(c => (c.feet.duty_l + c.feet.duty_r) / 2)), cyc = median(g.map(c => c.feet.step_period_s)), v = median(g.map(c => c.locomotion_speed_ms));
  P(`- ${k}: ${g.length} forward clips, median speed ${r2(v)} m/s, median foot-cycle ${r2(cyc)} s (gait.js cycleTime(${r2(v)}) = ${r2(cycleTime(v))} s), median duty ${r2(duty)} (gait.js ${k === 'walk' ? GAIT.walk.duty : GAIT.run.duty}).`);
}
P('');
P('## Quality');
P('');
P('| flag | all files | unique | examples |');
P('|---|---|---|---|');
const allFlags = [...new Set(clips.flatMap(c => c.flags))].sort();
for (const fl of allFlags) { const u = unique.filter(c => c.flags.includes(fl)); P(`| ${fl} | ${clips.filter(c => c.flags.includes(fl)).length} | ${u.length} | ${u.slice(0, 6).map(c => c.name).join(', ')} |`); }
P('');
P(`Thresholds: teleport > ${TH.teleport_m_per_frame} m pelvis step per frame or a joint > ${TH.joint_speed_max_ms} m/s; rapid rotation = a major bone local step > ${TH.rapid_step_deg} deg in one frame (flip > ${TH.flip_step_deg}); bone drift = (max-min)/mean local bone length > ${TH.drift_bad} (minor > ${TH.drift_warn}); skate = more than ${TH.skate_frac * 100}% of stance frames in plants that wire/plants.js marks sliding (in-place clips get in_place_treadmill instead); penetration = any joint > ${TH.sink_cm} cm below the ground; knee hyperextension = knee flexion < -${LIMITS.kneeFlex} deg (depthsign LIMITS.kneeFlex).`);
P('');
const worst = (key, k = 8) => [...unique].filter(c => Number.isFinite(c[key])).sort((a, b) => b[key] - a[key]).slice(0, k).map(c => `${c.name} (${c[key]})`).join(', ');
P(`- Highest local angular speed (deg/s): ${worst('max_joint_angular_speed_dps')}`);
P(`- Worst bone-length drift: ${worst('bone_length_drift')}`);
P(`- Most rapid-rotation steps: ${worst('rapid_rotation_steps')}`);
P('');
P('## Unusable or duplicated');
P('');
const bad = clips.filter(c => !c.usable && !c.duplicate_of);
P(`Unusable unique clips (${bad.length}): ` + bad.map(c => `${c.name} [${c.category}; ${c.flags.filter(f => !['penetration', 'foot_skate', 'bone_length_drift_minor', 'rapid_rotation', 'knee_hyperextension', 'name_motion_mismatch', 'floor_offset', 'bone_length_drift'].includes(f)).join(',') || c.category}]`).join('; '));
P('');
P('Duplicate groups (kept file <- duplicates):');
P('');
const groups = new Map();
for (const c of clips) if (c.duplicate_of) { if (!groups.has(c.duplicate_of)) groups.set(c.duplicate_of, []); groups.get(c.duplicate_of).push(c); }
const fileName = new Map(clips.map(c => [c.file, c.name]));
for (const [k, v] of groups) P(`- ${fileName.get(k)} (${k}) <- ${v.map(c => `${c.name} [${c.duplicate_kind}]`).join(', ')}`);
P('');
const mir = clips.filter(c => c.near_duplicates.some(d => d.kind === 'mirror'));
const slo = clips.filter(c => c.same_legs_other_travel?.length && !c.duplicate_of);
P(`Joint-angle curves within ${TH.near_dup_deg} deg of another clip but a different travel direction (kept: directional variants of one cycle, e.g. pelvis yaw and leg aim re-authored; useful as matched sets): ${slo.length} clips, e.g. ${slo.slice(0, 10).map(c => `${c.name} ~ ${c.same_legs_other_travel.map(f => fileName.get(f)).join('/')}`).join('; ')}`);
P('');
P(`Mirror pairs (kept as separate clips): ${[...new Set(mir.map(c => c.name))].join(', ') || 'none'}`);
P('');
P('## Skeleton mapping (wire/rigmap.js)');
P('');
P('| rig family | files | native rigmap confidence (median) | with alias layer | canonical bones present | missing of WANTED |');
P('|---|---|---|---|---|---|');
for (const fam of [...new Set(clips.map(c => c.rig_family))]) {
  const g = clips.filter(c => c.rig_family === fam && c.rigmap);
  const present = [...new Set(g.flatMap(c => Object.keys(c.rigmap.canonical)))];
  P(`| ${fam} | ${g.length} | ${median(g.map(c => c.rigmap_native.confidence))} (missing ${[...new Set(g.flatMap(c => c.rigmap_native.missing))].join(' ') || 'none'}) | ${median(g.map(c => c.rigmap.confidence))} | ${present.length} | ${[...new Set(g.flatMap(c => c.rigmap.missing))].join(' ') || 'none'} |`);
}
P('');
const famPresent = fam => new Set(clips.filter(c => c.rig_family === fam && c.rigmap).flatMap(c => Object.keys(c.rigmap.canonical)));
P('facts.js RIG_NAMES coverage: ' + RIG_NAMES.map(k => `${k}: ${[...new Set(clips.map(c => c.rig_family))].filter(f => famPresent(f).has(k)).join('/') || 'MISSING'}`).join('; '));
P('');
P('Not present in any rig: spine_04, spine_05 (the library spines stop at spine_03), weapon/prop bones, toe/ball on Mixamo comes from ToeBase. Fingers: 3 joints each on every rig.');
P('');
P('## Fulcrum phase segmentation (phases.js) on library clips');
P('');
P('| category | clips | majority phase from phases.js |');
P('|---|---|---|');
for (const k of cats) { const g = U.filter(c => c.category === k && c.fulcrum_phase_major); if (!g.length) continue; P(`| ${k} | ${g.length} | ${Object.entries(cnt(g, 'fulcrum_phase_major')).sort((a, b) => b[1] - a[1]).map(([a, b]) => `${a} ${b}`).join(', ')} |`); }
P('');
P('## Depth-sign anatomical limits (mocap/depthsign.js LIMITS) vs library');
P('');
const lf = k => U.filter(c => c.depthsign_limit_frames?.[k] > 0);
P(`Clips with frames outside limits: knee flexion < -${LIMITS.kneeFlex}: ${lf('knee_hyper').length}; elbow > ${LIMITS.elbow}: ${lf('elbow_over').length}; hip flexion > ${LIMITS.hipSag[1]}: ${lf('hip_flex_over').length}; hip extension beyond ${LIMITS.hipSag[0]}: ${lf('hip_ext_over').length}.`);
P(`Library envelopes (usable clips): knee flex max ${r2(Math.max(...U.map(c => c.angles.knee_flex_max)))} deg (median of clip maxima ${r2(median(U.map(c => c.angles.knee_flex_max)))}); knee flex min ${r2(Math.min(...U.map(c => c.angles.knee_flex_min)))}; elbow max ${r2(Math.max(...U.map(c => c.angles.elbow_flex_max)))}; hip flex max ${r2(Math.max(...U.map(c => c.angles.hip_flex_max)))}; hip ext max ${r2(Math.max(...U.map(c => c.angles.hip_ext_max)))}.`);
P('');

writeFileSync(join(OUT, 'index.json'), JSON.stringify({ generated_by: 'tools/motionlib_index.mjs', thresholds: TH, clips }, null, 1));
writeFileSync(join(OUT, 'summary.md'), lines.join('\n') + '\n');
console.log(`indexed ${clips.length} clips -> ${join(OUT, 'index.json')}, ${join(OUT, 'summary.md')}`);
