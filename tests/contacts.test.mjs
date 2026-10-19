import test from 'node:test';
import assert from 'node:assert/strict';
import { csv, near, nearVec, v3, q4, rng, deg, rad } from './helpers.mjs';
import * as C from '../contacts.js';

const CSV = 'C:/re_work/hik/results/floor_random.csv';
const rows = csv(CSV);
const OPT = { bottom: 8, front: 0, floor: 0 };
const qs = (r, p) => [r[p + 'qx'], r[p + 'qy'], r[p + 'qz'], r[p + 'qw']];
const qErr = (a, b) => deg(q4.angle(a, b));

test('floor_random.csv smooth model matches measured HumanIK output', () => {
  assert.equal(rows.length, 120);
  let pm = 0, ps = 0, rm = 0;
  for (const r of rows) {
    const out = C.floorContactSmooth({ p: [10, r.ay, 0], q: [r.qx, r.qy, r.qz, r.qw] }, OPT, false);
    const e = v3.dist(out.p, [r.ax, r.ay_out, r.az]);
    pm = Math.max(pm, e); ps += e;
    rm = Math.max(rm, qErr(out.q, [r.oqx, r.oqy, r.oqz, r.oqw]));
  }
  console.log(`smooth pos mean ${(ps / rows.length).toFixed(5)} max ${pm.toFixed(5)} cm, rot max ${rm.toFixed(5)} deg`);
  assert.ok(ps / rows.length < 0.01, `mean ${ps / rows.length}`);
  assert.ok(pm < 0.05, `max pos ${pm}`);
  assert.ok(rm < 0.03, `max rot ${rm}`);
});

test('floorContactStiff on the CSV poses lifts vertically to the floor and keeps rotation', () => {
  let lifted = 0, worst = 0;
  for (const r of rows) {
    const a = { p: [10, r.ay, 0], q: [r.qx, r.qy, r.qz, r.qw] };
    const out = C.floorContactStiff(a, OPT);
    nearVec(out.q, a.q, 1e-12);
    near(out.p[0], 10, 1e-12);
    near(out.p[2], 0, 1e-12);
    const mk = C.footMarkers(out, OPT), lo = Math.min(...Object.values(mk).map(p => p[1]));
    worst = Math.max(worst, Math.abs(lo - Math.max(0, Math.min(...Object.values(C.footMarkers(a, OPT)).map(p => p[1])))));
    if (out.pen > 0) { lifted++; near(lo, 0, 1e-9); }
  }
  assert.ok(lifted > 20, 'lifted ' + lifted);
  assert.ok(worst < 1e-9);
});

const qEul = (pitch, roll, yaw) => q4.mul(q4.axis([0, 1, 0], rad(yaw)), q4.mul(q4.axis([0, 0, 1], rad(roll)), q4.axis([1, 0, 0], rad(pitch))));
const ankleAt = (y, pitch, roll, yaw) => ({ p: [10, y, 0], q: qEul(pitch, roll, yaw) });
const toeFor = (ankle, local = [0, -6, 13]) => ({ p: v3.add(ankle.p, q4.rot(ankle.q, local)), q: ankle.q.slice() });
const minY = (mk, keys) => Math.min(...keys.map(k => mk[k][1]));
const FRONT = ['frontIn', 'frontOut'];
const BACKMID = ['backIn', 'backOut', 'midIn', 'midOut'];
const poseRng = r => ankleAt(2 + r() * 12, (r() - 0.5) * 70, (r() - 0.5) * 50, (r() - 0.5) * 60);

test('footMarkers places the six markers in the ankle frame, mirrored per side and scaled by contact size', () => {
  const ankle = { p: [1, 20, 3], q: [0, 0, 0, 1] };
  const m = C.footMarkers(ankle, { bottom: 8 });
  nearVec(m.backIn, [1 - 5, 12, 3 - 4.5], 1e-12);
  nearVec(m.backOut, [1 + 5, 12, 3 - 4.5], 1e-12);
  nearVec(m.midIn, [1 - 5, 12, 3 + 13], 1e-12);
  nearVec(m.midOut, [1 + 5, 12, 3 + 13], 1e-12);
  nearVec(m.frontIn, [1 - 5, 12, 3 + 20], 1e-12);
  nearVec(m.frontOut, [1 + 5, 12, 3 + 20], 1e-12);
  const r = C.footMarkers(ankle, { bottom: 8, side: -1 });
  nearVec(r.backIn, [1 + 5, 12, 3 - 4.5], 1e-12);
  nearVec(r.midOut, [1 - 5, 12, 3 + 13], 1e-12);
  const big = C.footMarkers(ankle, { bottom: 8, contactSize: 200 });
  nearVec(big.frontOut, [1 + 10, 20 - 16, 3 + 40], 1e-12);
});

test('footMarkers rotates markers with the ankle and keeps front markers rigid to a rotated toe', () => {
  const r = rng(5);
  for (let i = 0; i < 20; i++) {
    const q = q4.axis([r() - 0.5, r() - 0.5, r() - 0.5], r() * 3);
    const ankle = { p: [r() * 10, r() * 10, r() * 10], q };
    const m = C.footMarkers(ankle, {});
    nearVec(m.midOut, v3.add(ankle.p, q4.rot(q, [5, -7.5, 13])), 1e-12);
    nearVec(m.backIn, v3.add(ankle.p, q4.rot(q, [-5, -7.5, -4.5])), 1e-12);
  }
  const ankle = { p: [0, 20, 0], q: [0, 0, 0, 1] }, toe = { p: [0, 14, 13], q: q4.axis([1, 0, 0], rad(30)), q0: [0, 0, 0, 1] };
  const base = C.footMarkers(ankle, {}), bent = C.footMarkers(ankle, {}, toe);
  nearVec(bent.midIn, base.midIn, 1e-12);
  near(v3.dist(bent.frontIn, toe.p), v3.dist(base.frontIn, toe.p), 1e-12);
  nearVec(bent.frontIn, v3.add(toe.p, q4.rot(toe.q, v3.sub(base.frontIn, toe.p))), 1e-12);
});

test('floorContactStiff lifts straight up by the deepest marker and keeps rotation', () => {
  const r = rng(21);
  let lifted = 0;
  for (let i = 0; i < 60; i++) {
    const a = poseRng(r);
    const out = C.floorContactStiff(a, { floor: 0, bottom: 8 });
    nearVec(out.q, a.q, 1e-12);
    near(out.p[0], a.p[0], 1e-12);
    near(out.p[2], a.p[2], 1e-12);
    const mk = C.footMarkers(out, { bottom: 8 });
    const lo = minY(mk, Object.keys(mk));
    assert.ok(lo >= -1e-9, `still penetrating ${lo}`);
    if (out.pen > 1e-9) { lifted++; near(lo, 0, 1e-9); near(out.p[1] - a.p[1], out.pen, 1e-12); }
    else near(out.p[1], a.p[1], 1e-12);
  }
  assert.ok(lifted > 10);
});

test('floorContactStiff honours the floor height and contactType 1-3 ignoring the front markers', () => {
  const a = ankleAt(9, 35, 0, 0);
  const t0 = C.floorContactStiff(a, { bottom: 8, floor: 0 }), t1 = C.floorContactStiff(a, { bottom: 8, floor: 0, contactType: 1 });
  assert.ok(t0.pen > 0 && t0.pen > t1.pen + 1e-6);
  const raised = C.floorContactStiff({ p: [10, 9 + 5, 0], q: a.q }, { bottom: 8, floor: 5 });
  near(raised.pen, t0.pen, 1e-9);
  const high = C.floorContactStiff(ankleAt(30, 0, 0, 0), { bottom: 8 });
  assert.equal(high.pen, 0);
  nearVec(high.p, [10, 30, 0], 1e-12);
});

test('floorContactSmooth never mutates its input and is a no-op on a clear foot', () => {
  const a = ankleAt(30, 10, 5, 5), snap = JSON.stringify(a);
  const out = C.floorContactSmooth(a, { bottom: 8 }, false);
  assert.equal(JSON.stringify(a), snap);
  nearVec(out.p, a.p, 1e-12);
  nearVec(out.q, a.q, 1e-12);
  const low = ankleAt(2, 20, 0, 0), s2 = JSON.stringify(low);
  C.floorContactSmooth(low, { bottom: 8 }, false);
  assert.equal(JSON.stringify(low), s2);
});

test('floorContactSmooth leaves the back and mid markers on the floor within tolerance', () => {
  const r = rng(77);
  let worst = 0;
  for (let i = 0; i < 80; i++) {
    const out = C.floorContactSmooth(poseRng(r), { bottom: 8, front: 0 }, false);
    const mk = C.footMarkers(out, { bottom: 8, front: 0 });
    worst = Math.min(worst, minY(mk, BACKMID));
    near(Math.hypot(...out.q), 1, 1e-12);
  }
  assert.ok(worst > -0.5, `worst ${worst}`);
});

test('contact size scales every marker distance, front included', () => {
  const a = ankleAt(5, 25, 8, 10);
  const doubled = { contactSize: 100, back: 9, middle: 26, front: 14, inn: 10, out: 10, bottom: 15 };
  const big = C.floorContactSmooth(a, { contactSize: 200 }, false), manual = C.floorContactSmooth(a, doubled, false);
  nearVec(big.p, manual.p, 1e-9);
  nearVec(big.q, manual.q, 1e-9);
  near(C.floorContactStiff(a, { contactSize: 200 }).pen, C.floorContactStiff(a, doubled).pen, 1e-9);
});

test('floorContact stiffness blend: 0 is smooth, 1 is stiff, 0.5 is the midpoint', () => {
  const o = { bottom: 8 };
  const r = rng(9);
  for (let i = 0; i < 20; i++) {
    const ankle = ankleAt(3 + r() * 7, 10 + r() * 25, (r() - 0.5) * 30, (r() - 0.5) * 40);
    const sm = C.floorContactSmooth(ankle, o, false), st = C.floorContactStiff(ankle, o);
    const s0 = C.floorContact({ ankle }, { ...o, stiffness: 0 }), s1 = C.floorContact({ ankle }, { ...o, stiffness: 1 }), sh = C.floorContact({ ankle }, { ...o, stiffness: 0.5 });
    nearVec(s0.ankle.p, sm.p, 1e-12);
    nearVec(s0.ankle.q, sm.q, 1e-12);
    nearVec(s1.ankle.p, st.p, 1e-12);
    nearVec(s1.ankle.q, st.q, 1e-12);
    nearVec(sh.ankle.p, v3.mul(v3.add(sm.p, st.p), 0.5), 1e-12);
    near(qErr(sh.ankle.q, sm.q), qErr(sh.ankle.q, st.q), 1e-6);
    near(qErr(sh.ankle.q, sm.q) * 2, qErr(sm.q, st.q), 1e-6);
    near(Math.hypot(...sh.ankle.q), 1, 1e-12);
    assert.equal(s0.toe, null);
  }
  const a = ankleAt(4, 20, 0, 0);
  nearVec(C.floorContact({ ankle: a }, { ...o, stiffness: -3 }).ankle.p, C.floorContact({ ankle: a }, { ...o, stiffness: 0 }).ankle.p, 1e-12);
  nearVec(C.floorContact({ ankle: a }, { ...o, stiffness: 7 }).ankle.p, C.floorContactStiff(a, o).p, 1e-12);
});

test('floorContact reports penetration, and touched is false for a clear foot', () => {
  const hit = C.floorContact({ ankle: ankleAt(5, 20, 0, 0) }, { bottom: 8 });
  assert.ok(hit.touched && hit.penetration > 0);
  const clear = C.floorContact({ ankle: ankleAt(40, 20, 0, 0) }, { bottom: 8 });
  assert.ok(!clear.touched);
  near(clear.penetration, 0, 1e-12);
  nearVec(clear.ankle.p, [10, 40, 0], 1e-12);
});

test('pivot ankle restores the ankle x and z, pivot toes restores the toe x and z', () => {
  const r = rng(31);
  for (let i = 0; i < 15; i++) {
    const ankle = ankleAt(3 + r() * 6, 8 + r() * 25, (r() - 0.5) * 40, (r() - 0.5) * 50), toe = toeFor(ankle);
    const base = { bottom: 8, toeBend: false };
    for (const s of [0, 0.5, 1]) {
      const auto = C.floorContact({ ankle, toe }, { ...base, stiffness: s, pivot: 'auto' });
      const pa = C.floorContact({ ankle, toe }, { ...base, stiffness: s, pivot: 'ankle' });
      const pt = C.floorContact({ ankle, toe }, { ...base, stiffness: s, pivot: 'toes' });
      near(pa.ankle.p[0], ankle.p[0], 1e-9);
      near(pa.ankle.p[2], ankle.p[2], 1e-9);
      near(pa.ankle.p[1], auto.ankle.p[1], 1e-9);
      nearVec(pt.ankle.q, auto.ankle.q, 1e-12);
      if (s !== 0.5) {
        near(pt.toe.p[0], toe.p[0], 1e-9);
        near(pt.toe.p[2], toe.p[2], 1e-9);
      }
      if (s === 0) assert.ok(Math.abs(auto.ankle.p[0] - ankle.p[0]) + Math.abs(auto.ankle.p[2] - ankle.p[2]) > 1e-3);
    }
  }
});

test('pivot toes without a toe node is a no-op', () => {
  const ankle = ankleAt(4, 20, 5, 10);
  const a = C.floorContact({ ankle }, { bottom: 8, pivot: 'toes' }), b = C.floorContact({ ankle }, { bottom: 8, pivot: 'auto' });
  nearVec(a.ankle.p, b.ankle.p, 1e-12);
});

test('toe bend puts the lowest front marker on the floor when it penetrates', () => {
  const r = rng(101);
  let bent = 0, untouched = 0;
  for (let i = 0; i < 80; i++) {
    const ankle = ankleAt(3 + r() * 9, (r() - 0.3) * 60, (r() - 0.5) * 40, (r() - 0.5) * 50), toe = toeFor(ankle);
    for (const s of [0, 0.5]) {
      const o = { bottom: 8, stiffness: s };
      const flat = C.floorContact({ ankle, toe }, { ...o, toeBend: false }), out = C.floorContact({ ankle, toe }, o);
      nearVec(out.ankle.p, flat.ankle.p, 1e-12);
      nearVec(out.toe.p, flat.toe.p, 1e-12);
      const pre = C.footMarkers(flat.ankle, o, { p: flat.toe.p, q: flat.toe.q, q0: flat.toe.q });
      const post = C.footMarkers(out.ankle, o, { p: out.toe.p, q: out.toe.q, q0: flat.toe.q });
      if (minY(pre, FRONT) < 0) {
        bent++;
        near(minY(post, FRONT), 0, 1e-3, `pose ${i} s ${s}`);
        assert.ok(out.toeAngle !== undefined);
        if (Math.abs(out.toeAngle) > 1e-6) {
          const axis = v3.unit(q4.rot(out.ankle.q, [1, 0, 0])), dq = q4.mul(out.toe.q, q4.conj(flat.toe.q));
          near(Math.abs(v3.dot(v3.unit(dq.slice(0, 3)), axis)), 1, 1e-9);
          near(qErr(out.toe.q, flat.toe.q), Math.abs(deg(out.toeAngle)), 1e-6);
        }
      } else {
        untouched++;
        assert.equal(out.toeAngle, undefined);
        near(qErr(out.toe.q, flat.toe.q), 0, 1e-9);
      }
    }
  }
  assert.ok(bent > 10 && untouched > 5, `${bent} ${untouched}`);
});

test('toeBend solves a hand-built case: penetrating tip bends up until it touches', () => {
  const ankle = { p: [0, 8, 0], q: [0, 0, 0, 1] };
  const toe = { p: [0, 2, 13], q: [0, 0, 0, 1] };
  const o = { bottom: 8, floor: 1, front: 7 };
  const res = C.toeBend(toe, ankle, o);
  const mk = C.footMarkers(ankle, o, { p: toe.p, q: res.q, q0: toe.q });
  near(minY(mk, FRONT), 1, 1e-6);
  assert.ok(res.angle < 0);
  const clear = C.toeBend({ p: [0, 10, 13], q: [0, 0, 0, 1] }, { p: [0, 16, 0], q: [0, 0, 0, 1] }, o);
  assert.equal(clear.angle, 0);
  nearVec(clear.q, [0, 0, 0, 1], 1e-12);
  const touching = C.toeBend(toe, ankle, { bottom: 8, floor: 0, front: 7 });
  assert.equal(touching.angle, 0);
});

test('toeBend takes the solution nearest the unbent toe and survives an unreachable floor', () => {
  const ankle = { p: [0, 8, 0], q: [0, 0, 0, 1] }, toe = { p: [0, 2, 13], q: [0, 0, 0, 1] };
  const res = C.toeBend(toe, ankle, { bottom: 8, front: 7, floor: 1 });
  assert.ok(Math.abs(res.angle) < Math.PI / 4);
  const far = C.toeBend({ p: [0, -40, 13], q: [0, 0, 0, 1] }, ankle, { bottom: 8, front: 7, floor: 1 });
  assert.ok(Number.isFinite(far.angle));
  near(Math.hypot(...far.q), 1, 1e-12);
});

test('kneeFloor lifts by penetration of the knee radius above the floor, weighted', () => {
  const k = C.kneeFloor([1, 3, 2], { radius: 5 });
  near(k.penetration, 2, 1e-12);
  near(k.lift, 2, 1e-12);
  nearVec(k.p, [1, 5, 2], 1e-12);
  const w = C.kneeFloor([1, 3, 2], { radius: 5, weight: 0.5, floor: 1 });
  near(w.penetration, 3, 1e-12);
  near(w.lift, 1.5, 1e-12);
  nearVec(w.p, [1, 4.5, 2], 1e-12);
  const clear = C.kneeFloor([0, 9, 0], { radius: 5 });
  assert.equal(clear.lift, 0);
  nearVec(clear.p, [0, 9, 0], 1e-12);
  nearVec(C.liftOnPenetration([1, 2, 3], 4), [1, 6, 3], 1e-12);
  nearVec(C.liftOnPenetration([1, 2, 3], 4, [1, 0, 0]), [5, 2, 3], 1e-12);
});

test('lowestPoint picks the smallest y', () => {
  const l = C.lowestPoint({ a: [0, 3, 0], b: [1, -2, 0], c: [2, 5, 0] });
  assert.equal(l.name, 'b');
  assert.equal(l.y, -2);
  assert.equal(C.lowestPoint({}), null);
});

test('rollSegments follows the AdvancedSkeleton network', () => {
  let s = C.rollSegments(-10);
  near(s.heel, -10, 1e-12); near(s.toes, 0, 1e-12); near(s.toesEnd, 0, 1e-12);
  s = C.rollSegments(15, 30, 60);
  near(s.heel, 0, 1e-12); near(s.a, 0.5, 1e-12); near(s.b, 0, 1e-12); near(s.toes, 7.5, 1e-12); near(s.toesEnd, 0, 1e-12);
  s = C.rollSegments(30, 30, 60);
  near(s.toes, 30, 1e-12); near(s.toesEnd, 0, 1e-12);
  s = C.rollSegments(45, 30, 60);
  near(s.a, 1, 1e-12); near(s.b, 0.5, 1e-12); near(s.toes, 22.5, 1e-12); near(s.toesEnd, 22.5, 1e-12);
  s = C.rollSegments(90, 30, 60);
  near(s.toes, 0, 1e-12); near(s.toesEnd, 90, 1e-12);
  for (const r of [20, 31, 44, 49, 50, 61, 100]) {
    const t = C.rollSegments(r, 20, 50);
    near(t.heel + t.toes + t.toesEnd, r, 1e-9, `r ${r}`);
  }
  near(C.rollSegments(5, 20, 50).toes, 5 * 5 / 20, 1e-12);
  near(C.rollSegments(-30, 20, 50).heel, -30, 1e-12);
  near(C.rollSegments(10, 30, 30).b, 0, 1e-12);
  near(C.rollSegments(40, 30, 30).b, 1, 1e-12);
});

test('rockSegments splits rock into inner and outer clamps', () => {
  assert.deepEqual(C.rockSegments(-50), { inner: -50, outer: 0 });
  assert.deepEqual(C.rockSegments(70), { inner: 0, outer: 70 });
  assert.deepEqual(C.rockSegments(500), { inner: 0, outer: 200 });
  assert.deepEqual(C.rockSegments(-500), { inner: -200, outer: 0 });
});

test('rollFoot keeps rigid distances and pins the active pivot', () => {
  const pts = { heel: [0, 0, 0], ankle: [0, 8, -2], ball: [0, 0, 13], tip: [0, 0, 20], side: [4, 1, 6] };
  const pivots = { heel: [0, 0, 0], ball: [0, 0, 13], toe: [0, 0, 20] };
  const names = Object.keys(pts);
  for (const r of [-25, -5, 10, 15, 30, 45, 60, 75, 90]) {
    const out = C.rollFoot(pts, r, pivots);
    for (let i = 0; i < names.length; i++) for (let j = i + 1; j < names.length; j++)
      near(v3.dist(out.points[names[i]], out.points[names[j]]), v3.dist(pts[names[i]], pts[names[j]]), 1e-9, `r ${r}`);
  }
  nearVec(C.rollFoot(pts, -10, pivots).points.heel, pts.heel, 1e-12);
  nearVec(C.rollFoot(pts, 15, pivots).points.ball, pts.ball, 1e-12);
  nearVec(C.rollFoot(pts, 90, pivots).points.tip, pts.tip, 1e-12);
  nearVec(C.rollFoot(pts, 0, pivots).points.ankle, pts.ankle, 1e-12);
  const heelLift = C.rollFoot(pts, 15, pivots);
  assert.ok(heelLift.points.heel[1] > 0.5, 'positive roll lifts the heel');
  near(heelLift.toes, 7.5, 1e-12);
  const back = C.rollFoot(pts, -10, pivots);
  assert.ok(back.points.tip[1] < -0.5 || back.points.tip[1] > 0.5);
  near(back.heel, -10, 1e-12);
  const quarter = C.rollFoot({ p: [0, 0, 23] }, 15, pivots, { start: 15, end: 30 });
  near(quarter.toes, 15, 1e-12);
  near(v3.dist(quarter.points.p, [0, 0, 13]), 10, 1e-12);
  near(deg(Math.atan2(-quarter.points.p[1], quarter.points.p[2] - 13)), 15, 1e-9);
});

const track = frames => Float64Array.from(frames.flat());
const stanceSwingStance = () => {
  const f = [];
  for (let i = 0; i < 10; i++) f.push([0, 0, 0]);
  for (let k = 1; k <= 8; k++) f.push([0.5 * k, 0, 0]);
  for (let i = 0; i < 10; i++) f.push([4.5, 0, 0]);
  return f;
};
const runsOf = r => r.runs.map(x => [x.s, x.e]);

test('detectPlants Rule A finds stances, footprint frame and drops swing', () => {
  const r = C.detectPlants(track(stanceSwingStance()), {});
  assert.deepEqual(r.runs.map(x => [x.s, x.e, x.mid]), [[0, 9, 4], [18, 27, 22]]);
  for (let i = 0; i < 28; i++) assert.equal(r.planted[i], i <= 9 || i >= 18 ? 1 : 0, `frame ${i}`);
  assert.ok(r.runs.every(x => x.sliding === false));
});

test('detectPlants threshold scales with leg length', () => {
  const t = track(stanceSwingStance());
  assert.equal(C.detectPlants(t, { legLength: 10 }).runs.length, 1);
  assert.equal(C.detectPlants(t, { legLength: 1, tol: 0.5 }).runs.length, 1);
  assert.equal(C.detectPlants(t, { legLength: 1, tol: 0.49 }).runs.length, 2);
  assert.equal(C.detectPlants(t, { legLength: 1, tol: 0.4 }).runs.length, 2);
});

test('detectPlants lift above 0.07 S splits a stance, a 1- or 2-frame gap is filled, 3 frames is not', () => {
  const build = lifts => track(Array.from({ length: 30 }, (_, i) => [0, lifts[i] ?? 0, 0]));
  assert.deepEqual(runsOf(C.detectPlants(build({ 10: 0.2, 11: 0.4, 12: 0.6, 13: 0.4, 14: 0.2 }), {})), [[0, 9], [15, 29]]);
  assert.deepEqual(runsOf(C.detectPlants(build({ 10: 0.2, 11: 0.4, 12: 0.2 }), {})), [[0, 9], [13, 29]]);
  const two = C.detectPlants(build({ 10: 0.2, 11: 0.4 }), {});
  assert.deepEqual(runsOf(two), [[0, 29]]);
  assert.ok(two.planted.every(v => v === 1));
  near(two.runs[0].height, 0.4, 1e-12);
  const one = C.detectPlants(build({ 10: 0.2 }), {});
  assert.deepEqual(runsOf(one), [[0, 29]]);
  near(one.runs[0].height, 0.2, 1e-12);
  const small = C.detectPlants(build({ 10: 0.05, 11: 0.1, 12: 0.05 }), {});
  assert.deepEqual(runsOf(small), [[0, 29]]);
  near(small.runs[0].height, 0.1, 1e-12);
});

test('detectPlants drops runs of 3 frames or fewer and keeps runs of 4', () => {
  const build = plateau => {
    const f = [];
    for (let i = 0; i < 12; i++) f.push([0, 0, 0]);
    f.push([0.5, 0, 0], [1, 0, 0], [1.5, 0, 0]);
    for (let i = 0; i < plateau; i++) f.push([2, 0, 0]);
    f.push([2.5, 0, 0], [3, 0, 0], [3.5, 0, 0]);
    for (let i = 0; i < 12; i++) f.push([4, 0, 0]);
    return track(f);
  };
  const t3 = C.detectPlants(build(3), {});
  assert.deepEqual(runsOf(t3), [[0, 11], [21, 32]]);
  assert.equal(t3.planted.slice(12, 21).reduce((a, b) => a + b, 0), 0);
  const t4 = C.detectPlants(build(4), {});
  assert.deepEqual(t4.runs.map(x => [x.s, x.e, x.mid]), [[0, 11, 5], [15, 18, 16], [22, 33, 27]]);
  assert.deepEqual(runsOf(C.detectPlants(build(2), {})), [[0, 11], [20, 31]]);
  assert.equal(C.detectPlants(build(3), { minRun: 2 }).runs.length, 3);
});

test('detectPlants vertical gate rejects a still foot held in the air', () => {
  const f = [];
  for (let i = 0; i < 10; i++) f.push([0, 0, 0]);
  f.push([0, 0.4, 0]);
  for (let i = 0; i < 10; i++) f.push([0, 0.8, 0]);
  const t = track(f);
  assert.deepEqual(runsOf(C.detectPlants(t, {})), [[0, 20]]);
  const gated = C.detectPlants(t, { verticalTol: 0.2, floor: 0 });
  assert.deepEqual(runsOf(gated), [[0, 9]]);
  assert.equal(gated.planted[15], 0);
  assert.deepEqual(runsOf(C.detectPlants(t, { verticalTol: 0.2, floor: 0.8 })), [[11, 20]]);
});

test('detectPlants flags sliding by horizontal distance and by heading', () => {
  const creep = step => track(Array.from({ length: 14 }, (_, i) => [i * step, 0, 0]));
  assert.equal(C.detectPlants(creep(0.05), {}).runs[0].sliding, true);
  assert.equal(C.detectPlants(creep(0.02), {}).runs[0].sliding, false);
  assert.equal(C.detectPlants(creep(0.02), { footLength: 0.4 }).runs[0].sliding, true);
  assert.equal(C.detectPlants(creep(0.05), { slidingDist: 100 }).runs[0].sliding, false);
  const still = track(Array.from({ length: 14 }, () => [1, 0, 2]));
  const head = d => Float64Array.from({ length: 14 }, (_, i) => rad(d) * i / 13);
  assert.equal(C.detectPlants(still, { heading: head(15), slidingAngle: 10 }).runs[0].sliding, true);
  assert.equal(C.detectPlants(still, { heading: head(5), slidingAngle: 10 }).runs[0].sliding, false);
  assert.equal(C.detectPlants(still, { heading: head(1), slidingAngle: 0 }).runs[0].sliding, true);
  assert.equal(C.detectPlants(still, { heading: new Float64Array(14) }).runs[0].sliding, false);
  const wrapped = Float64Array.from({ length: 14 }, (_, i) => i % 2 ? rad(179) : rad(-179));
  assert.equal(C.detectPlants(still, { heading: wrapped, slidingAngle: 10 }).runs[0].sliding, false);
});

test('detectPlants uses the maximum height over the run for the footprint', () => {
  const ys = [0, 0.05, 0.1, 0.05, 0, 0.03, 0.06, 0.03, 0];
  const t = C.detectPlants(track(ys.map(y => [0, y, 0])), {});
  assert.equal(t.runs.length, 1);
  near(t.runs[0].height, 0.1, 1e-12);
  assert.equal(t.runs[0].mid, 4);
});

test('ankleLock holds a low steady foot at its first position', () => {
  const f = [];
  for (let i = 0; i < 10; i++) f.push([i % 2 * 0.5, 1, 0]);
  for (let i = 0; i < 5; i++) f.push([20, 10, 0]);
  for (let i = 0; i < 12; i++) f.push([50, 1 + (i % 2) * 0.5, i % 3 * 0.3]);
  const r = C.ankleLock(track(f), {});
  for (let i = 0; i < 27; i++) assert.equal(r.locked[i], i < 10 || i >= 15 ? 1 : 0, `frame ${i}`);
  nearVec(r.anchors[4], [0, 1, 0], 1e-12);
  nearVec(r.anchors[9], [0, 1, 0], 1e-12);
  assert.equal(r.anchors[11], null);
  nearVec(r.anchors[20], [50, 1, 0], 1e-12);
});

test('ankleLock needs the minimum duration and breaks when the perimeter is exceeded', () => {
  const brief = track([[0, 1, 0], [0, 1, 0], [0, 20, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]);
  assert.deepEqual(Array.from(C.ankleLock(brief, {}).locked), [0, 0, 0, 1, 1, 1]);
  assert.deepEqual(Array.from(C.ankleLock(brief, { time: 0.2 }).locked), [0, 0, 0, 0, 0, 0]);
  const drift = track(Array.from({ length: 10 }, (_, i) => [i, 1, 0]));
  const d = C.ankleLock(drift, { perimeter: 3 });
  assert.deepEqual(Array.from(d.locked), [1, 1, 1, 1, 1, 1, 1, 1, 0, 0]);
  nearVec(d.anchors[0], [0, 1, 0], 1e-12);
  nearVec(d.anchors[3], [0, 1, 0], 1e-12);
  nearVec(d.anchors[4], [4, 1, 0], 1e-12);
  nearVec(d.anchors[7], [4, 1, 0], 1e-12);
  const high = track([[0, 4, 0], [0, 4, 0], [0, 4, 0], [0, 4, 0]]);
  assert.equal(C.ankleLock(high, { floorProximity: 3 }).locked.reduce((s, v) => s + v, 0), 0);
  assert.equal(C.ankleLock(high, { floorProximity: 3, floor: 2 }).locked.reduce((s, v) => s + v, 0), 4);
});

test('ankleLock with maxSlide lets the anchor trail a drifting foot by the perimeter', () => {
  const drift = track(Array.from({ length: 12 }, (_, i) => [i, 1, 0]));
  const r = C.ankleLock(drift, { perimeter: 3, maxSlide: 60, fps: 30 });
  assert.ok(r.locked.every(v => v === 1));
  for (let k = 3; k < 12; k++) near(r.anchors[k][0], k - 3, 1e-9, `frame ${k}`);
  for (let k = 0; k < 12; k++) near(r.anchors[k][1], 1, 1e-12);
});

test('ankleLock angular perimeter releases when the foot turns', () => {
  const f = track(Array.from({ length: 12 }, () => [5, 1, 5]));
  const turnAt = k => Float64Array.from(Array.from({ length: 12 }, (_, i) => q4.axis([0, 1, 0], rad(i < k ? 0 : 20))).flat());
  assert.ok(C.ankleLock(f, {}).locked.every(v => v === 1));
  assert.deepEqual(Array.from(C.ankleLock(f, { rot: turnAt(10) }).locked), [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0]);
  assert.ok(C.ankleLock(f, { rot: turnAt(10), angularPerimeter: 30 }).locked.every(v => v === 1));
  const slow = Float64Array.from(Array.from({ length: 12 }, (_, i) => q4.axis([0, 1, 0], rad(i * 3))).flat());
  assert.ok(C.ankleLock(f, { rot: slow, angularPerimeter: 10 }).locked.every(v => v === 1));
  assert.ok(C.ankleLock(f, { rot: slow, angularPerimeter: 5 }).locked.every(v => v === 0));
});

test('recoveryBlend is 1 until release, then a critically damped decay', () => {
  const w = C.recoveryBlend(40, 10, 30, { recoveryTime: 0.15, damping: 5 });
  assert.equal(w.length, 40);
  for (let f = 0; f <= 10; f++) near(w[f], 1, 1e-12);
  for (let f = 11; f < 40; f++) assert.ok(w[f] < w[f - 1] && w[f] >= 0, `frame ${f}`);
  const x = 5 * 3 / (0.15 * 30);
  near(w[13], (1 + x) * Math.exp(-x), 1e-12);
  assert.ok(w[39] < 1e-6);
  assert.ok(C.recoveryBlend(40, 10, 30, { recoveryTime: 0.5 })[20] > w[20]);
  const hard = C.recoveryBlend(20, 5, 30, { recoveryTime: 0 });
  assert.equal(hard[5], 1);
  assert.equal(hard[6], 0);
  assert.ok(C.recoveryBlend(10, 99, 30, {}).every(v => v === 1));
});

test('knee extension detector flags only fully IK-blended frames above the limit', () => {
  const knee = Float64Array.from([10, 20, 30, 50, 70, 50, 30, 20, 10]);
  assert.deepEqual(C.kneeExtensionFlags(knee, null, 60), [4]);
  assert.deepEqual(C.kneeExtensionFlags(knee, Float64Array.from([1, 1, 1, 1, 1, 1, 1, 1, 1]), 60), [4]);
  assert.deepEqual(C.kneeExtensionFlags(knee, Float64Array.from([1, 1, 1, 1, 0.5, 1, 1, 1, 1]), 60), []);
  assert.deepEqual(C.kneeExtensionFlags(knee, null, 100), []);
  assert.deepEqual(C.kneeExtensionFlags(knee, null, 40), [3, 4, 5]);
});

test('knee extension fix clamps at the limit and smooths only the flagged window', () => {
  const knee = Float64Array.from([10, 20, 30, 50, 70, 50, 30, 20, 10]);
  const r = C.kneeExtensionFix(knee, null, 60);
  assert.deepEqual(r.flags, [4]);
  assert.equal(r.knee.length, knee.length);
  for (const v of r.knee) assert.ok(v <= 60 + 1e-12);
  for (const i of [0, 1, 7, 8]) assert.equal(r.knee[i], knee[i]);
  assert.equal(knee[4], 70);
  const expect = [34, 42, 44, 42, 34];
  for (let i = 2; i <= 6; i++) near(r.knee[i], expect[i - 2], 1e-9, `frame ${i}`);
  const none = C.kneeExtensionFix(knee, null, 90);
  assert.deepEqual(Array.from(none.knee), Array.from(knee));
  assert.deepEqual(none.flags, []);
  const half = C.kneeExtensionFix(knee, Float64Array.from([1, 1, 1, 1, 0.5, 1, 1, 1, 1]), 60);
  assert.deepEqual(Array.from(half.knee), Array.from(knee));
  const edge = C.kneeExtensionFix(Float64Array.from([80, 20, 10]), null, 60);
  assert.ok(edge.knee.every(v => v <= 60 && Number.isFinite(v)));
});

test('knee wobble detection needs two close direction reversals above the threshold', () => {
  const ramp = Array.from({ length: 20 }, (_, i) => i * 0.1);
  ramp[4] = 0.4;
  ramp[5] = 0.3;
  const knee = Float64Array.from(ramp);
  const r = C.kneeWobbleRanges(knee, { threshold: 0.02, window: 6 });
  assert.deepEqual(r.events, [4, 5]);
  assert.deepEqual(r.ranges, [{ s: 4, e: 4 }]);
  const sine = Float64Array.from({ length: 80 }, (_, i) => 0.5 * Math.sin(i * Math.PI / 20));
  const c = C.kneeWobbleRanges(sine, { threshold: 0.001, window: 6 });
  assert.deepEqual(c.events, [10, 30, 50, 70]);
  assert.deepEqual(c.ranges, []);
  const dip = Float64Array.from(Array.from({ length: 20 }, (_, i) => i === 8 ? 0.69 : i * 0.1));
  assert.deepEqual(C.kneeWobbleRanges(dip, { threshold: 0.02 }).events, [8]);
  const tight = C.kneeWobbleRanges(dip, { threshold: 0.001 });
  assert.deepEqual(tight.events, [7, 8]);
  assert.deepEqual(tight.ranges, [{ s: 7, e: 7 }]);
  assert.deepEqual(C.kneeWobbleRanges(Float64Array.from(Array.from({ length: 20 }, (_, i) => i * 0.1)), {}).events, []);
  assert.deepEqual(C.kneeWobbleRanges(knee, { threshold: 0.02, window: 1 }).ranges, []);
});

test('knee wobble fix replaces a wobble range with the line between its neighbours', () => {
  const knee = Float64Array.from([0, 0.1, 0.2, 0.3, 0.9, 0.2, 0.5, 0.6]);
  const out = C.kneeWobbleFix(knee, [{ s: 3, e: 5 }]);
  const a = knee[2], b = knee[6];
  for (let i = 3; i <= 5; i++) near(out[i], a + (b - a) * (i - 2) / 4, 1e-12, `frame ${i}`);
  for (const i of [0, 1, 2, 6, 7]) assert.equal(out[i], knee[i]);
  const half = C.kneeWobbleFix(knee, [{ s: 3, e: 5 }], { strength: 0.5 });
  near(half[4], (knee[4] + (a + (b - a) * 0.5)) / 2, 1e-12);
  assert.deepEqual(Array.from(C.kneeWobbleFix(knee, [{ s: 3, e: 5 }], { strength: 0 })), Array.from(knee));
  const pipe = Float64Array.from([0, 0.1, 0.2, 0.3, 0.4, 0.3, 0.4, 0.5, 0.6, 0.7]);
  const det = C.kneeWobbleRanges(pipe, { threshold: 0.02 });
  assert.deepEqual(det.ranges, [{ s: 4, e: 4 }]);
  const fixed = C.kneeWobbleFix(pipe, det.ranges);
  near(fixed[4], 0.3, 1e-12);
  assert.deepEqual(C.kneeWobbleRanges(fixed, { threshold: 0.02 }).ranges, []);
  assert.ok(C.kneeWobbleFix(Float64Array.from([5, 1, 2, 3]), [{ s: 0, e: 1 }]).every(Number.isFinite));
});

const spin = (n, step) => Float64Array.from(Array.from({ length: n }, (_, i) => q4.axis([0.2, 1, 0.3], step * i)).flat());

test('spikeFrames flags a single-sample reversal and not a smooth rotation', () => {
  assert.deepEqual(C.spikeFrames(spin(30, 0.1)), []);
  const t = spin(30, 0.1);
  t.set(q4.axis([0.2, 1, 0.3], 0.2), 5 * 4);
  const f = C.spikeFrames(t);
  assert.ok(f.includes(5), `${f}`);
  assert.ok(f.every(i => i >= 4 && i <= 5), `${f}`);
});

test('spikeFrames ignores quaternion sign flips and static tracks', () => {
  const t = spin(30, 0.1);
  for (const i of [7, 8, 9, 20]) for (let k = 0; k < 4; k++) t[i * 4 + k] = -t[i * 4 + k];
  assert.deepEqual(C.spikeFrames(t), []);
  assert.deepEqual(C.spikeFrames(Float64Array.from(Array.from({ length: 10 }, () => [0, 0, 0, 1]).flat())), []);
  assert.deepEqual(C.spikeFrames(new Float64Array(0)), []);
  assert.deepEqual(C.spikeFrames(Float64Array.from([0, 0, 0, 1, 0, 0, 0, 1])), []);
});

test('spikeFrames flags a velocity reversal', () => {
  const ang = [0, 0.1, 0.2, 0.3, 0.2, 0.1, 0, 0.1, 0.2];
  assert.deepEqual(C.spikeFrames(Float64Array.from(ang.map(a => q4.axis([0, 0, 1], a)).flat())), [3, 6]);
});
