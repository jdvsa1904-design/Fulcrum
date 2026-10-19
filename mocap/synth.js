import { add, sub, scale, dot, cross, norm, mid } from './math.js';
import { NB, NLM, LM, fk, getV, landmarksFromPose, lmAt } from './skeleton.js';
import { IMG_STRIDE } from './landmarks.js';

export function lookAtCamera(pos, target, o = {}) {
  const f = norm(sub(target, pos)), r = norm(cross(f, [0, 1, 0])), dn = cross(f, r);
  const width = o.width ?? 1280, height = o.height ?? 720, hfov = o.hfov ?? 60;
  return { pos, f, r, dn, width, height, hfov, focal: width / 2 / Math.tan(hfov * Math.PI / 360) };
}

export const toCam = (cam, p) => { const d = sub(p, cam.pos); return [dot(d, cam.r), dot(d, cam.dn), dot(d, cam.f)]; };

export function toReconstructionFrame(cam, p) {
  const fh = norm([cam.f[0], 0, cam.f[2]]), rh = norm([cam.r[0], 0, cam.r[2]]), d = [p[0] - cam.pos[0], p[1], p[2] - cam.pos[2]];
  return [d[0] * rh[0] + d[2] * rh[2], d[1], -(d[0] * fh[0] + d[2] * fh[2])];
}

export function synthCapture(skel, take, cam, o = {}) {
  const rand = o.rand ?? Math.random, gauss = () => { let u = 0, v = 0; while (!u) u = rand(); v = rand(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const noiseImg = o.noiseImg ?? 0, noiseWorld = o.noiseWorld ?? 0, noiseDepth = o.noiseDepth ?? noiseWorld * 2, fps = take.fps ?? o.fps ?? 30;
  const frames = [], tmp = { pos: new Float64Array(NB * 3), q: new Float64Array(NB * 4) }, truth = [];
  for (let i = 0; i < take.n; i++) {
    fk(skel, getV(take.root, i), take.rot.subarray(i * NB * 4, (i + 1) * NB * 4), tmp);
    const L = landmarksFromPose(skel, tmp);
    truth.push(L);
    const C = Array.from({ length: NLM }, (_, j) => toCam(cam, lmAt(L, j))), hip = mid(C[LM.lHip], C[LM.rHip]);
    const img = new Float32Array(NLM * IMG_STRIDE), world = new Float32Array(NLM * 3);
    const facing = norm(sub(C[LM.lSh], C[LM.rSh]));
    for (let j = 0; j < NLM; j++) {
      const c = C[j];
      img[j * IMG_STRIDE] = (c[0] / c[2] * cam.focal + cam.width / 2) / cam.width + gauss() * noiseImg;
      img[j * IMG_STRIDE + 1] = (c[1] / c[2] * cam.focal + cam.height / 2) / cam.height + gauss() * noiseImg * cam.width / cam.height;
      img[j * IMG_STRIDE + 2] = (c[2] - hip[2]) / 2;
      const side = [1, 2, 3, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31].includes(j) ? 1 : j === 0 ? 0 : -1;
      img[j * IMG_STRIDE + 3] = o.occlusion ? Math.max(0.2, 0.95 - 0.5 * Math.max(0, side * facing[2])) : 0.95;
      img[j * IMG_STRIDE + 4] = 0.99;
      world[j * 3] = c[0] - hip[0] + gauss() * noiseWorld;
      world[j * 3 + 1] = c[1] - hip[1] + gauss() * noiseWorld;
      world[j * 3 + 2] = c[2] - hip[2] + gauss() * noiseDepth;
    }
    const poses = [{ img, world }];
    if (o.extraPerson) poses.push(o.extraPerson(i));
    frames.push({ t: i / fps, poses: o.shuffle && i % 2 ? poses.reverse() : poses });
  }
  return { capture: { version: 1, video: { name: o.name ?? 'synthetic', width: cam.width, height: cam.height, fps, duration: take.n / fps, frames: take.n }, model: 'synthetic', frames }, truth };
}

export function staticPerson(skel, cam, at, o = {}) {
  const local = o.local, root = add(at, [0, skel.rest[0][1], 0]), W = fk(skel, root, local), L = landmarksFromPose(skel, W);
  const C = Array.from({ length: NLM }, (_, j) => toCam(cam, lmAt(L, j))), hip = mid(C[LM.lHip], C[LM.rHip]);
  const img = new Float32Array(NLM * IMG_STRIDE), world = new Float32Array(NLM * 3);
  for (let j = 0; j < NLM; j++) {
    const c = C[j];
    img.set([(c[0] / c[2] * cam.focal + cam.width / 2) / cam.width, (c[1] / c[2] * cam.focal + cam.height / 2) / cam.height, 0, 0.9, 0.99], j * IMG_STRIDE);
    world.set(scale(sub(c, hip), 1), j * 3);
  }
  return () => ({ img, world });
}
