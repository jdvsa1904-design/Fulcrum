import * as THREE from 'three';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { BONES, NB } from './skeleton.js';
import { captureToJSON } from './capture.js';

export function download(data, name, type = 'application/octet-stream') {
  const blob = data instanceof Blob ? data : new Blob([data], { type }), a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 4000);
}

const frameCount = clip => clip.n + (clip.loop ? 1 : 0);

export function mannequinClip(clip, name = clip.name) {
  const n = frameCount(clip), times = new Float32Array(n), tracks = [];
  for (let i = 0; i < n; i++) times[i] = i / clip.fps;
  for (let b = 0; b < NB; b++) {
    const v = new Float32Array(n * 4);
    for (let i = 0; i < n; i++) {
      const s = (i % clip.n) * NB * 4 + b * 4;
      let q = [clip.rot[s], clip.rot[s + 1], clip.rot[s + 2], clip.rot[s + 3]];
      if (i > 0 && q[0] * v[i * 4 - 4] + q[1] * v[i * 4 - 3] + q[2] * v[i * 4 - 2] + q[3] * v[i * 4 - 1] < 0) q = q.map(x => -x);
      v.set(q, i * 4);
    }
    tracks.push(new THREE.QuaternionKeyframeTrack(`${BONES[b]}.quaternion`, times, v));
  }
  const p = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) p.set(clip.root.subarray((i % clip.n) * 3, (i % clip.n) * 3 + 3), i * 3);
  tracks.push(new THREE.VectorKeyframeTrack('Hips.position', times, p));
  return new THREE.AnimationClip(name, (n - 1) / clip.fps, tracks);
}

export function bakeCustomClip(character, clip, name = clip.name) {
  const n = frameCount(clip), times = new Float32Array(n), bones = character.bones, hips = character.setup.dst.slots.Hips;
  const q = bones.map(() => new Float32Array(n * 4)), p = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const s = i % clip.n;
    times[i] = i / clip.fps;
    character.applyPose(clip.root.subarray(s * 3, s * 3 + 3), clip.rot.subarray(s * NB * 4, (s + 1) * NB * 4));
    bones.forEach((b, k) => {
      let v = [b.quaternion.x, b.quaternion.y, b.quaternion.z, b.quaternion.w];
      if (i > 0 && v[0] * q[k][i * 4 - 4] + v[1] * q[k][i * 4 - 3] + v[2] * q[k][i * 4 - 2] + v[3] * q[k][i * 4 - 1] < 0) v = v.map(x => -x);
      q[k].set(v, i * 4);
    });
    p.set([bones[hips].position.x, bones[hips].position.y, bones[hips].position.z], i * 3);
  }
  const tracks = bones.map((b, k) => new THREE.QuaternionKeyframeTrack(`${b.uuid}.quaternion`, times, q[k]));
  tracks.push(new THREE.VectorKeyframeTrack(`${bones[hips].uuid}.position`, times, p));
  return new THREE.AnimationClip(name, (n - 1) / clip.fps, tracks);
}

export async function exportGLB(character, clips) {
  const used = new Set(), named = clips.map(c => {
    let name = (c.name || c.kind || 'Move').replace(/[^\w\- ()]+/g, '').trim() || 'Move', k = 2;
    while (used.has(name)) name = `${c.name} ${k++}`;
    used.add(name);
    return [c, name];
  });
  const animations = named.map(([c, name]) => character.kind === 'mannequin' ? mannequinClip(c, name) : bakeCustomClip(character, c, name));
  const glb = await new GLTFExporter().parseAsync(character.group, { binary: true, animations, onlyVisible: true });
  return new Blob([glb], { type: 'model/gltf-binary' });
}

export function projectJSON(state) {
  return JSON.stringify({ kind: 'fulcrum-mocap', version: 1, saved: new Date().toISOString(), video: state.video, settings: state.settings, capture: captureToJSON(state.capture) });
}
