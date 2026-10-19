import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

function gridTexture(dark) {
  const c = document.createElement('canvas'), s = 512;
  c.width = c.height = s;
  const g = c.getContext('2d');
  g.fillStyle = dark ? '#23272c' : '#e9ebee';
  g.fillRect(0, 0, s, s);
  g.strokeStyle = dark ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.05)';
  g.lineWidth = 2;
  for (let i = 1; i < 4; i++) { g.beginPath(); g.moveTo(i * s / 4, 0); g.lineTo(i * s / 4, s); g.moveTo(0, i * s / 4); g.lineTo(s, i * s / 4); g.stroke(); }
  g.strokeStyle = dark ? 'rgba(255,255,255,0.13)' : 'rgba(0,0,0,0.13)';
  g.lineWidth = 4;
  g.strokeRect(0, 0, s, s);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(200, 200);
  t.anisotropy = 8;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

export function createViewer(container, o = {}) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: !!o.preserve });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.appendChild(renderer.domElement);
  const scene = new THREE.Scene(), camera = new THREE.PerspectiveCamera(45, 1, 0.05, 400);
  camera.position.set(3, 2, 5);
  const hemi = new THREE.HemisphereLight(0xffffff, 0x8a8f96, 1.6), sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.position.set(4, 8, 5);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 0.5, far: 30 });
  sun.shadow.bias = -0.0005;
  scene.add(hemi, sun, sun.target);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.MeshStandardMaterial({ roughness: 0.95, metalness: 0 }));
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.9, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  const follow = { on: false, yaw: Math.PI, pitch: 0.28, dist: 4.2, target: new THREE.Vector3(0, 1, 0), drag: null, track: true };
  const v = {
    renderer, scene, camera, controls, floor, sun, follow, character: null, extras: new THREE.Group(),
    setTheme(dark) {
      const bg = new THREE.Color(dark ? '#1b1e22' : '#f3f4f6');
      scene.background = bg;
      scene.fog = new THREE.Fog(bg, 18, 70);
      floor.material.map?.dispose();
      floor.material.map = gridTexture(dark);
      floor.material.needsUpdate = true;
      hemi.groundColor.set(dark ? 0x3a3f46 : 0x9aa0a8);
    },
    setCharacter(ch) {
      if (v.character) scene.remove(v.character.group);
      v.character = ch;
      if (ch) scene.add(ch.group);
    },
    resize() {
      const w = container.clientWidth || 1, h = container.clientHeight || 1;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    },
    setFollow(on) {
      follow.on = on;
      controls.enabled = !on;
      if (on) {
        const f = v.character?.focus?.() ?? new THREE.Vector3();
        follow.target.copy(f);
      }
    },
    cameraYaw() { const d = camera.getWorldDirection(new THREE.Vector3()); return Math.atan2(d.x, d.z); },
    frame(target, dist = 4.5, dir = null) {
      const d = dir ? new THREE.Vector3(...dir).normalize() : camera.position.clone().sub(controls.target).normalize();
      controls.target.copy(target);
      camera.position.copy(target).addScaledVector(d, dist);
      controls.update();
    },
    fromVideo(cam) {
      camera.position.set(0, cam.height_m, 0);
      camera.up.set(0, 1, 0);
      const q = new THREE.Quaternion(cam.q[0], cam.q[1], cam.q[2], cam.q[3]), look = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
      camera.fov = 2 * Math.atan(Math.tan((cam.hfov * Math.PI / 180) / 2) / (cam.width / cam.height)) * 180 / Math.PI;
      camera.updateProjectionMatrix();
      controls.target.copy(camera.position).addScaledVector(look, 4);
      camera.lookAt(controls.target);
      controls.update();
    },
    update(dt) {
      if (follow.on && v.character) {
        const f = v.character.focus();
        follow.target.lerp(new THREE.Vector3(f.x, f.y + 0.35, f.z), 1 - Math.exp(-dt * 8));
        const cp = Math.cos(follow.pitch), off = new THREE.Vector3(Math.sin(follow.yaw) * cp, Math.sin(follow.pitch), Math.cos(follow.yaw) * cp).multiplyScalar(follow.dist);
        camera.position.copy(follow.target).add(off);
        camera.lookAt(follow.target);
      } else {
        const f = follow.track ? v.character?.focus?.() : null;
        if (f && Number.isFinite(f.x)) {
          const want = new THREE.Vector3(f.x, Math.max(0.6, f.y), f.z), d = want.sub(controls.target).multiplyScalar(1 - Math.exp(-dt * 6));
          controls.target.add(d);
          camera.position.add(d);
        }
        controls.update();
      }
      const c = v.character?.focus?.();
      if (c) {
        sun.position.set(c.x + 4, 8, c.z + 5);
        sun.target.position.set(c.x, 0, c.z);
      }
    },
    render() { renderer.render(scene, camera); },
    dispose() { renderer.dispose(); controls.dispose(); },
  };
  scene.add(v.extras);
  const el = renderer.domElement;
  el.addEventListener('pointerdown', e => { if (follow.on) { follow.drag = { x: e.clientX, y: e.clientY, yaw: follow.yaw, pitch: follow.pitch }; el.setPointerCapture(e.pointerId); } });
  el.addEventListener('pointermove', e => {
    if (!follow.on || !follow.drag) return;
    follow.yaw = follow.drag.yaw - (e.clientX - follow.drag.x) * 0.006;
    follow.pitch = Math.min(1.3, Math.max(-0.15, follow.drag.pitch + (e.clientY - follow.drag.y) * 0.004));
  });
  el.addEventListener('pointerup', () => { follow.drag = null; });
  el.addEventListener('wheel', e => { if (follow.on) { follow.dist = Math.min(14, Math.max(1.5, follow.dist * Math.exp(e.deltaY * 0.001))); e.preventDefault(); } }, { passive: false });
  v.setTheme(o.dark ?? false);
  v.resize();
  return v;
}
