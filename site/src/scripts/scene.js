import * as THREE from 'three';

const DEG = Math.PI / 180;
const HOLO = 0x35e0ff;

function buildBoard() {
  const board = new THREE.Group();

  const pcb = new THREE.Mesh(
    new THREE.BoxGeometry(5.6, 0.16, 3.6),
    new THREE.MeshStandardMaterial({ color: 0x0e7a3e, roughness: 0.55, metalness: 0.1 })
  );
  board.add(pcb);

  // GPIO header: black base + 2x20 gold pins along the back edge
  const header = new THREE.Mesh(
    new THREE.BoxGeometry(4.4, 0.2, 0.38),
    new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.7 })
  );
  header.position.set(0, 0.18, -1.45);
  board.add(header);

  const pinGeo = new THREE.BoxGeometry(0.045, 0.3, 0.045);
  const pinMat = new THREE.MeshStandardMaterial({ color: 0xd4af37, roughness: 0.25, metalness: 0.9 });
  for (let i = 0; i < 20; i++) {
    for (let j = 0; j < 2; j++) {
      const pin = new THREE.Mesh(pinGeo, pinMat);
      pin.position.set(-2.1 + i * 0.22, 0.4, -1.53 + j * 0.16);
      board.add(pin);
    }
  }

  // The MPU-9250: small black QFN with a pin-1 dot
  const chip = new THREE.Mesh(
    new THREE.BoxGeometry(0.62, 0.1, 0.62),
    new THREE.MeshStandardMaterial({ color: 0x151515, roughness: 0.4 })
  );
  chip.position.set(0.0, 0.13, 0.25);
  board.add(chip);
  const dot = new THREE.Mesh(
    new THREE.CylinderGeometry(0.035, 0.035, 0.02, 16),
    new THREE.MeshStandardMaterial({ color: 0xdddddd, roughness: 0.3 })
  );
  dot.position.set(-0.2, 0.185, 0.05);
  board.add(dot);

  // Holographic outline around the MPU, marking the sensed part
  const chipRing = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(0.8, 0.16, 0.8)),
    new THREE.LineBasicMaterial({ color: HOLO, transparent: true, opacity: 0.85 })
  );
  chipRing.position.copy(chip.position);
  board.add(chipRing);

  // A couple of larger packages so it reads as a Pi
  const soc = new THREE.Mesh(
    new THREE.BoxGeometry(0.9, 0.12, 0.9),
    new THREE.MeshStandardMaterial({ color: 0x2a2a2e, roughness: 0.35, metalness: 0.4 })
  );
  soc.position.set(-1.35, 0.14, 0.1);
  board.add(soc);

  const usbMat = new THREE.MeshStandardMaterial({ color: 0x9aa2ad, roughness: 0.3, metalness: 0.8 });
  const usb1 = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.42, 0.7), usbMat);
  usb1.position.set(2.35, 0.29, 0.75);
  board.add(usb1);
  const usb2 = usb1.clone();
  usb2.position.set(2.35, 0.29, -0.35);
  board.add(usb2);

  // Mounting holes (gold rings)
  const holeMat = new THREE.MeshStandardMaterial({ color: 0xd4af37, roughness: 0.3, metalness: 0.8 });
  for (const [x, z] of [[-2.55, -1.5], [-2.55, 1.5], [2.55, -1.5], [2.55, 1.5]]) {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.09, 0.03, 8, 24), holeMat);
    ring.rotation.x = Math.PI / 2;
    ring.position.set(x, 0.09, z);
    board.add(ring);
  }

  return { board, chip };
}

function buildCube() {
  const group = new THREE.Group();

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(1.15, 1.15, 1.15),
    new THREE.MeshBasicMaterial({
      color: 0x0891b2, transparent: true, opacity: 0.2,
      blending: THREE.AdditiveBlending, depthWrite: false,
    })
  );
  group.add(body);

  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(body.geometry),
    new THREE.LineBasicMaterial({ color: HOLO, transparent: true, opacity: 0.95 })
  );
  group.add(edges);

  // Body-frame axes: X red, Y green, Z blue
  group.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1.15, 0xff5f5f, 0.22, 0.1));
  group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 1.15, 0x4ade80, 0.22, 0.1));
  group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 1.15, 0x60a5fa, 0.22, 0.1));

  return group;
}

function buildHoloDeck() {
  const deck = new THREE.Group();

  const grid = new THREE.PolarGridHelper(4.4, 16, 6, 64, HOLO, 0x0e5a70);
  grid.material.transparent = true;
  grid.material.opacity = 0.22;
  deck.add(grid);

  // Radar sweep: a translucent sector that spins on the grid
  const sweep = new THREE.Mesh(
    new THREE.RingGeometry(0.15, 4.35, 40, 1, 0, 0.65),
    new THREE.MeshBasicMaterial({
      color: HOLO, transparent: true, opacity: 0.06, side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending, depthWrite: false,
    })
  );
  sweep.rotation.x = -Math.PI / 2;
  sweep.position.y = 0.01;
  deck.add(sweep);

  const needle = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0.15, 0, 0), new THREE.Vector3(4.35, 0, 0)]),
    new THREE.LineBasicMaterial({ color: HOLO, transparent: true, opacity: 0.5 })
  );
  needle.rotation.x = -Math.PI / 2;
  needle.position.y = 0.012;
  deck.add(needle);

  return { deck, sweep, needle };
}

function buildParticles() {
  const N = 140;
  const positions = new Float32Array(N * 3);
  for (let i = 0; i < N; i++) {
    const r = 1.5 + Math.random() * 3.2;
    const a = Math.random() * Math.PI * 2;
    positions[i * 3] = Math.cos(a) * r;
    positions[i * 3 + 1] = Math.random() * 3.4;
    positions[i * 3 + 2] = Math.sin(a) * r;
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const points = new THREE.Points(geo, new THREE.PointsMaterial({
    color: HOLO, size: 0.035, transparent: true, opacity: 0.5,
    blending: THREE.AdditiveBlending, depthWrite: false,
  }));
  return points;
}

// Standard mapping from DeviceOrientation angles to a quaternion
const zee = new THREE.Vector3(0, 0, 1);
const orientEuler = new THREE.Euler();
const qScreen = new THREE.Quaternion();
const qFlip = new THREE.Quaternion(-Math.sqrt(0.5), 0, 0, Math.sqrt(0.5));

function quaternionFromOrientation(out, alpha, beta, gamma, screenAngle) {
  orientEuler.set(beta * DEG, alpha * DEG, -gamma * DEG, 'YXZ');
  out.setFromEuler(orientEuler);
  out.multiply(qFlip);
  out.multiply(qScreen.setFromAxisAngle(zee, -screenAngle * DEG));
}

export function initScene(canvas, imuButton) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  const scene = new THREE.Scene();
  // Telephoto framing: a wide FOV up close bends the cube into a frustum, so
  // the camera sits 3x farther with a matching narrow FOV (same subject size)
  const camera = new THREE.PerspectiveCamera(15, 1, 0.1, 100);
  camera.position.set(0, 5.4, 22.2);
  camera.lookAt(0, 0.9, 0);

  scene.add(new THREE.AmbientLight(0x6b87a8, 0.55));
  const key = new THREE.DirectionalLight(0xdfefff, 1.5);
  key.position.set(5, 8, 4);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x3a86ff, 0.6);
  rim.position.set(-4, 2, -6);
  scene.add(rim);
  const accent = new THREE.PointLight(HOLO, 8, 12);
  accent.position.set(-3, 3, 2);
  scene.add(accent);

  const { board, chip } = buildBoard();
  board.position.y = -0.8;
  scene.add(board);

  const { deck, sweep, needle } = buildHoloDeck();
  deck.position.y = -0.9;
  scene.add(deck);

  scene.add(buildParticles());

  const cube = buildCube();
  cube.position.y = 1.45;
  scene.add(cube);

  // Data beam: the driver "reading" the chip — from the MPU up to the cube
  const beam = new THREE.Mesh(
    new THREE.CylinderGeometry(0.045, 0.1, 1, 12, 1, true),
    new THREE.MeshBasicMaterial({
      color: HOLO, transparent: true, opacity: 0.18, side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending, depthWrite: false,
    })
  );
  scene.add(beam);
  const chipWorld = new THREE.Vector3();
  const beamDir = new THREE.Vector3();
  const UP = new THREE.Vector3(0, 1, 0);

  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const speed = reduced ? 0.15 : 1.0;

  const targetQ = new THREE.Quaternion();
  const tmpQ = new THREE.Quaternion();
  const prevQ = new THREE.Quaternion();
  const deltaQ = new THREE.Quaternion();
  const AXIS_X = new THREE.Vector3(1, 0, 0);
  const AXIS_Y = new THREE.Vector3(0, 1, 0);

  // Live cube state, consumed by the HUD telemetry panel
  const state = { q: cube.quaternion, rates: { x: 0, y: 0, z: 0 } };

  let mode = 'auto'; // 'auto' | 'drag' | 'imu'
  let resumeAt = 0;

  // --- pointer drag ---
  let dragging = false, lastX = 0, lastY = 0;
  canvas.style.touchAction = 'pan-y';
  canvas.addEventListener('pointerdown', (e) => {
    dragging = true;
    lastX = e.clientX;
    lastY = e.clientY;
    if (mode !== 'imu') mode = 'drag';
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    const dx = e.clientX - lastX;
    const dy = e.clientY - lastY;
    lastX = e.clientX;
    lastY = e.clientY;
    targetQ.premultiply(tmpQ.setFromAxisAngle(AXIS_Y, dx * 0.008));
    targetQ.premultiply(tmpQ.setFromAxisAngle(AXIS_X, dy * 0.008));
  });
  const endDrag = () => {
    dragging = false;
    if (mode === 'drag') resumeAt = performance.now() + 3000;
  };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);

  // --- device IMU (phones/tablets) ---
  const onOrientation = (e) => {
    if (e.alpha === null && e.beta === null) return;
    mode = 'imu';
    const angle = (screen.orientation && screen.orientation.angle) || 0;
    quaternionFromOrientation(targetQ, e.alpha || 0, e.beta || 0, e.gamma || 0, angle);
  };
  if (imuButton) {
    if (typeof DeviceOrientationEvent !== 'undefined' && 'ontouchstart' in window) {
      imuButton.hidden = false;
      imuButton.addEventListener('click', async () => {
        try {
          if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            const answer = await DeviceOrientationEvent.requestPermission();
            if (answer !== 'granted') return;
          }
          window.addEventListener('deviceorientation', onOrientation);
          imuButton.textContent = 'IMU LIVE';
          imuButton.disabled = true;
        } catch {
          imuButton.hidden = true;
        }
      });
    }
  }

  // --- sizing ---
  const holder = canvas.parentElement;
  function resize() {
    const w = holder.clientWidth;
    const h = holder.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(holder);
  resize();

  // --- animation loop ---
  let prev = performance.now();
  prevQ.copy(cube.quaternion);
  renderer.setAnimationLoop((now) => {
    const dt = Math.min((now - prev) / 1000, 0.05);
    prev = now;
    const t = now / 1000;

    if (mode === 'drag' && !dragging && now > resumeAt) mode = 'auto';
    if (mode === 'auto') {
      // synthetic gyro rates, slightly incommensurate so the tumble never loops
      targetQ.premultiply(tmpQ.setFromAxisAngle(AXIS_Y, 0.35 * speed * dt));
      targetQ.premultiply(tmpQ.setFromAxisAngle(AXIS_X, Math.sin(t * 0.37) * 0.22 * speed * dt));
    }

    cube.quaternion.slerp(targetQ, 0.12);
    cube.position.y = 1.45 + Math.sin(t * 1.25) * 0.07 * speed;
    board.rotation.y = Math.sin(t * 0.21) * 0.16 * speed;

    // body-frame angular rate from the quaternion delta, in °/s (smoothed)
    if (dt > 0) {
      deltaQ.copy(prevQ).invert().multiply(cube.quaternion);
      const w = Math.min(Math.max(deltaQ.w, -1), 1);
      const angle = 2 * Math.acos(Math.abs(w));
      const s = Math.sqrt(Math.max(1 - w * w, 1e-12));
      const k = ((w < 0 ? -1 : 1) * angle) / (s * dt) / DEG;
      state.rates.x += (deltaQ.x * k - state.rates.x) * 0.25;
      state.rates.y += (deltaQ.y * k - state.rates.y) * 0.25;
      state.rates.z += (deltaQ.z * k - state.rates.z) * 0.25;
      prevQ.copy(cube.quaternion);
    }

    // holographic dressing
    sweep.rotation.z = -t * 0.9 * speed;
    needle.rotation.z = -t * 0.9 * speed;
    chip.getWorldPosition(chipWorld);
    beamDir.copy(cube.position).sub(chipWorld);
    const len = beamDir.length();
    beam.position.copy(chipWorld).addScaledVector(beamDir, 0.5);
    beam.quaternion.setFromUnitVectors(UP, beamDir.normalize());
    beam.scale.set(1, len, 1);
    beam.material.opacity = 0.14 + Math.sin(t * 2.6) * 0.06;

    renderer.render(scene, camera);
  });

  return state;
}
