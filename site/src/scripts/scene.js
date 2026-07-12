import * as THREE from 'three';

const DEG = Math.PI / 180;

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

  return board;
}

function buildCube() {
  const group = new THREE.Group();

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(1.15, 1.15, 1.15),
    new THREE.MeshStandardMaterial({
      color: 0x14b8a6, transparent: true, opacity: 0.16,
      roughness: 0.2, metalness: 0.1, depthWrite: false,
    })
  );
  group.add(body);

  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(body.geometry),
    new THREE.LineBasicMaterial({ color: 0x2dd4bf })
  );
  group.add(edges);

  // Body-frame axes: X red, Y green, Z blue
  group.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1.15, 0xef4444, 0.22, 0.1));
  group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 1.15, 0x22c55e, 0.22, 0.1));
  group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 1.15, 0x3b82f6, 0.22, 0.1));

  return group;
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
  const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
  camera.position.set(0, 2.3, 7.2);
  camera.lookAt(0, 0.8, 0);

  scene.add(new THREE.AmbientLight(0x8fa3bd, 0.7));
  const key = new THREE.DirectionalLight(0xffffff, 1.6);
  key.position.set(5, 8, 4);
  scene.add(key);
  const accent = new THREE.PointLight(0x2dd4bf, 6, 12);
  accent.position.set(-3, 3, 2);
  scene.add(accent);

  const board = buildBoard();
  board.position.y = -0.8;
  scene.add(board);

  const cube = buildCube();
  cube.position.y = 1.3;
  scene.add(cube);

  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const speed = reduced ? 0.15 : 1.0;

  const targetQ = new THREE.Quaternion();
  const tmpQ = new THREE.Quaternion();
  const AXIS_X = new THREE.Vector3(1, 0, 0);
  const AXIS_Y = new THREE.Vector3(0, 1, 0);

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
          imuButton.textContent = 'IMU live';
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
    cube.position.y = 1.3 + Math.sin(t * 1.25) * 0.07 * speed;
    board.rotation.y = Math.sin(t * 0.21) * 0.16 * speed;

    renderer.render(scene, camera);
  });
}
