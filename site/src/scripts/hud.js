// HUD dressing: boot sequence typing + live telemetry derived from the cube state.

const REDUCED = typeof window !== 'undefined'
  && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

export function typeBoot(el, lines) {
  if (REDUCED) {
    el.textContent = lines.join('\n');
    return;
  }
  let li = 0, ci = 0, out = '';
  const caret = '█';
  function tick() {
    if (li >= lines.length) {
      el.textContent = out;
      return;
    }
    const line = lines[li];
    if (ci < line.length) {
      ci += 1 + (Math.random() < 0.4 ? 1 : 0); // uneven typing
      el.textContent = out + line.slice(0, ci) + caret;
      setTimeout(tick, 14);
    } else {
      out += line + '\n';
      li += 1;
      ci = 0;
      el.textContent = out + (li < lines.length ? caret : '');
      setTimeout(tick, 160);
    }
  }
  tick();
}

// Rotates a world vector into the body frame (inverse of quaternion q).
function toBody(q, vx, vy, vz) {
  const x = -q.x, y = -q.y, z = -q.z, w = q.w;
  const ix = w * vx + y * vz - z * vy;
  const iy = w * vy + z * vx - x * vz;
  const iz = w * vz + x * vy - y * vx;
  const iw = -x * vx - y * vy - z * vz;
  return [
    ix * w + iw * -x + iy * -z - iz * -y,
    iy * w + iw * -y + iz * -x - ix * -z,
    iz * w + iw * -z + ix * -y - iy * -x,
  ];
}

const noise = (amp) => (Math.random() - 0.5) * amp;
const fmt = (v) => (v >= 0 ? '+' : '') + v.toFixed(2);

// Earth's field with a ~55° dip, |B| ≈ 38 µT, in the scene's world frame
const MAG_WORLD = [0, -31.1, 21.8];

export function startTelemetry(root, state) {
  const el = {};
  for (const node of root.querySelectorAll('[data-t]')) el[node.dataset.t] = node;
  if (!el.g1) return;

  const t0 = performance.now();
  setInterval(() => {
    const t = (performance.now() - t0) / 1000;
    const q = state.q;

    // vertical axis reads as the third component, like the driver's A3/G3/M3
    const g = toBody(q, 0, 1, 0);
    const m = toBody(q, MAG_WORLD[0], MAG_WORLD[1], MAG_WORLD[2]);

    el.g1.textContent = fmt(state.rates.x + noise(0.4));
    el.g2.textContent = fmt(state.rates.z + noise(0.4));
    el.g3.textContent = fmt(state.rates.y + noise(0.4));
    el.a1.textContent = fmt(g[0] + noise(0.008));
    el.a2.textContent = fmt(g[2] + noise(0.008));
    el.a3.textContent = fmt(g[1] + noise(0.008));
    el.m1.textContent = fmt(m[0] + noise(0.5));
    el.m2.textContent = fmt(m[2] + noise(0.5));
    el.m3.textContent = fmt(m[1] + noise(0.5));
    el.temp.textContent = fmt(36.3 + Math.sin(t / 9) * 0.3 + noise(0.06));
  }, REDUCED ? 500 : 120);
}
