// Bayani-6 dashboard: procedural camera feed (water surface + pollutants) with an
// AnomalyNet-style patch heatmap and HUD drawn on an overlay canvas.
(function () {
  const B6 = (window.B6 = window.B6 || {});
  const CW = 256, CH = 144, PX = 16, PY = 9; // feed resolution and patch grid

  // value noise
  const perm = new Uint8Array(512), rv = new Float32Array(256);
  (function () {
    let s = 1337;
    const r = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    const p = [...Array(256).keys()];
    for (let i = 255; i > 0; i--) { const j = Math.floor(r() * (i + 1)); [p[i], p[j]] = [p[j], p[i]]; }
    for (let i = 0; i < 512; i++) perm[i] = p[i & 255];
    for (let i = 0; i < 256; i++) rv[i] = r();
  })();
  function vnoise(x, y) {
    const xi = Math.floor(x), yi = Math.floor(y), xf = x - xi, yf = y - yi;
    const u = xf * xf * (3 - 2 * xf), v = yf * yf * (3 - 2 * yf);
    const X0 = xi & 255, X1 = (xi + 1) & 255, Y0 = yi & 255, Y1 = (yi + 1) & 255;
    const a = rv[perm[X0 + perm[Y0]]], b = rv[perm[X1 + perm[Y0]]];
    const c = rv[perm[X0 + perm[Y1]]], d = rv[perm[X1 + perm[Y1]]];
    return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
  }
  const smooth = (e0, e1, x) => { const t = Math.min(1, Math.max(0, (x - e0) / (e1 - e0))); return t * t * (3 - 2 * t); };
  function hsv(h, s, v) {
    const i = Math.floor(h * 6), f = h * 6 - i, p = v * (1 - s), q = v * (1 - f * s), t = v * (1 - (1 - f) * s);
    const m = [[v, t, p], [q, v, p], [p, v, t], [p, q, v], [t, p, v], [v, p, q]][((i % 6) + 6) % 6];
    return [m[0] * 255, m[1] * 255, m[2] * 255];
  }

  let feed, fctx, img, overlay, octx;
  const patch = new Float32Array(PX * PY);

  function init(feedCanvas, overlayCanvas) {
    feed = feedCanvas; fctx = feed.getContext("2d");
    img = fctx.createImageData(CW, CH);
    overlay = overlayCanvas; octx = overlay.getContext("2d");
  }

  function render(v) {
    const d = img.data, t = v.t, scroll = v.scroll;
    patch.fill(0);
    const items = v.items;
    for (let y = 0; y < CH; y++) {
      const depth = 0.72 + 0.28 * (y / CH);
      for (let x = 0; x < CW; x++) {
        const xs = x - scroll;
        const w1 = Math.sin(xs * 0.09 + t * 1.6 + Math.sin(y * 0.07 + t * 0.7) * 2.0);
        const w2 = Math.sin((xs * 0.6 + y) * 0.06 - t * 1.1);
        const n = vnoise(xs * 0.05, y * 0.09 + t * 0.25);
        const wave = 0.5 * w1 + 0.5 * w2;
        let r = 34, g = 62, b = 57;
        const k = depth * (0.82 + 0.13 * wave + 0.16 * (n - 0.5));
        r *= k; g *= k; b *= k;
        if (wave > 0.78) { const sp = (wave - 0.78) * 4.5 * depth; r += 70 * sp; g += 85 * sp; b += 80 * sp; }

        if (v.glare > 0) {
          const gl = v.glare * Math.exp(-((x - v.glareX) ** 2) / (2 * 18 * 18)) * (0.55 + 0.45 * Math.sin(y * 0.6 + t * 9 + x * 0.2));
          r += 190 * gl; g += 195 * gl; b += 185 * gl;
        }

        for (let i = 0; i < items.length; i++) {
          const it = items[i];
          const dx = (x - it.px) / it.R, dy = (y - it.py) / (it.R * 0.62);
          if (dx > 1.6 || dx < -1.6 || dy > 1.6 || dy < -1.6) continue;
          const dd = Math.sqrt(dx * dx + dy * dy) + (vnoise(x * 0.12 + it.seed, y * 0.12) - 0.5) * 0.65;
          const m = smooth(1.0, 0.68, dd) * it.vis;
          if (m <= 0.001) continue;
          let cr, cg, cb, a;
          if (it.type === "oil") {
            const h = (dd * 2.1 + vnoise(x * 0.05, y * 0.05 + t * 0.1) * 1.4 + t * 0.04) % 1;
            [cr, cg, cb] = hsv(h, 0.55, 0.6);
            a = m * 0.72;
          } else if (it.type === "foam") {
            const f = smooth(0.32, 0.6, vnoise(x * 0.35 + it.seed, y * 0.35 + t * 0.3));
            cr = 226; cg = 231; cb = 228; a = m * (0.25 + 0.72 * f);
          } else if (it.type === "sludge") {
            const q = 0.78 + 0.35 * vnoise(x * 0.2, y * 0.2);
            cr = 80 * q; cg = 59 * q; cb = 34 * q; a = m * 0.9;
          } else {
            const q = vnoise(x * 0.25 + it.seed, y * 0.6);
            cr = 52 * (0.7 + 0.6 * q); cg = 112 * (0.7 + 0.6 * q); cb = 42 * (0.7 + 0.6 * q); a = m * (0.55 + 0.45 * q);
          }
          r += (cr - r) * a; g += (cg - g) * a; b += (cb - b) * a;
          patch[Math.floor(y / (CH / PY)) * PX + Math.floor(x / (CW / PX))] += m;
        }
        const o = (y * CW + x) * 4;
        d[o] = r; d[o + 1] = g; d[o + 2] = b; d[o + 3] = 255;
      }
    }
    // a little sensor noise
    for (let k = 0; k < 600; k++) {
      const o = ((Math.random() * CW * CH) | 0) * 4, j = (Math.random() - 0.5) * 30;
      d[o] += j; d[o + 1] += j; d[o + 2] += j;
    }
    fctx.putImageData(img, 0, 0);
    drawOverlay(v);
  }

  function drawOverlay(v) {
    const dpr = window.devicePixelRatio || 1;
    const w = overlay.clientWidth, h = overlay.clientHeight;
    if (overlay.width !== Math.round(w * dpr) || overlay.height !== Math.round(h * dpr)) {
      overlay.width = Math.round(w * dpr); overlay.height = Math.round(h * dpr);
    }
    const c = octx;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);
    const pw = w / PX, ph = h / PY, cellPx = (CW / PX) * (CH / PY);
    const strength = Math.min(1, Math.max(0, (v.score - 0.25) / 0.45));

    // patch heatmap
    let minX = PX, minY = PY, maxX = -1, maxY = -1;
    for (let j = 0; j < PY; j++) {
      for (let i = 0; i < PX; i++) {
        const a = patch[j * PX + i] / cellPx;
        if (a > 0.1 && strength > 0) {
          c.fillStyle = `rgba(255, 77, 61, ${Math.min(0.55, a * 1.1) * strength})`;
          c.fillRect(i * pw + 1, j * ph + 1, pw - 2, ph - 2);
          if (a > 0.22) { minX = Math.min(minX, i); minY = Math.min(minY, j); maxX = Math.max(maxX, i); maxY = Math.max(maxY, j); }
        }
      }
    }
    // faint patch grid
    c.strokeStyle = "rgba(255,255,255,0.05)"; c.lineWidth = 1;
    c.beginPath();
    for (let i = 1; i < PX; i++) { c.moveTo(i * pw, 0); c.lineTo(i * pw, h); }
    for (let j = 1; j < PY; j++) { c.moveTo(0, j * ph); c.lineTo(w, j * ph); }
    c.stroke();

    const mono = "'IBM Plex Mono', monospace";
    if (maxX >= 0 && v.score > v.threshold * 0.85) {
      const bx = minX * pw - 3, by = minY * ph - 3, bw = (maxX - minX + 1) * pw + 6, bh = (maxY - minY + 1) * ph + 6;
      const hot = v.score > v.threshold;
      c.strokeStyle = hot ? "#ff4d3d" : "#ffb020"; c.lineWidth = 2;
      c.strokeRect(bx, by, bw, bh);
      const label = `ANOMALY ${v.score.toFixed(2)}`;
      c.font = `600 12px ${mono}`;
      const tw = c.measureText(label).width + 10;
      c.fillStyle = hot ? "#ff4d3d" : "#ffb020";
      c.fillRect(bx, Math.max(0, by - 18), tw, 18);
      c.fillStyle = "#111"; c.fillText(label, bx + 5, Math.max(13, by - 5));
    }

    // corner brackets + centre mark
    c.strokeStyle = "rgba(230, 237, 243, 0.65)"; c.lineWidth = 1.5;
    const L = 16, m = 10;
    c.beginPath();
    c.moveTo(m, m + L); c.lineTo(m, m); c.lineTo(m + L, m);
    c.moveTo(w - m - L, m); c.lineTo(w - m, m); c.lineTo(w - m, m + L);
    c.moveTo(m, h - m - L); c.lineTo(m, h - m); c.lineTo(m + L, h - m);
    c.moveTo(w - m - L, h - m); c.lineTo(w - m, h - m); c.lineTo(w - m, h - m - L);
    c.moveTo(w / 2 - 8, h / 2); c.lineTo(w / 2 + 8, h / 2); c.moveTo(w / 2, h / 2 - 8); c.lineTo(w / 2, h / 2 + 8);
    c.stroke();

    c.font = `11px ${mono}`;
    c.fillStyle = "rgba(230, 237, 243, 0.85)";
    c.fillText("640×480 · 30 fps · RGB-D", 16, 26);
    const right = `ANOMALYNET · TRT FP16 · ${v.fps.toFixed(1)} FPS`;
    c.fillText(right, w - 16 - c.measureText(right).width, 26);
    c.fillStyle = "rgba(230, 237, 243, 0.6)";
    c.fillText(`patches ${PX}×${PY}`, 16, h - 16);

    c.font = `600 22px ${mono}`;
    const sc = v.score.toFixed(2);
    c.fillStyle = v.score > v.threshold ? "#ff4d3d" : v.score > v.threshold * 0.75 ? "#ffb020" : "#e6edf3";
    c.fillText(sc, w - 16 - c.measureText(sc).width, h - 14);

    if (v.mode === "sample") {
      c.font = `600 12px ${mono}`;
      const t = "ARM IN FRAME · SAMPLING";
      const tw = c.measureText(t).width + 16;
      c.fillStyle = "rgba(255, 176, 32, 0.92)";
      c.fillRect(w / 2 - tw / 2, h - 34, tw, 20);
      c.fillStyle = "#111"; c.fillText(t, w / 2 - tw / 2 + 8, h - 20);
    }
    if (v.paused) {
      c.fillStyle = "rgba(10,14,19,0.55)"; c.fillRect(0, 0, w, h);
      c.font = `600 13px ${mono}`; c.fillStyle = "#e6edf3";
      const t = "PAUSED · press Run";
      c.fillText(t, w / 2 - c.measureText(t).width / 2, h / 2 + 24);
    }
  }

  B6.camera = { init, render, CW, CH };
})();
