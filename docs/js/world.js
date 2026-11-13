// Bayani-6 dashboard: world geometry (creek, banks, bridges, buildings), patrol route,
// LiDAR ray casting, contamination heat grid and the top-down map renderer.
(function () {
  const B6 = (window.B6 = window.B6 || {});

  const W = 36, H = 20;          // world size in metres
  const HALF = 1.5;              // creek half-width
  const BANK_V = 2.9;            // patrol path offset from the creek centre (vertical)
  const BRIDGES = [4, 32];
  const BRIDGE_HALF = 0.85;

  function mulberry32(a) {
    return function () {
      a |= 0; a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  const cy = (x) => 10 + 2.4 * Math.sin(x / 5.0 + 0.4) + 0.5 * Math.sin(x / 2.2 + 1.3);
  function tangent(x) {
    const e = 0.01, dy = (cy(x + e) - cy(x - e)) / (2 * e), l = Math.hypot(1, dy);
    return [1 / l, dy / l];
  }
  function northNormal(x) { const t = tangent(x); return [t[1], -t[0]]; }
  function bankPt(x, off) { const n = northNormal(x); return [x + n[0] * off, cy(x) + n[1] * off]; }

  // ---- Static geometry ----
  const centre = [], northBank = [], southBank = [];
  for (let x = -1; x <= W + 1.001; x += 0.25) {
    centre.push([x, cy(x)]);
    northBank.push(bankPt(x, HALF));
    southBank.push(bankPt(x, -HALF));
  }

  const nearBridge = (x, pad = 0) => BRIDGES.some((b) => Math.abs(x - b) < BRIDGE_HALF + pad);

  const segments = []; // LiDAR-visible line segments [x1, y1, x2, y2]
  function addPolyline(pts) {
    for (let i = 0; i < pts.length - 1; i++) {
      const a = pts[i], b = pts[i + 1];
      if (nearBridge(a[0]) || nearBridge(b[0])) continue;
      segments.push([a[0], a[1], b[0], b[1]]);
    }
  }
  addPolyline(northBank);
  addPolyline(southBank);
  for (const b of BRIDGES) {
    for (const s of [-1, 1]) {
      const x = b + s * BRIDGE_HALF;
      segments.push([x, cy(x) - 2.0, x, cy(x) + 2.0]);
    }
  }

  const buildings = [];
  const rnd = mulberry32(42);
  function buildRow(north) {
    let x = 0.4;
    while (x < W - 1.5) {
      const w = 2.6 + rnd() * 3.4;
      const x2 = Math.min(W - 0.4, x + w);
      let edge = north ? Infinity : -Infinity;
      for (let xx = x; xx <= x2; xx += 0.2) {
        const p = north ? cy(xx) - BANK_V : cy(xx) + BANK_V;
        edge = north ? Math.min(edge, p) : Math.max(edge, p);
      }
      const y1 = north ? 0.5 : edge + 1.3;
      const y2 = north ? edge - 1.3 : H - 0.5;
      if (y2 - y1 > 1.1) {
        buildings.push({ x: x, y: y1, w: x2 - x, h: y2 - y1 });
        segments.push([x, y1, x2, y1], [x2, y1, x2, y2], [x2, y2, x, y2], [x, y2, x, y1]);
      }
      x = x2 + 0.7 + rnd() * 1.1;
    }
  }
  buildRow(true);
  buildRow(false);

  // ---- Patrol route: north bank east, bridge, south bank west, bridge ----
  const route = [];
  const [bW, bE] = BRIDGES;
  for (let x = bW; x <= bE; x += 0.4) route.push([x, cy(x) - BANK_V]);
  for (let y = cy(bE) - BANK_V; y <= cy(bE) + BANK_V; y += 0.4) route.push([bE, y]);
  for (let x = bE; x >= bW; x -= 0.4) route.push([x, cy(x) + BANK_V]);
  for (let y = cy(bW) + BANK_V; y >= cy(bW) - BANK_V; y -= 0.4) route.push([bW, y]);
  const cum = [0];
  for (let i = 1; i <= route.length; i++) {
    const a = route[i - 1], b = route[i % route.length];
    cum.push(cum[i - 1] + Math.hypot(b[0] - a[0], b[1] - a[1]));
  }
  const routeLen = cum[cum.length - 1];

  function routeAt(s) {
    s = ((s % routeLen) + routeLen) % routeLen;
    let lo = 0, hi = cum.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (cum[m] <= s) lo = m; else hi = m; }
    const a = route[lo], b = route[(lo + 1) % route.length];
    const f = (s - cum[lo]) / (cum[lo + 1] - cum[lo] || 1);
    return { x: a[0] + (b[0] - a[0]) * f, y: a[1] + (b[1] - a[1]) * f, heading: Math.atan2(b[1] - a[1], b[0] - a[0]), seg: lo };
  }

  // Where the camera is looking: towards the creek centre, ~2.7 m away
  function cameraTarget(x, y) {
    let best = null, bd = Infinity;
    for (const c of centre) {
      const d = (c[0] - x) ** 2 + (c[1] - y) ** 2;
      if (d < bd) { bd = d; best = c; }
    }
    const d = Math.sqrt(bd);
    if (d < 0.3) return { x, y, dir: 0, dist: 0 };
    const ux = (best[0] - x) / d, uy = (best[1] - y) / d;
    const reach = Math.min(d, 2.9);
    return { x: x + ux * reach, y: y + uy * reach, dir: Math.atan2(uy, ux), dist: reach };
  }

  function inCreek(x, y) {
    if (x < 0 || x > W) return false;
    return Math.abs(y - cy(x)) < HALF * 0.85;
  }

  function raycast(ox, oy, ang, maxR) {
    const dx = Math.cos(ang), dy = Math.sin(ang);
    let best = maxR;
    for (const s of segments) {
      const ex = s[2] - s[0], ey = s[3] - s[1];
      const den = dx * ey - dy * ex;
      if (Math.abs(den) < 1e-9) continue;
      const qx = s[0] - ox, qy = s[1] - oy;
      const t = (qx * ey - qy * ex) / den;
      const u = (qx * dy - qy * dx) / den;
      if (t > 0.05 && t < best && u >= 0 && u <= 1) best = t;
    }
    return best;
  }

  // ---- Contamination heat grid (0.2 m cells) ----
  const CELL = 0.2, GW = Math.round(W / CELL), GH = Math.round(H / CELL);
  const heat = new Float32Array(GW * GH);
  const heatCanvas = document.createElement("canvas");
  heatCanvas.width = GW; heatCanvas.height = GH;
  const heatCtx = heatCanvas.getContext("2d");
  let heatDirty = true;

  function addHeat(x, y, amp, sigma) {
    const r = Math.ceil((sigma * 3) / CELL);
    const cx = Math.round(x / CELL), cyy = Math.round(y / CELL);
    for (let j = -r; j <= r; j++) {
      for (let i = -r; i <= r; i++) {
        const gx = cx + i, gy = cyy + j;
        if (gx < 0 || gy < 0 || gx >= GW || gy >= GH) continue;
        const d2 = (i * CELL) ** 2 + (j * CELL) ** 2;
        heat[gy * GW + gx] += amp * Math.exp(-d2 / (2 * sigma * sigma));
      }
    }
    heatDirty = true;
  }
  function clearHeat() { heat.fill(0); heatDirty = true; }
  function heatImage() {
    if (!heatDirty) return heatCanvas;
    const img = heatCtx.createImageData(GW, GH);
    for (let k = 0; k < heat.length; k++) {
      const t = Math.min(1, heat[k]);
      const o = k * 4;
      if (t < 0.04) { img.data[o + 3] = 0; continue; }
      let r, g, b;
      if (t < 0.5) { const f = t / 0.5; r = 255; g = 214 - 90 * f; b = 80 - 54 * f; }
      else { const f = (t - 0.5) / 0.5; r = 255; g = 124 - 66 * f; b = 26 + 22 * f; }
      img.data[o] = r; img.data[o + 1] = g; img.data[o + 2] = b;
      img.data[o + 3] = Math.min(215, 40 + t * 230);
    }
    heatCtx.putImageData(img, 0, 0);
    heatDirty = false;
    return heatCanvas;
  }

  // ---- Map renderer ----
  const POLLUTANT_COLORS = { oil: "#a77bff", foam: "#e8eef3", sludge: "#8a6337", weeds: "#4caf50" };

  function renderMap(ctx, cw, ch, st, opts) {
    const pad = 14;
    const s = Math.min((cw - pad * 2) / W, (ch - pad * 2) / H);
    const ox = (cw - W * s) / 2, oy = (ch - H * s) / 2;
    const X = (x) => ox + x * s, Y = (y) => oy + y * s;
    const view = { s, ox, oy };

    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 0, cw, ch);

    // ground + grid
    ctx.fillStyle = "#f6f8fa";
    ctx.fillRect(X(0), Y(0), W * s, H * s);
    ctx.lineWidth = 1;
    for (let x = 0; x <= W; x++) {
      ctx.strokeStyle = x % 5 === 0 ? "#dbe2e8" : "#eaeef2";
      ctx.beginPath(); ctx.moveTo(Math.round(X(x)) + 0.5, Y(0)); ctx.lineTo(Math.round(X(x)) + 0.5, Y(H)); ctx.stroke();
    }
    for (let y = 0; y <= H; y++) {
      ctx.strokeStyle = y % 5 === 0 ? "#dbe2e8" : "#eaeef2";
      ctx.beginPath(); ctx.moveTo(X(0), Math.round(Y(y)) + 0.5); ctx.lineTo(X(W), Math.round(Y(y)) + 0.5); ctx.stroke();
    }

    // buildings (hatched)
    for (const b of buildings) {
      ctx.fillStyle = "#e6ebf0";
      ctx.fillRect(X(b.x), Y(b.y), b.w * s, b.h * s);
      ctx.save();
      ctx.beginPath(); ctx.rect(X(b.x), Y(b.y), b.w * s, b.h * s); ctx.clip();
      ctx.strokeStyle = "#d3dbe3"; ctx.lineWidth = 1;
      for (let k = -b.h * s; k < b.w * s; k += 9) {
        ctx.beginPath(); ctx.moveTo(X(b.x) + k, Y(b.y) + b.h * s); ctx.lineTo(X(b.x) + k + b.h * s, Y(b.y)); ctx.stroke();
      }
      ctx.restore();
      ctx.strokeStyle = "#b4c0cb"; ctx.lineWidth = 1.2;
      ctx.strokeRect(X(b.x) + 0.5, Y(b.y) + 0.5, b.w * s - 1, b.h * s - 1);
    }

    // creek
    ctx.beginPath();
    northBank.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))));
    for (let i = southBank.length - 1; i >= 0; i--) ctx.lineTo(X(southBank[i][0]), Y(southBank[i][1]));
    ctx.closePath();
    ctx.fillStyle = "#bfe0e8";
    ctx.fill();
    ctx.save(); ctx.clip();
    ctx.strokeStyle = "rgba(20, 110, 130, 0.28)"; ctx.lineWidth = 1.2;
    const N = 70;
    for (let i = 0; i < N; i++) {
      const u = ((i / N) * (W + 2) + st.time * 0.35) % (W + 2) - 1;
      const lat = (((i * 37) % 11) / 11 - 0.5) * 2 * HALF * 0.75;
      const t = tangent(u), n = northNormal(u);
      const px = u + n[0] * lat, py = cy(u) + n[1] * lat;
      ctx.beginPath(); ctx.moveTo(X(px), Y(py)); ctx.lineTo(X(px + t[0] * 0.7), Y(py + t[1] * 0.7)); ctx.stroke();
    }
    ctx.restore();

    // heatmap
    if (opts.heat) {
      ctx.save();
      ctx.imageSmoothingEnabled = true;
      ctx.drawImage(heatImage(), X(0), Y(0), W * s, H * s);
      ctx.restore();
    }

    // pollutants (ground truth)
    if (opts.truth) {
      for (const a of st.anomalies) drawPollutant(ctx, X(a.x), Y(a.y), s, a, st.time);
    }

    // banks / railings
    ctx.strokeStyle = "#6f8ca0"; ctx.lineWidth = 1.8;
    ctx.beginPath();
    for (const bank of [northBank, southBank]) {
      let pen = false;
      for (const p of bank) {
        if (nearBridge(p[0])) { pen = false; continue; }
        if (!pen) { ctx.moveTo(X(p[0]), Y(p[1])); pen = true; } else ctx.lineTo(X(p[0]), Y(p[1]));
      }
    }
    ctx.stroke();

    // bridges
    for (const b of BRIDGES) {
      const x1 = X(b - BRIDGE_HALF), x2 = X(b + BRIDGE_HALF);
      const y1 = Y(cy(b) - 2.0), y2 = Y(cy(b) + 2.0);
      ctx.fillStyle = "#d9e0e6"; ctx.fillRect(x1, y1, x2 - x1, y2 - y1);
      ctx.strokeStyle = "#bcc7d1"; ctx.lineWidth = 1;
      for (let yy = y1 + 4; yy < y2; yy += 6) { ctx.beginPath(); ctx.moveTo(x1, yy); ctx.lineTo(x2, yy); ctx.stroke(); }
      ctx.strokeStyle = "#5f788b"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x1, y2); ctx.moveTo(x2, y1); ctx.lineTo(x2, y2); ctx.stroke();
    }

    // patrol route
    ctx.save();
    ctx.setLineDash([3, 6]); ctx.strokeStyle = "#9aabb9"; ctx.lineWidth = 1.2;
    ctx.beginPath();
    route.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))));
    ctx.closePath(); ctx.stroke();
    ctx.restore();

    // trail
    if (st.trail.length > 1) {
      ctx.strokeStyle = "rgba(11, 127, 140, 0.75)"; ctx.lineWidth = 2.2;
      ctx.beginPath();
      st.trail.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))));
      ctx.stroke();
    }

    const r = st.robot;

    // LiDAR
    if (opts.lidar && st.scan) {
      ctx.strokeStyle = "rgba(11, 127, 140, 0.09)"; ctx.lineWidth = 1;
      ctx.beginPath();
      for (const p of st.scan) { ctx.moveTo(X(r.x), Y(r.y)); ctx.lineTo(X(p[0]), Y(p[1])); }
      ctx.stroke();
      ctx.fillStyle = "#0b9aa8";
      for (const p of st.scan) if (p[2]) ctx.fillRect(X(p[0]) - 1.2, Y(p[1]) - 1.2, 2.4, 2.4);
    }

    // camera footprint
    const cam = st.cam;
    if (cam && cam.dist > 0.2) {
      const spread = 0.42, L = cam.dist + 0.9;
      ctx.beginPath();
      ctx.moveTo(X(r.x), Y(r.y));
      ctx.lineTo(X(r.x + Math.cos(cam.dir - spread) * L), Y(r.y + Math.sin(cam.dir - spread) * L));
      ctx.lineTo(X(r.x + Math.cos(cam.dir + spread) * L), Y(r.y + Math.sin(cam.dir + spread) * L));
      ctx.closePath();
      const hot = st.score > st.threshold;
      ctx.fillStyle = hot ? "rgba(217, 58, 43, 0.16)" : "rgba(224, 138, 0, 0.12)";
      ctx.fill();
      ctx.strokeStyle = hot ? "rgba(217, 58, 43, 0.7)" : "rgba(224, 138, 0, 0.55)";
      ctx.lineWidth = 1; ctx.stroke();
    }

    // hotspots
    ctx.font = "600 11px 'IBM Plex Mono', monospace";
    st.hotspots.forEach((h, i) => {
      const hx = X(h.x), hy = Y(h.y), k = 7;
      if (i === st.hotspots.length - 1 && st.time - h.t < 6) {
        const ph = ((st.time - h.t) % 1.5) / 1.5;
        ctx.strokeStyle = `rgba(217, 58, 43, ${1 - ph})`; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(hx, hy, 8 + ph * 22, 0, Math.PI * 2); ctx.stroke();
      }
      ctx.beginPath(); ctx.moveTo(hx, hy - k); ctx.lineTo(hx + k, hy); ctx.lineTo(hx, hy + k); ctx.lineTo(hx - k, hy); ctx.closePath();
      ctx.fillStyle = h.falsePositive ? "#8794a1" : "#d93a2b"; ctx.fill();
      ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.fillStyle = h.falsePositive ? "#5b6b7a" : "#a32a1e";
      ctx.fillText(h.id, hx + 10, hy - 6);
    });

    // robot
    const bodyL = Math.max(0.62 * s, 20), bodyW = Math.max(0.48 * s, 15);
    ctx.save();
    ctx.translate(X(r.x), Y(r.y));
    if (st.mode === "sample" || st.mode === "halt" || st.mode === "pin") {
      const ph = (st.time % 1) / 1;
      ctx.strokeStyle = `rgba(224, 138, 0, ${0.9 - ph * 0.9})`; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(0, 0, bodyL * 0.7 + ph * 16, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.rotate(r.heading);
    ctx.fillStyle = "#0b1016";
    const wl = bodyL * 0.3, ww = bodyW * 0.22;
    for (const [wx, wy] of [[-bodyL * 0.32, -bodyW / 2 - ww * 0.4], [bodyL * 0.32, -bodyW / 2 - ww * 0.4], [-bodyL * 0.32, bodyW / 2 + ww * 0.4], [bodyL * 0.32, bodyW / 2 + ww * 0.4]]) {
      ctx.fillStyle = "#4a5661"; ctx.fillRect(wx - wl / 2, wy - ww / 2, wl, ww);
    }
    ctx.fillStyle = "#26323e"; ctx.fillRect(-bodyL / 2, -bodyW / 2, bodyL, bodyW);
    ctx.fillStyle = "#e9eef2"; ctx.fillRect(-bodyL / 2 + 2, -bodyW / 2 + 2, bodyL * 0.42, bodyW - 4);
    ctx.fillStyle = "#e08a00";
    ctx.beginPath(); ctx.moveTo(bodyL / 2 + 6, 0); ctx.lineTo(bodyL / 2 - 1, -5); ctx.lineTo(bodyL / 2 - 1, 5); ctx.closePath(); ctx.fill();
    ctx.restore();

    // scale bar + north arrow
    const sx = X(0) + 14, sy = Y(H) - 16;
    ctx.strokeStyle = "#5b6b7a"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx + 5 * s, sy); ctx.moveTo(sx, sy - 4); ctx.lineTo(sx, sy + 4); ctx.moveTo(sx + 5 * s, sy - 4); ctx.lineTo(sx + 5 * s, sy + 4); ctx.stroke();
    ctx.fillStyle = "#5b6b7a"; ctx.font = "11px 'IBM Plex Mono', monospace";
    ctx.fillText("5 m", sx + 5 * s + 8, sy + 4);
    const nx = X(W) - 22, ny = Y(0) + 26;
    ctx.beginPath(); ctx.moveTo(nx, ny - 14); ctx.lineTo(nx + 6, ny + 2); ctx.lineTo(nx, ny - 2); ctx.lineTo(nx - 6, ny + 2); ctx.closePath();
    ctx.fillStyle = "#5b6b7a"; ctx.fill();
    ctx.fillText("N", nx - 4, ny + 16);

    return view;
  }

  function drawPollutant(ctx, x, y, s, a, t) {
    const R = 0.55 * s;
    ctx.save();
    ctx.globalAlpha = a.found ? 0.5 : 0.75;
    if (a.type === "oil") {
      const cols = ["rgba(167,123,255,0.45)", "rgba(80,200,255,0.4)", "rgba(120,255,170,0.35)", "rgba(255,210,90,0.35)"];
      cols.forEach((c, i) => {
        ctx.fillStyle = c;
        ctx.beginPath();
        ctx.ellipse(x + Math.sin(t * 0.6 + i) * 2, y + Math.cos(t * 0.5 + i) * 2, R * (1 - i * 0.18), R * (0.7 - i * 0.12), 0.4 + i * 0.3, 0, Math.PI * 2);
        ctx.fill();
      });
    } else if (a.type === "foam") {
      ctx.fillStyle = "rgba(255,255,255,0.95)"; ctx.strokeStyle = "rgba(90,110,125,0.6)"; ctx.lineWidth = 1;
      for (let i = 0; i < 9; i++) {
        const ang = i * 2.4, rr = R * 0.55 * ((i % 3) / 3 + 0.3);
        ctx.beginPath(); ctx.arc(x + Math.cos(ang) * rr, y + Math.sin(ang) * rr * 0.8, R * 0.22, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      }
    } else if (a.type === "sludge") {
      ctx.fillStyle = "rgba(118,82,40,0.85)";
      ctx.beginPath(); ctx.ellipse(x, y, R, R * 0.65, -0.3, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "rgba(70,48,22,0.9)";
      ctx.beginPath(); ctx.ellipse(x + R * 0.2, y - R * 0.1, R * 0.45, R * 0.3, 0.5, 0, Math.PI * 2); ctx.fill();
    } else {
      ctx.strokeStyle = "rgba(90,180,80,0.9)"; ctx.lineWidth = 2;
      for (let i = 0; i < 10; i++) {
        const ang = i * 0.63;
        ctx.beginPath(); ctx.moveTo(x, y);
        ctx.quadraticCurveTo(x + Math.cos(ang) * R * 0.5, y + Math.sin(ang) * R * 0.2, x + Math.cos(ang) * R, y + Math.sin(ang) * R * 0.6);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  B6.world = {
    W, H, HALF, cy, northNormal, bankPt, route, routeLen, routeAt, cameraTarget, inCreek, raycast,
    addHeat, clearHeat, renderMap, POLLUTANT_COLORS,
  };
})();
