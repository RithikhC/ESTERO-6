// Bayani-6 dashboard: mission simulation (patrol -> detect -> halt -> pin -> sample -> map)
// and all UI wiring.
(function () {
  const B6 = window.B6, world = B6.world;
  const $ = (id) => document.getElementById(id);

  const SPEED = 0.62;          // m/s patrol speed
  const STEP = 0.05;           // 20 Hz simulation tick
  const SIGMA = 0.85;          // camera footprint response (m)
  const WAYPOINTS = 14;
  const RACK = 6;               // sample tubes on the rear deck
  const TYPE_LABEL = { oil: "Oil sheen", foam: "Foam", sludge: "Sludge", weeds: "Weed mat" };

  const st = {};
  let mapView = null;

  function gauss() { let u = 0, v = 0; while (!u) u = Math.random(); while (!v) v = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
  function lerpAngle(a, b, f) { let d = b - a; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return a + d * f; }
  const fmt = (n, d = 2) => n.toFixed(d);
  function clock(t) {
    const s = Math.floor(t), h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), x = s % 60;
    return [h, m, x].map((v) => String(v).padStart(2, "0")).join(":");
  }

  // ---------------- state ----------------
  function makeAnomaly(id, type, x, off, sev) {
    const p = world.bankPt(x, off);
    return { id, type, x: p[0], y: p[1], sev, found: false };
  }

  function reset() {
    Object.assign(st, {
      running: false, speed: st.speed || 1, time: 0, anim: st.anim || 0, missionT: 0,
      s: 0.4, robot: { x: 0, y: 0, heading: 0 }, trail: [], scan: null, cam: null,
      score: 0.1, noise: 0.03, threshold: st.threshold || 0.8, mode: "standby", modeT: 0,
      glare: { t: 0, dur: 0, amp: 0, x: 0, val: 0 }, dist: 0, lastTrail: 0,
      samples: 0, falseHalts: 0, battery: 100, cooldown: 0, history: [], pending: null,
      hotspots: [], rack: Array(RACK).fill("empty"), lastSkip: null, scrollPx: 0, fps: 32.4, armK: 0, armFill: 0, ripple: -1, nextAnomId: 6,
      anomalies: [
        makeAnomaly(1, "oil", 9.0, 0.3, 0.93),
        makeAnomaly(2, "foam", 14.5, -0.4, 0.9),
        makeAnomaly(3, "sludge", 20.0, 0.1, 0.95),
        makeAnomaly(4, "weeds", 25.5, -0.3, 0.88),
        makeAnomaly(5, "oil", 29.5, 0.35, 0.86),
      ],
    });
    const p = world.routeAt(st.s);
    st.robot = { x: p.x, y: p.y, heading: p.heading };
    world.clearHeat();
    $("log").innerHTML = "";
    log("info", "bayani_master: node up · ROS 2 Humble · sim clock");
    log("info", `nav2: ${WAYPOINTS} waypoints loaded · patrol loop ${fmt(world.routeLen, 1)} m`);
    log("info", "cartographer: map frame ready · 2D LiDAR @ 10 Hz");
    log("info", "anomaly_detector: TensorRT engine loaded (anomalynet_fp16.engine)");
    log("info", "anomaly_detector: baseline = 60 s clean-water video (1800 frames)");
    log("info", "ready · press Run to start the patrol");
    renderTable();
    updateRunButton();
  }

  // ---------------- logging ----------------
  function log(level, msg) {
    const el = $("log");
    const p = document.createElement("p");
    p.className = level;
    const tag = level === "hot" ? "HOT " : level === "warn" ? "WARN" : "INFO";
    const t = st.missionT || 0;
    const ts = `${String(Math.floor(t / 60)).padStart(2, "0")}:${(t % 60).toFixed(1).padStart(4, "0")}`;
    p.innerHTML = `<span class="t">${ts}</span><span class="lv">${tag}</span>`;
    p.appendChild(document.createTextNode(msg));
    el.appendChild(p);
    while (el.children.length > 160) el.removeChild(el.firstChild);
    el.scrollTop = el.scrollHeight;
  }

  // ---------------- simulation ----------------
  function setMode(m) { st.mode = m; st.modeT = 0; }

  function step(dt) {
    st.time += dt;
    st.missionT += dt;
    st.modeT += dt;
    st.battery = Math.max(5, st.battery - dt * 0.012);
    st.fps = 32.4 + Math.sin(st.time * 0.7) * 0.5 + (Math.random() - 0.5) * 0.4;

    // motion
    let v = 0;
    if (st.mode === "patrol") v = SPEED;
    else if (st.mode === "halt") v = SPEED * Math.max(0, 1 - st.modeT / 0.8);
    if (v > 0) {
      st.s += v * dt;
      st.dist += v * dt;
      const p = world.routeAt(st.s);
      const ox = st.robot.x, oy = st.robot.y;
      st.robot.x = p.x; st.robot.y = p.y;
      // mecanum crab-walk: on the banks the body (and its forward camera) faces the water
      const look = world.cameraTarget(p.x, p.y);
      const want = look.dist < 0.3 ? p.heading : look.dir;
      st.robot.heading = lerpAngle(st.robot.heading, want, 1 - Math.exp(-dt * 6));
      if (st.dist - st.lastTrail > 0.25) {
        st.trail.push([p.x, p.y]); st.lastTrail = st.dist;
        if (st.trail.length > 320) st.trail.shift();
      }
      if (st.cam) {
        const rx = Math.cos(st.cam.dir + Math.PI / 2), ry = Math.sin(st.cam.dir + Math.PI / 2);
        st.scrollPx -= ((p.x - ox) * rx + (p.y - oy) * ry) * 58;
      }
    }

    // camera footprint
    const ct = world.cameraTarget(st.robot.x, st.robot.y);
    if (ct.dist < 0.3) {
      const h = st.robot.heading;
      st.cam = { x: st.robot.x + Math.cos(h) * 2, y: st.robot.y + Math.sin(h) * 2, dir: h, dist: 2 };
    } else st.cam = ct;

    // anomaly score
    let anom = 0, best = null;
    for (const a of st.anomalies) {
      const d2 = (a.x - st.cam.x) ** 2 + (a.y - st.cam.y) ** 2;
      const r = a.sev * Math.exp(-d2 / (2 * SIGMA * SIGMA));
      if (r > anom) { anom = r; best = a; }
    }
    const g = st.glare;
    if (g.t > 0) {
      g.t += dt;
      g.val = g.amp * Math.sin(Math.PI * Math.min(1, g.t / g.dur));
      if (g.t >= g.dur) { g.t = 0; g.val = 0; }
    } else if (st.mode === "patrol" && Math.random() < dt * 0.09) {
      Object.assign(g, { t: 0.0001, dur: 0.5 + Math.random() * 0.4, amp: 0.38 + Math.random() * 0.22, x: 40 + Math.random() * 176 });
    }
    st.noise = Math.min(0.08, Math.max(0, st.noise + (Math.random() - 0.5) * 0.03));
    const raw = Math.min(0.99, 0.09 + st.noise + Math.max(anom, g.val));
    st.score += (raw - st.score) * 0.5;
    st.history.push({ v: st.score, halt: false });
    if (st.history.length > 400) st.history.shift();

    if (st.score < st.threshold * 0.7) st.lastSkip = null;
    st.cooldown -= dt;

    switch (st.mode) {
      case "patrol":
        if (st.score > st.threshold && st.cooldown <= 0) {
          const known = st.hotspots.find((h) => Math.hypot(h.x - st.cam.x, h.y - st.cam.y) < 1.6);
          if (known) {
            if (st.lastSkip !== known.id) { log("info", `score ${fmt(st.score)} at known hotspot ${known.id} · continuing patrol`); st.lastSkip = known.id; }
          } else {
            const real = anom >= g.val && anom > 0.3 ? best : null;
            st.pending = { anomaly: real, score: st.score, camX: st.cam.x, camY: st.cam.y };
            st.history[st.history.length - 1].halt = true;
            log("warn", `anomaly_score ${fmt(st.score)} > ${fmt(st.threshold)} · pausing Nav2`);
            setMode("halt");
          }
        }
        break;
      case "halt":
        st.pending.score = Math.max(st.pending.score, st.score);
        if (st.modeT >= 0.8) { setMode("pin"); }
        break;
      case "pin":
        if (st.modeT >= 0.6) {
          pin();
          if (st.samples >= RACK) {
            const hs = st.hotspots[st.hotspots.length - 1];
            hs.status = "ns";
            log("warn", `sample rack full (${RACK}/${RACK}) · ${hs.id} logged without a sample`);
            renderTable();
            setMode("map");
          } else {
            setMode("sample");
            log("info", `arm_sampler: dip_sample · picking tube from rack slot ${st.samples + 1}`);
          }
        }
        break;
      case "sample": {
        const t = st.modeT, hs = st.hotspots[st.hotspots.length - 1];
        if (t < 1.5) { st.armK = t / 1.5; st.armFill = 0; st.ripple = -1; }
        else if (t < 4.5) {
          if (st.ripple < 0) log("info", `sample #${st.samples + 1}: gripper at water line · holding 3.0 s`);
          st.armK = 1; st.armFill = (t - 1.5) / 3; st.ripple = t - 1.5;
        } else if (t < 6) {
          if (st.ripple >= 0) { log("info", `sample #${st.samples + 1} captured · tube back in slot ${st.samples + 1}, stowing arm`); st.ripple = -1; }
          st.armK = 1 - (t - 4.5) / 1.5; st.armFill = 1;
        } else {
          st.armK = 0; st.rack[st.samples] = "full"; st.samples++;
          hs.status = hs.falsePositive ? "fp" : "ok";
          if (hs.falsePositive) { st.falseHalts++; log("warn", `ground truth: ${hs.id} was surface glare · false halt`); }
          renderTable();
          log("info", `hotspots.csv += ${hs.id}, ${fmt(hs.x)}, ${fmt(hs.y)}, ${fmt(hs.score)}`);
          setMode("map");
        }
        break;
      }
      case "map":
        if (st.modeT >= 0.6) {
          const wp = Math.floor((world.routeAt(st.s).seg / world.route.length) * WAYPOINTS) + 1;
          log("info", `nav2: resuming patrol · waypoint ${wp}/${WAYPOINTS}`);
          st.cooldown = 2.5; st.armFill = 0;
          setMode("patrol");
        }
        break;
    }
  }

  function pin() {
    const p = st.pending, a = p.anomaly;
    const ex = a ? a.x + gauss() * 0.09 : p.camX + gauss() * 0.15;
    const ey = a ? a.y + gauss() * 0.09 : p.camY + gauss() * 0.15;
    const id = "H" + (st.hotspots.length + 1);
    const h = {
      id, x: ex, y: ey, score: p.score, type: a ? a.type : null, falsePositive: !a,
      err: a ? Math.hypot(ex - a.x, ey - a.y) : null, status: "run", t: st.anim, missionT: st.missionT,
    };
    st.hotspots.push(h);
    if (a) a.found = true;
    world.addHeat(ex, ey, a ? 0.9 * a.sev : 0.25, 0.8);
    log("info", `tf2: map→base_link (${fmt(st.robot.x)}, ${fmt(st.robot.y)}) yaw ${fmt(st.robot.heading * 57.3, 0)}°`);
    log("hot", `hotspot ${id} pinned at (${fmt(ex)}, ${fmt(ey)}) · score ${fmt(p.score)}`);
    renderTable(true);
  }

  // ---------------- rendering ----------------
  const camItems = () => {
    const c = st.cam; if (!c) return [];
    const rx = Math.cos(c.dir + Math.PI / 2), ry = Math.sin(c.dir + Math.PI / 2);
    const fx = Math.cos(c.dir), fy = Math.sin(c.dir);
    const CW = B6.camera.CW, CH = B6.camera.CH, out = [];
    for (const a of st.anomalies) {
      const vx = a.x - c.x, vy = a.y - c.y;
      const u = vx * rx + vy * ry, dpt = vx * fx + vy * fy;
      if (Math.abs(u) > 2.8 || Math.abs(dpt) > 1.9) continue;
      out.push({
        type: a.type, seed: a.id * 13.7, vis: 1,
        px: CW / 2 + (u / 2.2) * (CW / 2), py: CH * 0.55 - (dpt / 1.9) * CH * 0.42,
        R: (0.62 / 2.2) * (CW / 2) * (1 - dpt * 0.08),
      });
    }
    return out;
  };

  function sizeCanvas(cv) {
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    const ctx = cv.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w, h };
  }

  function computeScan() {
    const pts = [], n = 120, R = 8;
    for (let i = 0; i < n; i++) {
      const ang = (i / n) * Math.PI * 2;
      const d = world.raycast(st.robot.x, st.robot.y, ang, R);
      pts.push([st.robot.x + Math.cos(ang) * d, st.robot.y + Math.sin(ang) * d, d < R]);
    }
    return pts;
  }

  function renderChart() {
    const { ctx: c, w, h } = sizeCanvas($("chartCanvas"));
    c.clearRect(0, 0, w, h);
    const L = 36, R = 12, T = 12, Bm = 22;
    const gw = w - L - R, gh = h - T - Bm;
    const Y = (v) => T + gh * (1 - v);
    c.font = "10.5px 'IBM Plex Mono', monospace";
    for (let v = 0; v <= 1.001; v += 0.2) {
      c.strokeStyle = "#e6ebf0"; c.lineWidth = 1;
      c.beginPath(); c.moveTo(L, Math.round(Y(v)) + 0.5); c.lineTo(w - R, Math.round(Y(v)) + 0.5); c.stroke();
      c.fillStyle = "#8794a1"; c.fillText(v.toFixed(1), 6, Y(v) + 4);
    }
    c.fillStyle = "#8794a1";
    c.fillText("last 20 s", w - R - 58, h - 6);
    const hist = st.history, N = 400;
    const X = (i) => L + gw * ((i + (N - hist.length)) / (N - 1));
    // halt markers
    hist.forEach((p, i) => {
      if (!p.halt) return;
      c.strokeStyle = "rgba(224,138,0,0.8)"; c.lineWidth = 1.5;
      c.beginPath(); c.moveTo(X(i), T); c.lineTo(X(i), T + gh); c.stroke();
      c.fillStyle = "#b06c00"; c.fillText("HALT", X(i) + 4, T + 10);
    });
    if (hist.length > 1) {
      const grad = c.createLinearGradient(0, T, 0, T + gh);
      grad.addColorStop(0, "rgba(11,127,140,0.22)"); grad.addColorStop(1, "rgba(11,127,140,0)");
      c.beginPath(); c.moveTo(X(0), Y(0));
      hist.forEach((p, i) => c.lineTo(X(i), Y(p.v)));
      c.lineTo(X(hist.length - 1), Y(0)); c.closePath();
      c.fillStyle = grad; c.fill();
      c.lineWidth = 1.8;
      for (let i = 1; i < hist.length; i++) {
        c.strokeStyle = hist[i].v > st.threshold ? "#d93a2b" : "#0b7f8c";
        c.beginPath(); c.moveTo(X(i - 1), Y(hist[i - 1].v)); c.lineTo(X(i), Y(hist[i].v)); c.stroke();
      }
    }
    c.save();
    c.setLineDash([6, 5]); c.strokeStyle = "#e08a00"; c.lineWidth = 1.2;
    c.beginPath(); c.moveTo(L, Y(st.threshold)); c.lineTo(w - R, Y(st.threshold)); c.stroke();
    c.restore();
    c.fillStyle = "#b06c00"; c.fillText(`trigger ${st.threshold.toFixed(2)}`, L + 6, Y(st.threshold) - 5);
  }

  function renderTable(flash) {
    const tb = document.querySelector("#hotTable tbody");
    $("hotEmpty").style.display = st.hotspots.length ? "none" : "block";
    tb.innerHTML = "";
    st.hotspots.slice().reverse().forEach((h, idx) => {
      const tr = document.createElement("tr");
      if (flash && idx === 0) tr.className = "new";
      const col = h.type ? world.POLLUTANT_COLORS[h.type] : "#8794a1";
      const app = h.type ? TYPE_LABEL[h.type] : "Glare";
      const stat = h.status === "ok" ? '<span class="st ok">SAMPLED</span>' : h.status === "fp" ? '<span class="st fp">FALSE +</span>' : h.status === "ns" ? '<span class="st ns">RACK FULL</span>' : '<span class="st run">IN PROGRESS</span>';
      tr.innerHTML = `<td class="id">${h.id}</td><td>${fmt(h.x)}</td><td>${fmt(h.y)}</td><td>${fmt(h.score)}</td><td><span class="swatch" style="background:${col}"></span>${app}</td><td>${stat}</td>`;
      tb.appendChild(tr);
    });
  }

  const STAGES = { patrol: [0, 1, 2], halt: [3], pin: [4], sample: [5], map: [6] };
  function renderChrome() {
    const pill = $("statusPill");
    const label = !st.running && st.mode !== "standby" ? "PAUSED" : { standby: "STANDBY", patrol: "PATROLLING", halt: "HALTED", pin: "PINNING", sample: "SAMPLING", map: "LOGGING" }[st.mode];
    $("statusText").textContent = label;
    pill.className = "status-pill " + (!st.running ? "" : st.mode === "patrol" ? "patrol" : st.mode === "sample" || st.mode === "pin" || st.mode === "map" ? "sample" : st.mode === "halt" ? "halt" : "");
    $("missionClock").textContent = clock(st.missionT);
    $("battery").textContent = `${Math.round(st.battery)}%`;
    const lit = STAGES[st.mode] || [];
    document.querySelectorAll("#pipeline li").forEach((li) => {
      const k = +li.dataset.stage;
      li.classList.toggle("on", st.mode === "patrol" && lit.includes(k));
      li.classList.toggle("alert", st.mode !== "patrol" && lit.includes(k));
    });
    $("statDist").textContent = `${fmt(st.dist, 1)} m`;
    $("statHot").textContent = st.hotspots.filter((h) => !h.falsePositive).length;
    $("statSamples").textContent = st.samples;
    $("statFalse").textContent = st.falseHalts;
    const errs = st.hotspots.filter((h) => h.err != null).map((h) => h.err);
    $("statErr").textContent = errs.length ? `${fmt(errs.reduce((a, b) => a + b, 0) / errs.length)} m` : "—";
    $("statFps").textContent = st.running ? fmt(st.fps, 1) : "—";
    const armTag = $("armState");
    const armLabel = st.mode === "sample" ? (st.modeT < 1.5 ? "DESCENDING" : st.modeT < 4.5 ? "SAMPLING" : "STOWING") : "STOWED";
    armTag.textContent = armLabel;
    armTag.classList.toggle("active", st.mode === "sample");
  }

  function renderMission() {
    const mc = sizeCanvas($("mapCanvas"));
    st.scan = $("togLidar").checked ? computeScan() : null;
    mapView = world.renderMap(mc.ctx, mc.w, mc.h, { ...st, time: st.anim }, {
      lidar: $("togLidar").checked, heat: $("togHeat").checked, truth: $("togTruth").checked,
    });
    // hotspot pulse uses sim time
    B6.camera.render({
      t: st.anim, scroll: st.scrollPx, items: camItems(), score: st.score, threshold: st.threshold,
      fps: st.running ? st.fps : 0, mode: st.mode, glare: st.glare.val / 0.6, glareX: st.glare.x, paused: !st.running,
    });
    const rack = st.rack.slice();
    if (st.mode === "sample" && st.samples < RACK) rack[st.samples] = "active";
    B6.arm.render(st.armK, st.armFill, st.ripple, st.anim, rack);
    renderChart();
  }

  // ---------------- main loop ----------------
  let last = performance.now(), acc = 0;
  function frame(now) {
    const dt = Math.min(0.1, (now - last) / 1000); last = now;
    st.anim += dt;
    if (st.running) {
      acc += dt * st.speed;
      let n = 0;
      while (acc >= STEP && n < 40) { step(STEP); acc -= STEP; n++; }
    }
    if ($("view-mission").classList.contains("active")) renderMission();
    renderChrome();
    requestAnimationFrame(frame);
  }

  // ---------------- UI ----------------
  function updateRunButton() {
    const b = $("btnRun");
    b.textContent = st.running ? "Pause" : "Run";
    b.classList.toggle("run", !st.running);
  }
  function setRunning(on) {
    if (on && st.mode === "standby") { setMode("patrol"); log("info", "nav2: following waypoints · patrol started"); }
    st.running = on;
    updateRunButton();
  }
  function showTab(name) {
    document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("active", t.dataset.tab === name));
    document.querySelectorAll(".view").forEach((v) => v.classList.toggle("active", v.id === "view-" + name));
    window.scrollTo(0, 0);
    history.replaceState(null, "", "#" + name);
  }
  function setSpeed(x) {
    st.speed = x;
    document.querySelectorAll("#speedSeg button").forEach((b) => b.classList.toggle("on", +b.dataset.speed === x));
  }
  function toggleDrop(on) {
    const wrap = $("mapWrap");
    const v = on === undefined ? !wrap.classList.contains("dropping") : on;
    wrap.classList.toggle("dropping", v);
    $("btnDrop").classList.toggle("on", v);
  }
  function exportCsv() {
    const rows = [["id", "mission_time_s", "x_m", "y_m", "score", "appearance", "rack_slot", "sample_status"]];
    st.hotspots.forEach((h) => rows.push([h.id, fmt(h.missionT, 1), fmt(h.x, 3), fmt(h.y, 3), fmt(h.score, 3), h.type || "glare", h.status === "ns" ? "" : "slot " + (st.hotspots.filter((x) => x.status !== "ns").indexOf(h) + 1), h.status === "ok" ? "sampled" : h.status === "fp" ? "false_positive" : h.status === "ns" ? "no_sample_rack_full" : "in_progress"]));
    const blob = new Blob([rows.map((r) => r.join(",")).join("\n") + "\n"], { type: "text/csv" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "bayani6_hotspots.csv";
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    log("info", `exported ${st.hotspots.length} hotspot(s) → bayani6_hotspots.csv`);
  }

  document.querySelectorAll(".tab").forEach((t) => t.addEventListener("click", () => showTab(t.dataset.tab)));
  document.querySelectorAll("[data-goto]").forEach((b) => b.addEventListener("click", () => {
    showTab(b.dataset.goto);
    if (b.dataset.goto === "mission" && !st.running) setRunning(true);
  }));
  $("btnRun").addEventListener("click", () => setRunning(!st.running));
  $("btnReset").addEventListener("click", () => { const sp = st.speed; reset(); setSpeed(sp); });
  document.querySelectorAll("#speedSeg button").forEach((b) => b.addEventListener("click", () => setSpeed(+b.dataset.speed)));
  $("thr").addEventListener("input", (e) => { st.threshold = +e.target.value; $("thrVal").textContent = st.threshold.toFixed(2); });
  $("thr").addEventListener("change", () => log("info", `operator: trigger threshold set to ${st.threshold.toFixed(2)}`));
  $("btnCsv").addEventListener("click", exportCsv);
  $("btnDrop").addEventListener("click", () => toggleDrop());

  const mapCanvas = $("mapCanvas");
  function toWorld(e) {
    if (!mapView) return null;
    const r = mapCanvas.getBoundingClientRect();
    return { x: (e.clientX - r.left - mapView.ox) / mapView.s, y: (e.clientY - r.top - mapView.oy) / mapView.s };
  }
  mapCanvas.addEventListener("mousemove", (e) => {
    const p = toWorld(e);
    if (p && p.x >= 0 && p.x <= world.W && p.y >= 0 && p.y <= world.H) $("cursorPos").textContent = `x ${fmt(p.x)} m   y ${fmt(p.y)} m`;
    else $("cursorPos").textContent = "x —  y —";
  });
  mapCanvas.addEventListener("click", (e) => {
    if (!$("mapWrap").classList.contains("dropping")) return;
    const p = toWorld(e);
    if (!p || !world.inCreek(p.x, p.y)) { log("warn", "drop rejected · pollutant must be inside the creek"); return; }
    const type = $("dropType").value;
    st.anomalies.push({ id: st.nextAnomId++, type, x: p.x, y: p.y, sev: 0.92, found: false });
    log("info", `operator: dropped ${TYPE_LABEL[type].toLowerCase()} at (${fmt(p.x)}, ${fmt(p.y)})`);
    toggleDrop(false);
  });

  document.addEventListener("keydown", (e) => {
    if (!$("view-mission").classList.contains("active")) return;
    if (["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement.tagName) && e.key !== "Escape") return;
    if (e.code === "Space") { e.preventDefault(); setRunning(!st.running); }
    else if (e.key === "1" || e.key === "2" || e.key === "4") setSpeed(+e.key);
    else if (e.key === "h" || e.key === "H") $("togHeat").click();
    else if (e.key === "l" || e.key === "L") $("togLidar").click();
    else if (e.key === "p" || e.key === "P") toggleDrop();
    else if (e.key === "Escape") toggleDrop(false);
  });

  B6.camera.init($("camCanvas"), $("camOverlay"));
  B6.arm.init($("armCanvas"), $("joints"));
  st.speed = 1; st.threshold = 0.8; st.anim = 0;
  reset();
  document.body.classList.add("instant"); // no fade on first load
  document.querySelectorAll(".tab, [data-goto]").forEach((b) => b.addEventListener("click", () => document.body.classList.remove("instant")));
  const initial = (location.hash || "#overview").slice(1);
  if (["overview", "mission", "how", "hardware"].includes(initial)) showTab(initial);
  // ?thr=0.6 sets the trigger threshold (for testing the glare trade-off)
  const thrQ = +new URLSearchParams(location.search).get("thr");
  if (thrQ >= 0.5 && thrQ <= 0.95) { st.threshold = thrQ; $("thr").value = thrQ; $("thrVal").textContent = thrQ.toFixed(2); }
  // ?demo opens the live mission already running (handy for presenting); ?demo=4 sets the speed
  const demo = new URLSearchParams(location.search).get("demo");
  if (demo !== null) {
    showTab("mission");
    if (["1", "2", "4"].includes(demo)) setSpeed(+demo);
    setRunning(true);
    // ?warp=N fast-forwards N simulated seconds on load (used for screenshots/testing)
    const warp = Math.min(600, +new URLSearchParams(location.search).get("warp") || 0);
    for (let i = 0; i < warp / STEP; i++) step(STEP);
  }
  requestAnimationFrame(frame);
})();
