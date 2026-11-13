// Bayani-6 dashboard: side-view myCobot arm animation for the dip-sampling sequence.
(function () {
  const B6 = (window.B6 = window.B6 || {});
  const deg = Math.PI / 180;
  const STOW = [85, 100, 70];     // absolute link angles (deg, y up)
  const DIP = [20, -60, -90];
  const J_STOW = [0, 0, 0, 0, 0, 0];
  const J_DIP = [-90, -58, -72, -40, 0, 12];

  let canvas, ctx, jointsEl, cells = [];

  function init(c, j) {
    canvas = c; ctx = c.getContext("2d"); jointsEl = j;
    jointsEl.innerHTML = "";
    for (let i = 0; i < 6; i++) {
      const d = document.createElement("div");
      d.innerHTML = `J${i + 1}<b>0.0°</b>`;
      jointsEl.appendChild(d);
      cells.push(d.querySelector("b"));
    }
  }

  const ease = (x) => (x < 0.5 ? 2 * x * x : 1 - Math.pow(-2 * x + 2, 2) / 2);

  // k: 0 = stowed, 1 = dipped. fill: sample vial 0..1. ripple: seconds since water contact (or -1)
  function render(k, fill, ripple, t, rack) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    const c = ctx;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);
    const e = ease(Math.min(1, Math.max(0, k)));

    const groundY = h * 0.78, waterY = groundY + h * 0.04;
    const robotX = w * 0.16, robotW = Math.min(w * 0.2, h * 0.55), robotH = h * 0.13;
    const baseX = robotX + robotW * 0.62, baseY = groundY - h * 0.14;
    const bankX = robotX + robotW + h * 0.05;

    // bank + water
    c.fillStyle = "#dfe5ea";
    c.fillRect(0, groundY, bankX, h - groundY);
    c.fillStyle = "#bfe0e8";
    c.fillRect(bankX, waterY, w - bankX, h - waterY);
    c.strokeStyle = "rgba(20,110,130,0.35)"; c.lineWidth = 1;
    for (let i = 0; i < 8; i++) {
      const x = bankX + ((i * 47 + t * 18) % (w - bankX));
      c.beginPath(); c.moveTo(x, waterY + 6 + (i % 3) * 7); c.lineTo(x + 14, waterY + 6 + (i % 3) * 7); c.stroke();
    }
    c.strokeStyle = "#6f8ca0"; c.lineWidth = 1.5;
    c.beginPath(); c.moveTo(0, groundY); c.lineTo(bankX, groundY); c.lineTo(bankX, h); c.stroke();
    c.strokeStyle = "rgba(11,127,140,0.7)";
    c.beginPath(); c.moveTo(bankX, waterY); c.lineTo(w, waterY); c.stroke();

    // robot body
    const wr = robotH * 0.42;
    c.fillStyle = "#4a5661";
    c.beginPath(); c.arc(robotX + wr * 1.1, groundY - wr, wr, 0, Math.PI * 2); c.fill();
    c.beginPath(); c.arc(robotX + robotW - wr * 1.1, groundY - wr, wr, 0, Math.PI * 2); c.fill();
    c.fillStyle = "#26323e";
    c.fillRect(robotX, groundY - wr - robotH * 0.85, robotW, robotH * 0.85);
    c.fillStyle = "#e9eef2";
    c.fillRect(robotX + 3, groundY - wr - robotH * 0.85 + 3, robotW * 0.35, robotH * 0.85 - 6);
    c.fillStyle = "#e08a00";
    c.fillRect(robotX + robotW * 0.42, groundY - wr - robotH * 0.5, robotW * 0.12, 3);
    // arm pedestal
    c.fillStyle = "#8193a3";
    c.fillRect(baseX - 7, baseY, 14, groundY - wr - robotH * 0.85 - baseY);

    // links
    const L = [h * 0.26, h * 0.24, h * 0.1];
    const ang = STOW.map((s, i) => (s + (DIP[i] - s) * e) * deg);
    let px = baseX, py = baseY;
    const pts = [[px, py]];
    for (let i = 0; i < 3; i++) {
      px += Math.cos(ang[i]) * L[i];
      py -= Math.sin(ang[i]) * L[i];
      pts.push([px, py]);
    }
    c.lineCap = "round";
    c.strokeStyle = "#7d8e9c"; c.lineWidth = 12;
    c.beginPath(); c.moveTo(pts[0][0], pts[0][1]); c.lineTo(pts[1][0], pts[1][1]); c.lineTo(pts[2][0], pts[2][1]); c.stroke();
    c.strokeStyle = "#f4f7f9"; c.lineWidth = 9;
    c.beginPath(); c.moveTo(pts[0][0], pts[0][1]); c.lineTo(pts[1][0], pts[1][1]); c.lineTo(pts[2][0], pts[2][1]); c.stroke();
    c.strokeStyle = "#7d8e9c"; c.lineWidth = 9;
    c.beginPath(); c.moveTo(pts[2][0], pts[2][1]); c.lineTo(pts[3][0], pts[3][1]); c.stroke();
    c.strokeStyle = "#f4f7f9"; c.lineWidth = 6;
    c.beginPath(); c.moveTo(pts[2][0], pts[2][1]); c.lineTo(pts[3][0], pts[3][1]); c.stroke();
    c.fillStyle = "#8193a3";
    for (let i = 0; i < 3; i++) { c.beginPath(); c.arc(pts[i][0], pts[i][1], 4.5, 0, Math.PI * 2); c.fill(); }
    // gripper
    const tip = pts[3], a3 = ang[2];
    const nx = Math.cos(a3 + Math.PI / 2), ny = -Math.sin(a3 + Math.PI / 2);
    const fx = Math.cos(a3), fy = -Math.sin(a3);
    c.strokeStyle = "#e08a00"; c.lineWidth = 3;
    for (const sgn of [-1, 1]) {
      c.beginPath();
      c.moveTo(tip[0] + nx * 5 * sgn, tip[1] + ny * 5 * sgn);
      c.lineTo(tip[0] + nx * 4 * sgn + fx * 9, tip[1] + ny * 4 * sgn + fy * 9);
      c.stroke();
    }
    c.lineCap = "butt";

    // ripples where the gripper meets the water
    if (ripple >= 0 && tip[1] + fy * 6 > waterY) {
      for (let i = 0; i < 3; i++) {
        const ph = ((ripple + i * 0.4) % 1.2) / 1.2;
        c.strokeStyle = `rgba(11, 127, 140, ${0.8 * (1 - ph)})`; c.lineWidth = 1.5;
        c.beginPath(); c.ellipse(tip[0], waterY, 6 + ph * 34, 2 + ph * 6, 0, 0, Math.PI * 2); c.stroke();
      }
    }

    // sample rack: 6 screw-cap tubes on the rear deck, slot n = hotspot Hn
    const slots = rack || [];
    const tw = 9, gap = 6, n = 6, th = h * 0.34;
    const rx = w - 14 - n * tw - (n - 1) * gap, ry = h * 0.1;
    c.fillStyle = "#eef2f5"; c.strokeStyle = "#b4c0cb"; c.lineWidth = 1;
    c.fillRect(rx - 6, ry + th - 6, n * tw + (n - 1) * gap + 12, 12);
    c.strokeRect(rx - 6 + 0.5, ry + th - 6 + 0.5, n * tw + (n - 1) * gap + 11, 11);
    c.font = "9px 'IBM Plex Mono', monospace";
    for (let i = 0; i < n; i++) {
      const x = rx + i * (tw + gap), state = slots[i] || "empty";
      const lvl = state === "full" ? 1 : state === "active" ? fill : 0;
      c.fillStyle = "#ffffff"; c.fillRect(x, ry, tw, th);
      if (lvl > 0) { c.fillStyle = "#0b9aa8"; c.fillRect(x + 1.5, ry + th - 1.5 - (th - 3) * lvl, tw - 3, (th - 3) * lvl); }
      c.strokeStyle = state === "active" ? "#e08a00" : "#7d8e9c"; c.lineWidth = state === "active" ? 2 : 1.2;
      c.strokeRect(x + 0.5, ry + 0.5, tw - 1, th - 1);
      c.fillStyle = state === "active" ? "#e08a00" : "#5b6b7a"; c.fillRect(x - 1, ry - 4, tw + 2, 4);
      c.fillStyle = state === "empty" ? "#a3afba" : "#3b4856";
      c.fillText(String(i + 1), x + 1.5, ry + th + 17);
    }
    c.fillStyle = "#5b6b7a"; c.font = "10px 'IBM Plex Mono', monospace";
    const lbl = "SAMPLE RACK";
    c.fillText(lbl, w - 8 - c.measureText(lbl).width, ry + th + 30);

    // joints readout
    for (let i = 0; i < 6; i++) {
      const v = J_STOW[i] + (J_DIP[i] - J_STOW[i]) * e;
      cells[i].textContent = `${v.toFixed(1)}°`;
    }
  }

  B6.arm = { init, render };
})();
