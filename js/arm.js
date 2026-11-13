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
  function render(k, fill, ripple, t) {
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
    c.fillStyle = "#16202a";
    c.fillRect(0, groundY, bankX, h - groundY);
    c.fillStyle = "#0d3540";
    c.fillRect(bankX, waterY, w - bankX, h - waterY);
    c.strokeStyle = "rgba(80,190,205,0.35)"; c.lineWidth = 1;
    for (let i = 0; i < 8; i++) {
      const x = bankX + ((i * 47 + t * 18) % (w - bankX));
      c.beginPath(); c.moveTo(x, waterY + 6 + (i % 3) * 7); c.lineTo(x + 14, waterY + 6 + (i % 3) * 7); c.stroke();
    }
    c.strokeStyle = "#3a5468"; c.lineWidth = 1.5;
    c.beginPath(); c.moveTo(0, groundY); c.lineTo(bankX, groundY); c.lineTo(bankX, h); c.stroke();
    c.strokeStyle = "rgba(43,179,192,0.6)";
    c.beginPath(); c.moveTo(bankX, waterY); c.lineTo(w, waterY); c.stroke();

    // robot body
    const wr = robotH * 0.42;
    c.fillStyle = "#2a3540";
    c.beginPath(); c.arc(robotX + wr * 1.1, groundY - wr, wr, 0, Math.PI * 2); c.fill();
    c.beginPath(); c.arc(robotX + robotW - wr * 1.1, groundY - wr, wr, 0, Math.PI * 2); c.fill();
    c.fillStyle = "#e6edf3";
    c.fillRect(robotX, groundY - wr - robotH * 0.85, robotW, robotH * 0.85);
    c.fillStyle = "#1b232c";
    c.fillRect(robotX + 3, groundY - wr - robotH * 0.85 + 3, robotW * 0.35, robotH * 0.85 - 6);
    c.fillStyle = "#ffb020";
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
    c.strokeStyle = "#e9eef2"; c.lineWidth = 10;
    c.beginPath(); c.moveTo(pts[0][0], pts[0][1]); c.lineTo(pts[1][0], pts[1][1]); c.lineTo(pts[2][0], pts[2][1]); c.stroke();
    c.lineWidth = 7;
    c.beginPath(); c.moveTo(pts[2][0], pts[2][1]); c.lineTo(pts[3][0], pts[3][1]); c.stroke();
    c.fillStyle = "#8193a3";
    for (let i = 0; i < 3; i++) { c.beginPath(); c.arc(pts[i][0], pts[i][1], 4.5, 0, Math.PI * 2); c.fill(); }
    // gripper
    const tip = pts[3], a3 = ang[2];
    const nx = Math.cos(a3 + Math.PI / 2), ny = -Math.sin(a3 + Math.PI / 2);
    const fx = Math.cos(a3), fy = -Math.sin(a3);
    c.strokeStyle = "#ffb020"; c.lineWidth = 3;
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
        c.strokeStyle = `rgba(150, 220, 230, ${0.7 * (1 - ph)})`; c.lineWidth = 1.5;
        c.beginPath(); c.ellipse(tip[0], waterY, 6 + ph * 34, 2 + ph * 6, 0, 0, Math.PI * 2); c.stroke();
      }
    }

    // sample vial
    const vx = w - 46, vy = h * 0.12, vw = 18, vh = h * 0.42;
    c.strokeStyle = "#7f8d9b"; c.lineWidth = 1.5;
    c.strokeRect(vx, vy, vw, vh);
    c.fillStyle = "#2bb3c0";
    c.fillRect(vx + 2, vy + vh - 2 - (vh - 4) * fill, vw - 4, (vh - 4) * fill);
    c.fillStyle = "#7f8d9b"; c.font = "10px 'IBM Plex Mono', monospace";
    c.fillText("VIAL", vx - 2, vy + vh + 14);
    c.fillStyle = fill > 0 ? "#2bb3c0" : "#55636f";
    c.fillText(`${Math.round(fill * 100)}%`, vx - 2, vy + vh + 26);

    // joints readout
    for (let i = 0; i < 6; i++) {
      const v = J_STOW[i] + (J_DIP[i] - J_STOW[i]) * e;
      cells[i].textContent = `${v.toFixed(1)}°`;
    }
  }

  B6.arm = { init, render };
})();
