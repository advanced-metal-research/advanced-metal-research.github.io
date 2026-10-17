/* Rosie page hero: after the pen-plot intro (rosie-plot.js) the drawing comes alive, for whichever Rosie is shown.
   The live hidden-line renderer (assets/js/rosie-hlr.js) is prepared during the plot at the SAME home pose,
   camera, scale and position as the plotted SVG (iso-front, ortho), then cross-fades in when the plot ends.
   From then on a seamless motion showcase loops:
     Linear X, Y, Z (TCP on dead-straight lines, constant tool orientation) -> Circle (constant orientation)
     -> Reorient about TCP (tool cone sweep, TCP pinned) -> Joint J4 . J6 -> Joint J1 -> through home -> again
   Cartesian moves are solved by damped least-squares IK at load (sampled tables, warm-started, joint limits
   clamped); the joint moves are C1 Hermite splines, so the loop wraps with matching velocity.
   One choreography for every model: the work region is placed from each arm's own geometry (MODELS below).
   Overlay (Canvas2D): traced TCP paths (active red, finished faint ink fading out), TCP triad with X/Y/Z labels
   (active axis red), dash-dot tool centre line, pinned-TCP marker during the pivot; HUD with the motion name.
   Model switch: buttons with data-rosie-model="600|1000|1400" re-plot the chosen robot, then run its loop;
   elements with data-rosie-fact="<key>" show that model's figures (FACTS), links with data-rosie-href="<key>"
   take that model's URL (the hero pre-order button); a 'rosie:model' event is dispatched.
   - reduced motion / no WebGL2: the switch swaps the static drawing and the figures; nothing moves
   - paused offscreen and while the tab is hidden; colours follow the theme tokens
   Node: require()-able for the planning checks (exports the core when there is no document). */
(function (global) {
  'use strict';

  /* ================================================================ planning core (no DOM) */
  var DEG = Math.PI / 180;
  var MARGIN = 3;                          // deg kept clear of every limit by the planner
  var RATE = 60;                           // IK samples per second of Cartesian motion
  var I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function scl(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function rotAxis(k, ang) {
    var c = Math.cos(ang), s = Math.sin(ang), C = 1 - c, x = k[0], y = k[1], z = k[2];
    return [x * x * C + c, x * y * C - z * s, x * z * C + y * s,
            y * x * C + z * s, y * y * C + c, y * z * C - x * s,
            z * x * C - y * s, z * y * C + x * s, z * z * C + c];
  }
  function mulR(A, B) {
    var o = new Array(9);
    for (var i = 0; i < 3; i++) for (var j = 0; j < 3; j++) o[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
    return o;
  }
  function mulRv(R, v) { return [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]]; }
  function compose(A, B) { return { R: mulR(A.R, B.R), t: add(mulRv(A.R, B.t), A.t) }; }

  // The working tool orientation of the Cartesian moves: tool axis pitched TOOL_PITCH deg nose-down, pointing +X.
  // Each model's TCP frame is turned (RT) so that in this orientation it reads as world X / Y / Z.
  var TOOL_PITCH = 55;
  // Kinematic chains of the rosie-*-hlr-edges.bin files (pivots in the parent frame, axes parallel to world at
  // home), joint limits in that convention, the TCP on the flange axis, and the showcase's work region (world mm):
  // A = start of the linear moves, B = their far corner (X1), circle centre C and radius, cone tilt.
  function chain(d1, a1, a2, c3x, a3, l5) {
    return [
      { par: -1, piv: [0, 0, 0], ax: null },
      { par: 0, piv: [0, 0, 0], ax: [0, 0, 1] },
      { par: 1, piv: [a1, 0, d1], ax: [0, 1, 0] },
      { par: 2, piv: [0, 0, a2], ax: [0, 1, 0] },
      { par: 3, piv: [c3x, 0, a3], ax: [1, 0, 0] },
      { par: 4, piv: [l5, 0, 0], ax: [0, 1, 0] },
      { par: 5, piv: [0, 0, -77.25], ax: [0, 0, 1] }
    ];
  }
  var DOWN = { tool: [0, 0, -90], RT: rotAxis([0, 1, 0], (90 - TOOL_PITCH) * DEG), j5: -90 };   // flange faces -Z at home
  var MODELS = {
    '1400': {
      name: 'Rosie 1400', reach: 1400,
      edges: 'assets/model/rosie-hlr-edges.bin', occluder: 'assets/model/rosie-occluder.bin',
      plot: { src: 'assets/model/rosie-1400-iso.svg', vb: [880, 846.71], k: 1.000006, shift: [201.88, 706.068], bbox: [0, 0, 880, 846.71] },
      KIN: [
        { par: -1, piv: [0, 0, 0], ax: null },
        { par: 0, piv: [0, 0, 25.5], ax: [0, 0, 1] },
        { par: 1, piv: [160, -125.5, 124.5], ax: [0, 1, 0] },
        { par: 2, piv: [0, 0, 600], ax: [0, 1, 0] },
        { par: 3, piv: [159, 125.5, 105], ax: [1, 0, 0] },
        { par: 4, piv: [514.7, 0, 0], ax: [0, 1, 0] },
        { par: 5, piv: [99.3, 0, 0], ax: [1, 0, 0] }
      ],
      // this model's convention, as mapped in cell-hlr.js: J5 = URDF J5 + 90, J6 = -URDF J6
      LIM: [[-180, 180], [-108.8, 108.8], [-89.9, 87.6], [-180, 180], [-103.1, 164.4], [-179.9, 120]],
      tool: [110, 0, 0], RT: rotAxis([0, 1, 0], -TOOL_PITCH * DEG), j5: 0,     // flange faces +X at home
      A: [920, -190, 440], X1: 720, C: [720, 10, 640], RAD: 140, CONE: 24, TRIAD: 70
    }
  };

  function fk(M, qd) {                     // joint angles in degrees -> world frames of the 7 links
    var out = new Array(7);
    for (var i = 0; i < 7; i++) {
      var l = M.KIN[i];
      out[i] = l.par < 0 ? { R: I3, t: l.piv.slice() } : compose(out[l.par], { R: l.ax ? rotAxis(l.ax, qd[i - 1] * DEG) : I3, t: l.piv });
    }
    return out;
  }
  function tcp(M, qd) { var F = fk(M, qd)[6]; return { R: mulR(F.R, M.RT), t: add(F.t, mulRv(F.R, M.tool)) }; }

  /* IK: damped least squares on the 6D TCP pose (orientation error weighted to mm), limits clamped */
  var W_ROT = 300;
  function solve6(A, b) {
    var n = 6, Mx = A.map(function (r, i) { return r.concat([b[i]]); }), r, k;
    for (var c = 0; c < n; c++) {
      var p = c; for (r = c + 1; r < n; r++) if (Math.abs(Mx[r][c]) > Math.abs(Mx[p][c])) p = r;
      var tmp = Mx[c]; Mx[c] = Mx[p]; Mx[p] = tmp;
      for (r = c + 1; r < n; r++) { var f = Mx[r][c] / Mx[c][c]; for (k = c; k <= n; k++) Mx[r][k] -= f * Mx[c][k]; }
    }
    var x = new Array(n);
    for (r = n - 1; r >= 0; r--) { var s = Mx[r][n]; for (k = r + 1; k < n; k++) s -= Mx[r][k] * x[k]; x[r] = s / Mx[r][r]; }
    return x;
  }
  function poseErr(T, D) {
    var ep = sub(D.t, T.t), Rt = [T.R[0], T.R[3], T.R[6], T.R[1], T.R[4], T.R[7], T.R[2], T.R[5], T.R[8]];
    var E = mulR(D.R, Rt), w = [E[7] - E[5], E[2] - E[6], E[3] - E[1]];
    var sn = 0.5 * Math.hypot(w[0], w[1], w[2]), cs = 0.5 * (E[0] + E[4] + E[8] - 1), th = Math.atan2(sn, cs);
    var eo = sn > 1e-9 ? scl(w, th / (2 * sn)) : [0, 0, 0];
    return [ep[0], ep[1], ep[2], eo[0] * W_ROT, eo[1] * W_ROT, eo[2] * W_ROT];
  }
  function clampQ(M, q) { for (var j = 0; j < 6; j++) q[j] = Math.min(M.LIM[j][1] - MARGIN, Math.max(M.LIM[j][0] + MARGIN, q[j])); return q; }
  function ik(M, q0, D, iters) {
    var q = q0.slice(), h = 1e-4, lam2 = 1, e, r, j;
    for (var it = 0; it < (iters || 60); it++) {
      e = poseErr(tcp(M, q), D);
      if (Math.hypot.apply(null, e) < 0.002) break;
      var J = [];
      for (r = 0; r < 6; r++) J.push([0, 0, 0, 0, 0, 0]);
      for (j = 0; j < 6; j++) {
        var qp = q.slice(); qp[j] += h;
        var ej = poseErr(tcp(M, qp), D);
        for (r = 0; r < 6; r++) J[r][j] = (e[r] - ej[r]) / h;          // per degree
      }
      var A = [];
      for (r = 0; r < 6; r++) { A.push([]); for (var c = 0; c < 6; c++) { var s = 0; for (var k = 0; k < 6; k++) s += J[r][k] * J[c][k]; A[r].push(s + (r === c ? lam2 : 0)); } }
      var y = solve6(A, e), dq = [0, 0, 0, 0, 0, 0], mx = 0;
      for (j = 0; j < 6; j++) { for (r = 0; r < 6; r++) dq[j] += J[r][j] * y[r]; mx = Math.max(mx, Math.abs(dq[j])); }
      if (mx > 6) for (j = 0; j < 6; j++) dq[j] *= 6 / mx;
      for (j = 0; j < 6; j++) q[j] += dq[j];
      clampQ(M, q);
    }
    return { q: q, err: Math.hypot.apply(null, poseErr(tcp(M, q), D)) };
  }

  var ease = function (u) { u = u < 0 ? 0 : u > 1 ? 1 : u; return u * u * u * (u * (u * 6 - 15) + 10); };   // S-curve
  var HOME = [0, 0, 0, 0, 0, 0];

  /* ------------------------------------------------------------------ the choreography */
  // Tool orientation for the Cartesian moves: the working orientation (TCP frame = world), so the triad reads as
  // world X / Y / Z. Joint keys are written for a flange that faces +X at home; j5 shifts J5 for one facing -Z.
  function plan(M) {
    var segs = [], j5 = M.j5;
    var q = [0, -12, 25, 0, 50 + j5, 0];                               // natural seed for the first IK
    var last;
    var C = M.C, RAD = M.RAD, P_A = M.A, P_B = [M.X1, M.A[1], M.A[2]];
    var P_C = [M.X1, C[1] + RAD, M.A[2]], P_D = [M.X1, C[1] + RAD, C[2]];
    function cart(name, dur, target, extra) {                         // target(u) -> {t, R}; u eased 0..1
      var n = Math.max(2, Math.round(dur * RATE) + 1), tab = new Float64Array(n * 6), path = [], worst = 0;
      for (var i = 0; i < n; i++) {
        var D = target(ease(i / (n - 1)));
        var s = ik(M, q, D); q = s.q; worst = Math.max(worst, s.err);
        for (var j = 0; j < 6; j++) tab[i * 6 + j] = q[j];
        path.push(D.t);
      }
      var sg = { kind: 'cart', name: name, dur: dur, n: n, tab: tab, path: path, worst: worst };
      for (var k in extra) sg[k] = extra[k];
      segs.push(sg); last = q.slice();
      return sg;
    }
    var R0 = I3;
    // tool axis in the working orientation, and two unit vectors perpendicular to it
    var E1 = [0, 1, 0], E2 = [Math.sin(TOOL_PITCH * DEG), 0, Math.cos(TOOL_PITCH * DEG)];
    var line = function (a, b) { return function (u) { return { t: [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u], R: R0 }; }; };

    // settle the IK at A first (not a segment), so the tables start converged
    q = ik(M, q, { t: P_A, R: R0 }, 200).q;
    var qA = q.slice();

    cart('lx', 1.9, line(P_A, P_B), { hud: ['Linear', 'X'], axis: 0, trace: true });
    cart('ly', 1.9, line(P_B, P_C), { hud: ['Linear', 'Y'], axis: 1, trace: true });
    cart('lz', 1.6, line(P_C, P_D), { hud: ['Linear', 'Z'], axis: 2, trace: true });
    cart('circle', 4.4, function (u) {
      var th = 2 * Math.PI * u;
      return { t: [C[0], C[1] + RAD * Math.cos(th), C[2] + RAD * Math.sin(th)], R: R0 };
    }, { hud: ['Circle', ''], trace: true });
    cart('toC', 1.2, line(P_D, C), { hud: ['Linear', 'Y'], axis: 1, trace: true });
    cart('pivot', 6.4, function (u) {
      // tool axis sweeps a cone about the pinned TCP: tilt in, one turn, tilt out (u is already eased)
      var phi = 2 * Math.PI * u;
      var a = M.CONE * DEG * ease(Math.min(1, u / 0.18)) * ease(Math.min(1, (1 - u) / 0.18));
      var k = add(scl(E1, Math.cos(phi)), scl(E2, Math.sin(phi)));      // tilt axis, perpendicular to the tool axis
      return { t: C, R: mulR(rotAxis(k, a), R0) };
    }, { hud: ['Reorient', 'about TCP'], pin: C });
    var qC = last.slice();

    // joint-space: one C1 spline from the pivot end, through the wrist roll, the J1 swing and home, back to A.
    // [time from segment start (s), q (deg), hud]
    var keys = [
      [0.0, qC, ['Joint', 'J4 · J6']],
      [1.5, [qC[0], qC[1], qC[2], qC[3] + 70, qC[4] + 22, qC[5] - 95]],
      [3.0, [qC[0], qC[1], qC[2], qC[3] - 60, qC[4] + 28, qC[5] + 80]],
      [4.4, [14, -22, 50, 0, 40 + j5, 0], ['Joint', 'J1']],
      [6.1, [46, -20, 48, 0, 42 + j5, 0]],
      [8.7, [-34, -20, 48, 0, 42 + j5, 0]],
      [10.6, HOME, ['Home', '']],
      [12.3, qA, ['Joint', 'move']]
    ];
    var js = spline('joint', keys);
    js.hudAt = [[0, ['Joint', 'J4 · J6']], [3.7, ['Joint', 'J1']], [10.0, ['Home', '']], [11.2, ['Joint', 'move']]];
    segs.push(js);
    var show = finish(segs);
    // played once after the handoff: from rest at home to rest at A, where the loop starts
    show.intro = spline('intro', [[0, HOME, ['Joint', 'move']], [2.2, qA]]);
    show.qA = qA;
    return show;
  }

  /* cubic Hermite through joint keys, Catmull-Rom tangents from the key times, zero velocity at both ends */
  function spline(name, keys) {
    var n = keys.length, T = keys.map(function (k) { return k[0]; }), Q = keys.map(function (k) { return k[1]; });
    var V = Q.map(function (q, i) {
      if (i === 0 || i === n - 1) return [0, 0, 0, 0, 0, 0];
      return q.map(function (_, j) { return (Q[i + 1][j] - Q[i - 1][j]) / (T[i + 1] - T[i - 1]); });
    });
    var dur = T[n - 1], m = Math.round(dur * RATE) + 1, tab = new Float64Array(m * 6), hud = [];
    for (var s = 0; s < m; s++) {
      var t = (s / (m - 1)) * dur, i = 0;
      while (i < n - 2 && t > T[i + 1]) i++;
      var h = T[i + 1] - T[i], u = (t - T[i]) / h, u2 = u * u, u3 = u2 * u;
      var h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
      for (var j = 0; j < 6; j++) tab[s * 6 + j] = h00 * Q[i][j] + h10 * h * V[i][j] + h01 * Q[i + 1][j] + h11 * h * V[i + 1][j];
    }
    keys.forEach(function (k) { if (k[2]) hud.push([k[0], k[2]]); });
    return { kind: 'joint', name: name, dur: dur, n: m, tab: tab, hudAt: hud, worst: 0 };
  }

  function finish(segs) {
    var t = 0;
    segs.forEach(function (s) { s.t0 = t; t += s.dur; });
    var show = { segs: segs, T: t };
    show.sample = function (tt, out) {                       // loop time -> joints (deg) + segment
      tt = ((tt % show.T) + show.T) % show.T;
      var i = 0; while (i < segs.length - 1 && tt >= segs[i].t0 + segs[i].dur) i++;
      var s = segs[i], u = Math.min(1, Math.max(0, (tt - s.t0) / s.dur)), f = u * (s.n - 1), k = Math.min(s.n - 2, Math.floor(f)), r = f - k;
      out = out || [0, 0, 0, 0, 0, 0];
      for (var j = 0; j < 6; j++) out[j] = s.tab[k * 6 + j] + (s.tab[(k + 1) * 6 + j] - s.tab[k * 6 + j]) * r;
      return { q: out, seg: s, index: i, u: u, local: tt - s.t0, f: f };
    };
    return show;
  }

  var Core = { DEG: DEG, MODELS: MODELS, TOOL_PITCH: TOOL_PITCH, fk: fk, tcp: tcp, ik: ik, plan: plan, ease: ease, HOME: HOME, rotAxis: rotAxis };
  global.RosieMovesCore = Core;
  if (typeof document === 'undefined') { if (typeof module !== 'undefined') module.exports = Core; return; }

  /* ================================================================ page */
  var fig = document.querySelector('[data-rosie-moves]');
  var frame = fig && fig.querySelector('.plot-frame');
  if (!frame) return;
  var img = frame.querySelector('img');
  var buttons = Array.prototype.slice.call(document.querySelectorAll('[data-rosie-model]'));
  var cur = fig.getAttribute('data-model') || '1400';
  var mqReduce = matchMedia('(prefers-reduced-motion: reduce)');
  var canMove = !!global.RosieHLR && 'IntersectionObserver' in global && !mqReduce.matches && global.RosieHLR.supported();

  /* ---------- the model switch (works without motion too: drawing, label and figures) ---------- */
  var FACTS = global.RosieFacts || {};
  function showFacts(m) {
    var f = FACTS[m] || {};
    document.querySelectorAll('[data-rosie-fact]').forEach(function (el) {
      var v = f[el.getAttribute('data-rosie-fact')];
      if (v != null) el.innerHTML = v;
    });
    document.querySelectorAll('[data-rosie-href]').forEach(function (el) {
      var v = f[el.getAttribute('data-rosie-href')];
      if (v) el.setAttribute('href', v);
    });
  }
  function choose(m) {
    if (!MODELS[m] || m === cur) return;
    cur = m;
    buttons.forEach(function (b) { b.setAttribute('aria-pressed', String(b.getAttribute('data-rosie-model') === m)); });
    fig.setAttribute('data-model', m);
    if (fig.dataset['label' + m]) fig.setAttribute('aria-label', fig.dataset['label' + m]);
    showFacts(m);
    try { document.dispatchEvent(new CustomEvent('rosie:model', { detail: { model: m } })); } catch (_) { /* old engines */ }
    if (canMove) live(m); else img.src = MODELS[m].plot.src;
  }
  buttons.forEach(function (b) {
    b.addEventListener('click', function () { choose(b.getAttribute('data-rosie-model')); });
  });
  var picker = document.querySelector('[data-rosie-switch]');
  if (picker) picker.hidden = false;
  if (!canMove) return;                          // static drawing / plot only

  var HOLD = 0.7;                               // s at home after the handoff before the first move
  var TRACE_FADE = 3.2;                         // s for a finished path to fade out
  var OVERLAY_PAD = 24;                         // CSS px kept for the triad labels around the showcase envelope
  var CAM = { ortho: true, yaw: -45, pitch: 35.2644, target: [0, 0, 0] };   // iso-front, eye (+X, -Y, +Z)

  var hud = document.createElement('div');
  hud.className = 'rm-hud';
  hud.setAttribute('aria-hidden', 'true');
  hud.innerHTML = '<span class="lbl"></span>';
  var hudLbl = hud.firstChild, hudText = '';
  fig.appendChild(hud);

  var R = {};                                   // per model: { M, show, hlr, canvas, overlay, V1, V2, fitKey, ready }
  var Rc = null;                                // the model on screen
  var isLive = false, visible = true, raf = 0, last = 0, clock = 0, frozen = null, S = null;
  var C = {};
  var qNow = [0, 0, 0, 0, 0, 0];

  function readStyle() {
    var cs = getComputedStyle(fig), lbl = getComputedStyle(hudLbl);
    var v = function (n, fb) { return (cs.getPropertyValue(n) || '').trim() || fb; };
    C = { ink: v('--ink', '#1b2a30'), red: v('--red', '#b8020d'), ground: v('--ice', '#d9e7eb'),
      font: lbl.fontWeight + ' ' + lbl.fontSize + ' ' + lbl.fontFamily, spacing: lbl.letterSpacing };
  }

  /* ---------- views: V1 = exactly the plotted SVG (home pose), V2 = the showcase framing ---------- */
  function fit(r) {
    var W = frame.clientWidth, H = frame.clientHeight, key = W + 'x' + H, p = r.M.plot;
    if (key === r.fitKey || !W || !H) return;
    r.fitKey = key;
    var hlr = r.hlr, c = Math.min(W / p.vb[0], H / p.vb[1]), ox = (W - p.vb[0] * c) / 2, oy = (H - p.vb[1] * c) / 2;
    r.V1 = { zoom: c * p.k, shift: [ox + c * p.shift[0] - W / 2, oy + c * p.shift[1] - H / 2] };
    // showcase envelope at zoom 1 (relative to the box centre): every pose of the loop, from home
    hlr.setCamera(Object.assign({}, CAM, { zoom: 1, shift: [0, 0] }));
    hlr.setPose(HOME);
    var e = hlr.extent(1), q = [0, 0, 0, 0, 0, 0], T = r.M.TRIAD;
    var x0 = e.minX - W / 2, x1 = e.maxX - W / 2, y0 = e.minY - H / 2, y1 = e.maxY - H / 2;
    for (var t = 0; t <= r.show.T; t += 0.2) {
      hlr.setPose(r.show.sample(t, q).q);
      var b = hlr.extent(6);
      x0 = Math.min(x0, b.minX - W / 2); x1 = Math.max(x1, b.maxX - W / 2); y0 = Math.min(y0, b.minY - H / 2); y1 = Math.max(y1, b.maxY - H / 2);
      // the TCP and its triad (a sphere of TRIAD mm round the TCP) must stay in view too
      var cc = hlr.project(hlr.toWorld(r.M.tool, 'flange'));
      x0 = Math.min(x0, cc.x - T - W / 2); x1 = Math.max(x1, cc.x + T - W / 2); y0 = Math.min(y0, cc.y - T - H / 2); y1 = Math.max(y1, cc.y + T - H / 2);
    }
    var m = 10 * c, pad = OVERLAY_PAD;
    var z = Math.min(r.V1.zoom, (W - 2 * m - 2 * pad) / (x1 - x0), (H - 2 * m - pad) / (y1 - y0));
    // centred across, standing on the same floor line as the home drawing
    r.V2 = { zoom: z, shift: [-z * (x0 + x1) / 2, (oy + p.bbox[3] * c) - H / 2 - z * y1] };
  }
  function setView(r, u) {
    var a = r.V1, b = r.V2, zoom = a.zoom + (b.zoom - a.zoom) * u;
    r.hlr.setCamera(Object.assign({}, CAM, { zoom: zoom, shift: [a.shift[0] + (b.shift[0] - a.shift[0]) * u, a.shift[1] + (b.shift[1] - a.shift[1]) * u] }));
  }

  /* ---------- timeline: hold at home -> intro move (view eases out) -> the loop, forever ---------- */
  function sampleSeg(sg, u) {
    var f = Math.min(1, Math.max(0, u)) * (sg.n - 1), k = Math.min(sg.n - 2, Math.floor(f)), rr = f - k, q = qNow;
    for (var j = 0; j < 6; j++) q[j] = sg.tab[k * 6 + j] + (sg.tab[(k + 1) * 6 + j] - sg.tab[k * 6 + j]) * rr;
    return q;
  }
  function state(show, t) {
    if (t < HOLD) return { q: HOME, view: 0, seg: null, hud: ['Home', ''], ov: 0 };
    t -= HOLD;
    if (t < show.intro.dur) {
      var u = t / show.intro.dur;
      return { q: sampleSeg(show.intro, u), view: ease(u), seg: null, hud: ['Joint', 'move'], ov: ease(Math.min(1, u * 1.6)) };
    }
    var r = show.sample(t - show.intro.dur, qNow), sg = r.seg, h = sg.hud || ['', ''];
    if (sg.hudAt) for (var i = 0; i < sg.hudAt.length; i++) if (r.local >= sg.hudAt[i][0]) h = sg.hudAt[i][1];
    return { q: r.q, view: 1, seg: sg, u: r.u, f: r.f, tt: sg.t0 + r.local, hud: h, ov: 1 };
  }

  /* ---------- overlay ---------- */
  function drawOverlay(ctx, hlr) {
    var s = S, r = Rc;
    if (!s || s.ov <= 0 || !r || r.hlr !== hlr) return;
    var show = r.show, M = r.M;
    function P(p) { var q = hlr.project(p); return [q.x, q.y]; }
    function polyline(pts, n, tail) {
      ctx.beginPath();
      var a = P(pts[0]); ctx.moveTo(a[0], a[1]);
      for (var i = 1; i <= n; i++) { a = P(pts[i]); ctx.lineTo(a[0], a[1]); }
      if (tail) ctx.lineTo(tail[0], tail[1]);
      ctx.stroke();
    }
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    var segs = show.segs, i, g;
    // traced paths: the active one in red up to the TCP, finished ones in faint ink fading out (wraps with the loop)
    if (s.seg) {
      for (i = 0; i < segs.length; i++) {
        g = segs[i];
        if (!g.trace) continue;
        if (g === s.seg) {
          var f = s.f, k = Math.min(g.n - 2, Math.floor(f)), rr = f - k, a = g.path[k], b = g.path[k + 1];
          var head = P([a[0] + (b[0] - a[0]) * rr, a[1] + (b[1] - a[1]) * rr, a[2] + (b[2] - a[2]) * rr]);
          ctx.globalAlpha = 1; ctx.strokeStyle = C.red; ctx.lineWidth = 1.3;
          polyline(g.path, k, head);
        } else {
          var age = ((s.tt - (g.t0 + g.dur)) % show.T + show.T) % show.T;
          var al = 0.5 * (1 - smooth(0, TRACE_FADE, age));
          if (al <= 0.004) continue;
          ctx.globalAlpha = al; ctx.strokeStyle = C.ink; ctx.lineWidth = 1;
          polyline(g.path, g.n - 1);
        }
      }
    }
    // pinned TCP marker during the reorientation
    if (s.seg && s.seg.pin) {
      var pa = smooth(0, 0.08, s.u) * (1 - smooth(0.92, 1, s.u)), c = P(s.seg.pin);
      ctx.globalAlpha = pa; ctx.strokeStyle = C.red; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(c[0], c[1], 6, 0, 2 * Math.PI); ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(c[0] - 12, c[1]); ctx.lineTo(c[0] - 8, c[1]); ctx.moveTo(c[0] + 8, c[1]); ctx.lineTo(c[0] + 12, c[1]);
      ctx.moveTo(c[0], c[1] - 12); ctx.lineTo(c[0], c[1] - 8); ctx.moveTo(c[0], c[1] + 8); ctx.lineTo(c[0], c[1] + 12);
      ctx.stroke();
      label(ctx, 'TCP', c[0] - 15, c[1] + 13, C.red, 'right');
    }
    // tool centre line (dash-dot) from the flange face to the TCP, and the TCP triad
    var ov = s.ov, ow = hlr.toWorld(M.tool, 'flange');
    var f0 = P(hlr.toWorld([0, 0, 0], 'flange')), tc = P(ow);
    ctx.globalAlpha = 0.6 * ov; ctx.strokeStyle = C.ink; ctx.lineWidth = 0.9; ctx.setLineDash([7, 2.5, 1.2, 2.5]);
    ctx.beginPath(); ctx.moveTo(f0[0], f0[1]); ctx.lineTo(tc[0], tc[1]); ctx.stroke();
    ctx.setLineDash([]);
    var F = hlr.frameMatrix('flange'), ax = s.seg && s.seg.axis !== undefined ? s.seg.axis : -1;
    for (var j = 0; j < 3; j++) {
      var cl = [M.RT[j], M.RT[3 + j], M.RT[6 + j]];          // TCP frame axis j in the flange frame
      var dw = [F[0] * cl[0] + F[4] * cl[1] + F[8] * cl[2], F[1] * cl[0] + F[5] * cl[1] + F[9] * cl[2], F[2] * cl[0] + F[6] * cl[1] + F[10] * cl[2]];
      var e = P([ow[0] + dw[0] * M.TRIAD, ow[1] + dw[1] * M.TRIAD, ow[2] + dw[2] * M.TRIAD]);
      var hot = j === ax, colr = hot ? C.red : C.ink;
      ctx.globalAlpha = (hot ? 1 : 0.8) * ov; ctx.strokeStyle = colr; ctx.lineWidth = hot ? 1.4 : 1;
      ctx.beginPath(); ctx.moveTo(tc[0], tc[1]); ctx.lineTo(e[0], e[1]); ctx.stroke();
      var dx = e[0] - tc[0], dy = e[1] - tc[1], L = Math.hypot(dx, dy) || 1;
      label(ctx, 'XYZ'[j], e[0] + dx / L * 8, e[1] + dy / L * 8, colr);
    }
    ctx.globalAlpha = ov; ctx.fillStyle = C.ink;
    ctx.beginPath(); ctx.arc(tc[0], tc[1], 1.8, 0, 2 * Math.PI); ctx.fill();
    ctx.globalAlpha = 1;
  }
  function label(ctx, txt, x, y, col, align) {
    ctx.font = C.font;
    try { ctx.letterSpacing = C.spacing; } catch (_) { /* older engines */ }
    ctx.textAlign = align || 'center'; ctx.textBaseline = 'middle';
    ctx.lineWidth = 3; ctx.lineJoin = 'round'; ctx.strokeStyle = C.ground; ctx.strokeText(txt, x, y);
    ctx.fillStyle = col; ctx.fillText(txt, x, y);
  }
  var smooth = function (a, b, x) { var t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

  function setHud(h) {
    var t = h[1] ? h[0] + ' <b>' + h[1] + '</b>' : h[0];
    if (t !== hudText) { hudLbl.innerHTML = t; hudText = t; }
  }

  /* ---------- loop ---------- */
  function draw() {
    var r = Rc;
    if (!r || !r.hlr) return;
    fit(r);
    S = isLive ? state(r.show, frozen != null ? frozen : clock) : { q: HOME, view: 0, seg: null, hud: ['Home', ''], ov: 0 };
    setView(r, S.view);
    r.hlr.setPose(S.q);
    r.hlr.render();
    if (isLive) setHud(S.hud);
  }
  function tick(now) {
    raf = 0;
    var dt = last ? Math.min(0.05, (now - last) / 1000) : 0;
    last = now;
    clock += dt;
    draw();
    schedule();
  }
  function schedule() { if (!raf && isLive && visible && !document.hidden && frozen == null) raf = requestAnimationFrame(tick); }
  function stop() { if (raf) cancelAnimationFrame(raf); raf = 0; last = 0; }

  function goLive(r) {
    if (r !== Rc || isLive) return;
    isLive = true; clock = 0;
    readStyle(); draw();
    r.canvas.classList.add('is-cur'); r.overlay.classList.add('is-cur');
    frame.classList.add('is-live'); fig.classList.add('is-live');
    schedule();
  }
  function whenPlotted(cb) {
    var st = frame.getAttribute('data-plot');
    if (st === 'done' || st === 'static' || !frame.classList.contains('is-plot')) { cb(); return; }
    frame.addEventListener('plotdone', function once() { frame.removeEventListener('plotdone', once); cb(); });
  }

  function prepare(m) {                         // renderer + plan for one model, created on first use
    if (R[m]) return R[m].ready;
    var M = MODELS[m], r = R[m] = { M: M, show: plan(M), fitKey: '' };
    r.canvas = document.createElement('canvas');
    r.overlay = document.createElement('canvas');
    r.canvas.className = 'rm-canvas'; r.overlay.className = 'rm-canvas rm-overlay';
    r.canvas.setAttribute('aria-hidden', 'true'); r.overlay.setAttribute('aria-hidden', 'true');
    frame.appendChild(r.canvas); frame.appendChild(r.overlay);
    r.ready = RosieHLR.create(r.canvas, {
      edges: M.edges, occluder: M.occluder,
      weights: { outline: 1.35, detail: 0.9 },
      overlay: r.overlay,
      drawOverlay: drawOverlay,
      onChange: function () { readStyle(); if (Rc === r && !raf) draw(); },
    }).then(function (hlr) { r.hlr = hlr; return r; }, function (err) { r.canvas.remove(); r.overlay.remove(); throw err; });
    return r.ready;
  }

  // show model m: re-plot its drawing (the static image while it loads), then hand over to its live loop
  function live(m) {
    stop();
    isLive = false; S = null;
    frame.classList.remove('is-live'); fig.classList.remove('is-live');
    frame.querySelectorAll('.rm-canvas.is-cur').forEach(function (c) { c.classList.remove('is-cur'); });
    var M = MODELS[m];
    img.src = M.plot.src;
    if (global.RosiePlot) global.RosiePlot.play(frame, M.plot.src);
    prepare(m).then(function (r) {
      if (cur !== m) return;
      Rc = r; r.fitKey = '';
      readStyle(); draw();                        // first frame rendered, hidden, while the plot is still drawing
      whenPlotted(function () { if (cur === m) goLive(r); });
    }).catch(function () { /* the plot holds as the static drawing */ });
  }

  new ResizeObserver(function () { if (Rc) { Rc.fitKey = ''; if (!raf) draw(); } }).observe(frame);
  new IntersectionObserver(function (es) {
    visible = es[es.length - 1].isIntersecting;
    if (visible) schedule(); else stop();
  }).observe(fig);
  document.addEventListener('visibilitychange', function () { if (document.hidden) stop(); else schedule(); });
  if (document.fonts && document.fonts.ready) document.fonts.ready.then(function () { readStyle(); if (!raf) draw(); });
  var onReduce = function () { if (mqReduce.matches) { isLive = false; stop(); draw(); } };
  mqReduce.addEventListener ? mqReduce.addEventListener('change', onReduce) : mqReduce.addListener(onReduce);
  // test hook: freeze at a time in seconds since the handoff (null resumes); live() forces the handoff
  global.__rosieMoves = {
    model: function (m) { choose(m); }, current: function () { return cur; },
    show: function () { return Rc && Rc.show; }, hold: HOLD,
    seek: function (t) { frozen = t; stop(); draw(); schedule(); },
    live: function () { if (Rc) { frame.setAttribute('data-plot', 'done'); goLive(Rc); } }, state: function () { return S; }
  };
  // the first model: rosie-plot.js is already plotting the hero image
  prepare(cur).then(function (r) {
    Rc = r; readStyle(); draw();
    whenPlotted(function () { if (Rc === r) goLive(r); });
  }).catch(function () { /* plot only */ });
})(typeof window !== 'undefined' ? window : globalThis);
