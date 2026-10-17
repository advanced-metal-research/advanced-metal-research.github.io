/* Rosie page hero: plots the isometric line drawing in, like a pen plotter.
   Loaded (not deferred) straight after the hero figure so the static image can be hidden
   before first paint. Without JS, or with reduced motion, the static image stays.
   RosiePlot.play(frame, src) plots another drawing into the same frame (the model switch in rosie-moves.js);
   a running plot is cancelled first. */
(function () {
  var supported = !!(window.fetch && window.requestAnimationFrame) &&
    !(window.matchMedia && matchMedia('(prefers-reduced-motion: reduce)').matches);

  var TOTAL = 3200;        // ms of plotting, then hold
  var LINE_PX = 1.35;      // outline weight on screen
  var FINE_PX = 0.9;       // detail weight on screen
  var FINE_LEN = 28;       // strokes shorter than this (viewBox units) are "detail"
  // when each link starts and ends, as fractions of the eased timeline: base first, flange last
  var WINDOWS = {
    base: [0, 0.2], turret: [0.08, 0.34], upper_arm: [0.22, 0.52], elbow: [0.42, 0.72],
    forearm: [0.56, 0.88], wrist: [0.72, 0.92], flange: [0.78, 1]
  };

  function play(frame, src) {
    if (!frame || !supported) return;
    if (frame._plotCancel) frame._plotCancel();
    var old = frame.querySelector('svg.rosie-plot');
    if (old) old.remove();
    var img = frame.querySelector('img');
    src = src || frame.getAttribute('data-plot-src') || (img && img.getAttribute('src'));
    if (!src) return;
    var cancelled = false, raf = 0, io = null, onVis = null;
    frame._plotCancel = function () {
      cancelled = true; clearTimeout(bail);
      if (raf) cancelAnimationFrame(raf);
      if (io) io.disconnect();
      if (onVis) document.removeEventListener('visibilitychange', onVis);
    };

    frame.removeAttribute('data-plot');
    frame.classList.add('is-plot');
    // "plot finished" hook for the live renderer (rosie-moves.js): data-plot="done" | "static", plus a plotdone event
    var signal = function (st) {
      frame.setAttribute('data-plot', st);
      try { frame.dispatchEvent(new Event('plotdone')); } catch (e) { /* very old engines */ }
    };
    var reveal = function () { if (cancelled) return; frame.classList.remove('is-plot'); signal('static'); };
    var bail = setTimeout(reveal, 2500);   // slow network: show the static drawing instead

    fetch(src).then(function (r) { if (!r.ok) throw 0; return r.text(); }).then(function (text) {
      if (cancelled) return;
      var doc = new DOMParser().parseFromString(text, 'image/svg+xml');
      var svg = doc.documentElement;
      if (!svg || svg.nodeName !== 'svg' || !frame.classList.contains('is-plot')) throw 0;
      clearTimeout(bail);
      svg = document.importNode(svg, true);
      svg.removeAttribute('width'); svg.removeAttribute('height');
      svg.setAttribute('class', 'rosie-plot');
      svg.setAttribute('aria-hidden', 'true');
      svg.setAttribute('focusable', 'false');
      svg.setAttribute('preserveAspectRatio', 'xMidYMid meet');
      var ink = svg.querySelector('g'); if (ink) ink.removeAttribute('stroke-width');
      svg.querySelectorAll('path[stroke-width]').forEach(function (p) { p.removeAttribute('stroke-width'); });

      frame.appendChild(svg);      // attached first so stroke lengths can be measured

      // schedule every stroke
      var strokes = [];
      svg.querySelectorAll('g[data-link]').forEach(function (g) {
        var w = WINDOWS[g.getAttribute('data-link')] || [0, 1];
        var paths = g.querySelectorAll('path'), n = paths.length, lens = [], max = 0;
        for (var i = 0; i < n; i++) { lens[i] = paths[i].getTotalLength(); if (lens[i] > max) max = lens[i]; }
        var span = w[1] - w[0], dmax = Math.min(0.2, span * 0.55), dmin = Math.min(0.035, dmax);
        for (i = 0; i < n; i++) {
          var p = paths[i], L = lens[i];
          var dur = dmin + (dmax - dmin) * Math.sqrt(L / (max || 1));
          var start = w[0] + (span - dur) * (n > 1 ? i / (n - 1) : 0);
          if (L < FINE_LEN) p.classList.add('fine');
          p.style.strokeDasharray = L + ' ' + (L + 2);
          p.style.strokeDashoffset = L + 1;
          p.style.visibility = 'hidden';
          strokes.push({ p: p, L: L, a: start, b: start + dur, s: 0 });
        }
      });
      strokes.sort(function (x, y) { return x.a - y.a; });

      // keep the on-screen line weight constant whatever the hero size
      var vb = svg.viewBox.baseVal;
      var weigh = function () {
        var r = svg.getBoundingClientRect();
        var k = Math.min(r.width / vb.width, r.height / vb.height) || 1;
        var thin = Math.max(0.8, Math.min(1, k / 0.7));   // a touch lighter when drawn small (phones)
        svg.style.setProperty('--plot-sw', (LINE_PX * thin / k).toFixed(3));
        svg.style.setProperty('--plot-sw-fine', (FINE_PX * thin / k).toFixed(3));
      };
      weigh();
      if (window.ResizeObserver) new ResizeObserver(weigh).observe(frame);
      else addEventListener('resize', weigh);

      // clock that only runs while the hero is visible and the tab is shown
      var t = 0, last = 0, onScreen = true, first = 0, done = false;
      var ease = function (x) { return x < 0.5 ? 4 * x * x * x : 1 - Math.pow(-2 * x + 2, 3) / 2; };
      var soften = function (x) { return 0.35 * x + 0.65 * ease(x); };   // gentle in/out, never stalls
      var running = function () { return onScreen && !document.hidden && !done && !cancelled; };
      var frameFn = function (now) {
        raf = 0;
        if (cancelled) return;
        if (!last) last = now;
        t += Math.min(now - last, 50); last = now;
        var tau = soften(Math.min(1, t / TOTAL));
        for (var i = first; i < strokes.length; i++) {
          var s = strokes[i];
          if (s.a > tau) break;
          if (s.s === 2) { if (i === first) first++; continue; }
          var q = Math.min(1, (tau - s.a) / (s.b - s.a));
          if (s.s === 0) { s.p.style.visibility = ''; s.s = 1; }
          if (q >= 1) {
            s.p.style.strokeDasharray = s.p.style.strokeDashoffset = ''; s.s = 2;
          } else s.p.style.strokeDashoffset = s.L * (1 - q);
        }
        if (t >= TOTAL) { done = true; strokes.forEach(finish); signal('done'); return; }
        schedule();
      };
      var finish = function (s) { s.p.style.visibility = s.p.style.strokeDasharray = s.p.style.strokeDashoffset = ''; };
      var schedule = function () { if (!raf && running()) raf = requestAnimationFrame(frameFn); };
      var pause = function () { if (raf) cancelAnimationFrame(raf); raf = 0; last = 0; };
      onVis = function () { document.hidden ? pause() : schedule(); };
      document.addEventListener('visibilitychange', onVis);
      if (window.IntersectionObserver) {
        io = new IntersectionObserver(function (es) {
          onScreen = es[es.length - 1].isIntersecting; onScreen ? schedule() : pause();
        });
        io.observe(frame);
      }
      schedule();
    }).catch(function () { clearTimeout(bail); reveal(); });
  }

  window.RosiePlot = { play: play };
  play(document.querySelector('.hero-figure .plot-frame'));
})();
