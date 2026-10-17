/* rosie-hlr.js: real-time hidden-line renderer for Rosie 1400 (WebGL2, own shaders, no libraries).

   It draws the robot the way the static OCC hidden-line drawings look: crisp line art with hidden lines removed,
   two line weights (outline / detail), transparent background, colour from a CSS token.

   Data (public, simplified exterior only):
     assets/model/rosie-hlr-edges.bin   kinematic chain, slimmed feature edges per link, analytic tube silhouettes
     assets/model/rosie-occluder.bin    coarse exterior occluder mesh per link (depth only, never drawn as such)

   How it works
     pass 1  posed occluder triangles -> depth texture + coverage mask (colour writes off for the page;
             polygonOffset pushes the surfaces slightly back)
     pass 2  feature edges + per-frame silhouettes as screen-space quads with an analytic antialiased edge.
             Each fragment is depth-tested at the line's CENTRELINE against the occluder depth (+ a small view-space
             tolerance), so lines keep their full width at silhouettes and coplanar edges never stipple.
             Weight: a line runs at the outline weight where the coverage mask shows background within ~2 px
             (the outer contour and see-through gaps, as in the static drawings), else at the detail weight.
     silhouettes  smooth contours of the curved occluder faces (zero crossings of the interpolated n.(p - eye) over
             each triangle, so a coarse mesh still gives smooth, non-faceted outlines) plus exact analytic generators
             for untrimmed cylinders and cones (`revolved`).

   ---------------------------------------------------------------------------------------------------------------
   API
     RosieHLR.supported() -> boolean                 WebGL2 available? (if not: use the Canvas2D fallback)
     RosieHLR.loadEdges(url) -> Promise<data>        edge data in the legacy rosie-edges.json shape (no WebGL needed):
                                                     { links: [{ name, parent, joint, pivot, axis, polylines:[[[x,y,z],..],..],
                                                       kinds:[0 sharp | 1 tangent], revolved:[..] }], frames, units }
     RosieHLR.create(canvas, opts) -> Promise<renderer>   (alias: window.createRosieRenderer). Rejects if no WebGL2.
       opts.edges       URL of rosie-hlr-edges.bin   (default assets/model/rosie-hlr-edges.bin)
       opts.occluder    URL of rosie-occluder.bin    (default assets/model/rosie-occluder.bin)
       opts.color       CSS colour string, or omitted: the canvas's computed `color` (set it with CSS: var(--ink),
                        var(--on-blue) ...). Re-read automatically when data-theme or prefers-color-scheme changes.
       opts.weights     { outline: 1.35, detail: 0.9 }  CSS px
       opts.silhouettes 'auto' (default: weight by the outline rule) | 'outline' (always the heavy weight)
       opts.maxDpr      device-pixel-ratio cap (default 3)
       opts.overlay     true: add a Canvas2D layer on top (same box, pointer-events none), or pass your own <canvas>
       opts.drawOverlay (ctx, renderer) => {}  called at the end of every render(); ctx is in CSS px, already cleared
       opts.onChange    () => {}  called when something outside your control needs a re-render (theme, context restore)

     renderer.setPose(joints)            degrees, right-hand rule: { J1, .., J6 } or [j1, .., j6]
     renderer.setCamera(cam)             merged into the current camera:
         yaw     eye azimuth, degrees CCW from +X seen from above (-90 = eye on -Y, the side view; -45 = iso-front)
         pitch   eye elevation, degrees          dist  eye distance from target, mm (perspective)
         target  [x, y, z] world mm              fov   vertical field of view, degrees (perspective)
         ortho   true for orthographic           zoom  CSS px per mm at the target (ortho; in perspective it
                                                       overrides fov: focal = zoom * dist)
         shift   [dx, dy] CSS px: where the target lands relative to the canvas centre (+y down)
         up      [x, y, z] screen-up hint (default [0, 0, 1])
     renderer.addGeometry(g) -> handle   extra geometry (torch, table, workpiece, bead ...):
         frame    'world' | link name ('base','turret','upper_arm','elbow','forearm','wrist','flange') | 'J1'..'J6'
                  (= the link that joint moves; 'J6' is the flange). Geometry follows that link's pose.
         matrix   optional 16 numbers, column-major, applied before the frame (geometry -> frame)
         edges    polylines: [[[x,y,z], ..], ..] or [Float32Array(xyz..), ..] (mm, in the frame)
         tris     { positions: flat xyz, indices?: flat, normals?: flat xyz }  occluders (depth only). With normals
                  (and contour !== false) their smooth silhouettes are drawn every frame, like the robot's.
         revolved [{ p0, p1, r0, r1, x?, a?: [deg0, deg1] }]  analytic cylinder/cone silhouettes (rigid frames only)
         weight   'auto' (default) | 'outline' | 'detail'
         alpha    0..1 (default 1)        color  optional CSS colour for this group (else the line colour)
         occlude  tris write depth (default true)        depthTest  false: lines draw over everything (default true)
       handle.update(partial g)  replace edges / tris / revolved / options;   handle.setMatrix(m16)
       handle.setVisible(bool);   handle.remove()
     renderer.project([x,y,z], frame = 'world') -> { x, y, depth, inFront }   CSS px in the canvas box
     renderer.frameMatrix(frame) -> Float32Array(16)   current world transform of a frame (column-major)
     renderer.toWorld([x,y,z], frame) -> [x,y,z]
     renderer.worldPoints(step = 1) -> Float32Array   posed occluder vertices (world), for framing/fit maths
     renderer.extent(step = 4) -> { minX, maxX, minY, maxY }  projected CSS px bounds of the robot for the camera
     renderer.render() -> stats { segments, silhouettes, ms }   draws one frame (resizes itself to the CSS box)
     renderer.setColor(css?)  force a colour (or re-read the CSS colour);   renderer.setWeights({outline, detail})
     renderer.overlay         { canvas, ctx } or null;   renderer.gl;   renderer.destroy()
   --------------------------------------------------------------------------------------------------------------- */
(function (global) {
  'use strict';

  const DEG = Math.PI / 180;
  const DEFAULTS = { edges: 'assets/model/rosie-hlr-edges.bin', occluder: 'assets/model/rosie-occluder.bin' };
  const KIND = { auto: 0, detail: 1, outline: 2, silhouette: 3 };
  const STRIDE = 8;                       // floats per line instance: p0 xyz, p1 xyz, kind, alpha

  /* ------------------------------------------------------------------ small mat4 (column-major) */
  const M4 = {
    id() { const m = new Float32Array(16); m[0] = m[5] = m[10] = m[15] = 1; return m; },
    mul(a, b) {
      const o = new Float32Array(16);
      for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
        o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
      }
      return o;
    },
    translate(t) { const m = M4.id(); m[12] = t[0]; m[13] = t[1]; m[14] = t[2]; return m; },
    scaleT(s, t) { const m = M4.id(); m[0] = m[5] = m[10] = s; m[12] = t[0]; m[13] = t[1]; m[14] = t[2]; return m; },
    rot(ax, a) {
      const l = Math.hypot(ax[0], ax[1], ax[2]) || 1, x = ax[0] / l, y = ax[1] / l, z = ax[2] / l;
      const c = Math.cos(a), s = Math.sin(a), t = 1 - c, m = M4.id();
      m[0] = t * x * x + c; m[1] = t * x * y + s * z; m[2] = t * x * z - s * y;
      m[4] = t * x * y - s * z; m[5] = t * y * y + c; m[6] = t * y * z + s * x;
      m[8] = t * x * z + s * y; m[9] = t * y * z - s * x; m[10] = t * z * z + c;
      return m;
    },
    inv(m) {
      const o = new Float32Array(16), [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = m;
      const b00 = a00 * a11 - a01 * a10, b01 = a00 * a12 - a02 * a10, b02 = a00 * a13 - a03 * a10, b03 = a01 * a12 - a02 * a11;
      const b04 = a01 * a13 - a03 * a11, b05 = a02 * a13 - a03 * a12, b06 = a20 * a31 - a21 * a30, b07 = a20 * a32 - a22 * a30;
      const b08 = a20 * a33 - a23 * a30, b09 = a21 * a32 - a22 * a31, b10 = a21 * a33 - a23 * a31, b11 = a22 * a33 - a23 * a32;
      let d = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
      if (!d) return M4.id();
      d = 1 / d;
      o[0] = (a11 * b11 - a12 * b10 + a13 * b09) * d; o[1] = (a02 * b10 - a01 * b11 - a03 * b09) * d;
      o[2] = (a31 * b05 - a32 * b04 + a33 * b03) * d; o[3] = (a22 * b04 - a21 * b05 - a23 * b03) * d;
      o[4] = (a12 * b08 - a10 * b11 - a13 * b07) * d; o[5] = (a00 * b11 - a02 * b08 + a03 * b07) * d;
      o[6] = (a32 * b02 - a30 * b05 - a33 * b01) * d; o[7] = (a20 * b05 - a22 * b02 + a23 * b01) * d;
      o[8] = (a10 * b10 - a11 * b08 + a13 * b06) * d; o[9] = (a01 * b08 - a00 * b10 - a03 * b06) * d;
      o[10] = (a30 * b04 - a31 * b02 + a33 * b00) * d; o[11] = (a21 * b02 - a20 * b04 - a23 * b00) * d;
      o[12] = (a11 * b07 - a10 * b09 - a12 * b06) * d; o[13] = (a00 * b09 - a01 * b07 + a02 * b06) * d;
      o[14] = (a31 * b01 - a30 * b03 - a32 * b00) * d; o[15] = (a20 * b03 - a21 * b01 + a22 * b00) * d;
      return o;
    },
    pt(m, x, y, z, out, o) {
      out[o] = m[0] * x + m[4] * y + m[8] * z + m[12];
      out[o + 1] = m[1] * x + m[5] * y + m[9] * z + m[13];
      out[o + 2] = m[2] * x + m[6] * y + m[10] * z + m[14];
    },
    dir(m, v) { return [m[0] * v[0] + m[4] * v[1] + m[8] * v[2], m[1] * v[0] + m[5] * v[1] + m[9] * v[2], m[2] * v[0] + m[6] * v[1] + m[10] * v[2]]; },
    p3(m, v) { return [m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12], m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13], m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14]]; },
  };
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

  /* ------------------------------------------------------------------ data files */
  const cache = {};
  function fetchRHL(url) {
    if (!cache[url]) {
      cache[url] = fetch(url).then((r) => { if (!r.ok) throw new Error(url + ' ' + r.status); return r.arrayBuffer(); }).then((buf) => {
        const dv = new DataView(buf);
        if (dv.getUint32(0, true) !== 0x314c4852) throw new Error('not an RHL1 file: ' + url);
        const n = dv.getUint32(4, true);
        const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 8, n)));
        return { header, buf, base: 8 + n };
      });
      cache[url].catch(() => { delete cache[url]; });
    }
    return cache[url];
  }

  function parseEdges(f) {
    const { header: H, buf, base } = f, q = H.q;
    const links = H.links.map((L) => {
      const cnt = new Uint16Array(buf, base + L.cnt, L.np), kinds = new Uint8Array(buf, base + L.kind, L.np);
      const P = new Int16Array(buf, base + L.pts, L.npts * 3), o = L.off;
      const polys = [];
      for (let i = 0, k = 0; i < L.np; i++) {
        const a = new Float32Array(cnt[i] * 3);
        for (let j = 0; j < a.length; j += 3, k += 3) { a[j] = P[k] * q + o[0]; a[j + 1] = P[k + 1] * q + o[1]; a[j + 2] = P[k + 2] * q + o[2]; }
        polys.push(a);
      }
      return { name: L.name, parent: L.parent, joint: L.joint, pivot: L.pivot, axis: L.axis, polys, kinds: Array.from(kinds), revolved: L.revolved || [] };
    });
    return { units: H.units, frames: H.frames, links };
  }

  function loadEdges(url) {
    return fetchRHL(url || DEFAULTS.edges).then((f) => {
      const d = parseEdges(f);
      return {
        units: d.units, frames: d.frames,
        links: d.links.map((l) => ({
          name: l.name, parent: l.parent, joint: l.joint, pivot: l.pivot, axis: l.axis, kinds: l.kinds, revolved: l.revolved,
          polylines: l.polys.map((a) => { const o = []; for (let i = 0; i < a.length; i += 3) o.push([a[i], a[i + 1], a[i + 2]]); return o; }),
        })),
      };
    });
  }

  function parseOccluder(f) {
    const { header: H, buf, base } = f, q = H.q, out = {};
    for (const L of H.links) {
      const nvc = L.nvc != null ? L.nvc : L.nv;
      const pos = new Int16Array(buf, base + L.pos, L.nv * 3), nrm = new Int8Array(buf, base + L.nrm, nvc * 2);
      const idx = new Uint16Array(buf, base + L.idx, L.nt * 3);
      out[L.name] = { q, off: L.off, nv: L.nv, nvc, nt: L.nt, tc: L.tc, pos, nrm, idx };
    }
    return out;
  }

  function octDecode(e, n) {
    const out = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      let x = e[i * 2] / 127, y = e[i * 2 + 1] / 127; const z = 1 - Math.abs(x) - Math.abs(y);
      const t = Math.max(-z, 0); x += x >= 0 ? -t : t; y += y >= 0 ? -t : t;
      const l = Math.hypot(x, y, z) || 1; out[i * 3] = x / l; out[i * 3 + 1] = y / l; out[i * 3 + 2] = z / l;
    }
    return out;
  }

  /* Compact contour mesh: only the vertices the curved (contour-capable) triangles use. */
  function contourMesh(P, N, idx, tc) {
    const map = new Map(), vs = [];
    const t = new Uint32Array(tc * 3);
    for (let i = 0; i < tc * 3; i++) {
      const v = idx[i]; let j = map.get(v);
      if (j === undefined) { j = vs.length; map.set(v, j); vs.push(v); }
      t[i] = j;
    }
    const p = new Float32Array(vs.length * 3), n = new Float32Array(vs.length * 3);
    vs.forEach((v, j) => { for (let k = 0; k < 3; k++) { p[j * 3 + k] = P[v * 3 + k]; n[j * 3 + k] = N[v * 3 + k]; } });
    return { p, n, t, g: new Float32Array(vs.length), nv: vs.length, nt: tc };
  }

  /* ------------------------------------------------------------------ silhouettes (CPU, per frame) */
  // Smooth contour: zero set of g = n . (p - eye) (perspective) or n . v (ortho), linear over each triangle.
  function emitContours(cm, eyeL, ortho, M, out, kind, alpha) {
    const { p, n, t, g } = cm;
    const ex = eyeL[0], ey = eyeL[1], ez = eyeL[2];
    for (let i = 0, j = 0; i < cm.nv; i++, j += 3) {
      g[i] = ortho ? n[j] * ex + n[j + 1] * ey + n[j + 2] * ez
        : n[j] * (p[j] - ex) + n[j + 1] * (p[j + 1] - ey) + n[j + 2] * (p[j + 2] - ez);
    }
    const tmp = [0, 0, 0, 0, 0, 0];
    for (let k = 0; k < cm.nt * 3; k += 3) {
      const a = t[k], b = t[k + 1], c = t[k + 2];
      const ga = g[a], gb = g[b], gc = g[c];
      const sa = ga >= 0, sb = gb >= 0, sc = gc >= 0;
      if (sa === sb && sb === sc) continue;
      let m = 0;
      const cut = (u, v, gu, gv) => {
        const s = gu / (gu - gv), u3 = u * 3, v3 = v * 3;
        tmp[m] = p[u3] + (p[v3] - p[u3]) * s; tmp[m + 1] = p[u3 + 1] + (p[v3 + 1] - p[u3 + 1]) * s; tmp[m + 2] = p[u3 + 2] + (p[v3 + 2] - p[u3 + 2]) * s;
        m += 3;
      };
      if (sa !== sb) cut(a, b, ga, gb);
      if (sb !== sc) cut(b, c, gb, gc);
      if (m < 6 && sc !== sa) cut(c, a, gc, ga);
      const o = out.reserve(1);
      M4.pt(M, tmp[0], tmp[1], tmp[2], out.a, o);
      M4.pt(M, tmp[3], tmp[4], tmp[5], out.a, o + 3);
      out.a[o + 6] = kind; out.a[o + 7] = alpha;
    }
  }

  // Analytic cylinder/cone generators where the surface normal is perpendicular to the view ray.
  function emitRevolved(revs, eyeL, ortho, M, out, kind, alpha) {
    for (const r of revs) {
      const w = ortho ? eyeL : [r.p0[0] - eyeL[0], r.p0[1] - eyeL[1], r.p0[2] - eyeL[2]];
      const A = dot(r.xd, w), B = dot(r.yd, w);
      const C = ortho ? r.k * dot(r.d, w) : r.k * dot(r.d, w) - r.r0;
      const R = Math.hypot(A, B);
      if (R < 1e-9 || Math.abs(C) > R) continue;
      const base = Math.atan2(B, A), del = Math.acos(C / R);
      for (const th of [base + del, base - del]) {
        if (!r.full) {
          let tt = th - r.a0; tt -= Math.floor(tt / (2 * Math.PI)) * 2 * Math.PI;
          if (r.a0 + tt > r.a1 + 1e-6) continue;
        }
        const c = Math.cos(th), s = Math.sin(th);
        const ux = c * r.xd[0] + s * r.yd[0], uy = c * r.xd[1] + s * r.yd[1], uz = c * r.xd[2] + s * r.yd[2];
        const o = out.reserve(1);
        M4.pt(M, r.p0[0] + r.r0 * ux, r.p0[1] + r.r0 * uy, r.p0[2] + r.r0 * uz, out.a, o);
        M4.pt(M, r.p1[0] + r.r1 * ux, r.p1[1] + r.r1 * uy, r.p1[2] + r.r1 * uz, out.a, o + 3);
        out.a[o + 6] = kind; out.a[o + 7] = alpha;
      }
    }
  }

  function prepRevolved(list) {
    return (list || []).map((r) => {
      const ax = [r.p1[0] - r.p0[0], r.p1[1] - r.p0[1], r.p1[2] - r.p0[2]], L = Math.hypot(ax[0], ax[1], ax[2]);
      if (!L) return null;
      const d = norm(ax);
      let x = r.x || (Math.abs(d[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]);
      const xd0 = dot(x, d); x = norm([x[0] - xd0 * d[0], x[1] - xd0 * d[1], x[2] - xd0 * d[2]]);
      const a = r.a || [0, 360];
      return { p0: r.p0, p1: r.p1, d, xd: x, yd: cross(d, x), r0: r.r0, r1: r.r1 == null ? r.r0 : r.r1,
        k: ((r.r1 == null ? r.r0 : r.r1) - r.r0) / L, a0: a[0] * DEG, a1: a[1] * DEG, full: a[1] - a[0] >= 359.9 };
    }).filter(Boolean);
  }

  function growable(n) {
    return {
      a: new Float32Array(Math.max(64, n) * STRIDE), n: 0,
      reserve(k) {
        const need = (this.n + k) * STRIDE;
        if (need > this.a.length) { const b = new Float32Array(Math.max(need, this.a.length * 2)); b.set(this.a); this.a = b; }
        const o = this.n * STRIDE; this.n += k; return o;
      },
      clear() { this.n = 0; },
    };
  }

  function polysToSegments(polys, kindOf, alpha) {
    let n = 0;
    for (const p of polys) n += Math.max(0, p.length / 3 - 1);
    const a = new Float32Array(n * STRIDE);
    let o = 0;
    polys.forEach((p, i) => {
      const k = kindOf(i);
      for (let j = 0; j + 5 < p.length; j += 3, o += STRIDE) {
        a[o] = p[j]; a[o + 1] = p[j + 1]; a[o + 2] = p[j + 2]; a[o + 3] = p[j + 3]; a[o + 4] = p[j + 4]; a[o + 5] = p[j + 5];
        a[o + 6] = k; a[o + 7] = alpha;
      }
    });
    return a;
  }

  function flatPolys(edges) {
    return (edges || []).map((pl) => {
      if (pl instanceof Float32Array) return pl;
      if (pl.length && typeof pl[0] === 'number') return Float32Array.from(pl);
      const a = new Float32Array(pl.length * 3);
      pl.forEach((q, i) => { a[i * 3] = q[0]; a[i * 3 + 1] = q[1]; a[i * 3 + 2] = q[2]; });
      return a;
    }).filter((a) => a.length >= 6);
  }

  /* ------------------------------------------------------------------ colour */
  let probe = null;
  function cssToRGBA(css) {
    if (!probe) { const c = document.createElement('canvas'); c.width = c.height = 1; probe = c.getContext('2d', { willReadFrequently: true }); }
    probe.clearRect(0, 0, 1, 1); probe.fillStyle = '#000'; probe.fillStyle = css || '#000'; probe.fillRect(0, 0, 1, 1);
    const d = probe.getImageData(0, 0, 1, 1).data;
    const a = d[3] / 255 || 1;
    return [d[0] / 255 / a, d[1] / 255 / a, d[2] / 255 / a, d[3] / 255];
  }

  /* ------------------------------------------------------------------ shaders */
  const VS_DEPTH = `#version 300 es
  in vec3 a_pos; uniform mat4 u_mvp;
  void main() { gl_Position = u_mvp * vec4(a_pos, 1.0); }`;
  const FS_DEPTH = `#version 300 es
  precision mediump float; out vec4 o;
  void main() { o = vec4(1.0); }`;
  const VS_LINE = `#version 300 es
  precision highp float;
  in vec2 a_corner; in vec3 a_p0; in vec3 a_p1; in vec2 a_ka;
  uniform mat4 u_mvp; uniform vec2 u_vp; uniform float u_ext;
  flat out vec2 v_a; flat out vec2 v_b; flat out vec2 v_z; flat out vec2 v_ka;
  void main() {
    vec4 c0 = u_mvp * vec4(a_p0, 1.0), c1 = u_mvp * vec4(a_p1, 1.0);
    if (c0.w <= 1e-6 || c1.w <= 1e-6) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
    vec2 s0 = (c0.xy / c0.w * 0.5 + 0.5) * u_vp, s1 = (c1.xy / c1.w * 0.5 + 0.5) * u_vp;
    vec2 d = s1 - s0; float L = length(d);
    vec2 dir = L > 1e-4 ? d / L : vec2(1.0, 0.0), nrm = vec2(-dir.y, dir.x);
    vec2 p = (a_corner.x < 0.5 ? s0 - dir * u_ext : s1 + dir * u_ext) + nrm * (a_corner.y * u_ext);
    v_a = s0; v_b = s1; v_z = vec2(c0.z / c0.w, c1.z / c1.w); v_ka = a_ka;
    gl_Position = vec4(p / u_vp * 2.0 - 1.0, 0.0, 1.0);
  }`;
  const FS_LINE = `#version 300 es
  precision highp float; precision highp sampler2D;
  uniform sampler2D u_depth; uniform sampler2D u_mask;
  uniform vec3 u_rgb; uniform float u_alpha; uniform vec2 u_hw; uniform float u_R;
  uniform vec2 u_nf; uniform float u_ortho; uniform float u_tol; uniform float u_depthTest; uniform float u_silOut;
  uniform ivec2 u_size;
  flat in vec2 v_a; flat in vec2 v_b; flat in vec2 v_z; flat in vec2 v_ka;
  out vec4 o;
  float lin(float d) {
    if (u_ortho > 0.5) return mix(u_nf.x, u_nf.y, d);
    float z = d * 2.0 - 1.0;
    return 2.0 * u_nf.x * u_nf.y / (u_nf.y + u_nf.x - z * (u_nf.y - u_nf.x));
  }
  float cov(vec2 q) { return texelFetch(u_mask, clamp(ivec2(floor(q)), ivec2(0), u_size - 1), 0).r; }
  void main() {
    vec2 p = gl_FragCoord.xy, ab = v_b - v_a;
    float L2 = dot(ab, ab);
    float t = L2 > 1e-8 ? clamp(dot(p - v_a, ab) / L2, 0.0, 1.0) : 0.0;
    vec2 c = v_a + ab * t;
    float dist = length(p - c);
    if (dist > u_hw.x + 1.0) discard;
    if (u_depthTest > 0.5) {
      ivec2 ic = ivec2(floor(c));
      if (ic.x >= 0 && ic.y >= 0 && ic.x < u_size.x && ic.y < u_size.y) {
        float dOcc = texelFetch(u_depth, ic, 0).r;
        if (dOcc < 1.0) {
          float zl = mix(v_z.x, v_z.y, t) * 0.5 + 0.5;
          if (lin(zl) > lin(dOcc) + u_tol) discard;
        }
      }
    }
    int k = int(v_ka.x + 0.5);
    float hw;
    if (k == 1) hw = u_hw.y;
    else if (k == 2 || (k == 3 && u_silOut > 0.5)) hw = u_hw.x;
    else {
      float m = 1.0, r = u_R, s = u_R * 0.7071;
      m = min(m, cov(c + vec2(r, 0.0))); m = min(m, cov(c - vec2(r, 0.0)));
      m = min(m, cov(c + vec2(0.0, r))); m = min(m, cov(c - vec2(0.0, r)));
      m = min(m, cov(c + vec2(s, s))); m = min(m, cov(c - vec2(s, s)));
      m = min(m, cov(c + vec2(s, -s))); m = min(m, cov(c - vec2(s, -s)));
      hw = m < 0.5 ? u_hw.x : u_hw.y;
    }
    float a = clamp(hw + 0.5 - dist, 0.0, 1.0) * v_ka.y * u_alpha;
    if (a <= 0.002) discard;
    o = vec4(u_rgb * a, a);
  }`;

  function compile(gl, vs, fs) {
    const sh = (type, src) => {
      const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS) && !gl.isContextLost()) throw new Error(gl.getShaderInfoLog(s));
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, sh(gl.VERTEX_SHADER, vs)); gl.attachShader(p, sh(gl.FRAGMENT_SHADER, fs)); gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS) && !gl.isContextLost()) throw new Error(gl.getProgramInfoLog(p));
    const u = {}, n = gl.getProgramParameter(p, gl.ACTIVE_UNIFORMS);
    for (let i = 0; i < n; i++) { const a = gl.getActiveUniform(p, i); u[a.name] = gl.getUniformLocation(p, a.name); }
    return { p, u, a: (name) => gl.getAttribLocation(p, name) };
  }

  function supported() {
    try { const c = document.createElement('canvas'); return !!(global.WebGL2RenderingContext && c.getContext('webgl2')); } catch (_) { return false; }
  }

  /* ------------------------------------------------------------------ renderer */
  async function create(canvas, opts) {
    opts = Object.assign({}, opts);
    const gl = canvas.getContext('webgl2', { alpha: true, premultipliedAlpha: true, antialias: false, depth: false, stencil: false, powerPreference: 'low-power' });
    if (!gl) throw new Error('no-webgl2');
    const [fe, fo] = await Promise.all([fetchRHL(opts.edges || DEFAULTS.edges), fetchRHL(opts.occluder || DEFAULTS.occluder)]);
    const E = parseEdges(fe), O = parseOccluder(fo);

    const weights = Object.assign({ outline: 1.35, detail: 0.9 }, opts.weights);
    const cam = { yaw: -45, pitch: 35.264, dist: 6000, target: [450, 0, 450], fov: 20, ortho: false, zoom: null, shift: [0, 0], up: [0, 0, 1] };
    let pose = {};
    let rgb = [0.1, 0.16, 0.19];
    let lost = false, destroyed = false;

    /* links: CPU data (kept for context restore) */
    const links = E.links.map((l) => {
      const oc = O[l.name];
      const L = { name: l.name, parent: l.parent, joint: l.joint, pivot: l.pivot, axis: l.axis, W: M4.id(),
        seg: polysToSegments(l.polys, (i) => (l.kinds[i] === 1 ? KIND.detail : KIND.auto), 1),
        rev: prepRevolved(l.revolved), occ: oc, cm: null, P: null, bs: [0, 0, 0, 0], gpu: {} };
      if (oc) {
        const P = new Float32Array(oc.nv * 3);
        for (let i = 0; i < oc.nv * 3; i += 3) { P[i] = oc.pos[i] * oc.q + oc.off[0]; P[i + 1] = oc.pos[i + 1] * oc.q + oc.off[1]; P[i + 2] = oc.pos[i + 2] * oc.q + oc.off[2]; }
        L.P = P;
        L.cm = contourMesh(P, octDecode(oc.nrm, oc.nvc), oc.idx, oc.tc);
        let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
        for (let i = 0; i < P.length; i += 3) for (let k = 0; k < 3; k++) { if (P[i + k] < lo[k]) lo[k] = P[i + k]; if (P[i + k] > hi[k]) hi[k] = P[i + k]; }
        L.bs = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2, Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) / 2];
      }
      return L;
    });
    const byName = {}; links.forEach((l) => { byName[l.name] = l; if (l.joint) byName[l.joint] = l; });
    const groups = [];
    const dyn = growable(4096);

    function fk() {
      const W = {};
      for (const l of links) {
        let M = M4.mul(l.parent ? W[l.parent] : M4.id(), M4.translate(l.pivot));
        if (l.axis) M = M4.mul(M, M4.rot(l.axis, (pose[l.joint] || 0) * DEG));
        W[l.name] = M; l.W = M;
      }
    }
    fk();
    function frameM(frame) {
      if (!frame || frame === 'world') return M4.id();
      const l = byName[frame];
      if (!l) throw new Error('unknown frame ' + frame);
      return l.W;
    }

    /* GPU resources */
    let progD, progL, fbo, texDepth, texMask, fbW = 0, fbH = 0, dynBuf, idxBuf = null, idxSegs = 0;
    const holder = { dynCap: 0 }, lineVAOs = new Set();
    // Lines are plain indexed quads (4 vertices per segment, no instancing: much cheaper on software GL).
    const VSTRIDE = STRIDE + 2;                 // + corner (along 0|1, across -1|1)
    let scratch = new Float32Array(0);
    function expand(seg, n) {
      if (scratch.length < n * 4 * VSTRIDE) scratch = new Float32Array(Math.max(n * 4 * VSTRIDE, scratch.length * 2));
      const o = scratch;
      for (let i = 0, k = 0; i < n; i++) {
        const b = i * STRIDE;
        for (let c = 0; c < 4; c++, k += VSTRIDE) {
          for (let j = 0; j < STRIDE; j++) o[k + j] = seg[b + j];
          o[k + STRIDE] = c & 1; o[k + STRIDE + 1] = c & 2 ? 1 : -1;
        }
      }
      return o.subarray(0, n * 4 * VSTRIDE);
    }
    function ensureIdx(n) {
      if (n <= idxSegs && idxBuf) return;
      idxSegs = Math.max(n, idxSegs * 2, 16384);
      const I = new Uint32Array(idxSegs * 6);
      for (let i = 0; i < idxSegs; i++) { const v = i * 4, k = i * 6; I[k] = v; I[k + 1] = v + 1; I[k + 2] = v + 2; I[k + 3] = v + 2; I[k + 4] = v + 1; I[k + 5] = v + 3; }
      if (idxBuf) gl.deleteBuffer(idxBuf);
      idxBuf = gl.createBuffer();
      gl.bindVertexArray(null); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, idxBuf); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, I, gl.STATIC_DRAW);
      for (const v of lineVAOs) { gl.bindVertexArray(v.vao); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, idxBuf); }
      gl.bindVertexArray(null);
    }
    function makeVAO(buf, count) {
      ensureIdx(count);
      const vao = gl.createVertexArray(); gl.bindVertexArray(vao);
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      const sp = [['a_p0', 3, 0], ['a_p1', 3, 12], ['a_ka', 2, 24], ['a_corner', 2, 32]];
      for (const [n, s, off] of sp) {
        const loc = progL.a(n); gl.enableVertexAttribArray(loc); gl.vertexAttribPointer(loc, s, gl.FLOAT, false, VSTRIDE * 4, off);
      }
      gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, idxBuf);
      gl.bindVertexArray(null);
      const v = { vao, buf, count, line: true }; lineVAOs.add(v);
      return v;
    }
    function lineBuffer(data) {
      const n = data.length / STRIDE;
      const b = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, b); gl.bufferData(gl.ARRAY_BUFFER, expand(data, n), gl.STATIC_DRAW);
      return makeVAO(b, n);
    }
    function meshVAO(pos, idx, type, norm16) {
      const vao = gl.createVertexArray(); gl.bindVertexArray(vao);
      const vb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, vb); gl.bufferData(gl.ARRAY_BUFFER, pos, gl.STATIC_DRAW);
      const loc = progD.a('a_pos'); gl.enableVertexAttribArray(loc); gl.vertexAttribPointer(loc, 3, norm16 ? gl.SHORT : gl.FLOAT, false, 0, 0);
      let ib = null;
      if (idx) { ib = gl.createBuffer(); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, idx, gl.STATIC_DRAW); }
      gl.bindVertexArray(null);
      return { vao, vb, ib, count: idx ? idx.length : pos.length / 3, type };
    }
    function freeVAO(v) { if (!v) return; lineVAOs.delete(v); gl.deleteVertexArray(v.vao); if (v.buf) gl.deleteBuffer(v.buf); if (v.vb) gl.deleteBuffer(v.vb); if (v.ib) gl.deleteBuffer(v.ib); }

    function initGL() {
      progD = compile(gl, VS_DEPTH, FS_DEPTH);
      progL = compile(gl, VS_LINE, FS_LINE);
      lineVAOs.clear(); idxBuf = null; idxSegs = 0;
      for (const l of links) {
        l.gpu.lines = l.seg.length ? lineBuffer(l.seg) : null;
        l.gpu.mesh = l.occ ? meshVAO(l.occ.pos, l.occ.idx, gl.UNSIGNED_SHORT, true) : null;
      }
      for (const g of groups) uploadGroup(g);
      dynBuf = gl.createBuffer(); holder.dynCap = 0;
      dyn.vao = makeVAO(dynBuf, 0);
      fbW = fbH = 0;
    }

    function ensureFBO(w, h) {
      if (w === fbW && h === fbH && fbo) return;
      if (fbo) { gl.deleteFramebuffer(fbo); gl.deleteTexture(texDepth); gl.deleteTexture(texMask); }
      const tex = (ifmt, fmt, type) => {
        const t = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, t);
        gl.texImage2D(gl.TEXTURE_2D, 0, ifmt, w, h, 0, fmt, type, null);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        return t;
      };
      texMask = tex(gl.R8, gl.RED, gl.UNSIGNED_BYTE);
      texDepth = tex(gl.DEPTH_COMPONENT24, gl.DEPTH_COMPONENT, gl.UNSIGNED_INT);
      fbo = gl.createFramebuffer(); gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, texMask, 0);
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.TEXTURE_2D, texDepth, 0);
      gl.bindFramebuffer(gl.FRAMEBUFFER, null);
      fbW = w; fbH = h;
    }

    /* user geometry */
    function buildGroup(g, spec) {
      if ('frame' in spec) g.frame = spec.frame || 'world';
      if ('matrix' in spec) g.matrix = spec.matrix ? Float32Array.from(spec.matrix) : null;
      for (const k of ['alpha', 'occlude', 'depthTest', 'visible']) if (k in spec) g[k] = spec[k];
      if ('weight' in spec) g.kind = KIND[spec.weight] != null ? KIND[spec.weight] : KIND.auto;
      if ('color' in spec) g.color = spec.color ? cssToRGBA(spec.color) : null;
      if ('edges' in spec) { g.seg = polysToSegments(flatPolys(spec.edges), () => g.kind, 1); g.dirtyE = true; }
      if ('revolved' in spec) g.rev = prepRevolved(spec.revolved);
      if ('tris' in spec || 'contour' in spec) {
        const t = spec.tris || g.trisSpec; g.trisSpec = t;
        if ('contour' in spec) g.contour = spec.contour;
        g.pos = t ? Float32Array.from(t.positions) : null;
        g.idx = t && t.indices ? Uint32Array.from(t.indices) : null;
        g.cm = null;
        if (t && t.normals && g.contour !== false) {
          const idx = g.idx || Uint32Array.from({ length: g.pos.length / 3 }, (_, i) => i);
          g.cm = contourMesh(g.pos, Float32Array.from(t.normals), idx, idx.length / 3);
        }
        g.dirtyT = true;
      }
      if (!lost) uploadGroup(g);
    }
    function uploadGroup(g) {
      if (g.dirtyE || !g.gpuL) { freeVAO(g.gpuL); g.gpuL = g.seg && g.seg.length ? lineBuffer(g.seg) : null; g.dirtyE = false; }
      if (g.dirtyT || !g.gpuT) { freeVAO(g.gpuT); g.gpuT = g.pos && g.pos.length ? meshVAO(g.pos, g.idx, gl.UNSIGNED_INT, false) : null; g.dirtyT = false; }
      if (!g.dyn) g.dyn = growable(256);
      if (!g.dynBuf) { g.dynBuf = gl.createBuffer(); g.dynCap = 0; g.dyn.vao = makeVAO(g.dynBuf, 0); }
    }
    function addGeometry(spec) {
      const g = { frame: 'world', matrix: null, kind: KIND.auto, alpha: 1, color: null, occlude: true, depthTest: true, visible: true,
        seg: null, rev: [], pos: null, idx: null, cm: null, contour: true };
      buildGroup(g, Object.assign({ frame: 'world' }, spec));
      groups.push(g);
      return {
        update(s) { buildGroup(g, s); },
        setMatrix(m) { g.matrix = m ? Float32Array.from(m) : null; },
        setVisible(v) { g.visible = !!v; },
        remove() {
          const i = groups.indexOf(g); if (i >= 0) groups.splice(i, 1);
          if (!lost) { freeVAO(g.gpuL); freeVAO(g.gpuT); if (g.dynBuf) freeVAO(g.dyn.vao); }
        },
      };
    }
    const groupM = (g) => (g.matrix ? M4.mul(frameM(g.frame), g.matrix) : frameM(g.frame));

    /* camera */
    let W = 1, H = 1, dpr = 1, V = M4.id(), Pm = M4.id(), PV = M4.id(), eye = [0, 0, 0], viewDir = [0, 0, -1], near = 1, far = 10;
    function sceneSphere() {
      const c = cam.target; let R = 1;
      for (const l of links) {
        if (!l.occ) continue;
        const w = M4.p3(l.W, l.bs);
        R = Math.max(R, Math.hypot(w[0] - c[0], w[1] - c[1], w[2] - c[2]) + l.bs[3]);
      }
      for (const g of groups) {
        if (!g.visible) continue;
        const M = groupM(g);
        const pts = [];
        if (g.pos) for (let i = 0; i < g.pos.length; i += 3 * Math.max(1, (g.pos.length / 300) | 0)) pts.push([g.pos[i], g.pos[i + 1], g.pos[i + 2]]);
        if (g.seg) for (let i = 0; i < g.seg.length; i += STRIDE * Math.max(1, (g.seg.length / STRIDE / 200) | 0)) pts.push([g.seg[i], g.seg[i + 1], g.seg[i + 2]]);
        for (const p of pts) { const w = M4.p3(M, p); R = Math.max(R, Math.hypot(w[0] - c[0], w[1] - c[1], w[2] - c[2]) + 5); }
      }
      return R * 1.02 + 10;
    }
    function updateCamera() {
      const y = cam.yaw * DEG, p = cam.pitch * DEG, t = cam.target;
      const toEye = [Math.cos(p) * Math.cos(y), Math.cos(p) * Math.sin(y), Math.sin(p)];
      const R = sceneSphere();
      const dist = cam.ortho ? R + 50 : cam.dist;
      eye = [t[0] + toEye[0] * dist, t[1] + toEye[1] * dist, t[2] + toEye[2] * dist];
      const f = norm([-toEye[0], -toEye[1], -toEye[2]]);
      let up = cam.up || [0, 0, 1];
      if (Math.abs(dot(norm(up), f)) > 0.999) up = [Math.cos(y + Math.PI), Math.sin(y + Math.PI), 0];
      const s = norm(cross(f, up)), u = cross(s, f);
      V = M4.id();
      V[0] = s[0]; V[4] = s[1]; V[8] = s[2]; V[1] = u[0]; V[5] = u[1]; V[9] = u[2]; V[2] = -f[0]; V[6] = -f[1]; V[10] = -f[2];
      V[12] = -dot(s, eye); V[13] = -dot(u, eye); V[14] = dot(f, eye);
      viewDir = f;
      near = Math.max(dist - R, dist * 0.01, 0.5); far = dist + R;
      const sx = cam.shift[0] / (W / 2), sy = cam.shift[1] / (H / 2);
      Pm = new Float32Array(16);
      if (cam.ortho) {
        const z = cam.zoom || 0.5;
        Pm[0] = z / (W / 2); Pm[5] = z / (H / 2); Pm[10] = -2 / (far - near); Pm[14] = -(far + near) / (far - near);
        Pm[12] = sx; Pm[13] = -sy; Pm[15] = 1;
      } else {
        const fpx = cam.zoom ? cam.zoom * dist : (H / 2) / Math.tan((cam.fov || 20) * DEG / 2);
        Pm[0] = fpx / (W / 2); Pm[5] = fpx / (H / 2);
        Pm[8] = -sx; Pm[9] = sy;
        Pm[10] = -(far + near) / (far - near); Pm[11] = -1; Pm[14] = -2 * far * near / (far - near);
      }
      PV = M4.mul(Pm, V);
    }

    function project(p, frame) {
      const w = frame && frame !== 'world' ? M4.p3(frameM(frame), p) : p;
      const m = PV;
      const x = m[0] * w[0] + m[4] * w[1] + m[8] * w[2] + m[12], y = m[1] * w[0] + m[5] * w[1] + m[9] * w[2] + m[13];
      const z = m[2] * w[0] + m[6] * w[1] + m[10] * w[2] + m[14], ww = m[3] * w[0] + m[7] * w[1] + m[11] * w[2] + m[15];
      return { x: (x / ww * 0.5 + 0.5) * W, y: (0.5 - y / ww * 0.5) * H, depth: cam.ortho ? z : ww, inFront: ww > 0 };
    }

    /* theme colour */
    function readColor() {
      rgb = cssToRGBA(opts.color || getComputedStyle(canvas).color).slice(0, 3);
    }
    const onTheme = () => { readColor(); if (opts.onChange) opts.onChange(); };
    const mo = new MutationObserver(onTheme);
    mo.observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class'] });
    const mq = global.matchMedia ? matchMedia('(prefers-color-scheme: dark)') : null;
    if (mq) mq.addEventListener ? mq.addEventListener('change', onTheme) : mq.addListener(onTheme);
    readColor();

    /* overlay */
    let overlay = null;
    if (opts.overlay) {
      const oc = opts.overlay instanceof HTMLCanvasElement ? opts.overlay : document.createElement('canvas');
      if (!(opts.overlay instanceof HTMLCanvasElement)) {
        oc.className = 'rosie-hlr-overlay'; oc.setAttribute('aria-hidden', 'true');
        oc.style.cssText = 'position:absolute;inset:0;width:100%;height:100%;pointer-events:none';
        canvas.insertAdjacentElement('afterend', oc);
      }
      overlay = { canvas: oc, ctx: oc.getContext('2d') };
    }

    canvas.addEventListener('webglcontextlost', (e) => { e.preventDefault(); lost = true; fbo = null; });
    canvas.addEventListener('webglcontextrestored', () => {
      lost = false; for (const g of groups) { g.gpuL = g.gpuT = null; g.dynBuf = null; }
      initGL(); if (opts.onChange) opts.onChange();
    });
    initGL();

    function resize() {
      dpr = Math.min(global.devicePixelRatio || 1, opts.maxDpr || 3);
      W = Math.max(1, canvas.clientWidth); H = Math.max(1, canvas.clientHeight);
      const w = Math.max(1, Math.round(W * dpr)), h = Math.max(1, Math.round(H * dpr));
      if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
      if (overlay && (overlay.canvas.width !== w || overlay.canvas.height !== h)) { overlay.canvas.width = w; overlay.canvas.height = h; }
      return [w, h];
    }

    function uploadDyn(d, buf, capKey, holder) {
      const bytes = d.n * 4 * VSTRIDE * 4;
      ensureIdx(d.n);
      gl.bindVertexArray(null);
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      if (bytes > holder[capKey]) { holder[capKey] = Math.max(bytes, holder[capKey] * 2, 16384); gl.bufferData(gl.ARRAY_BUFFER, holder[capKey], gl.DYNAMIC_DRAW); }
      if (bytes) gl.bufferSubData(gl.ARRAY_BUFFER, 0, expand(d.a, d.n));
      d.vao.count = d.n;
    }

    function render() {
      if (destroyed || lost || gl.isContextLost()) return null;
      const t0 = performance.now();
      const [w, h] = resize();
      ensureFBO(w, h);
      updateCamera();
      const ortho = !!cam.ortho;
      const silKind = KIND.silhouette;

      /* per-frame silhouettes */
      dyn.clear();
      for (const l of links) {
        const Mi = M4.inv(l.W);
        const e = ortho ? norm(M4.dir(Mi, [-viewDir[0], -viewDir[1], -viewDir[2]])) : M4.p3(Mi, eye);
        if (l.cm) emitContours(l.cm, e, ortho, l.W, dyn, silKind, 1);
        if (l.rev.length) emitRevolved(l.rev, e, ortho, l.W, dyn, silKind, 1);
      }
      uploadDyn(dyn, dynBuf, 'dynCap', holder);
      for (const g of groups) {
        g.dyn.clear();
        if (!g.visible || (!g.cm && !g.rev.length)) { g.dyn.vao.count = 0; continue; }
        const M = groupM(g), Mi = M4.inv(M);
        const e = ortho ? M4.dir(Mi, [-viewDir[0], -viewDir[1], -viewDir[2]]) : M4.p3(Mi, eye);
        const k = g.kind === KIND.auto ? silKind : g.kind;
        if (g.cm) emitContours(g.cm, e, ortho, M, g.dyn, k, 1);
        if (g.rev.length) emitRevolved(g.rev, ortho ? norm(e) : e, ortho, M, g.dyn, k, 1);
        uploadDyn(g.dyn, g.dynBuf, 'dynCap', g);
      }

      /* pass 1: occluders -> depth + coverage */
      gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
      gl.viewport(0, 0, w, h);
      gl.colorMask(true, true, true, true); gl.depthMask(true);   // depthMask must be on for the depth clear
      gl.clearColor(0, 0, 0, 0); gl.clearDepth(1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST); gl.depthFunc(gl.LESS); gl.depthMask(true);
      gl.disable(gl.BLEND); gl.disable(gl.CULL_FACE);
      gl.enable(gl.POLYGON_OFFSET_FILL); gl.polygonOffset(opts.offsetFactor != null ? opts.offsetFactor : 3, opts.offsetUnits != null ? opts.offsetUnits : 2);
      gl.useProgram(progD.p);
      for (const l of links) {
        if (!l.gpu.mesh) continue;
        gl.uniformMatrix4fv(progD.u.u_mvp, false, M4.mul(PV, M4.mul(l.W, M4.scaleT(l.occ.q, l.occ.off))));
        gl.bindVertexArray(l.gpu.mesh.vao);
        gl.drawElements(gl.TRIANGLES, l.gpu.mesh.count, gl.UNSIGNED_SHORT, 0);
      }
      for (const g of groups) {
        if (!g.visible || !g.occlude || !g.gpuT) continue;
        gl.uniformMatrix4fv(progD.u.u_mvp, false, M4.mul(PV, groupM(g)));
        gl.bindVertexArray(g.gpuT.vao);
        if (g.gpuT.ib) gl.drawElements(gl.TRIANGLES, g.gpuT.count, gl.UNSIGNED_INT, 0); else gl.drawArrays(gl.TRIANGLES, 0, g.gpuT.count);
      }
      gl.disable(gl.POLYGON_OFFSET_FILL);

      /* pass 2: lines */
      gl.bindFramebuffer(gl.FRAMEBUFFER, null);
      gl.viewport(0, 0, w, h);
      gl.clearColor(0, 0, 0, 0); gl.clear(gl.COLOR_BUFFER_BIT);
      gl.disable(gl.DEPTH_TEST); gl.depthMask(false);
      gl.enable(gl.BLEND); gl.blendEquation(gl.MAX); gl.blendFunc(gl.ONE, gl.ONE);
      gl.useProgram(progL.p);
      const U = progL.u;
      gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D, texDepth); gl.uniform1i(U.u_depth, 0);
      gl.activeTexture(gl.TEXTURE1); gl.bindTexture(gl.TEXTURE_2D, texMask); gl.uniform1i(U.u_mask, 1);
      const hwO = weights.outline * dpr / 2, hwD = weights.detail * dpr / 2;
      gl.uniform2f(U.u_vp, w, h); gl.uniform1f(U.u_ext, hwO + 1.5);
      gl.uniform2f(U.u_hw, hwO, hwD); gl.uniform1f(U.u_R, (opts.outlineRadius || 2.2) * dpr);
      gl.uniform2f(U.u_nf, near, far); gl.uniform1f(U.u_ortho, ortho ? 1 : 0);
      gl.uniform2i(U.u_size, w, h);
      // depth tolerance (mm): the constant covers the coarse-mesh deviation; polygonOffset covers slope
      const mmPerPx = ortho ? 1 / ((cam.zoom || 0.5) * dpr) : cam.dist / ((cam.zoom ? cam.zoom * cam.dist : (H / 2) / Math.tan((cam.fov || 20) * DEG / 2)) * dpr);
      gl.uniform1f(U.u_tol, (opts.tolMM != null ? opts.tolMM : 0.9) + (opts.tolPx != null ? opts.tolPx : 0) * mmPerPx);
      gl.uniform1f(U.u_silOut, opts.silhouettes === 'outline' ? 1 : 0);
      let segs = 0;
      const draw = (v, M, alpha, depthTest, color) => {
        if (!v || !v.count) return;
        gl.uniformMatrix4fv(U.u_mvp, false, M ? M4.mul(PV, M) : PV);
        gl.uniform1f(U.u_alpha, alpha); gl.uniform1f(U.u_depthTest, depthTest ? 1 : 0);
        const c = color || rgb; gl.uniform3f(U.u_rgb, c[0], c[1], c[2]);
        gl.bindVertexArray(v.vao); gl.drawElements(gl.TRIANGLES, v.count * 6, gl.UNSIGNED_INT, 0);
        segs += v.count;
      };
      if (!opts.hideFeatures) for (const l of links) draw(l.gpu.lines, l.W, 1, true);
      if (!opts.hideSilhouettes) draw(dyn.vao, null, 1, true);
      const coloured = [];
      for (const g of groups) {
        if (!g.visible) continue;
        if (g.color) { coloured.push(g); continue; }
        const M = groupM(g);
        draw(g.gpuL, M, g.alpha, g.depthTest !== false);
        draw(g.dyn.vao, null, g.alpha, g.depthTest !== false);
      }
      if (coloured.length) {
        gl.blendEquation(gl.FUNC_ADD); gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
        for (const g of coloured) {
          const M = groupM(g);
          draw(g.gpuL, M, g.alpha * g.color[3], g.depthTest !== false, g.color);
          draw(g.dyn.vao, null, g.alpha * g.color[3], g.depthTest !== false, g.color);
        }
      }
      gl.bindVertexArray(null);
      gl.disable(gl.BLEND); gl.blendEquation(gl.FUNC_ADD);

      if (overlay) {
        const c = overlay.ctx;
        c.setTransform(1, 0, 0, 1, 0, 0); c.clearRect(0, 0, overlay.canvas.width, overlay.canvas.height);
        c.setTransform(dpr, 0, 0, dpr, 0, 0);
        if (opts.drawOverlay) opts.drawOverlay(c, api);
      }
      return { segments: segs, silhouettes: dyn.n, ms: performance.now() - t0 };
    }

    const api = {
      gl, canvas, overlay,
      get camera() { return Object.assign({}, cam); },
      get pose() { return Object.assign({}, pose); },
      setPose(j) {
        if (Array.isArray(j)) { pose = {}; j.forEach((v, i) => { pose['J' + (i + 1)] = v; }); } else pose = Object.assign({}, pose, j);
        fk();
      },
      setCamera(c) { Object.assign(cam, c); if (c.target) cam.target = c.target.slice(); if (c.shift) cam.shift = c.shift.slice(); },
      addGeometry,
      project,
      frameMatrix: (f) => Float32Array.from(frameM(f)),
      toWorld: (p, f) => M4.p3(frameM(f), p),
      worldPoints(step = 1) {
        const out = [];
        for (const l of links) {
          if (!l.P) continue;
          for (let i = 0; i < l.P.length; i += 3 * step) out.push(...M4.p3(l.W, [l.P[i], l.P[i + 1], l.P[i + 2]]));
        }
        return Float32Array.from(out);
      },
      extent(step = 4) {
        resize(); updateCamera();
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        const P = api.worldPoints(step);
        for (let i = 0; i < P.length; i += 3) {
          const q = project([P[i], P[i + 1], P[i + 2]]);
          if (q.x < minX) minX = q.x; if (q.x > maxX) maxX = q.x; if (q.y < minY) minY = q.y; if (q.y > maxY) maxY = q.y;
        }
        return { minX, maxX, minY, maxY };
      },
      render,
      resize,
      setColor(css) { if (css !== undefined) opts.color = css; readColor(); },
      setWeights(wt) { Object.assign(weights, wt); },
      setOptions(o) { Object.assign(opts, o); },
      destroy() {
        destroyed = true; mo.disconnect();
        if (mq) mq.removeEventListener ? mq.removeEventListener('change', onTheme) : mq.removeListener(onTheme);
        if (overlay && !(opts.overlay instanceof HTMLCanvasElement)) overlay.canvas.remove();
        const ext = gl.getExtension('WEBGL_lose_context'); if (ext) ext.loseContext();
      },
    };
    return api;
  }

  const RosieHLR = { create, supported, loadEdges, M4 };
  global.RosieHLR = RosieHLR;
  global.createRosieRenderer = create;
})(window);
