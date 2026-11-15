// Group analyzer: measures a group from hole positions on a photo of the target, and finds likely
// bullet holes in the photo. Pure logic with no page code (the page draws and takes the taps), so the
// spotter unit can reuse it. Works in the browser (global `GroupCalc`) and in node (module.exports).
(function (root) {
  'use strict';
  const MOA_PER_IN_AT_100YD = 1 / 1.047;   // 1 MOA = 1.047 in at 100 yd

  // pts: hole centres in image pixels [{x, y}]. pxPerIn: image pixels per inch (from the scale taps).
  // aim: optional aim point {x, y}. distanceYd: how far the target was.
  // Returns sizes in inches and MOA. Up and right are + (image y grows downward).
  function stats(pts, pxPerIn, distanceYd, aim) {
    if (!(pxPerIn > 0) || !(distanceYd > 0)) return { err: 'Set the scale and the distance first.' };
    if (!pts || pts.length < 2) return { err: 'Mark at least two holes.' };
    const n = pts.length, toIn = (px) => px / pxPerIn, moa = (inch) => inch / (distanceYd / 100) * MOA_PER_IN_AT_100YD;
    const cx = pts.reduce((a, p) => a + p.x, 0) / n, cy = pts.reduce((a, p) => a + p.y, 0) / n;
    let es = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) es = Math.max(es, Math.hypot(pts[i].x - pts[j].x, pts[i].y - pts[j].y));
    const mr = pts.reduce((a, p) => a + Math.hypot(p.x - cx, p.y - cy), 0) / n;
    const xs = pts.map((p) => p.x), ys = pts.map((p) => p.y);
    const width = Math.max.apply(null, xs) - Math.min.apply(null, xs), height = Math.max.apply(null, ys) - Math.min.apply(null, ys);
    const out = {
      n, esIn: toIn(es), esMoa: moa(toIn(es)), meanRadiusIn: toIn(mr), meanRadiusMoa: moa(toIn(mr)),
      widthIn: toIn(width), heightIn: toIn(height), widthMoa: moa(toIn(width)), heightMoa: moa(toIn(height)),
      center: { x: cx, y: cy }, offRightIn: null, offUpIn: null, offRightMoa: null, offUpMoa: null, err: null,
    };
    if (aim) {
      out.offRightIn = toIn(cx - aim.x); out.offUpIn = toIn(aim.y - cy);
      out.offRightMoa = moa(out.offRightIn); out.offUpMoa = moa(out.offUpIn);
    }
    return out;
  }

  // Pixels per inch from two scale taps a known distance apart.
  function scaleFrom(a, b, inches) {
    const d = Math.hypot(b.x - a.x, b.y - a.y);
    return d > 0 && inches > 0 ? d / inches : null;
  }

  // Find likely bullet holes: dark, round spots about one bullet wide, darker than the paper around them.
  // gray: Uint8 array (w*h, 0 = black). holeDiaPx: expected hole size in pixels (bullet diameter x scale).
  // Holes inside a black aiming mark are not found; the shooter taps those in.
  // Returns [{x, y, areaPx}] sorted by how round and dark they are.
  function findHoles(gray, w, h, holeDiaPx) {
    if (!(holeDiaPx >= 3)) return [];
    const r = holeDiaPx / 2, area1 = Math.PI * r * r;
    // Local mean over a window a few holes wide (integral image), so shading across the photo does not matter.
    const W = w + 1, I = new Float64Array(W * (h + 1));
    for (let y = 0; y < h; y++) { let row = 0; for (let x = 0; x < w; x++) { row += gray[y * w + x]; I[(y + 1) * W + x + 1] = I[y * W + x + 1] + row; } }
    const half = Math.max(4, Math.round(holeDiaPx * 2.5));
    const dark = new Uint8Array(w * h);
    for (let y = 0; y < h; y++) {
      const y0 = Math.max(0, y - half), y1 = Math.min(h, y + half + 1);
      for (let x = 0; x < w; x++) {
        const x0 = Math.max(0, x - half), x1 = Math.min(w, x + half + 1);
        const sum = I[y1 * W + x1] - I[y0 * W + x1] - I[y1 * W + x0] + I[y0 * W + x0], m = sum / ((y1 - y0) * (x1 - x0));
        const g = gray[y * w + x];
        if (g < m * 0.62 && m > 70) dark[y * w + x] = 1;      // clearly darker than bright paper around it
      }
    }
    // Connected spots (4-neighbour flood fill).
    const seen = new Uint8Array(w * h), out = [], stack = [];
    for (let i = 0; i < w * h; i++) {
      if (!dark[i] || seen[i]) continue;
      let n = 0, sx = 0, sy = 0, minx = w, maxx = 0, miny = h, maxy = 0, dsum = 0;
      stack.push(i); seen[i] = 1;
      while (stack.length) {
        const k = stack.pop(), x = k % w, y = (k - x) / w;
        n++; sx += x; sy += y; dsum += gray[k];
        if (x < minx) minx = x; if (x > maxx) maxx = x; if (y < miny) miny = y; if (y > maxy) maxy = y;
        const nb = [x > 0 ? k - 1 : -1, x < w - 1 ? k + 1 : -1, y > 0 ? k - w : -1, y < h - 1 ? k + w : -1];
        for (const j of nb) if (j >= 0 && dark[j] && !seen[j]) { seen[j] = 1; stack.push(j); }
        if (n > area1 * 8) break;                              // far too big: a printed mark, not a hole
      }
      if (stack.length) { stack.length = 0; continue; }
      const bw = maxx - minx + 1, bh = maxy - miny + 1, fill = n / (bw * bh), aspect = Math.max(bw, bh) / Math.min(bw, bh);
      if (n < area1 * 0.35 || n > area1 * 2.6 || aspect > 1.7 || fill < 0.5) continue;
      out.push({ x: sx / n, y: sy / n, areaPx: n, score: fill / aspect - dsum / n / 255 });
    }
    return out.sort((a, b) => b.score - a.score).map(({ x, y, areaPx }) => ({ x, y, areaPx }));
  }

  // Nearest point within `radius` pixels (for removing a hole by tapping it), or -1.
  function nearest(pts, p, radius) {
    let best = -1, bd = radius;
    pts.forEach((q, i) => { const d = Math.hypot(q.x - p.x, q.y - p.y); if (d <= bd) { bd = d; best = i; } });
    return best;
  }

  const GroupCalc = { stats, scaleFrom, findHoles, nearest };
  if (typeof module !== 'undefined' && module.exports) module.exports = GroupCalc;
  else root.GroupCalc = GroupCalc;
})(typeof self !== 'undefined' ? self : this);
