// Checks web/groupcalc.js: group size maths and the hole finder on drawn test images.
'use strict';
const G = require('../web/groupcalc.js');
let fails = 0;
const check = (n, ok, d) => { console.log(`[${ok ? 'PASS' : 'FAIL'}] ${n}${d !== undefined ? '  ' + d : ''}`); if (!ok) fails++; };

// ---- maths: 100 px per inch, 100 yd ----
const pts = [{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 0, y: 100 }, { x: 100, y: 100 }];
const s = G.stats(pts, 100, 100, { x: 50, y: 150 });
check('ES of a 1 in square is the diagonal, 1.414 in', Math.abs(s.esIn - Math.SQRT2) < 1e-9, s.esIn);
check('ES in MOA at 100 yd (1 MOA = 1.047 in)', Math.abs(s.esMoa - Math.SQRT2 / 1.047) < 1e-9, s.esMoa.toFixed(3));
check('mean radius 0.707 in', Math.abs(s.meanRadiusIn - Math.SQRT1_2) < 1e-9);
check('group 1 in high and centred left/right of the aim point', Math.abs(s.offUpIn - 1) < 1e-9 && Math.abs(s.offRightIn) < 1e-9, s.offUpIn + ' ' + s.offRightIn);
check('MOA halves at 200 yd', Math.abs(G.stats(pts, 100, 200).esMoa - s.esMoa / 2) < 1e-9);
check('no aim point: no offset', G.stats(pts, 100, 100).offUpIn === null);
check('needs two holes', !!G.stats([{ x: 1, y: 1 }], 100, 100).err);
check('needs a scale', !!G.stats(pts, 0, 100).err);
check('scale from two taps 300 px apart = 3 in', G.scaleFrom({ x: 0, y: 0 }, { x: 300, y: 0 }, 3) === 100);
check('nearest tap finds the hole', G.nearest(pts, { x: 96, y: 4 }, 10) === 1 && G.nearest(pts, { x: 50, y: 50 }, 10) === -1);

// ---- hole finder on a drawn target: light paper, shading, a black aiming square, 5 holes ----
const w = 400, h = 300, img = new Uint8Array(w * h);
for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) img[y * w + x] = 200 + Math.round(40 * x / w);   // uneven light
const disc = (cx, cy, r, v) => { for (let y = Math.floor(cy - r); y <= cy + r; y++) for (let x = Math.floor(cx - r); x <= cx + r; x++) if ((x - cx) ** 2 + (y - cy) ** 2 <= r * r) img[y * w + x] = v; };
for (let y = 120; y < 180; y++) for (let x = 40; x < 100; x++) img[y * w + x] = 15;                    // printed aiming square
for (let y = 0; y < h; y++) img[y * w + 200] = 60;                                                       // thin printed line
const holes = [[250, 100], [270, 120], [262, 90], [300, 150], [240, 140]];
holes.forEach(([x, y]) => disc(x, y, 5.5, 40));
const found = G.findHoles(img, w, h, 11);
const hit = holes.filter(([x, y]) => found.some((f) => Math.hypot(f.x - x, f.y - y) < 2));
check('finds all 5 holes', hit.length === 5, found.length + ' found');
check('does not mistake the aiming square or the printed line for holes', found.length === 5, found.map((f) => f.x.toFixed(0) + ',' + f.y.toFixed(0)).join(' '));
check('no scale means no search', G.findHoles(img, w, h, 0).length === 0);

console.log(fails ? `\nFAILED (${fails})` : '\nALL PASSED');
process.exit(fails ? 1 : 0);
