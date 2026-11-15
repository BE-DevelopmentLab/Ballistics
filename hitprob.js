// Hit probability: the chance of a first-round hit on a target, from the shooter's group size,
// muzzle velocity spread, wind call error and range error.
// Pure maths with no page code, so the spotter unit can reuse it later.
// Works in the browser (global `HitProb`) and in node (module.exports).
//
// The board's solver gives the hold. This file only needs how much the impact MOVES when each input
// is a little wrong, worked out on the phone from the board's solution (range, time of flight,
// velocity at the target) plus the muzzle velocity, zero range and sight height:
//   - wind:  lag-time rule (Didion): drift = crosswind x (time of flight - range / muzzle velocity).
//            This is exact for a point-mass bullet in a steady crosswind.
//   - range: the slope of the come-up curve at the target range, from a drop curve fitted to the
//            board's time of flight and remaining velocity.
//   - muzzle velocity: drop scaled with 1 / MV^2 (drag that grows with speed squared), times 1.15
//            because real G1/G7 drag rises as the bullet slows. Checked against the board's own
//            solver in tests/test_hitprob.js (within about 10%).
// Then a Monte Carlo run throws thousands of simulated shots and counts the hits.
(function (root) {
  'use strict';
  const G = 9.80665;
  const MV_DRAG_FACTOR = 1.15;
  // Average extreme spread of an n-shot group, in units of the per-axis standard deviation
  // (round normal scatter, found by simulation).
  const ES_PER_SIGMA = { 3: 2.41, 5: 3.06, 10: 3.81 };
  const MOA_TO_MRAD = 0.290888;

  // Drop below the bore line, and its slope, from 0 to X metres, for a bullet whose slowness (1/v)
  // follows a quadratic in distance fitted to the muzzle velocity, the velocity at the target and
  // the time of flight. Gravity pulls the path down by g / v^2 per metre per metre.
  function dropCurve(v0, vt, tof, R) {
    const a = 1 / v0, A = 1 / vt - a, B = tof - a * R;
    let c = (B - A * R / 2) / (R * R * R / 3 - R * R * R / 2), b = (A - c * R * R) / R;
    const iv = (x) => a + b * x + c * x * x;
    // A bad fit (slowness going negative or the bullet speeding up) falls back to a straight line.
    if (!(iv(R / 2) > 0) || b < 0 || b + 2 * c * R < 0) { c = 0; b = A / R; }
    return function at(X) {
      const N = 120, dx = X / N;
      let s = 0, d = 0, prev = a * a;
      for (let i = 1; i <= N; i++) {
        const v = iv(i * dx), cur = v * v, ns = s + (prev + cur) / 2 * dx;
        d += (s + ns) / 2 * dx; s = ns; prev = cur;
      }
      return { drop: G * d, slope: G * s };
    };
  }

  // How far the impact moves (mrad) per unit error in each input.
  // p: { muzzleMps, tofS, velocityMps (at target), rangeM, zeroRangeM, sightHeightM }
  // Returns null if the inputs cannot describe a real shot.
  function sensitivities(p) {
    const v0 = p.muzzleMps, vt = p.velocityMps, t = p.tofS, R = p.rangeM, Z = p.zeroRangeM, h = p.sightHeightM || 0;
    if (![v0, vt, t, R, Z].every((x) => Number.isFinite(x) && x > 0)) return null;
    if (!(vt < v0) || !(t > R / v0) || !(t < R / vt)) return null;     // must slow down, and tof must fit between the speeds
    const at = dropCurve(v0, vt, t, R), r = at(R), z = at(Z);
    return {
      mradPerWindMps: 1000 * (t - R / v0) / R,
      mradPerRangeM: 1000 * (r.slope / R - (r.drop + h) / (R * R)),
      mradPerMvMps: MV_DRAG_FACTOR * 1000 * (-2 / v0) * (r.drop / R - z.drop / Z),
    };
  }

  // Per-axis standard deviation (mrad) of a group measured as extreme spread in MOA.
  function groupSigmaMrad(esMoa, shots) {
    const k = ES_PER_SIGMA[shots] || ES_PER_SIGMA[5];
    return (esMoa * MOA_TO_MRAD) / k;
  }

  // Small, repeatable random numbers (so the same inputs always give the same answer).
  function rng(seed) {
    let s = seed >>> 0 || 1;
    return function () { s = (s + 0x6D2B79F5) >>> 0; let t = s; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  }

  // Chance (0..1) that a shot lands on the target. sx, sy: spread (1 sigma) at the target in metres.
  // shape 'rect' (width x height) or 'circle' (width = diameter). offX, offY: aim error in metres.
  function monteCarlo(sx, sy, shape, widthM, heightM, opts) {
    const o = opts || {}, n = o.trials || 20000, rand = rng(o.seed || 12345);
    const offX = o.offX || 0, offY = o.offY || 0, hw = widthM / 2, hh = (shape === 'circle' ? widthM : heightM) / 2;
    let hits = 0;
    for (let i = 0; i < n; i += 2) {
      // Box-Muller gives two independent normals per pair of uniforms.
      const u = rand() || 1e-12, v = rand(), m = Math.sqrt(-2 * Math.log(u));
      const x = offX + sx * m * Math.cos(2 * Math.PI * v), y = offY + sy * m * Math.sin(2 * Math.PI * v);
      const hit = shape === 'circle' ? (x * x + y * y <= hw * hw) : (Math.abs(x) <= hw && Math.abs(y) <= hh);
      if (hit) hits++;
    }
    return hits / Math.ceil(n / 2);
  }

  // Everything together.
  // in: { muzzleMps, tofS, velocityMps, rangeM, zeroRangeM, sightHeightM,       (from the board and profile)
  //       groupMoa, groupShots, mvSdMps, windErrMps, rangeErrM,               (how good the inputs are, 1 sigma)
  //       shape, widthM, heightM, trials, seed }
  // out: { p, sigmaXMrad, sigmaYMrad, parts: { group, mv, range, wind } (mrad, 1 sigma), biggest, err }
  function assess(i) {
    if (!(i.widthM > 0) || (i.shape !== 'circle' && !(i.heightM > 0))) return { err: 'Enter the target size.' };
    if (![i.groupMoa, i.mvSdMps, i.windErrMps, i.rangeErrM].every((x) => Number.isFinite(x) && x >= 0)) return { err: 'Spreads must be 0 or more.' };
    const s = sensitivities(i);
    if (!s) return { err: 'Needs a solution from the board first.' };
    const parts = {
      group: groupSigmaMrad(i.groupMoa, i.groupShots),
      mv: Math.abs(s.mradPerMvMps) * i.mvSdMps,
      range: Math.abs(s.mradPerRangeM) * i.rangeErrM,
      wind: Math.abs(s.mradPerWindMps) * i.windErrMps,
    };
    const sigmaXMrad = Math.hypot(parts.group, parts.wind);
    const sigmaYMrad = Math.hypot(parts.group, parts.mv, parts.range);
    const toM = i.rangeM / 1000;
    const p = monteCarlo(sigmaXMrad * toM, sigmaYMrad * toM, i.shape, i.widthM, i.heightM, { trials: i.trials, seed: i.seed });
    // The group counts on both axes; the others on one.
    const share = { group: 2 * parts.group ** 2, mv: parts.mv ** 2, range: parts.range ** 2, wind: parts.wind ** 2 };
    const biggest = Object.keys(share).reduce((a, b) => (share[b] > share[a] ? b : a));
    return { p, sigmaXMrad, sigmaYMrad, parts, biggest, sens: s, err: null };
  }

  const HitProb = { sensitivities, groupSigmaMrad, monteCarlo, assess, ES_PER_SIGMA };
  if (typeof module !== 'undefined' && module.exports) module.exports = HitProb;
  else root.HitProb = HitProb;
})(typeof self !== 'undefined' ? self : this);
