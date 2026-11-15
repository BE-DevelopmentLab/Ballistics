// JavaScript copy of the board's point-mass solver (core/ballistics.h), so the phone can solve and
// true a load with the board switched off, using the conditions logged with each shot.
// It is a line-for-line port: keep it in step with core/ballistics.h and core/drag_tables.h.
// tests/test_solver_js.js checks that both give the same answers (tests/solver_ref.cpp).
// Works in the browser (global `Solver`) and in node (module.exports).
(function (root) {
  'use strict';
  const PI = Math.PI, G = 9.80665, LB_IN2_TO_KG_M2 = 703.0696, FPS_TO_MPS = 0.3048, YD_TO_M = 0.9144;
  const MRAD_PER_MOA = 0.290888, MPS_TO_MPH = 2.236936, EARTH_RATE = 7.2921159e-5;
  const CORIOLIS_MIN_RANGE = 1000 * YD_TO_M;

  // ---- drag tables (core/drag_tables.h): [mach, cd] pairs ----
  const G1 = [0,.2629,.05,.2558,.1,.2487,.15,.2413,.2,.2344,.25,.2278,.3,.2214,.35,.2155,.4,.2104,.45,.2061,.5,.2032,.55,.202,
    .6,.2034,.7,.2165,.725,.223,.75,.2313,.775,.2417,.8,.2546,.825,.2706,.85,.2901,.875,.3136,.9,.3415,.925,.3734,.95,.4084,
    .975,.4448,1,.4805,1.025,.5136,1.05,.5427,1.075,.5677,1.1,.5883,1.125,.6053,1.15,.6191,1.2,.6393,1.25,.6518,1.3,.6589,
    1.35,.6621,1.4,.6625,1.45,.6607,1.5,.6573,1.55,.6528,1.6,.6474,1.65,.6413,1.7,.6347,1.75,.628,1.8,.621,1.85,.6141,
    1.9,.6072,1.95,.6003,2,.5934,2.05,.5867,2.1,.5804,2.15,.5743,2.2,.5685,2.25,.563,2.3,.5577,2.35,.5527,2.4,.5481,
    2.45,.5438,2.5,.5397,2.6,.5325,2.7,.5264,2.8,.5211,2.9,.5168,3,.5133,3.1,.5105,3.2,.5084,3.3,.5067,3.4,.5054,3.5,.504,
    3.6,.503,3.7,.5022,3.8,.5016,3.9,.501,4,.5006,4.2,.4998,4.4,.4995,4.6,.4992,4.8,.499,5,.4988];
  const G7 = [0,.1198,.05,.1197,.1,.1196,.15,.1194,.2,.1193,.25,.1194,.3,.1194,.35,.1194,.4,.1193,.45,.1193,.5,.1194,.55,.1193,
    .6,.1194,.65,.1197,.7,.1202,.725,.1207,.75,.1215,.775,.1226,.8,.1242,.825,.1266,.85,.1306,.875,.1368,.9,.1464,.925,.166,
    .95,.2054,.975,.2993,1,.3803,1.025,.4015,1.05,.4043,1.075,.4034,1.1,.4014,1.125,.3987,1.15,.3955,1.2,.3884,1.25,.381,
    1.3,.3732,1.35,.3657,1.4,.358,1.45,.351,1.5,.344,1.55,.3376,1.6,.3315,1.65,.326,1.7,.3209,1.75,.316,1.8,.3117,1.85,.3078,
    1.9,.3042,1.95,.301,2,.298,2.05,.2951,2.1,.2922,2.15,.2892,2.2,.2864,2.25,.2835,2.3,.2807,2.35,.2779,2.4,.2752,2.45,.2725,
    2.5,.2697,2.55,.267,2.6,.2643,2.65,.2615,2.7,.2588,2.75,.2561,2.8,.2533,2.85,.2506,2.9,.2479,2.95,.2451,3,.2424,3.1,.2368,
    3.2,.2313,3.3,.2258,3.4,.2205,3.5,.2154,3.6,.2106,3.7,.206,3.8,.2017,3.9,.1975,4,.1935,4.2,.1861,4.4,.1793,4.6,.173,
    4.8,.1672,5,.1618];
  // The C++ tables are float; round the same way so both copies agree to the last digit.
  const f32 = Math.fround;
  const TABLE = { G1: G1.map(f32), G7: G7.map(f32) };

  function dragCd(model, mach) {
    const t = TABLE[model] || TABLE.G7, n = t.length / 2;
    if (mach <= t[0]) return t[1];
    if (mach >= t[2 * (n - 1)]) return t[2 * (n - 1) + 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (t[2 * mid] <= mach) lo = mid; else hi = mid; }
    const f = (mach - t[2 * lo]) / (t[2 * hi] - t[2 * lo]);
    return t[2 * lo + 1] + f * (t[2 * hi + 1] - t[2 * lo + 1]);
  }

  // ---- atmosphere ----
  function airState(a) {
    const tk = a.tempC + 273.15;
    const es = 6.1121 * Math.exp((18.678 - a.tempC / 234.5) * (a.tempC / (257.14 + a.tempC)));
    let e = (a.rhPct / 100) * es;
    if (e > a.pressureHpa) e = a.pressureHpa;
    const pd = a.pressureHpa - e;
    const tv = tk / (1 - 0.378 * e / a.pressureHpa);
    return { density: (pd * 100) / (287.058 * tk) + (e * 100) / (461.495 * tk), soundSpeed: 20.0468 * Math.sqrt(tv) };
  }

  // ---- integrator ----
  function deriv(e, s) {
    const rx = s[3] - e.wx, ry = s[4], rz = s[5] - e.wz;
    const vr = Math.sqrt(rx * rx + ry * ry + rz * rz);
    const k = (PI / 8) * e.rho * dragCd(e.model, vr / e.c) * vr / e.bcSi;
    return [s[3], s[4], s[5],
      -k * rx + e.gx - 2 * (e.oy * s[5] - e.oz * s[4]),
      -k * ry + e.gy - 2 * (e.oz * s[3] - e.ox * s[5]),
      -k * rz - 2 * (e.ox * s[4] - e.oy * s[3])];
  }
  const adv = (s, d, h) => s.map((v, i) => v + d[i] * h);
  function rk4(e, s, h) {
    const k1 = deriv(e, s), k2 = deriv(e, adv(s, k1, h / 2)), k3 = deriv(e, adv(s, k2, h / 2)), k4 = deriv(e, adv(s, k3, h));
    const h6 = h / 6;
    return s.map((v, i) => v + h6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }

  function fly(e, b, sightHeight, alpha, range, dt) {
    dt = dt || 0.001;
    let s = [0, -sightHeight, 0, b.mv * Math.cos(alpha), b.mv * Math.sin(alpha), 0], t = 0;
    for (let i = 0; i < 20000; i++) {
      if (s[0] >= range) break;
      const prev = s;
      s = rk4(e, s, dt);
      const tp = t; t += dt;
      if (s[3] <= 1) return { ok: false };
      if (s[0] >= range) {
        const f = (range - prev[0]) / (s[0] - prev[0]), L = (j) => prev[j] + f * (s[j] - prev[j]);
        return { ok: true, y: L(1), z: L(2), tof: tp + f * dt, speed: Math.sqrt(L(3) ** 2 + L(4) ** 2 + L(5) ** 2) };
      }
    }
    return { ok: false };
  }

  function solveAngle(e, b, sightHeight, range) {
    let a0 = sightHeight / range + G * range / (2 * b.mv * b.mv), a1 = a0 * 1.05 + 1e-4;
    let i0 = fly(e, b, sightHeight, a0, range), i1 = fly(e, b, sightHeight, a1, range);
    if (!i0.ok || !i1.ok) return null;
    for (let it = 0; it < 20; it++) {
      if (Math.abs(i1.y) < 1e-4) break;
      const denom = i1.y - i0.y;
      if (Math.abs(denom) < 1e-9) break;
      const a2 = a1 - i1.y * (a1 - a0) / denom;
      a0 = a1; i0 = i1; a1 = a2;
      i1 = fly(e, b, sightHeight, a1, range);
      if (!i1.ok) return null;
    }
    return { alpha: a1, at: i1 };
  }

  function makeEnv(b, atm, windSpeed, windClockDeg, slopeRad, coriolis, latDeg, azDeg) {
    const air = airState(atm), ang = windClockDeg * PI / 180;
    const e = { bcSi: b.bc * LB_IN2_TO_KG_M2, model: b.model, rho: air.density, c: air.soundSpeed,
      wx: -windSpeed * Math.cos(ang), wz: -windSpeed * Math.sin(ang), gx: -G * Math.sin(slopeRad), gy: -G * Math.cos(slopeRad), ox: 0, oy: 0, oz: 0 };
    if (coriolis) {
      const lat = latDeg * PI / 180, az = azDeg * PI / 180;
      const wf = EARTH_RATE * Math.cos(lat) * Math.cos(az), wr = -EARTH_RATE * Math.cos(lat) * Math.sin(az), wu = EARTH_RATE * Math.sin(lat);
      const cs = Math.cos(slopeRad), sn = Math.sin(slopeRad);
      e.ox = wf * cs + wu * sn; e.oy = -wf * sn + wu * cs; e.oz = wr;
    }
    return e;
  }

  function stabilityFactor(b, atm) {
    if (!(b.massGr > 0 && b.diameterIn > 0 && b.lengthIn > 0 && b.twistIn !== 0)) return 0;
    const d = b.diameterIn, t = Math.abs(b.twistIn) / d, l = b.lengthIn / d;
    let sg = 30 * b.massGr / (t * t * d * d * d * l * (1 + l * l));
    sg *= Math.cbrt((b.mv / FPS_TO_MPS) / 2800);
    sg *= (atm.tempC * 1.8 + 32 + 460) / 519;
    sg *= 29.92 / (atm.pressureHpa / 33.8639);
    return sg;
  }
  function spinDriftM(b, atm, tof) {
    const sg = stabilityFactor(b, atm);
    if (!(sg > 0) || !(tof > 0)) return 0;
    return (b.twistIn > 0 ? 1 : -1) * 1.25 * (sg + 1.2) * Math.pow(tof, 1.83) * 0.0254;
  }
  function aeroJumpMrad(b, atm, windSpeed, windClockDeg) {
    const sg = stabilityFactor(b, atm);
    if (!(sg > 0)) return 0;
    let y = 0.01 * sg - 0.0024 * (b.lengthIn / b.diameterIn) + 0.032;
    if (y < 0) y = 0;
    const crossMph = windSpeed * Math.sin(windClockDeg * PI / 180) * MPS_TO_MPH;
    return (b.twistIn > 0 ? 1 : -1) * y * crossMph * MRAD_PER_MOA;
  }

  // The app's config (SI, as sent to the board) -> the solver's bullet, like the firmware does.
  function bulletOf(cfg) {
    const b = { bc: cfg.bc, model: cfg.model === 'G1' ? 'G1' : 'G7', mv: cfg.muzzleMps, massGr: 0, diameterIn: 0, lengthIn: 0, twistIn: 0 };
    if (cfg.massGr > 0 && cfg.diameterM > 0 && cfg.lengthM > 0 && cfg.twistM) {
      b.massGr = cfg.massGr; b.diameterIn = cfg.diameterM / 0.0254; b.lengthIn = cfg.lengthM / 0.0254; b.twistIn = cfg.twistM / 0.0254;
    }
    return b;
  }
  const zeroAtmOf = (cfg) => ({ tempC: cfg.zeroTempC, pressureHpa: cfg.zeroPressureHpa, rhPct: cfg.zeroRhPct });

  // Full solution, same fields as the board's solution packet.
  // cfg: app config (readConfig()). atm: { tempC, pressureHpa, rhPct } (station pressure).
  // shot: { windSpeedMps, windClockDeg, slopeDeg, spinDrift, aeroJump, coriolis, latitudeDeg, azimuthDeg }
  function solveB(b, cfg, atm, shot, range) {
    const fail = { ok: false, rangeM: range };
    const z = solveAngle(makeEnv(b, zeroAtmOf(cfg), 0, 0, 0), b, cfg.sightHeightM, cfg.zeroRangeM);
    if (!z) return fail;
    const ws = shot.windSpeedMps || 0, wd = shot.windClockDeg || 0, slope = (shot.slopeDeg || 0) * PI / 180;
    const e = makeEnv(b, atm, ws, wd, slope, !!shot.coriolis && range > CORIOLIS_MIN_RANGE, shot.latitudeDeg || 0, shot.azimuthDeg || 0);
    const r = solveAngle(e, b, cfg.sightHeightM, range);
    if (!r) return fail;
    const hasSpin = b.twistIn !== 0;
    const aj = shot.aeroJump && hasSpin ? aeroJumpMrad(b, atm, ws, wd) : 0;
    const sd = shot.spinDrift && hasSpin ? spinDriftM(b, atm, r.at.tof) : 0;
    return { ok: true, rangeM: range, elevationMrad: (r.alpha - z.alpha) * 1000 + aj, windageMrad: -((r.at.z + sd) / range) * 1000,
      spinDriftMrad: -(sd / range) * 1000, aeroJumpMrad: aj, tofS: r.at.tof, velocityMps: r.at.speed, dropM: r.at.y, windMps: ws };
  }
  const solve = (cfg, atm, shot, range) => solveB(bulletOf(cfg), cfg, atm, shot, range);

  // ---- truing (core/ballistics.h true_muzzle_velocity / true_bc / true_load) ----
  // An observation: { atm, shot, rangeM, observedMrad } where observedMrad is the come-up that actually hit.
  function findRoot(f, lo, hi, tol) {
    let flo = f(lo), fhi = f(hi);
    if (!Number.isFinite(flo) || !Number.isFinite(fhi) || flo * fhi > 0) return null;
    for (let it = 0; it < 60; it++) {
      let x = hi - fhi * (hi - lo) / (fhi - flo);
      if (!(x > lo && x < hi)) x = (lo + hi) / 2;
      const fx = f(x);
      if (!Number.isFinite(fx)) return null;
      if (Math.abs(fx) < tol || (hi - lo) < tol * 1e-3) return x;
      if ((fx < 0) === (flo < 0)) { lo = x; flo = fx; } else { hi = x; fhi = fx; }
      const mid = (lo + hi) / 2, fm = f(mid);
      if (!Number.isFinite(fm)) return null;
      if ((fm < 0) === (flo < 0)) { lo = mid; flo = fm; } else { hi = mid; fhi = fm; }
    }
    return (lo + hi) / 2;
  }
  function comeUpError(b, cfg, o) {
    const s = solveB(b, cfg, o.atm, o.shot, o.rangeM);
    return s.ok ? s.elevationMrad - o.observedMrad : NaN;
  }
  // Returns the trued muzzle velocity (m/s), or null if it cannot match.
  function trueMv(cfg, o, tol) {
    if (!(o.rangeM > cfg.zeroRangeM) || !(o.observedMrad > 0)) return null;
    const b = bulletOf(cfg), mv0 = b.mv;
    return findRoot((mv) => { b.mv = mv; return comeUpError(b, cfg, o); }, mv0 * 0.6, mv0 * 1.4, tol || 0.005);
  }
  // Returns the trued BC, or null.
  function trueBc(cfg, o, tol) {
    if (!(o.rangeM > cfg.zeroRangeM) || !(o.observedMrad > 0)) return null;
    const b = bulletOf(cfg), bc0 = b.bc;
    return findRoot((bc) => { b.bc = bc; return comeUpError(b, cfg, o); }, bc0 * 0.4, bc0 * 2.5, tol || 0.005);
  }
  // Both: MV from the mid-range observation, BC from the far one, alternating until both agree.
  // Returns { muzzleMps, bc } or null.
  function trueLoad(cfg, mid, far) {
    const c = Object.assign({}, cfg), tol = 0.0005;
    for (let pass = 0; pass < 10; pass++) {
      const mv = trueMv(c, mid, tol); if (mv === null) return null; c.muzzleMps = mv;
      const bc = trueBc(c, far, tol); if (bc === null) return null; c.bc = bc;
      const b = bulletOf(c);
      if (Math.abs(comeUpError(b, c, mid)) < 0.002 && Math.abs(comeUpError(b, c, far)) < 0.002) break;
    }
    return { muzzleMps: c.muzzleMps, bc: c.bc };
  }

  const Solver = { solve, trueMv, trueBc, trueLoad, airState, dragCd, stabilityFactor };
  if (typeof module !== 'undefined' && module.exports) module.exports = Solver;
  else root.Solver = Solver;
})(typeof self !== 'undefined' ? self : this);
