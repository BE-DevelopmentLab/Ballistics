// Checks that the phone's copy of the solver (web/solver.js) gives the same answers as the board's
// (core/ballistics.h, run through tests/solver_ref in double and in float like the ESP32).
'use strict';
const path = require('path'), { execFileSync } = require('child_process');
const S = require('../web/solver.js');
let fails = 0;
const check = (n, ok, d) => { console.log(`[${ok ? 'PASS' : 'FAIL'}] ${n}${d !== undefined ? '  ' + d : ''}`); if (!ok) fails++; };
const lines = (exe, arg) => execFileSync(path.join(__dirname, exe), arg ? [arg] : []).toString().trim().split('\n').map(JSON.parse);
const IN = 0.0254;
const cfgOf = (r) => ({ model: r.g7 ? 'G7' : 'G1', bc: r.bc, muzzleMps: r.mv, sightHeightM: r.sight, zeroRangeM: r.zero, zeroTempC: r.zt,
  zeroPressureHpa: r.zp, zeroRhPct: r.zh, massGr: r.mass, diameterM: r.dia * IN, lengthM: r.len * IN, twistM: r.twist * IN });
const run = (r) => S.solve(cfgOf(r), { tempC: r.t, pressureHpa: r.p, rhPct: r.rh },
  { windSpeedMps: r.wind, windClockDeg: r.wdir, slopeDeg: r.slope, spinDrift: !!r.spin, aeroJump: !!r.aj, coriolis: !!r.cor, latitudeDeg: r.lat, azimuthDeg: r.az }, r.range);

for (const [exe, tol, name] of [['solver_ref', 0.002, 'double'], ['solver_ref_f', 0.02, 'float, like the board']]) {
  const all = lines(exe), cases = all.filter((x) => !x.truing);
  for (const r of cases) {
    const j = run(r), tag = `${r.range} m ${r.g7 ? 'G7' : 'G1'} ${r.bc} (${name})`;
    check(`solves (${tag})`, j.ok === !!r.ok);
    check(`elevation matches within ${tol} mrad (${tag})`, Math.abs(j.elevationMrad - r.el) < tol, j.elevationMrad.toFixed(4) + ' vs ' + r.el.toFixed(4));
    check(`windage matches within ${tol} mrad (${tag})`, Math.abs(j.windageMrad - r.wi) < tol, j.windageMrad.toFixed(4) + ' vs ' + r.wi.toFixed(4));
    check(`time of flight and velocity match (${tag})`, Math.abs(j.tofS - r.tof) < 1e-3 && Math.abs(j.velocityMps - r.vel) < 0.5, j.tofS.toFixed(4) + ' ' + j.velocityMps.toFixed(1));
  }
  if (name !== 'double') continue;
  const tr = all.find((x) => x.truing);
  const cfg = { model: 'G7', bc: 0.176, muzzleMps: 780, sightHeightM: 1.5 * IN, zeroRangeM: 91.44, zeroTempC: 15, zeroPressureHpa: 1013.25, zeroRhPct: 50 };
  const atm = { tempC: 22, pressureHpa: 985, rhPct: 40 }, shot = { windSpeedMps: 0, windClockDeg: 0, slopeDeg: 0 };
  const mid = { atm, shot, rangeM: 457.2, observedMrad: tr.midObs }, far = { atm, shot, rangeM: 914.4, observedMrad: tr.farObs };
  const mv = S.trueMv(cfg, mid);
  check('truing: MV from one mid-range shot matches the board', mv !== null && Math.abs(mv - tr.mv) < 0.5, mv && mv.toFixed(2) + ' vs ' + tr.mv.toFixed(2));
  const load = S.trueLoad(cfg, mid, far);
  check('truing: MV + BC from two shots matches the board', load && Math.abs(load.muzzleMps - tr.loadMv) < 0.5 && Math.abs(load.bc - tr.loadBc) < 0.0005, load && load.muzzleMps.toFixed(1) + ' / ' + load.bc.toFixed(4));
  check('truing: recovers the true load (765 m/s, BC 0.170)', load && Math.abs(load.muzzleMps - 765) < 1 && Math.abs(load.bc - 0.170) < 0.001);
  check('truing: shot inside zero range is refused', S.trueMv(cfg, Object.assign({}, mid, { rangeM: 50 })) === null);
}
console.log(fails ? `\nFAILED (${fails})` : '\nALL PASSED');
process.exit(fails ? 1 : 0);
