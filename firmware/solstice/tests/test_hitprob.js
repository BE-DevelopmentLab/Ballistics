// Checks web/hitprob.js: sensitivities against the board's own solver (tests/hit_ref), and the
// Monte Carlo hit chance against exact formulas.
'use strict';
const path = require('path'), { execFileSync } = require('child_process');
const H = require('../web/hitprob.js');
let fails = 0;
const check = (n, ok, d) => { console.log(`[${ok ? 'PASS' : 'FAIL'}] ${n}${d !== undefined ? '  ' + d : ''}`); if (!ok) fails++; };
const near = (a, b, rel) => Math.abs(a - b) <= rel * Math.abs(b);

// ---- sensitivities vs the real solver ----
const refs = execFileSync(path.join(__dirname, 'hit_ref')).toString().trim().split('\n').map(JSON.parse);
for (const r of refs) {
  const s = H.sensitivities({ muzzleMps: r.mv, tofS: r.tof, velocityMps: r.vel, rangeM: r.range, zeroRangeM: 91.44, sightHeightM: 0.0381 });
  const tag = `${r.range} m, MV ${r.mv}`;
  check(`wind sensitivity matches solver within 2% (${tag})`, near(s.mradPerWindMps, Math.abs(r.dWind), 0.02), s.mradPerWindMps.toFixed(3) + ' vs ' + Math.abs(r.dWind).toFixed(3));
  check(`range sensitivity matches solver within 7% (${tag})`, near(s.mradPerRangeM, r.dR, 0.07), s.mradPerRangeM.toFixed(4) + ' vs ' + r.dR.toFixed(4));
  check(`MV sensitivity matches solver within 15% (${tag})`, near(s.mradPerMvMps, r.dMv, 0.15), s.mradPerMvMps.toFixed(4) + ' vs ' + r.dMv.toFixed(4));
}
// ---- BC sensitivity: the phone's solver copy vs the real solver ----
const atm0 = { tempC: 15, pressureHpa: 1013.25, rhPct: 50 };
for (const r of refs) {
  const cfg = { bc: r.bc, model: r.g1 ? 'G1' : 'G7', muzzleMps: r.mv, sightHeightM: 0.0381, zeroRangeM: 91.44, zeroTempC: 15, zeroPressureHpa: 1013.25, zeroRhPct: 50 };
  const d = H.bcSensitivity(cfg, atm0, {}, r.range);
  check(`BC sensitivity matches solver within 3% (${r.range} m, BC ${r.bc})`, d !== null && near(d, r.dBc, 0.03), (d || 0).toFixed(4) + ' vs ' + r.dBc.toFixed(4));
}
check('BC sensitivity: more BC means less come-up', refs.every((r) => r.dBc < 0));
check('BC sensitivity: no solver input gives null', H.bcSensitivity(null, atm0, {}, 500) === null);

check('nonsense solution gives null', H.sensitivities({ muzzleMps: 800, tofS: 0.1, velocityMps: 900, rangeM: 500, zeroRangeM: 100 }) === null);
check('missing solution gives null', H.sensitivities({ muzzleMps: 800, rangeM: 500, zeroRangeM: 100 }) === null);

// ---- group size ----
check('1 MOA 5-shot group = 0.095 mrad per axis', Math.abs(H.groupSigmaMrad(1, 5) - 0.290888 / 3.06) < 1e-9);
check('3-shot groups look smaller, so the same size means more spread', H.groupSigmaMrad(1, 3) > H.groupSigmaMrad(1, 5) && H.groupSigmaMrad(1, 5) > H.groupSigmaMrad(1, 10));

// ---- Monte Carlo vs exact ----
const erf = (x) => { const t = 1 / (1 + 0.3275911 * Math.abs(x)); const y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-x * x); return x >= 0 ? y : -y; };
const rectExact = (sx, sy, w, h) => erf(w / 2 / (sx * Math.SQRT2)) * erf(h / 2 / (sy * Math.SQRT2));
const p1 = H.monteCarlo(0.1, 0.15, 'rect', 0.3, 0.3, { trials: 100000 });
check('rectangle: Monte Carlo matches exact within 1%', Math.abs(p1 - rectExact(0.1, 0.15, 0.3, 0.3)) < 0.01, p1.toFixed(3) + ' vs ' + rectExact(0.1, 0.15, 0.3, 0.3).toFixed(3));
const p2 = H.monteCarlo(0.1, 0.1, 'circle', 0.25, 0, { trials: 100000 }), e2 = 1 - Math.exp(-(0.125 ** 2) / (2 * 0.01));
check('circle: Monte Carlo matches exact (Rayleigh) within 1%', Math.abs(p2 - e2) < 0.01, p2.toFixed(3) + ' vs ' + e2.toFixed(3));
check('same inputs give the same answer', H.monteCarlo(0.1, 0.1, 'circle', 0.25, 0) === H.monteCarlo(0.1, 0.1, 'circle', 0.25, 0));
check('aim error lowers the chance', H.monteCarlo(0.1, 0.1, 'circle', 0.25, 0, { offX: 0.1 }) < p2);

// ---- the whole thing ----
const r = refs[2];   // 1000 yd, 77 gr at 780 m/s
const base = { muzzleMps: r.mv, tofS: r.tof, velocityMps: r.vel, rangeM: r.range, zeroRangeM: 91.44, sightHeightM: 0.0381,
  groupMoa: 1, groupShots: 5, mvSdMps: 3, windErrMps: 0.9, rangeErrM: 2, shape: 'rect', widthM: 0.4572, heightM: 0.4572 };
const a = H.assess(base);
check('assess: gives a chance between 0 and 1', !a.err && a.p > 0 && a.p < 1, a.p);
check('assess: at 1000 yd a 2 mph wind error is the biggest factor', a.biggest === 'wind', a.biggest);
check('assess: horizontal spread bigger than vertical here', a.sigmaXMrad > a.sigmaYMrad);
check('assess: a perfect wind call raises the chance', H.assess(Object.assign({}, base, { windErrMps: 0 })).p > a.p);
check('assess: bigger target raises the chance', H.assess(Object.assign({}, base, { widthM: 0.9, heightM: 0.9 })).p > a.p);
check('assess: no target size is an error', !!H.assess(Object.assign({}, base, { widthM: 0 })).err);
check('assess: negative spread is an error', !!H.assess(Object.assign({}, base, { mvSdMps: -1 })).err);
check('assess: no Solstice solution is an error', /Solstice/.test(H.assess(Object.assign({}, base, { tofS: 0 })).err));
check('assess: circle needs no height', !H.assess(Object.assign({}, base, { shape: 'circle', heightM: 0 })).err);

// ---- BC uncertainty ----
const cfgR = { bc: 0.176, model: 'G7', muzzleMps: r.mv, sightHeightM: 0.0381, zeroRangeM: 91.44, zeroTempC: 15, zeroPressureHpa: 1013.25, zeroRhPct: 50 };
const kBc = H.bcSensitivity(cfgR, atm0, {}, r.range);
const withBc = H.assess(Object.assign({}, base, { bcErrPct: 3, mradPerBcPct: kBc }));
check('BC: 3% BC uncertainty adds to the vertical spread', !withBc.err && withBc.sigmaYMrad > a.sigmaYMrad && Math.abs(withBc.parts.bc - 3 * Math.abs(kBc)) < 1e-12, withBc.parts.bc);
check('BC: and lowers the hit chance', withBc.p < a.p, withBc.p + ' vs ' + a.p);
check('BC: does not touch the horizontal spread', withBc.sigmaXMrad === a.sigmaXMrad);
check('BC: 0% is the same as leaving it out', H.assess(Object.assign({}, base, { bcErrPct: 0 })).p === a.p);
check('BC: uncertainty without a BC effect is an error', !!H.assess(Object.assign({}, base, { bcErrPct: 2 })).err);
check('BC: big BC error with a perfect wind call is the biggest factor',
  H.assess(Object.assign({}, base, { bcErrPct: 10, mradPerBcPct: kBc, windErrMps: 0 })).biggest === 'bc');

// ---- target size decides it: one target, its own size ----
const MOA_M = (range) => range * 0.290888 / 1000;      // 1 MOA in metres at this range
const tgt = (moa) => Object.assign({}, base, { groupMoa: 2, mvSdMps: 0, windErrMps: 0, rangeErrM: 0, shape: 'circle', widthM: moa * MOA_M(r.range), heightM: 0 });
const small = H.assess(tgt(0.5)), mid = H.assess(tgt(2)), big = H.assess(tgt(8));
check('size: 2 MOA rifle on a 0.5 MOA target is a hard shot (under 15%)', small.p < 0.15, (small.p * 100).toFixed(1) + '%');
check('size: matches the exact answer for a round target', Math.abs(small.p - (1 - Math.exp(-((0.25 * MOA_M(r.range)) ** 2) / (2 * (small.sigmaXMrad * r.range / 1000) ** 2)))) < 0.01);
check('size: chance rises with target size', small.p < mid.p && mid.p < big.p, [small.p, mid.p, big.p].map((x) => x.toFixed(2)).join(' < '));
check('size: an 8 MOA target with a 2 MOA rifle is nearly certain', big.p > 0.99, big.p);
check('size: the same small target is just as hard at any range (angular size is what counts)',
  Math.abs(small.p - H.assess(Object.assign(tgt(0.5), { rangeM: refs[0].range, tofS: refs[0].tof, velocityMps: refs[0].vel, widthM: 0.5 * MOA_M(refs[0].range) })).p) < 0.01);
check('size: target size reported in mrad', Math.abs(small.targetWMrad - 0.5 * 0.290888) < 1e-9 && small.targetHMrad === small.targetWMrad);

console.log(fails ? `\nFAILED (${fails})` : '\nALL PASSED');
process.exit(fails ? 1 : 0);
