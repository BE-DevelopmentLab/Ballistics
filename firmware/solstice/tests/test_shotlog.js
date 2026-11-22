// Checks web/shotlog.js: logging, truing from logged shots (synthetic truth), chronograph stats and CSV export.
'use strict';
const S = require('../web/solver.js'), L = require('../web/shotlog.js');
let fails = 0;
const check = (n, ok, d) => { console.log(`[${ok ? 'PASS' : 'FAIL'}] ${n}${d !== undefined ? '  ' + d : ''}`); if (!ok) fails++; };
const IN = 0.0254, YD = 0.9144;
const guess = { model: 'G7', bc: 0.176, muzzleMps: 780, sightHeightM: 1.5 * IN, zeroRangeM: 100 * YD, zeroTempC: 15, zeroPressureHpa: 1013.25, zeroRhPct: 50,
  massGr: 77, diameterM: 0.224 * IN, lengthM: 0.99 * IN, twistM: 8 * IN };
const truth = Object.assign({}, guess, { muzzleMps: 765, bc: 0.170 });
const shotOpts = { windSpeedMps: 0, windClockDeg: 90, slopeDeg: 0, spinDrift: true, aeroJump: true };
// A shot fired with the guessed load's hold; the impact is where the TRUE load put it.
function fire(yd, atm, extraUp) {
  const r = yd * YD, hold = S.solve(guess, atm, shotOpts, r).elevationMrad, real = S.solve(truth, atm, shotOpts, r).elevationMrad;
  return L.makeShot({ cfg: guess, profile: '77 SMK', atm, shot: shotOpts, rangeM: r, holdUpMrad: hold, impactUpMrad: hold - real + (extraUp || 0), time: 1794700000000 + yd });
}
const hot = { tempC: 30, pressureHpa: 960, rhPct: 30 }, cold = { tempC: 2, pressureHpa: 1005, rhPct: 80 };

const s1 = fire(400, hot);
check('log: impact low when the real MV is slower', s1.impactUpMrad < 0, s1.impactUpMrad.toFixed(2));
check('log: stores what the solver said', Math.abs(s1.predUpMrad - s1.holdUpMrad) < 1e-9);
check('log: come-up that hit = hold - impact', Math.abs(L.observedUp(s1) - S.solve(truth, hot, shotOpts, 400 * YD).elevationMrad) < 1e-9);

let t = L.trueFromLog([s1], guess);
check('truing: one 400 yd shot trues the MV only', t.ok && t.fitted === 'mv' && t.bc === guess.bc && /velocity only/.test(t.msg), t.msg);
check('truing: MV moves toward the truth', t.muzzleMps < 775 && t.muzzleMps > 755, t.muzzleMps.toFixed(1));
check('truing: that shot now fits', t.after < 0.01, t.after);

const band = [fire(300, hot), fire(400, cold), fire(500, hot), fire(600, cold)];
t = L.trueFromLog(band, guess);
check('truing: 300-600 yd shots alone fit both MV and BC (exact impacts recover 765 / 0.170)', t.ok && t.fitted === 'both' && Math.abs(t.muzzleMps - 765) < 1.5 && Math.abs(t.bc - 0.170) < 0.0015, t.muzzleMps.toFixed(1) + ' / ' + t.bc.toFixed(4));
check('truing: error shrinks', t.after < t.before / 5, t.before.toFixed(3) + ' -> ' + t.after.toFixed(3));
check('truing: one residual per shot used', t.residuals.length === 4);

const wide = [fire(400, hot), fire(500, cold), fire(800, hot), fire(1000, cold)];
t = L.trueFromLog(wide, guess);
check('truing: shots past 600 yd still count', t.ok && t.used === 4 && Math.abs(t.muzzleMps - 765) < 1.5 && Math.abs(t.bc - 0.170) < 0.0015, t.muzzleMps.toFixed(1) + ' / ' + t.bc.toFixed(4));

const chrono = Object.assign({}, guess, { muzzleMps: 765 });
const bandC = [300, 400, 500, 600].map((yd) => { const r = yd * YD, hold = S.solve(chrono, hot, shotOpts, r).elevationMrad; return L.makeShot({ cfg: chrono, atm: hot, shot: shotOpts, rangeM: r, holdUpMrad: hold, impactUpMrad: hold - S.solve(truth, hot, shotOpts, r).elevationMrad }); });
t = L.trueFromLog(bandC, chrono, { mvKnown: true });
check('truing: chronograph box keeps the MV and fits the BC from 300-600 yd', t.ok && t.fitted === 'bc' && t.muzzleMps === 765 && Math.abs(t.bc - 0.170) < 0.001 && /chronograph velocity/.test(t.msg), t.bc.toFixed(4));

// Realistic +/-0.1 mil impact errors: the joint 300-600 yd fit is flagged as rough; the chronograph route stays close.
const noise = [0.1, -0.1, 0.08, -0.06, 0.1, -0.1];
const noisy = [300, 350, 400, 450, 500, 600].map((yd, k) => fire(yd, k % 2 ? hot : cold, noise[k]));
t = L.trueFromLog(noisy, guess);
check('truing: noisy 300-600 yd joint fit gives a +/- and says the BC is rough', t.ok && t.bcSd > 0 && t.weak && /rough/.test(t.msg), 'BC ' + t.bc.toFixed(3) + ' +/- ' + (t.bcSd || 0).toFixed(3));
const noisyC = [300, 350, 400, 450, 500, 600].map((yd, k) => { const r = yd * YD, a = k % 2 ? hot : cold, hold = S.solve(chrono, a, shotOpts, r).elevationMrad;
  return L.makeShot({ cfg: chrono, atm: a, shot: shotOpts, rangeM: r, holdUpMrad: hold, impactUpMrad: hold - S.solve(truth, a, shotOpts, r).elevationMrad + noise[k] }); });
t = L.trueFromLog(noisyC, chrono, { mvKnown: true });
check('truing: same noise with a chronographed MV keeps the BC within 0.01', t.ok && Math.abs(t.bc - 0.170) < 0.01, t.bc.toFixed(4) + ' +/- ' + (t.bcSd || 0).toFixed(4));

t = L.trueFromLog([fire(400, hot), fire(450, cold)], guess);
check('truing: shots less than 150 yd apart true the MV only', t.ok && t.fitted === 'mv' && /150 yd apart/.test(t.msg));

const skipped = [fire(400, hot), Object.assign(fire(800, hot), { use: false })];
t = L.trueFromLog(skipped, guess);
check('truing: shots turned off are left out', t.used === 1);
check('truing: nothing from 300 yd out is refused', !L.trueFromLog([fire(200, hot)], guess).ok);

check('validate: needs range', !!L.validateShot({ rangeM: 0, holdUpMrad: 1, atm: hot }));
check('validate: needs hold', !!L.validateShot({ rangeM: 400, holdUpMrad: NaN, atm: hot }));
check('validate: needs conditions', !!L.validateShot({ rangeM: 400, holdUpMrad: 1, atm: {} }));
check('validate: good shot passes', L.validateShot({ rangeM: 400, holdUpMrad: 1, atm: hot }) === null);

const v = L.parseVelocities('2551, 2560 2548\n2565;2556 x');
const st = L.chronoStats(v);
check('chrono: parses numbers separated by commas, spaces, lines', v.length === 5, v.join(' '));
check('chrono: mean, ES, SD', Math.abs(st.mean - 2556) < 1e-9 && st.es === 17 && Math.abs(st.sd - 6.819) < 0.01, `${st.mean} ${st.es} ${st.sd.toFixed(3)}`);
check('chrono: empty string gives null', L.chronoStats(L.parseVelocities('')) === null);

const csv = L.toCsv([Object.assign(fire(400, hot), { note: 'gust, "left"', bullet: { name: 'Barnes', weightGr: 77, diaIn: 0.224, sd: 0.219, type: 'OTM', bc: 0.176, model: 'G7' } })]);
const lines = csv.trim().split('\n');
check('csv: header plus one row', lines.length === 2 && /^time,profile,range_m/.test(lines[0]));
check('csv: quotes notes with commas and quotes', /"gust, ""left"""/.test(lines[1]), lines[1].slice(-20));
check('csv: range in yards', lines[1].split(',')[3] === '400');
check('csv: bullet columns', /bullet,bullet_gr,bullet_dia_in,bullet_sd,bullet_type,bullet_bc,bc_model/.test(lines[0]) && /,Barnes,77,0\.224,0\.219,OTM,0\.176,G7,/.test(lines[1]), lines[1]);

console.log(fails ? `\nFAILED (${fails})` : '\nALL PASSED');
process.exit(fails ? 1 : 0);
