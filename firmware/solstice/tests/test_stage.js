// Checks web/stage.js: per-target holds match the solver, run orders, dial changes, low-odds flags, results vs plan.
'use strict';
const S = require('../web/solver.js'), St = require('../web/stage.js');
let fails = 0;
const check = (n, ok, d) => { console.log(`[${ok ? 'PASS' : 'FAIL'}] ${n}${d !== undefined ? '  ' + d : ''}`); if (!ok) fails++; };
const IN = 0.0254, YD = 0.9144;
const cfg = { model: 'G7', bc: 0.176, muzzleMps: 765, sightHeightM: 1.5 * IN, zeroRangeM: 100 * YD, zeroTempC: 15, zeroPressureHpa: 1013.25, zeroRhPct: 50 };
const atm = { tempC: 20, pressureHpa: 1000, rhPct: 50 }, shot = { windSpeedMps: 3, windClockDeg: 90, slopeDeg: 0 };
const hit = { groupMoa: 1, groupShots: 5, mvSdMps: 3, windErrMps: 0.9, rangeErrM: 2 };
const T = (name, yd, inch, shots) => St.makeTarget({ name, rangeM: yd * YD, shape: 'circle', widthM: inch * IN, shots: shots || 2 });
const targets = [T('Big far', 800, 24), T('Small near', 300, 4), T('Mid', 500, 10), T('Tiny far', 900, 6)];
const o = { cfg, atm, shot, hit, order: 'entered', lowPct: 40 };

let p = St.plan(targets, o);
check('every target solved', p.rows.every((r) => r.sol.ok));
check('holds match the solver directly', Math.abs(p.rows[2].sol.elevationMrad - S.solve(cfg, atm, shot, 500 * YD).elevationMrad) < 1e-12);
check('entered order kept', p.rows.map((r) => r.target.name).join() === 'Big far,Small near,Mid,Tiny far');
check('first target has no dial change, the next one does', p.rows[0].dialUpMrad === null && Math.abs(p.rows[1].dialUpMrad - (p.rows[1].sol.elevationMrad - p.rows[0].sol.elevationMrad)) < 1e-12);
check('hit chance for each target', p.rows.every((r) => r.p > 0 && r.p <= 1), p.rows.map((r) => Math.round(r.p * 100)).join(' '));
check('a 6 in plate at 900 yd is flagged as low odds', p.rows[3].low && !p.rows[0].low, Math.round(p.rows[3].p * 100) + '%');
check('total shots', p.totals.shots === 8);

p = St.plan(targets, Object.assign({}, o, { order: 'near' }));
check('near to far order', p.rows.map((r) => r.target.name).join() === 'Small near,Mid,Big far,Tiny far' && p.rows.map((r) => r.step).join() === '1,2,3,4');
check('near to far: every dial change is up', p.rows.slice(1).every((r) => r.dialUpMrad > 0));
p = St.plan(targets, Object.assign({}, o, { order: 'far' }));
check('far to near order', p.rows[0].target.name === 'Tiny far' && p.rows[3].target.name === 'Small near');
p = St.plan(targets, Object.assign({}, o, { order: 'odds' }));
check('best odds first', p.rows.every((r, i) => i === 0 || r.p <= p.rows[i - 1].p), p.rows.map((r) => Math.round(r.p * 100)).join(' '));

// results vs plan
targets[0].results = ['H', 'H']; targets[3].results = ['M', 'M']; targets[1].results = ['H'];
p = St.plan(targets, o);
const c = St.compare(p);
check('compare: counts shots fired and hits', c.fired === 5 && c.hits === 3);
check('compare: expected hits from the plan', Math.abs(c.expected - (2 * p.rows[0].p + p.rows[1].p + 2 * p.rows[3].p)) < 1e-9, c.expected.toFixed(2));
const sure = [St.makeTarget({ name: 'Sure thing', rangeM: 200 * YD, shape: 'rect', widthM: 0.6, heightM: 0.6, shots: 3, results: ['M', 'M', 'H'] })];
check('compare: flags a target that went much worse than planned', St.compare(St.plan(sure, o)).notes.length === 1, St.compare(St.plan(sure, o)).notes[0]);

check('validate: range', !!St.validateTarget(T('x', 10, 10)));
check('validate: size', !!St.validateTarget(T('x', 400, 0)));
check('validate: good target', St.validateTarget(T('x', 400, 10)) === null);
check('no hit settings: no hit chance, still solves', St.plan([T('a', 400, 10)], Object.assign({}, o, { hit: null })).rows[0].p === null);

console.log(fails ? `\nFAILED (${fails})` : '\nALL PASSED');
process.exit(fails ? 1 : 0);
