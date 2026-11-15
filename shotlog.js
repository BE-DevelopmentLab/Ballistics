// Shot log, truing from logged shots, and chronograph string stats.
// Pure logic with no page code (the page keeps the list in localStorage), so the spotter unit can reuse it.
// Needs Solver (web/solver.js) for predictions and truing.
// Works in the browser (global `ShotLog`) and in node (module.exports).
(function (root) {
  'use strict';
  const Solver = (typeof module !== 'undefined' && module.exports) ? require('./solver.js') : root.Solver;
  const MID_MIN_M = 300 * 0.9144, FAR_MIN_M = 600 * 0.9144;     // the brief: MV from 300-600 yd, BC past 600 yd

  // A logged shot. Angles in mrad, + = up / right. Impact is where the bullet landed relative to the
  // aim point (+ high / right), so the come-up that would have centred it is hold - impact.
  // in: { cfg, profile, atm, atmSource, shot, rangeM, holdUpMrad, holdRightMrad, impactUpMrad, impactRightMrad, note, time }
  function makeShot(i) {
    const s = {
      id: (i.time || Date.now()).toString(36) + Math.random().toString(36).slice(2, 6),
      time: i.time || Date.now(), profile: i.profile || '', note: String(i.note || '').slice(0, 200),
      cfg: Object.assign({}, i.cfg), atm: Object.assign({}, i.atm), atmSource: i.atmSource || 'solstice',
      shot: Object.assign({}, i.shot), rangeM: +i.rangeM,
      holdUpMrad: +i.holdUpMrad, holdRightMrad: +i.holdRightMrad || 0, impactUpMrad: +i.impactUpMrad || 0, impactRightMrad: +i.impactRightMrad || 0,
      powder: String(i.powder || '').slice(0, 60), chargeGr: Number.isFinite(+i.chargeGr) && String(i.chargeGr).trim() !== '' ? +i.chargeGr : null,
      use: true,
    };
    const p = Solver.solve(s.cfg, s.atm, s.shot, s.rangeM);
    s.predUpMrad = p.ok ? p.elevationMrad : null;
    s.predRightMrad = p.ok ? p.windageMrad : null;
    return s;
  }
  function validateShot(i) {
    if (!(i.rangeM > 0) || i.rangeM > 3000) return 'Enter the range.';
    if (!Number.isFinite(+i.holdUpMrad)) return 'Enter the elevation you dialed or held.';
    if (![i.holdRightMrad, i.impactUpMrad, i.impactRightMrad].every((v) => v === undefined || v === '' || Number.isFinite(+v))) return 'Impact and windage must be numbers.';
    if (!i.atm || ![i.atm.tempC, i.atm.pressureHpa, i.atm.rhPct].every(Number.isFinite)) return 'No conditions to log with this shot.';
    return null;
  }
  const observedUp = (s) => s.holdUpMrad - s.impactUpMrad;
  const observedRight = (s) => s.holdRightMrad - s.impactRightMrad;
  const obsOf = (s) => ({ atm: s.atm, shot: s.shot, rangeM: s.rangeM, observedMrad: observedUp(s) });
  const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;

  // True the load from logged shots, using each shot's own conditions. All shots from 300 yd out are used.
  //   mvKnown (the MV came from a chronograph): only the BC is fitted. This is the reliable way.
  //   Otherwise MV and BC are fitted together (least squares) when the shots cover at least 150 yd of range;
  //   with a narrower spread only the MV is trued, since the two cannot be told apart.
  //   Inside 600 yd, MV and BC change the come-up in nearly the same way, so a fit of both from 300-600 yd
  //   shots is easily thrown off by small impact errors: the result carries a +/- (1 SD) so the shooter can see it.
  // cfg: the current profile (sight height, zero, bullet). Returns
  //   { ok, muzzleMps, bc, mvSd, bcSd, fitted: 'bc'|'mv'|'both', used, before: rms mrad, after: rms mrad, residuals, msg, weak }
  const SPREAD_MIN_M = 150 * 0.9144;
  function fit(usable, cfg, params) {
    // Gauss-Newton on the come-up residuals. params: names in cfg ('muzzleMps', 'bc').
    const c = Object.assign({}, cfg), step = { muzzleMps: 1, bc: cfg.bc * 0.01 };
    const resid = (cc) => usable.map((s) => { const p = Solver.solve(cc, s.atm, s.shot, s.rangeM); return p.ok ? observedUp(s) - p.elevationMrad : NaN; });
    let r = resid(c), JtJ = null;
    for (let it = 0; it < 12; it++) {
      if (!r.every(Number.isFinite)) return null;
      const J = params.map((k) => { const cc = Object.assign({}, c); cc[k] += step[k]; const rr = resid(cc); return rr.map((v, i) => -(v - r[i]) / step[k]); });
      const n = params.length, A = [], b = [];
      for (let a = 0; a < n; a++) { A.push([]); b.push(J[a].reduce((t, v, i) => t + v * r[i], 0)); for (let q = 0; q < n; q++) A[a].push(J[a].reduce((t, v, i) => t + v * J[q][i], 0)); }
      JtJ = A;
      let dx;
      if (n === 1) dx = [b[0] / A[0][0]];
      else { const det = A[0][0] * A[1][1] - A[0][1] * A[1][0]; if (!(Math.abs(det) > 0)) return null; dx = [(A[1][1] * b[0] - A[0][1] * b[1]) / det, (A[0][0] * b[1] - A[1][0] * b[0]) / det]; }
      params.forEach((k, a) => { c[k] += dx[a]; });
      if (!(c.bc > 0.02 && c.bc < 2 && c.muzzleMps > 150 && c.muzzleMps < 1600)) return null;
      r = resid(c);
      if (params.every((k, a) => Math.abs(dx[a]) < step[k] * 0.01)) break;
    }
    // 1 SD of each fitted value from the scatter left over (needs more shots than values).
    const dof = usable.length - params.length, sd = {};
    if (dof > 0 && JtJ) {
      const s2 = r.reduce((t, v) => t + v * v, 0) / dof, n = params.length;
      const inv = n === 1 ? [[1 / JtJ[0][0]]] : (() => { const d = JtJ[0][0] * JtJ[1][1] - JtJ[0][1] * JtJ[1][0]; return [[JtJ[1][1] / d, -JtJ[0][1] / d], [-JtJ[1][0] / d, JtJ[0][0] / d]]; })();
      params.forEach((k, a) => { sd[k] = Math.sqrt(Math.max(0, s2 * inv[a][a])); });
    }
    return { c, sd };
  }
  function trueFromLog(shots, cfg, opts) {
    const o = opts || {};
    const usable = shots.filter((s) => s.use !== false && s.rangeM >= MID_MIN_M && s.rangeM > cfg.zeroRangeM && observedUp(s) > 0);
    if (!usable.length) return { ok: false, msg: 'Log at least one shot from 300 yd (274 m) out, with its impact, to true the load.' };
    const rms = (cc) => {
      const r = usable.map((s) => { const p = Solver.solve(cc, s.atm, s.shot, s.rangeM); return { id: s.id, mrad: p.ok ? observedUp(s) - p.elevationMrad : NaN }; });
      return { list: r, rms: Math.sqrt(mean(r.map((x) => x.mrad * x.mrad))) };
    };
    const before = rms(cfg).rms;
    const ranges = usable.map((s) => s.rangeM), spread = Math.max.apply(null, ranges) - Math.min.apply(null, ranges);
    const fitted = o.mvKnown ? 'bc' : (usable.length >= 2 && spread >= SPREAD_MIN_M ? 'both' : 'mv');
    const f = fit(usable, cfg, fitted === 'bc' ? ['bc'] : fitted === 'mv' ? ['muzzleMps'] : ['muzzleMps', 'bc']);
    if (!f) return { ok: false, msg: 'Could not match the logged shots. Check the range, hold and impact you logged.' };
    const after = rms(f.c), bcSd = f.sd.bc === undefined ? null : f.sd.bc, mvSd = f.sd.muzzleMps === undefined ? null : f.sd.muzzleMps;
    const weak = fitted === 'both' && (bcSd === null || bcSd > 0.05 * f.c.bc);
    const msg = fitted === 'bc' ? 'Trued the BC from ' + usable.length + ' shot(s), keeping your chronograph velocity.'
      : fitted === 'mv' ? 'Trued the velocity only: ' + (usable.length < 2 ? 'one shot' : 'the shots are all at about the same range') + ', and that cannot separate velocity from BC. Log shots at least 150 yd apart, or tick the chronograph box to true the BC.'
      : 'Trued velocity and BC together from ' + usable.length + ' shots.' + (weak ? ' Between 300 and 600 yd the two look alike, so this BC is rough. A chronographed velocity (tick the box) gives a much firmer BC.' : '');
    return { ok: true, muzzleMps: f.c.muzzleMps, bc: f.c.bc, mvSd, bcSd, fitted, used: usable.length, before, after: after.rms, residuals: after.list, msg, weak };
  }

  // Chronograph string: velocities in any one unit -> { n, mean, es, sd } in the same unit (sd is the sample SD).
  function parseVelocities(text) {
    return String(text || '').split(/[\s,;]+/).map((x) => x.trim()).filter(Boolean).map(Number).filter((v) => Number.isFinite(v) && v > 0);
  }
  function chronoStats(v) {
    if (!v.length) return null;
    const m = mean(v), es = Math.max.apply(null, v) - Math.min.apply(null, v);
    const sd = v.length > 1 ? Math.sqrt(v.reduce((a, x) => a + (x - m) ** 2, 0) / (v.length - 1)) : 0;
    return { n: v.length, mean: m, es, sd };
  }

  // Spreadsheet export (one row per shot).
  function toCsv(shots) {
    const head = ['time', 'profile', 'range_m', 'range_yd', 'hold_up_mrad', 'hold_right_mrad', 'impact_up_mrad', 'impact_right_mrad',
      'come_up_that_hit_mrad', 'solver_said_mrad', 'temp_c', 'pressure_hpa', 'rh_pct', 'conditions_from', 'wind_mps', 'wind_from_deg', 'slope_deg',
      'muzzle_fps', 'bc', 'drag_model', 'powder', 'charge_gr', 'used_for_truing', 'note'];
    const q = (v) => { const t = String(v === null || v === undefined ? '' : v); return /[",\n]/.test(t) ? '"' + t.replace(/"/g, '""') + '"' : t; };
    const n = (v, d) => (Number.isFinite(v) ? (+v.toFixed(d)).toString() : '');
    const rows = shots.map((s) => [new Date(s.time).toISOString(), s.profile, n(s.rangeM, 1), n(s.rangeM / 0.9144, 1), n(s.holdUpMrad, 2), n(s.holdRightMrad, 2),
      n(s.impactUpMrad, 2), n(s.impactRightMrad, 2), n(observedUp(s), 2), n(s.predUpMrad, 2), n(s.atm.tempC, 1), n(s.atm.pressureHpa, 1), n(s.atm.rhPct, 0),
      s.atmSource, n(s.shot.windSpeedMps, 2), n(s.shot.windClockDeg, 0), n(s.shot.slopeDeg, 1), n(s.cfg.muzzleMps / 0.3048, 0), n(s.cfg.bc, 3), s.cfg.model, s.powder || '', n(s.chargeGr, 1),
      s.use === false ? 'no' : 'yes', s.note].map(q).join(','));
    return [head.join(',')].concat(rows).join('\n') + '\n';
  }

  const ShotLog = { makeShot, validateShot, observedUp, observedRight, trueFromLog, parseVelocities, chronoStats, toCsv, MID_MIN_M, FAR_MIN_M };
  if (typeof module !== 'undefined' && module.exports) module.exports = ShotLog;
  else root.ShotLog = ShotLog;
})(typeof self !== 'undefined' ? self : this);
