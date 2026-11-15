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
      cfg: Object.assign({}, i.cfg), atm: Object.assign({}, i.atm), atmSource: i.atmSource || 'board',
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

  // True the load from logged shots, using each shot's own conditions.
  //   shots past 600 yd true the BC; shots from 300 to 600 yd true the muzzle velocity.
  //   With only one band, only that value changes (with only far shots, the MV is taken as right, e.g. chronographed).
  // cfg: the current profile (sight height, zero, bullet). Returns
  //   { ok, muzzleMps, bc, usedMid, usedFar, before: rms mrad, after: rms mrad, residuals: [{id, mrad}], msg }
  function trueFromLog(shots, cfg) {
    const usable = shots.filter((s) => s.use !== false && s.rangeM >= MID_MIN_M && s.rangeM > cfg.zeroRangeM && observedUp(s) > 0);
    const mid = usable.filter((s) => s.rangeM < FAR_MIN_M), far = usable.filter((s) => s.rangeM >= FAR_MIN_M);
    if (!mid.length && !far.length) return { ok: false, msg: 'Log at least one shot past 300 yd (274 m) with its impact to true the load.' };
    const c = Object.assign({}, cfg);
    const rms = (cc) => {
      const r = usable.map((s) => { const p = Solver.solve(cc, s.atm, s.shot, s.rangeM); return { id: s.id, mrad: p.ok ? observedUp(s) - p.elevationMrad : NaN }; });
      return { list: r, rms: Math.sqrt(mean(r.map((x) => x.mrad * x.mrad))) };
    };
    const before = rms(c).rms;
    for (let pass = 0; pass < (mid.length && far.length ? 4 : 1); pass++) {
      if (mid.length) {
        const v = mid.map((s) => Solver.trueMv(c, obsOf(s))).filter((x) => x !== null);
        if (!v.length) return { ok: false, msg: 'Could not match the 300-600 yd shots. Check the range, hold and impact you logged.' };
        c.muzzleMps = mean(v);
      }
      if (far.length) {
        const v = far.map((s) => Solver.trueBc(c, obsOf(s))).filter((x) => x !== null);
        if (!v.length) return { ok: false, msg: 'Could not match the shots past 600 yd. Check the range, hold and impact you logged.' };
        c.bc = mean(v);
      }
    }
    const after = rms(c);
    let msg = mid.length && far.length ? 'Trued velocity from ' + mid.length + ' mid-range and BC from ' + far.length + ' long-range shot(s).'
      : mid.length ? 'Trued velocity only (no shots past 600 yd yet, so the BC is unchanged).'
      : 'Trued BC only (no 300-600 yd shots, so the velocity is taken as right; chronograph it if you can).';
    return { ok: true, muzzleMps: c.muzzleMps, bc: c.bc, usedMid: mid.length, usedFar: far.length, before, after: after.rms, residuals: after.list, msg };
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
