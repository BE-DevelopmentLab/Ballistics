// Stage planner: holds, hit chance and a run order for a list of targets, plus how the stage went.
// Solved on the phone with Solver (web/solver.js) and HitProb (web/hitprob.js), so it works with Solstice off.
// Pure logic with no page code, so the spotter unit can reuse it (it will add GPS/heading/laser target pins).
// Works in the browser (global `Stage`) and in node (module.exports).
(function (root) {
  'use strict';
  const isNode = typeof module !== 'undefined' && module.exports;
  const Solver = isNode ? require('./solver.js') : root.Solver;
  const HitProb = isNode ? require('./hitprob.js') : root.HitProb;

  // A target: { id, name, rangeM, shape: 'circle'|'rect', widthM, heightM, shots, results: ['H'|'M', ...] }
  function makeTarget(t) {
    return { id: t.id || Date.now().toString(36) + Math.random().toString(36).slice(2, 6), name: String(t.name || '').slice(0, 30),
      rangeM: +t.rangeM, shape: t.shape === 'rect' ? 'rect' : 'circle', widthM: +t.widthM, heightM: t.shape === 'rect' ? +t.heightM : +t.widthM,
      shots: Math.max(1, Math.min(20, Math.round(+t.shots || 1))), results: Array.isArray(t.results) ? t.results.slice() : [] };
  }
  function validateTarget(t) {
    if (!(t.rangeM >= 25 && t.rangeM <= 2000)) return 'Range should be 25 m to 2000 m (27 to 2187 yd).';
    if (!(t.widthM > 0) || (t.shape === 'rect' && !(t.heightM > 0))) return 'Enter the target size.';
    if (!(t.shots >= 1 && t.shots <= 20)) return 'Shots should be 1 to 20.';
    return null;
  }

  // Plan the stage.
  // targets: list of targets (in the shooter's order).
  // o: { cfg (profile, SI), atm {tempC, pressureHpa, rhPct}, shot (Solver shot: wind, direction, slope, switches),
  //      hit: { groupMoa, groupShots, mvSdMps, windErrMps, rangeErrM } (from the Hit chance section),
  //      order: 'entered'|'near'|'far'|'odds', lowPct (flag below this, 0-100) }
  // Returns rows in run order: { target, step, sol (Solver solution), p (0-1 or null), low, dialUpMrad (change from the
  //   previous target), dialRightMrad }, plus totals { shots, expectedHits, hits, fired }.
  function plan(targets, o) {
    const rows = targets.map((t, idx) => {
      const sol = Solver.solve(o.cfg, o.atm, o.shot, t.rangeM);
      let p = null;
      if (sol.ok && o.hit) {
        const a = HitProb.assess(Object.assign({}, o.hit, { muzzleMps: o.cfg.muzzleMps, zeroRangeM: o.cfg.zeroRangeM, sightHeightM: o.cfg.sightHeightM,
          tofS: sol.tofS, velocityMps: sol.velocityMps, rangeM: t.rangeM, shape: t.shape, widthM: t.widthM, heightM: t.heightM, trials: 8000 }));
        p = a.err ? null : a.p;
      }
      return { target: t, entered: idx, sol, p, low: p !== null && p * 100 < (o.lowPct || 0) };
    });
    const by = { near: (a, b) => a.target.rangeM - b.target.rangeM, far: (a, b) => b.target.rangeM - a.target.rangeM,
      odds: (a, b) => (b.p === null ? -1 : b.p) - (a.p === null ? -1 : a.p) || a.target.rangeM - b.target.rangeM };
    if (by[o.order]) rows.sort((a, b) => by[o.order](a, b) || a.entered - b.entered);
    let prev = null, shots = 0, expected = 0, hits = 0, fired = 0;
    rows.forEach((r, i) => {
      r.step = i + 1;
      r.dialUpMrad = r.sol.ok && prev && prev.sol.ok ? r.sol.elevationMrad - prev.sol.elevationMrad : null;
      r.dialRightMrad = r.sol.ok && prev && prev.sol.ok ? r.sol.windageMrad - prev.sol.windageMrad : null;
      shots += r.target.shots;
      if (r.p !== null) expected += r.p * r.target.shots;
      hits += r.target.results.filter((x) => x === 'H').length;
      fired += r.target.results.length;
      prev = r;
    });
    return { rows, totals: { shots, expectedHits: expected, hits, fired } };
  }

  // How the stage went against the plan: for the shots actually fired, the hits the plan expected vs the hits made.
  function compare(planned) {
    let expected = 0, hits = 0, fired = 0;
    const notes = [];
    planned.rows.forEach((r) => {
      const n = r.target.results.length;
      if (!n) return;
      const h = r.target.results.filter((x) => x === 'H').length;
      fired += n; hits += h;
      if (r.p !== null) {
        expected += r.p * n;
        if (n >= 2 && h / n < r.p - 0.35) notes.push((r.target.name || 'Target ' + r.step) + ': ' + h + ' of ' + n + ' hits, plan said ' + Math.round(r.p * 100) + '%. Check the range, wind call or come-up there.');
      }
    });
    return { fired, hits, expected, notes };
  }

  const Stage = { makeTarget, validateTarget, plan, compare };
  if (isNode) module.exports = Stage;
  else root.Stage = Stage;
})(typeof self !== 'undefined' ? self : this);
