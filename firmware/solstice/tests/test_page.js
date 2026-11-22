// Runs web/index.html's script against a minimal fake DOM to catch runtime errors
// and check the rendering logic. Does not test real Bluetooth.
'use strict';
const fs = require('fs'), path = require('path'), vm = require('vm');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const inline = html.match(/<script>\n([\s\S]*?)<\/script>/)[1];
const protoSrc = fs.readFileSync(path.join(__dirname, '../web/protocol.js'), 'utf8');

const ids = [...html.matchAll(/id="(\w+)"/g)].map(m => m[1]);
const defaults = {}; // initial values copied from the HTML for inputs
for (const m of html.matchAll(/<(?:input|select)\b[^>]*>/g)) {
  const id = (m[0].match(/id="(\w+)"/) || [])[1], val = (m[0].match(/value="([^"]*)"/) || [])[1];
  if (id) defaults[id] = val ?? '';
}
const els = {};
function mkEl(id) {
  const listeners = {};
  return { id, value: defaults[id] ?? '', textContent: '', style: {}, className: '', children: [], onclick: null,
    classList: { _s: new Set(), add(c){this._s.add(c)}, remove(c){this._s.delete(c)}, toggle(c,f){ f ? this._s.add(c) : this._s.delete(c) }, contains(c){return this._s.has(c)} },
    addEventListener(t, f){ (listeners[t] = listeners[t] || []).push(f) }, _fire(t){ (listeners[t]||[]).forEach(f=>f({})) },
    appendChild(c){ this.children.push(c) } };
}
ids.forEach(id => els[id] = mkEl(id));
els.model.value = 'G7'; els.windSrc.value = 'sensor'; els.slopeSrc.value = 'imu';
els.angUnit.value = 'MIL'; els.units.value = 'imp'; els.windDir.value = '90';
els.spinOn.value = 'on'; els.corOn.value = 'off'; els.azSrc.value = 'manual'; els.btdir.value = 'R';
els.aeroOn.value = 'on';
// slopeSrc/azSrc start as 'imu' (like settings saved by an older version); the page must switch them to manual.
const ctx = { console, setTimeout, clearTimeout, setInterval, clearInterval, Math, Number, Uint8Array, DataView, Array, Error, JSON, Object,
  document: { getElementById: id => els[id], createElement: () => ({ value: '', textContent: '' }) },
  localStorage: { _d: {}, getItem(k){ return this._d[k] ?? null }, setItem(k,v){ this._d[k]=v } },
  navigator: {} };
ctx.self = ctx; vm.createContext(ctx);
vm.runInContext(protoSrc, ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/hitprob.js'), 'utf8'), ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/solver.js'), 'utf8'), ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/shotlog.js'), 'utf8'), ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/groupcalc.js'), 'utf8'), ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/stage.js'), 'utf8'), ctx);
vm.runInContext(inline + '\n;globalThis.__t = { leadMrad, renderSolution, renderEnv, hideBattery, refreshLabels, setEnv: (e) => { lastEnv = e; renderEnv(); }, readConfig, readRequest, convertFormUnits, windFromPoint, windDesc, windDirDeg, setWindDir, setTab, load, refreshSlopeNote, onSleepClick, sleepBoard, setConn: (d, c) => { device = d; chars = c; }, unit, saveProfile, selectProfile, deleteProfile, requestCard, onSolution, readCard, validateCard, renderCard, validateConfig, earthError, bulletComplete, profiles: () => profiles, cardRows: () => cardRows, renderHit, hitInputs, logShot, renderLog, toggleShot, trueLoad, applyTrued, exportCsv, renderChrono, fillLogFromSolution, shots: () => shots, setLastEnv: (e) => { lastEnv = e; }, grpTap, grpUndo, grpTarget, grpMoveTo, grpRemovePhoto, renderGroup, grpUseInHit, setGrpMode, saveLoadTest, renderLoadTests, grpState: () => grp, loadTests: () => loadTests, addTarget, stageAction, saveStage, selectStage, deleteStage, renderStage, loadStages, stage: () => stage, stages: () => stages, readBullet, renderBullet, bulletFromProfile, bulletToProfile };', ctx);
const t = ctx.__t;

let fails = 0;
const check = (n, ok, d) => { console.log(`[${ok ? 'PASS' : 'FAIL'}] ${n}${d ? '  ' + d : ''}`); if (!ok) fails++; };

// config / request read from the default form (imperial -> SI)
const cfg = t.readConfig();
check('config converts 2600 fps to m/s', Math.abs(cfg.muzzleMps - 792.48) < 0.01, cfg.muzzleMps);
check('config converts 1.5 in to m', Math.abs(cfg.sightHeightM - 0.0381) < 1e-6, cfg.sightHeightM);
check('config converts 59 F to 15 C', Math.abs(cfg.zeroTempC - 15) < 1e-9, cfg.zeroTempC);
check('config converts 29.92 inHg to hPa', Math.abs(cfg.zeroPressureHpa - 1013.2) < 0.1, cfg.zeroPressureHpa);
const req = t.readRequest();
check('request: 500 yd in metres', Math.abs(req.rangeM - 457.2) < 0.01, req.rangeM);
check('request: wind from 90 deg sent as 90', req.windClockDeg === 90, req.windClockDeg);

// ---- wind direction compass ----
// The arrow head follows the finger and shows where the wind blows TO; the value is where it comes FROM.
check('compass: arrow pointing at target (up) = tailwind, from 180', t.windFromPoint(0, -50) === 180, t.windFromPoint(0, -50));
check('compass: arrow pointing at shooter (down) = headwind, from 0', t.windFromPoint(0, 50) === 0, t.windFromPoint(0, 50));
check('compass: arrow pointing left = from the right, 90', t.windFromPoint(-50, 0) === 90, t.windFromPoint(-50, 0));
check('compass: arrow pointing right = from the left, 270', t.windFromPoint(50, 0) === 270, t.windFromPoint(50, 0));
check('compass: snaps to 5 deg', t.windFromPoint(-50, -2) === 90 && t.windFromPoint(-50, -10) % 5 === 0, t.windFromPoint(-50, -10));
t.setWindDir(365);
check('compass: wraps past 360 and is sent in degrees', els.windDir.value === '5' && t.readRequest().windClockDeg === 5, els.windDir.value);
t.setWindDir(-5);
check('compass: wraps below 0', els.windDir.value === '355' && els.windDeg.textContent === '355°', els.windDir.value);
check('compass: describes headwind / tailwind', t.windDesc(0) === 'headwind' && t.windDesc(180) === 'tailwind');
check('compass: describes full value from the right', t.windDesc(90) === 'from the right, 100% crosswind', t.windDesc(90));
check('compass: describes quartering wind', t.windDesc(315) === 'from the front-left, 71% crosswind', t.windDesc(315));
t.setWindDir(90);
check('compass: saved with the form', JSON.parse(ctx.localStorage.getItem('bc.v1')).windDir === '90');
ctx.localStorage.setItem('bc.v1', JSON.stringify({ clock: '4' }));
els.windDir.value = '0'; t.load();
check('compass: older saved clock (4 o\'clock) becomes 120 deg', els.windDir.value === '120', els.windDir.value);
els.windDir.value = '90';
check('compass: no clock picker left in the page', !/id="clock"/.test(html));

// ---- Simple / Advanced tabs ----
const secTag = (id) => (html.match(new RegExp('<section id="' + id + '"[^>]*>')) || [''])[0];
check('tabs: opens on Simple by default', els.app.classList.contains('simple') && els.tabSimple.classList.contains('on'));
check('tabs: Simple shows profile, solution, shot, live conditions',
  ['secProf','secSol','secShot','secLive'].every(id => secTag(id) && !/adv-only/.test(secTag(id))));
check('tabs: range card, earth effects, rifle and load are Advanced only',
  ['secCard','secEarth','secRifle'].every(id => /adv-only/.test(secTag(id))));
check('tabs: slope row and moving target are Advanced only',
  /<div class="row gap adv-only">\s*<div><label>Slope source/.test(html) && /<div class="adv-only">\s*<h2[^>]*>Moving target/.test(html));
check('tabs: profile save/delete are Advanced only', /<div class="row gap adv-only">\s*<div><label>Name/.test(html));
t.setTab('adv');
check('tabs: Advanced shows everything', !els.app.classList.contains('simple') && els.tabAdv.classList.contains('on') && ctx.localStorage.getItem('bc.tab') === 'adv');
els.slopeManual.value = '5'; t.refreshSlopeNote();
check('tabs: no slope note in Advanced', els.slopeNote.classList.contains('hide'));
t.setTab('simple');
check('tabs: Simple warns when an Advanced slope is applied', !els.slopeNote.classList.contains('hide') && /5° slope/.test(els.slopeNote.textContent), els.slopeNote.textContent);
els.slopeManual.value = '0'; t.refreshSlopeNote();
check('tabs: no slope note when flat', els.slopeNote.classList.contains('hide'));
check('demo mode removed', !/demo/i.test(html));
check('request: sensor wind flag set', req.sensorWind);
check('no tilt sensor: saved "measured" slope switched to manual', els.slopeSrc.value === 'manual' && !req.imuSlope, els.slopeSrc.value);
check('no compass: saved "measured" direction switched to manual', els.azSrc.value === 'manual' && !req.imuAzimuth, els.azSrc.value);
els.slopeSrc.value = 'imu'; els.azSrc.value = 'imu';
check('IMU flags never sent even if a sensor option is picked', !t.readRequest().imuSlope && !t.readRequest().imuAzimuth);
els.slopeSrc.value = 'manual'; els.azSrc.value = 'manual';
check('sensor options are disabled in the page', /<option value="imu" disabled>Measured by sensor/.test(html) && /<option value="imu" disabled>Measured by compass/.test(html));
check('slope and direction default to manual in the page', /<select id="slopeSrc"><option value="manual" selected>/.test(html) && /<select id="azSrc"><option value="manual" selected>/.test(html));
check('aero jump flag follows the switch (on)', req.aeroJump === true);
els.aeroOn.value = 'off'; check('aero jump flag follows the switch (off)', t.readRequest().aeroJump === false); els.aeroOn.value = 'on';

// rendering
t.renderSolution({ ok: true, elevationMrad: 6.15, windageMrad: -1.91, tofS: 1.066, velocityMps: 450, dropM: 0, rangeM: 640, windMps: 4.47, slopeDeg: 0 });
check('renders UP MIL', els.outElev.textContent === '6.15' && els.outElevK.textContent === 'UP MIL', els.outElev.textContent + ' ' + els.outElevK.textContent);
check('renders LEFT for negative windage', els.outWind.textContent === '1.91' && els.outWindK.textContent === 'LEFT MIL', els.outWindK.textContent);
els.angUnit.value = 'MOA';
t.renderSolution({ ok: true, elevationMrad: 1, windageMrad: 1, tofS: 1, velocityMps: 300, dropM: 0, rangeM: 100, windMps: 0, slopeDeg: 0 });
check('MOA conversion (1 mrad = 3.44 MOA)', els.outElev.textContent === '3.44', els.outElev.textContent);
t.renderSolution({ ok: false });
check('failed solution shows N/A', els.outElev.textContent === 'N/A');

// ---- moving target lead ----
els.angUnit.value = 'MIL'; els.units.value = 'imp';
els.tgtDir.value = 'none';
check('lead: off when target not moving', t.leadMrad(1, 500).mrad === null);
t.renderSolution({ ok: true, elevationMrad: 2, windageMrad: 0.5, tofS: 0.8, velocityMps: 500, dropM: 0, rangeM: 500, windMps: 0, slopeDeg: 0 });
check('lead: no lead line when not moving', els.outLead.textContent === '', els.outLead.textContent);
els.tgtDir.value = 'R'; els.tgtSpeed.value = '10'; els.tgtDelay.value = '0';   // 10 mph = 4.4704 m/s
const L1 = t.leadMrad(0.8, 500);                                            // 4.4704*0.8/500*1000 = 7.1526 mrad
check('lead: 10 mph, 0.8 s, 500 m = +7.15 mrad (right)', Math.abs(L1.mrad - 7.15264) < 1e-4, L1.mrad);
els.tgtDir.value = 'L';
check('lead: moving left gives negative (hold left)', t.leadMrad(0.8, 500).mrad < 0);
els.tgtDelay.value = '200';
const L2 = t.leadMrad(0.8, 500);                                            // 4.4704*1.0/500*1000 = 8.9408
check('lead: 200 ms delay adds to time', Math.abs(L2.mrad + 8.9408) < 1e-4, L2.mrad);
els.tgtDelay.value = '0'; els.tgtDir.value = 'R';
t.renderSolution({ ok: true, elevationMrad: 2, windageMrad: 0.5, tofS: 0.8, velocityMps: 500, dropM: 0, rangeM: 500, windMps: 0, slopeDeg: 0 });
check('lead: line shows lead and total with wind', /Lead RIGHT 7\.15 MIL/.test(els.outLead.textContent) && /Total hold with wind: RIGHT 7\.65 MIL/.test(els.outLead.textContent), els.outLead.textContent);
els.tgtDir.value = 'L';
t.renderSolution({ ok: true, elevationMrad: 2, windageMrad: 0.5, tofS: 0.8, velocityMps: 500, dropM: 0, rangeM: 500, windMps: 0, slopeDeg: 0 });
check('lead: left lead offsets right wind', /Lead LEFT 7\.15/.test(els.outLead.textContent) && /Total hold with wind: LEFT 6\.65/.test(els.outLead.textContent), els.outLead.textContent);
els.angUnit.value = 'MOA';
t.renderSolution({ ok: true, elevationMrad: 2, windageMrad: 0, tofS: 0.8, velocityMps: 500, dropM: 0, rangeM: 500, windMps: 0, slopeDeg: 0 });
check('lead: MOA units', /Lead LEFT 24\.59 MOA/.test(els.outLead.textContent), els.outLead.textContent);
els.angUnit.value = 'MIL';
els.tgtSpeed.value = '-1'; check('lead: negative speed rejected', /speed/.test(t.leadMrad(1, 500).err || ''));
els.tgtSpeed.value = '100'; check('lead: 100 mph rejected', /too high/.test(t.leadMrad(1, 500).err || ''));
els.tgtSpeed.value = '10'; els.tgtDelay.value = '5000'; check('lead: 5 s delay rejected', /Delay/.test(t.leadMrad(1, 500).err || ''));
els.units.value = 'met'; els.tgtSpeed.value = '5'; els.tgtDelay.value = '0'; els.tgtDir.value = 'R';
check('lead: metric 5 m/s, 1 s, 500 m = 10 mrad', Math.abs(t.leadMrad(1, 500).mrad - 10) < 1e-9);
els.units.value = 'imp'; els.tgtDir.value = 'none'; els.tgtSpeed.value = '3';

// ---- saved profiles ----
els.angUnit.value = 'MIL';
let r = t.saveProfile('');
check('profile: empty name rejected', !r.ok, r.msg);
els.bc.value = '5';
r = t.saveProfile('bad');
check('profile: invalid load rejected', !r.ok && t.profiles().list.length === 0, r.msg);
els.bc.value = '0.243';
r = t.saveProfile('6.5 CM');
check('profile: saved', r.ok && t.profiles().list.length === 1 && t.profiles().active === r.id);
check('profile: stored in SI', Math.abs(t.profiles().list[0].cfg.muzzleMps - 792.48) < 0.01);
els.mv.value = '2700'; els.bc.value = '0.3';
r = t.saveProfile('308 Win');
check('profile: second saved, sorted by name', t.profiles().list.length === 2 && t.profiles().list[0].name === '308 Win');
r = t.saveProfile('308 win');
check('profile: same name (any case) replaces', t.profiles().list.length === 2);
const id65 = t.profiles().list.find(p => p.name === '6.5 CM').id;
check('profile: select restores fields', t.selectProfile(id65) && +els.mv.value === 2600 && +els.bc.value === 0.243, els.mv.value + ' ' + els.bc.value);
check('profile: not dirty right after select', !/Edited/.test(els.profNote.textContent), els.profNote.textContent);
els.mv.value = '2650'; els.mv._fire('input');
check('profile: dirty after edit', /Edited/.test(els.profNote.textContent), els.profNote.textContent);
check('profile: persisted to storage', JSON.parse(ctx.localStorage.getItem('bc.profiles.v1')).list.length === 2);
els.mv.value = '2600';

// ---- bullet data / spin drift ----
check('no bullet data by default (all zero)', (c => !c.massGr && !c.diameterM && !c.lengthM && !c.twistM)(t.readConfig()));
check('spin drift on but no bullet: warned', /needs bullet/.test(els.earthNote.textContent) || (t.bulletComplete(t.readConfig()) === false));
els.bmass.value = '175'; els.bdia.value = '0.308'; els.blen.value = '1.24'; els.btw.value = '10';
let bc = t.readConfig();
check('bullet: grains/inches -> SI', bc.massGr === 175 && Math.abs(bc.diameterM - 0.0078232) < 1e-6 && Math.abs(bc.lengthM - 0.031496) < 1e-6 && Math.abs(bc.twistM - 0.254) < 1e-6, JSON.stringify([bc.massGr, bc.diameterM, bc.lengthM, bc.twistM]));
check('bullet: valid data passes validation', t.validateConfig(bc) === null, t.validateConfig(bc));
els.btdir.value = 'L'; check('bullet: left-hand twist is negative', t.readConfig().twistM < 0); els.btdir.value = 'R';
els.btw.value = ''; check('bullet: three of four fields rejected', /all four/.test(t.validateConfig(t.readConfig()))); els.btw.value = '10';
els.btw.value = '1'; check('bullet: absurd twist (1:1 in) rejected', /Twist/.test(t.validateConfig(t.readConfig())), t.validateConfig(t.readConfig())); els.btw.value = '10';
els.blen.value = '0.2'; check('bullet: too-short length rejected', /length/i.test(t.validateConfig(t.readConfig()))); els.blen.value = '1.24';
els.bmass.value = '5'; check('bullet: 5 grain rejected', /weight/i.test(t.validateConfig(t.readConfig()))); els.bmass.value = '175';
check('bullet: spin drift flag follows the switch', t.readRequest().spinDrift === true);
els.spinOn.value = 'off'; check('bullet: spin drift off -> flag clear', t.readRequest().spinDrift === false); els.spinOn.value = 'on';

// ---- Coriolis inputs ----
let rq = t.readRequest();
check('coriolis off by default, latitude sent as 0', !rq.coriolis && rq.latitudeDeg === 0);
els.corOn.value = 'on';
check('coriolis on without latitude is an error', /latitude/i.test(t.earthError(t.readRequest()) || ''));
els.lat.value = '44.25'; els.azSrc.value = 'manual'; els.azManual.value = '0';
check('coriolis with latitude + typed direction is valid', t.earthError(t.readRequest()) === null);
check('coriolis request carries latitude, no IMU-azimuth flag', t.readRequest().latitudeDeg === 44.25 && t.readRequest().imuAzimuth === false);
els.azManual.value = '9999';
check('manual direction out of range rejected', /direction/i.test(t.earthError(t.readRequest()) || ''));
els.azManual.value = '135'; check('manual direction accepted', t.earthError(t.readRequest()) === null && t.readRequest().azimuthDeg === 135);
els.lat.value = '120'; check('latitude 120 rejected', t.earthError(t.readRequest()) !== null); els.lat.value = '44.25';
const card2 = t.readCard(); check('card carries Earth settings', card2.coriolis && card2.spinDrift && card2.latitudeDeg === 44.25 && card2.azimuthDeg === 135);
const wire = ctx.Proto.encodeCard(Object.assign({}, card2, { count: 3 }));
check('card encodes to 32 bytes', wire.length === 32, wire.length);
els.corOn.value = 'off'; els.azSrc.value = 'imu';

// solution note shows spin drift
els.angUnit.value = 'MIL';
t.renderSolution({ ok: true, elevationMrad: 6, windageMrad: -1, tofS: 1, velocityMps: 400, rangeM: 900, windMps: 2, spinDriftMrad: -0.37 });
check('solution note reports spin drift left', /0\.37 MIL left/.test(els.outNote.textContent), els.outNote.textContent);
t.renderSolution({ ok: true, elevationMrad: 6, windageMrad: -1, tofS: 1, velocityMps: 400, rangeM: 900, windMps: 2, spinDriftMrad: 0 });
check('no spin drift note when zero', els.outNote.textContent === '');

// profiles carry bullet data
r = t.saveProfile('308 175 SMK');
check('profile with bullet saved', r.ok && t.profiles().list.find(p => p.name === '308 175 SMK').cfg.twistM > 0.25);
els.bmass.value = '168'; els.bdia.value = '0.3'; els.btdir.value = 'L';
t.selectProfile(r.id);
check('profile restores bullet fields', +els.bmass.value === 175 && +els.bdia.value === 0.308 && +els.blen.value === 1.24 && +els.btw.value === 10 && els.btdir.value === 'R',
      [els.bmass.value, els.bdia.value, els.blen.value, els.btw.value, els.btdir.value].join(' '));
els.bmass.value = '180'; els.bmass._fire('input');
check('profile marked edited when bullet changes', /Edited/.test(els.profNote.textContent), els.profNote.textContent);
els.bmass.value = '175'; t.deleteProfile(r.id);

// ---- range card ----
const card = t.readCard();
check('card: reads form (100 yd start, 10 rows)', card.count === 10 && Math.abs(card.startM - 91.44) < 0.01 && Math.abs(card.stepM - 91.44) < 0.01, JSON.stringify(card));
els.cardCount.value = '25'; check('card: 25 rows rejected', !!t.validateCard(t.readCard()));
els.cardCount.value = '0'; check('card: 0 rows rejected', !!t.validateCard(t.readCard()));
els.cardCount.value = '10'; els.cardStep.value = '300'; check('card: sweep past 2000 m rejected', !!t.validateCard(t.readCard()));
els.cardStep.value = '100'; check('card: valid sweep accepted', t.validateCard(t.readCard()) === null);

// ---- unit flip keeps meaning
els.units.value = 'met';
t.convertFormUnits();
check('flip to metric: muzzle velocity stays 2600 fps', +els.mv.value === 2600, els.mv.value);
check('flip to metric: zero temperature stays 59 F', +els.zt.value === 59, els.zt.value);
check('flip to metric: zero pressure stays 29.92 inHg', +els.zp.value === 29.92, els.zp.value);
check('flip to metric: bullet stays 0.308 in, 1:10 in', +els.bdia.value === 0.308 && +els.btw.value === 10, els.bdia.value + ' ' + els.btw.value);
check('flip to metric: zero range 100 yd -> 91 m', +els.zero.value === 91, els.zero.value);
check('flip to metric: 500 yd -> 457 m', Math.abs(+els.range.value - 457) < 1, els.range.value);
check('metric: rifle config unchanged in SI', Math.abs(t.readConfig().muzzleMps - 792.48) < 0.01 && Math.abs(t.readConfig().zeroTempC - 15) < 1e-9);
t.refreshLabels();
check('metric: rifle labels stay imperial', /fps/.test(els.lMv.textContent) && /°F/.test(els.lZt.textContent) && /inHg/.test(els.lZp.textContent) && /\(in\)/.test(els.lBdia.textContent) && /\(m\)/.test(els.lZero.textContent),
  [els.lMv.textContent, els.lZt.textContent, els.lZp.textContent, els.lBdia.textContent, els.lZero.textContent].join(' | '));
t.setEnv({ tempC: 15, rhPct: 50, pressureHpa: 1013.25, windMps: 2, headingDeg: 0, pitchDeg: 0, densityAltM: 0, imuOk: false, bmeOk: true });
check('metric: live temperature still in F', els.eTemp.textContent === '59.0°F', els.eTemp.textContent);
// ---- battery bar ----
const envBase = { tempC: 15, rhPct: 50, pressureHpa: 1013.25, windMps: 2, headingDeg: 0, pitchDeg: 0, densityAltM: 0, imuOk: false, bmeOk: true };
t.setEnv(Object.assign({}, envBase, { batteryPct: 82 }));
check('battery: shows percent and fills the bar', !els.batt.classList.contains('hide') && els.battTxt.textContent === '82%' && els.battFill.style.width === '82%', els.battTxt.textContent + ' ' + els.battFill.style.width);
check('battery: normal level is not flagged', !els.batt.classList.contains('low') && !els.batt.classList.contains('crit'));
t.setEnv(Object.assign({}, envBase, { batteryPct: 82, charging: true }));
check('battery: charging shows the bolt', !els.battBolt.classList.contains('hide'));
t.setEnv(Object.assign({}, envBase, { batteryPct: 82, charging: false }));
check('battery: not charging hides the bolt', els.battBolt.classList.contains('hide'));
t.setEnv(Object.assign({}, envBase, { batteryPct: null, charging: true }));
check('battery: no bolt when the percent is unknown', els.battBolt.classList.contains('hide'));
t.setEnv(Object.assign({}, envBase, { batteryPct: 25 }));
check('battery: 25 % is flagged low', els.batt.classList.contains('low') && !els.batt.classList.contains('crit'));
t.setEnv(Object.assign({}, envBase, { batteryPct: 10 }));
check('battery: 10 % is flagged critical', els.batt.classList.contains('crit') && !els.batt.classList.contains('low'));
t.setEnv(Object.assign({}, envBase, { batteryPct: 0 }));
check('battery: 0 % reads 0% with an empty bar', els.battTxt.textContent === '0%' && els.battFill.style.width === '0%', els.battTxt.textContent + ' ' + els.battFill.style.width);
t.setEnv(Object.assign({}, envBase, { batteryPct: null }));
check('battery: unknown shows -- and no flag', els.battTxt.textContent === '--' && els.battFill.style.width === '0%' && !els.batt.classList.contains('crit'));
t.hideBattery();
check('battery: hidden again on disconnect', els.batt.classList.contains('hide'));
check('metric: live pressure follows units (hPa)', /hPa/.test(els.ePress.textContent), els.ePress.textContent);

check('profile survives unit flip (still SI)', Math.abs(t.profiles().list.find(p => p.name === '6.5 CM').cfg.muzzleMps - 792.48) < 0.01);
t.selectProfile(id65);
check('profile restores in metric: velocity in fps, zero range in m', +els.mv.value === 2600 && +els.zero.value === 91, els.mv.value + ' ' + els.zero.value);

// delete
check('profile: delete', t.deleteProfile(id65) && t.profiles().list.length === 1 && t.profiles().active === null);
check('profile: delete unknown id is false', t.deleteProfile('nope') === false);

// building a card while not connected clears the table and asks to connect
t.requestCard();
check('card: not connected asks to connect', /Connect/.test(els.cardNote.textContent), els.cardNote.textContent);
check('card header uses current units', els.thRange.textContent === 'Range m', els.thRange.textContent);

// live card rows arriving over Bluetooth (not demo)
t.requestCard();
t.onSolution({ row: 3, ok: true, elevationMrad: 2, windageMrad: -0.5, tofS: 0.5, velocityMps: 700, rangeM: 300, windMps: 1 });
check('card: row 3 lands in slot 3', t.cardRows()[2] && t.cardRows()[2].rangeM === 300);
t.onSolution({ row: 99, ok: true, elevationMrad: 1, windageMrad: 0, tofS: 1, velocityMps: 1, rangeM: 1, windMps: 0 });
check('card: out-of-range row ignored', t.cardRows().length === 10);
t.onSolution({ row: 0, ok: true, elevationMrad: 9, windageMrad: 0, tofS: 1, velocityMps: 1, rangeM: 1, windMps: 0 });
check('live row 0 updates the main readout, not the card', els.outElev.textContent === '9.00' || els.outElev.textContent === '9' || /^9/.test(els.outElev.textContent), els.outElev.textContent);
t.onSolution({ row: 4, ok: false, elevationMrad: 0, windageMrad: 0, tofS: 0, velocityMps: 0, rangeM: 400, windMps: 0 });
check('card: failed row says no solution', /no solution/.test(els.cardBody.innerHTML));
// older metric saves are converted once to the imperial rifle fields
ctx.localStorage.setItem('bc.v1', JSON.stringify({ units: 'met', mv: '792', zt: '15', zp: '1013.2', sight: '3.8', bdia: '7.82', btw: '254', zero: '91' }));
t.load();
check('old metric save: rifle fields converted to imperial', +els.mv.value === 2598 && +els.zt.value === 59 && +els.zp.value === 29.92 && +els.sight.value === 1.5 && +els.bdia.value === 0.308 && +els.btw.value === 10 && +els.zero.value === 91,
  [els.mv.value, els.zt.value, els.zp.value, els.sight.value, els.bdia.value, els.btw.value, els.zero.value].join(' '));
ctx.localStorage.setItem('bc.v1', JSON.stringify({ units: 'met', rifleImp: 1, mv: '2600', zt: '59' }));
t.load();
check('new metric save: rifle fields not converted again', +els.mv.value === 2600 && +els.zt.value === 59, els.mv.value + ' ' + els.zt.value);
// ---- hit chance ----
{
  const secTagH = (html.match(/<section id="secHit"[^>]*>/) || [''])[0];
  check('hit: section exists and is Advanced only; the one-line result shows in Solution on both tabs', /adv-only/.test(secTagH) && /<section id="secSol">[\s\S]*id="outHit"[\s\S]*?<\/section>/.test(html));
  check('hit: hitprob.js is loaded by the page and cached for offline use', /<script src="hitprob.js"><\/script>/.test(html) && /'\.\/hitprob\.js'/.test(fs.readFileSync(path.join(__dirname, '../web/sw.js'), 'utf8')));
  els.units.value = 'imp'; t.convertFormUnits(); t.refreshLabels();
  els.mv.value = '2560'; els.zero.value = '100'; els.sight.value = '1.5';
  t.renderSolution(null);
  check('hit: no solution asks to connect', els.hitPct.textContent === '--' && /Connect/.test(els.hitNote.textContent), els.hitNote.textContent);
  const sol = { row: 0, ok: true, elevationMrad: 8, windageMrad: 1, tofS: 1.25, velocityMps: 400, rangeM: 731.5, windMps: 2 };
  els.hitShape.value = 'rect'; els.hitW.value = '18'; els.hitH.value = '18'; els.hitGroup.value = '1'; els.hitShots.value = '5';
  els.hitSd.value = '10'; els.hitWind.value = '2'; els.hitRange.value = '5'; els.hitBc.value = '0';
  t.renderSolution(sol);
  const pct = parseInt(els.hitPct.textContent, 10);
  check('hit: shows a percentage for an 800 yd shot', pct > 0 && pct < 100, els.hitPct.textContent);
  check('hit: the solution card repeats it', new RegExp('Hit chance ' + pct + '% \\(18 x 18 in target\\)').test(els.outHit.textContent), els.outHit.textContent);
  check('hit: names the biggest factor', /Biggest factor: wind call error/.test(els.hitNote.textContent), els.hitNote.textContent);
  const hi = t.hitInputs(sol);
  check('hit: inputs converted to SI (18 in, 10 fps, 2 mph, 5 yd)', Math.abs(hi.widthM - 0.4572) < 1e-6 && Math.abs(hi.mvSdMps - 3.048) < 1e-6 && Math.abs(hi.windErrMps - 0.89408) < 1e-6 && Math.abs(hi.rangeErrM - 4.572) < 1e-6);
  els.hitWind.value = '0'; t.renderSolution(sol);
  check('hit: perfect wind call raises the chance', parseInt(els.hitPct.textContent, 10) > pct, els.hitPct.textContent);
  const pNoBc = t.renderHit(sol).p;
  check('hit: BC effect worked out on the phone for this range', hi.mradPerBcPct < 0 && hi.bcErrPct === 0, hi.mradPerBcPct);
  els.hitBc.value = '5'; const wBc = t.renderHit(sol);
  check('hit: 5% BC uncertainty lowers the chance and adds vertical spread', wBc.p < pNoBc && wBc.parts.bc > 0, wBc.p + ' vs ' + pNoBc);
  els.hitBc.value = '-1'; t.renderHit(sol);
  check('hit: negative BC uncertainty is an error', els.hitPct.textContent === '--', els.hitNote.textContent);
  els.hitBc.value = '2';
  check('hit: note gives the target size in MOA and MIL', /The target is [\d.]+ x [\d.]+ MOA \([\d.]+ x [\d.]+ MIL\) at this range/.test(t.renderHit(sol) && els.hitNote.textContent), els.hitNote.textContent);
  // A 0.5 MOA plate with a 2 MOA rifle: hard whatever the hold.
  els.hitShape.value = 'circle'; els.hitGroup.value = '2'; els.hitW.value = String(+(0.5 * 1.047 * 731.5 / 0.9144 / 100).toFixed(2));
  t.renderHit(sol);
  check('hit: 2 MOA rifle on a 0.5 MOA plate is under 15%', parseInt(els.hitPct.textContent, 10) < 15, els.hitPct.textContent);
  check('hit: and the note says the target is smaller than the spread', /0\.5\d? MOA .*smaller than your likely spread/.test(els.hitNote.textContent), els.hitNote.textContent);
  check('hit: one target only, no range chart', !/hitChart|hitTable|hit chance by range/i.test(html));
  els.hitShape.value = 'rect'; els.hitGroup.value = '1'; els.hitW.value = '18';
  els.hitShape.value = 'circle'; t.renderHit(sol);
  check('hit: round plate hides the height box and says Diameter', els.hitHBox.classList.contains('hide') && /Diameter \(in\)/.test(els.lHitW.textContent));
  els.hitW.value = ''; t.renderHit(sol);
  check('hit: empty size is an error', els.hitPct.textContent === '--' && /target size/.test(els.hitNote.textContent), els.hitNote.textContent);
  els.hitW.value = '18';
  els.units.value = 'met'; t.convertFormUnits(); t.refreshLabels();
  check('hit: switching to metric converts target size to cm and range error to m', +els.hitW.value === 45.7 && +els.hitRange.value === 5 && /cm/.test(els.lHitW.textContent), els.hitW.value + ' ' + els.hitRange.value);
  check('hit: settings are saved with the form', /'hitShape','hitW','hitH','hitGroup','hitShots','hitSd','hitWind','hitRange','hitBc'/.test(inline));
}
// ---- load development tab: shot log, truing, chronograph ----
{
  const tag = (id) => (html.match(new RegExp('<section id="' + id + '"[^>]*>')) || [''])[0];
  check('load tab: has its own tab button', /<button id="tabLoad">Load development<\/button>/.test(html));
  check('load tab: shot log lives only on the Load development tab, with the profile picker', /load-only/.test(tag('secLog')) && /load-tab/.test(tag('secLog')) && /load-tab/.test(tag('secProf')) && !/load-tab/.test(tag('secSol')));
  t.setTab('load');
  check('load tab: selecting it marks the app and remembers it', els.app.classList.contains('load') && !els.app.classList.contains('simple') && els.tabLoad.classList.contains('on') && ctx.localStorage.getItem('bc.tab') === 'load');
  t.setTab('adv');
  check('load tab: leaving it clears the mark', !els.app.classList.contains('load') && els.tabAdv.classList.contains('on') && !els.tabLoad.classList.contains('on'));
  t.setTab('simple');

  els.units.value = 'imp'; t.convertFormUnits(); els.angUnit.value = 'MIL'; t.refreshLabels();
  els.model.value = 'G7'; els.bc.value = '0.176'; els.mv.value = '2560'; els.zero.value = '100'; els.sight.value = '1.5';
  els.zt.value = '59'; els.zp.value = '29.92'; els.zh.value = '50';
  els.bmass.value = ''; els.bdia.value = ''; els.blen.value = ''; els.btw.value = '';
  t.setConn(null, {}); t.setLastEnv(null);
  t.renderSolution({ row: 0, ok: true, elevationMrad: 4.6, windageMrad: 0.3, tofS: 0.62, velocityMps: 600, rangeM: 365.76, windMps: 1 });
  els.range.value = '400';
  els.logRange.value = ''; els.logUp.value = ''; els.logRight.value = ''; els.logImpUp.value = '-0.2'; els.logImpRight.value = '0.1';
  const before = t.shots().length;
  const s = t.logShot();
  check('log: blank range and hold are filled from the Shot range and the solution', s && Math.abs(s.rangeM - 365.76) < 0.01 && Math.abs(s.holdUpMrad - 4.6) < 1e-9, s && s.rangeM);
  check('log: impact stored in mrad, and come-up that hit = 4.6 + 0.2', s && Math.abs(s.impactUpMrad + 0.2) < 1e-9 && Math.abs(window_obs(s) - 4.8) < 1e-9);
  function window_obs(x) { return x.holdUpMrad - x.impactUpMrad; }
  check('log: not connected, so it says the zero conditions were used', /zero conditions/.test(els.logNoteOut.textContent) && s.atmSource === 'zero', els.logNoteOut.textContent);
  check('log: saved to storage and shown in the table', t.shots().length === before + 1 && JSON.parse(ctx.localStorage.getItem('bc.shots.v1')).length === before + 1 && /4\.80/.test(els.logBody.innerHTML));
  check('log: phone solver prediction stored', s.predUpMrad !== null && s.predUpMrad > 2 && s.predUpMrad < 5, s.predUpMrad);
  check('log: impact boxes reset to 0 after logging', els.logImpUp.value === '0' && els.logImpRight.value === '0');
  t.setConn({ gatt: { connected: true } }, { config: {} });
  t.setLastEnv({ tempC: 30, rhPct: 20, pressureHpa: 950, bmeOk: true });
  els.logRange.value = '700'; els.logUp.value = '11'; els.logImpUp.value = '0';
  const s2 = t.logShot();
  check('log: connected, the board\'s conditions are logged', s2 && s2.atmSource === 'solstice' && s2.atm.tempC === 30 && s2.atm.pressureHpa === 950);
  t.setConn(null, {});
  t.toggleShot(s2.id);
  check('log: a shot can be left out of truing', t.shots().find(x => x.id === s2.id).use === false && /class="off"/.test(els.logBody.innerHTML));
  t.toggleShot(s2.id);
  els.logRange.value = '50'; els.logUp.value = 'x';
  check('log: bad input is refused with a message', t.logShot() === null && els.logNoteOut.className.includes('bad'), els.logNoteOut.textContent);

  const r = t.trueLoad();
  check('truing: runs from the page and offers Apply', r && r.ok && !els.btnTrueApply.classList.contains('hide') && /Muzzle velocity \d+ fps \(was 2560\)/.test(els.trueNote.textContent), els.trueNote.textContent);
  const mvNew = Math.round(r.muzzleMps / 0.3048);
  t.applyTrued();
  check('truing: Apply writes the new MV and BC into the form', +els.mv.value === mvNew && +els.bc.value === +r.bc.toFixed(3), els.mv.value + ' ' + els.bc.value);
  check('csv: export gives a header and the logged shots', /^time,profile/.test(t.exportCsv()) && t.exportCsv().trim().split('\n').length === t.shots().filter(x => x.profile === '').length + 1);

  // powder and charge go with each logged shot
  els.ldPowder.value = 'Varget'; els.ldCharge.value = '24.5';
  els.logRange.value = '300'; els.logUp.value = '2'; const s3 = t.logShot();
  check('log: powder and charge saved with the shot', s3 && s3.powder === 'Varget' && s3.chargeGr === 24.5);
  check('load: powder, charge and notes are saved with the form', /'ldPowder','ldCharge','ldNotes'/.test(inline));

  // group from a photo (taps given straight to the handler; no real image needed for the maths)
  check('group: Take photo opens the rear camera, Choose photo the gallery', /<input id="grpCam" type="file" accept="image\/\*" capture="environment"/.test(html) && /<input id="grpPick" type="file" accept="image\/\*" class="hide">/.test(html));
  els.grpDist.value = '100'; els.grpScaleIn.value = '1';
  t.setGrpMode('scale'); t.grpTap({ x: 0, y: 0 }); t.grpTap({ x: 200, y: 0 });
  check('group: two scale taps set the scale and go back to marking holes', t.grpState().scale.length === 2 && t.grpState().mode === 'holes' && els.grpModeHoles.classList.contains('on'));
  [[100, 100], [300, 100], [100, 300], [300, 300]].forEach(([x, y]) => t.grpTap({ x, y }));
  check('group: four holes 1 in apart measure 1.41 in = 1.35 MOA at 100 yd', /4 shots: group 1\.41 in = 1\.35 MOA/.test(els.grpOut.textContent), els.grpOut.textContent);
  t.setGrpMode('aim'); t.grpTap({ x: 200, y: 400 });
  check('group: aim point gives the offset (1 in high, centred)', /Centre 1\.00 in high, 0\.00 in right/.test(els.grpOut.textContent), els.grpOut.textContent);
  t.grpTap({ x: 302, y: 298 });
  check('group: tapping a marked hole removes it', t.grpState().pts.length === 3);
  t.grpTap({ x: 300, y: 300 }); t.grpUndo();
  check('group: Undo removes the last hole', t.grpState().pts.length === 3);
  t.grpTap({ x: 300, y: 300 });
  const tg = t.grpTarget({ x: 305, y: 296 });
  check('group: pressing near a circle grabs that hole', tg && tg.kind === 'hole' && t.grpState().pts[tg.i].x === 300);
  t.grpMoveTo(tg, { x: 320, y: 300 });
  check('group: dragging moves the hole and re-measures', t.grpState().pts[tg.i].x === 320 && /4 shots: group 1\.49 in/.test(els.grpOut.textContent), els.grpOut.textContent);
  t.grpMoveTo(tg, { x: 300, y: 300 });
  const ts = t.grpTarget({ x: 2, y: 3 });
  check('group: scale points and the aim point can be grabbed too', ts && ts.kind === 'scale' && t.grpTarget({ x: 201, y: 401 }).kind === 'aim');
  check('group: Use in hit chance copies the group size and shot count', t.grpUseInHit() && +els.hitGroup.value === 1.35 && els.hitShots.value === '5', els.hitGroup.value + ' ' + els.hitShots.value);
  check('group: Remove photo button exists', /id="btnGrpRemove"/.test(html));
  els.chronoIn.value = '2551 2560 2548 2565 2556';
  const st = t.renderChrono();
  check('chrono: shows average, ES, SD', st.n === 5 && /average 2556 fps, ES 17, SD 6\.8/.test(els.chronoOut.textContent), els.chronoOut.textContent);
  els.btnChronoSd.onclick();
  check('chrono: Use SD puts it in hit chance', +els.hitSd.value === 6.8, els.hitSd.value);
  const lt = t.saveLoadTest();
  check('load test: saves powder, charge, chronograph and group together', lt && lt.powder === 'Varget' && lt.chargeGr === 24.5 && Math.round(lt.avgFps) === 2556 && Math.abs(lt.groupMoa - 1.35) < 0.01);
  check('load test: shown in the comparison table', /Varget<\/td><td>24\.5<\/td><td>2556<\/td><td>6\.8<\/td><td>17<\/td><td class="up">1\.35/.test(els.testBody.innerHTML), els.testBody.innerHTML);
  els.ldPowder.value = ''; els.ldCharge.value = '';
  t.grpRemovePhoto();
  check('group: Remove photo clears the image, marks and scale', t.grpState().img === null && t.grpState().pts.length === 0 && t.grpState().scale.length === 0 && els.grpCanvas.classList.contains('hide'));
  check('wording: the app calls the device Solstice, not "the board"', !/the board|Board battery|Board asleep/.test(html.replace(/\/\/.*$/gm, '')), (html.replace(/\/\/.*$/gm, '').match(/.{30}(the board|Board battery|Board asleep).{10}/) || [''])[0]);
  check('load test: needs a powder or charge', t.saveLoadTest() === null && /powder/.test(els.testNote.textContent));
}
// ---- bullet section (Load development) ----
{
  const tag = (id) => (html.match(new RegExp('<section id="' + id + '"[^>]*>')) || [''])[0];
  check('bullet: own section on the Load development tab', /load-only/.test(tag('secBullet')) && /load-tab/.test(tag('secBullet')));
  els.blName.value = 'Barnes Match Burner'; els.blWeight.value = '77'; els.blDia.value = '0.224'; els.blType.value = 'Match hollow point (HPBT/OTM)'; els.blBc.value = '0.176'; els.blModel.value = 'G7';
  const b = t.renderBullet();
  check('bullet: sectional density worked out (77 gr .224 = 0.219)', els.blSd.value === '0.219' && b.sd === 0.219, els.blSd.value);
  check('bullet: fields saved with the form', /'blName','blWeight','blDia','blType','blBc','blModel'/.test(inline));
  els.bc.value = '0.3'; els.model.value = 'G1'; els.bmass.value = ''; els.bdia.value = ''; els.blen.value = ''; els.btw.value = '';
  check('bullet: Use in profile copies BC and model, and leaves the spin-drift fields alone while length and twist are empty', t.bulletToProfile() && +els.bc.value === 0.176 && els.model.value === 'G7' && els.bmass.value === '' && /length and twist/.test(els.blNote.textContent));
  els.blen.value = '0.99'; els.btw.value = '8';
  check('bullet: with length and twist filled in, weight and diameter are copied too', t.bulletToProfile() && +els.bmass.value === 77 && +els.bdia.value === 0.224);
  els.blBc.value = ''; t.bulletFromProfile();
  check('bullet: Copy from profile fills the BC back in', +els.blBc.value === 0.176 && els.blModel.value === 'G7');
  els.logRange.value = '400'; els.logUp.value = '3'; const sb = t.logShot();
  check('bullet: saved with each logged shot', sb && sb.bullet && sb.bullet.weightGr === 77 && sb.bullet.sd === 0.219 && sb.bullet.name === 'Barnes Match Burner');
  els.ldPowder.value = 'Varget'; els.ldCharge.value = '24.0'; els.chronoIn.value = '2500 2510';
  const lt = t.saveLoadTest();
  check('bullet: saved with the load test and shown under the powder', lt && lt.bullet.bc === 0.176 && /Barnes Match Burner 77 gr Match hollow point/.test(els.testBody.innerHTML));
  els.ldPowder.value = ''; els.ldCharge.value = ''; els.chronoIn.value = '';
}
// ---- stage tab ----
{
  const tag = (id) => (html.match(new RegExp('<section id="' + id + '"[^>]*>')) || [''])[0];
  check('stage tab: has its own tab button, and shows the planner, profile and live conditions', /<button id="tabStage">Stage<\/button>/.test(html) && /stage-tab/.test(tag('secStage')) && /stage-tab/.test(tag('secProf')) && /stage-tab/.test(tag('secLive')) && !/stage-tab/.test(tag('secSol')));
  t.setTab('stage');
  check('stage tab: selecting it marks the app', els.app.classList.contains('stage') && els.tabStage.classList.contains('on') && !els.tabLoad.classList.contains('on'));
  els.units.value = 'imp'; t.convertFormUnits(); els.angUnit.value = 'MIL'; t.refreshLabels();
  els.mv.value = '2510'; els.bc.value = '0.176'; els.zero.value = '100'; els.windSrc.value = 'manual'; els.windManual.value = '5'; els.windDir.value = '90';
  els.hitGroup.value = '1'; els.hitSd.value = '10'; els.hitWind.value = '2'; els.hitRange.value = '5';
  els.stgOrder.value = 'entered'; els.stgLow.value = '50';
  const add = (name, yd, dia) => { els.tgName.value = name; els.tgRange.value = String(yd); els.tgShape.value = 'circle'; els.tgW.value = String(dia); els.tgShots.value = '2'; return t.addTarget(); };
  add('Far', 700, 12); add('Near', 300, 8); add('Tiny', 800, 4);
  check('stage: three targets added and solved', t.stage().targets.length === 3 && (els.stgList.innerHTML.match(/class="tgt/g) || []).length === 3);
  check('stage: each target shows UP, wind and a hit chance', (els.stgList.innerHTML.match(/UP \d+\.\d\d/g) || []).length === 3 && (els.stgList.innerHTML.match(/\d+%/g) || []).length === 3);
  check('stage: not connected says it uses the zero conditions, with the Shot section wind', /zero conditions/.test(els.stgCond.textContent) && /Wind 5\.0 mph from 90/.test(els.stgCond.textContent), els.stgCond.textContent);
  check('stage: 4 in plate at 800 yd is flagged low', /class="tgt low"[\s\S]*Tiny/.test(els.stgList.innerHTML) && /low-odds/.test(els.stgSum.textContent));
  els.stgOrder.value = 'near'; t.renderStage();
  check('stage: near to far puts Near first, with dial changes', /1\. Near/.test(els.stgList.innerHTML) && /dial \+/.test(els.stgList.innerHTML) && !/data-act="up"/.test(els.stgList.innerHTML));
  els.stgOrder.value = 'entered'; t.renderStage();
  const ids = t.stage().targets.map(x => x.id);
  t.stageAction('up', ids[1]);
  check('stage: arrows move a target in the entered order', t.stage().targets[0].name === 'Near');
  t.stageAction('H', ids[0]); t.stageAction('M', ids[0]);
  check('stage: hit and miss are recorded and summed against the plan', /1\/2 hits \(HM\)/.test(els.stgList.innerHTML) && /So far: 1 hit from 2 shots/.test(els.stgSum.textContent), els.stgSum.textContent);
  check('stage: an unsaved stage is kept as a draft', JSON.parse(ctx.localStorage.getItem('bc.stages.v1')).draft.targets.length === 3);
  check('stage: save by name', t.saveStage('Barricade 3') && t.stages().list.length === 1 && t.stage().name === 'Barricade 3' && !t.stages().draft);
  t.stageAction('del', ids[2]);
  check('stage: remove a target, saved list follows', t.stage().targets.length === 2 && t.stages().list[0].targets.length === 2);
  t.selectStage('');
  check('stage: New/blank stage', t.stage().targets.length === 0);
  t.selectStage(t.stages().list[0].id);
  check('stage: pick the saved stage again', t.stage().targets.length === 2 && t.stage().name === 'Barricade 3');
  els.tgRange.value = '5'; els.tgName.value = 'bad';
  check('stage: bad range refused', t.addTarget() === null && /Range/.test(els.tgNote.textContent));
  check('stage: delete', t.deleteStage() && t.stages().list.length === 0);
  t.setTab('simple');
}
// ---- sleep button ----
(async () => {
  check('sleep: button is at the very bottom of the page', html.lastIndexOf('id="secSleep"') > html.lastIndexOf('id="secRifle"') && /<section id="secSleep">[\s\S]*<\/section>\s*<\/main>/.test(html));
  check('sleep: not in an Advanced-only section', !/id="secSleep"[^>]*adv-only/.test(html));
  t.setConn(null, {});
  els.btnSleep.textContent = 'Sleep mode';
  t.onSleepClick();
  check('sleep: not connected asks to connect', /Connect/.test(els.sleepNote.textContent), els.sleepNote.textContent);
  let wrote = null;
  const fake = { gatt: { connected: true, disconnect() { this.connected = false; } } };
  t.setConn(fake, { config: {}, request: { writeValueWithResponse: async (b) => { if (!wrote) wrote = b; } } });
  t.onSleepClick();
  check('sleep: first tap only arms it, nothing sent', wrote === null && /again/.test(els.btnSleep.textContent), els.btnSleep.textContent);
  t.onSleepClick();
  await new Promise(r => setTimeout(r, 20));
  check('sleep: second tap sends a 28-byte request with only bit6 set', wrote && wrote.length === 28 && wrote[0] === 0x40, wrote && wrote[0]);
  check('sleep: status tells how to wake it', /BOOT/.test(els.status.textContent) && /BOOT/.test(els.sleepNote.textContent), els.status.textContent);
  check('sleep: button label reset after sending', els.btnSleep.textContent === 'Sleep mode', els.btnSleep.textContent);
  check('sleep: battery readout hidden', els.batt.classList.contains('hide'));
  console.log(fails ? `\nFAILED (${fails})` : '\nALL PASSED');
  process.exit(fails ? 1 : 0);
})();
