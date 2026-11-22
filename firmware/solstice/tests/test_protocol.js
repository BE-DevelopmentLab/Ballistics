// Cross-check C++ (core/protocol.h) against JS (web/protocol.js).
//   node test_protocol.js   (build proto_tool first; see run_tests.sh)
'use strict';
const { execFileSync } = require('child_process');
const path = require('path');
const Proto = require('../web/protocol.js');

const tool = path.join(__dirname, 'proto_tool');
const run = (...args) => execFileSync(tool, args, { encoding: 'utf8' }).trim();

let fails = 0;
function check(name, ok, detail) {
  console.log(`[${ok ? 'PASS' : 'FAIL'}] ${name}${detail ? '  ' + detail : ''}`);
  if (!ok) fails++;
}
const close = (a, b, tol) => Math.abs(a - b) <= tol;

// JS -> C++: config
const cfg = { model: 'G7', bc: 0.243, muzzleMps: 792.5, sightHeightM: 0.0381,
              zeroRangeM: 91.44, zeroTempC: 15, zeroPressureHpa: 1013.25, zeroRhPct: 50,
              massGr: 175, diameterM: 0.00782, lengthM: 0.0315, twistM: 0.254 };
{
  const out = run('config', Proto.toHex(Proto.encodeConfig(cfg))).split(' ').map(Number);
  const want = [1, cfg.bc, cfg.muzzleMps, cfg.sightHeightM, cfg.zeroRangeM, cfg.zeroTempC,
                cfg.zeroPressureHpa, cfg.zeroRhPct, cfg.massGr, cfg.diameterM, cfg.lengthM, cfg.twistM];
  check('config JS->C++ round trip', want.every((w, i) => close(out[i], w, 1e-4 * Math.max(1, Math.abs(w)))),
        out.join(' '));
}

// config without bullet data (spin drift off) and with a left-hand twist
{
  const { massGr, diameterM, lengthM, twistM, ...plain } = cfg;
  let out = run('config', Proto.toHex(Proto.encodeConfig(plain))).split(' ').map(Number);
  check('config without bullet data accepted (spin fields 0)', out.slice(8).every(x => x === 0), out.join(' '));
  out = run('config', Proto.toHex(Proto.encodeConfig({ ...cfg, twistM: -0.254 }))).split(' ').map(Number);
  check('left-hand twist keeps its sign', out[11] < 0, out.join(' '));
  for (const [name, bad] of [['mass but no twist', { ...cfg, twistM: 0 }], ['absurd twist', { ...cfg, twistM: 5 }],
                              ['length shorter than 1.5 cal', { ...cfg, lengthM: 0.005 }], ['negative mass', { ...cfg, massGr: -5 }]]) {
    let r; try { r = run('config', Proto.toHex(Proto.encodeConfig(bad))); } catch (e) { r = 'REJECT'; }
    check(`config rejected: ${name}`, r === 'REJECT', r);
  }
}

// JS -> C++: request (flag combinations incl. Coriolis / spin drift / IMU azimuth)
for (const [sensorWind, imuSlope, coriolis, spinDrift, imuAzimuth, aeroJump] of
     [[true, true, false, false, false, false], [false, false, false, false, false, false], [false, true, true, true, true, false], [true, false, true, false, false, true], [false, false, false, true, false, true]]) {
  const req = { sensorWind, imuSlope, coriolis, spinDrift, imuAzimuth, aeroJump, rangeM: 640.1, windClockDeg: 90,
                windSpeedMps: 4.4704, slopeDeg: -12, latitudeDeg: -33.5, azimuthDeg: 217.5 };
  const out = run('request', Proto.toHex(Proto.encodeRequest(req))).split(' ').map(Number);
  const flags = (sensorWind ? 1 : 0) | (imuSlope ? 2 : 0) | (coriolis ? 4 : 0) | (spinDrift ? 8 : 0) | (imuAzimuth ? 16 : 0) | (aeroJump ? 32 : 0);
  const want = [flags, req.rangeM, req.windClockDeg, req.windSpeedMps, req.slopeDeg, req.latitudeDeg, req.azimuthDeg];
  check(`request JS->C++ flags=${flags}`, want.every((w, i) => close(out[i], w, 1e-4 * Math.max(1, Math.abs(w)))),
        out.join(' '));
}

// JS -> C++: range card
{
  const card = { sensorWind: true, imuSlope: false, coriolis: true, spinDrift: true, imuAzimuth: false, count: 10,
                 startM: 91.44, stepM: 91.44, windClockDeg: 90, windSpeedMps: 4.4704, slopeDeg: 5,
                 latitudeDeg: 44.2, azimuthDeg: 301 };
  const out = run('card', Proto.toHex(Proto.encodeCard(card))).split(' ').map(Number);
  const want = [1 | 4 | 8, 10, card.startM, card.stepM, card.windClockDeg, card.windSpeedMps, card.slopeDeg, card.latitudeDeg, card.azimuthDeg];
  check('range card JS->C++ round trip', want.every((w, i) => close(out[i], w, 1e-4 * Math.max(1, Math.abs(w)))), out.join(' '));
  for (const [name, bad] of [
    ['zero rows', { ...card, count: 0 }], ['25 rows', { ...card, count: 25 }],
    ['past 2000 m', { ...card, count: 24, startM: 500, stepM: 100 }], ['NaN start', { ...card, startM: NaN }],
    ['zero step', { ...card, stepM: 0 }], ['latitude 95', { ...card, latitudeDeg: 95 }]]) {
    let r; try { r = run('card', Proto.toHex(Proto.encodeCard(bad))); } catch (e) { r = 'REJECT'; }
    check(`range card rejected: ${name}`, r === 'REJECT', r);
  }
  // exactly 24 rows ending exactly at the 2000 m limit must still be accepted
  const edge = { ...card, count: 24, startM: 10, stepM: (2000 - 10) / 23 };
  let r; try { r = run('card', Proto.toHex(Proto.encodeCard(edge))); } catch (e) { r = 'REJECT'; }
  check('range card accepted at the 24-row / 2000 m limit', r !== 'REJECT', r);
}

// C++ -> JS: solution
{
  const s = Proto.decodeSolution(Proto.fromHex(run('solution')));
  const ok = s.ok && s.row === 7 && close(s.elevationMrad, 6.15, 0.006) && close(s.windageMrad, -1.91, 0.006) &&
    close(s.tofS, 1.066, 0.001) && close(s.velocityMps, 450.2, 0.06) && close(s.dropM, -3.21, 0.006) &&
    close(s.rangeM, 640.1, 0.06) && close(s.windMps, 4.47, 0.006) && s.slopeDeg === -12 && close(s.spinDriftMrad, -0.37, 0.006);
  check('solution C++->JS (incl. negative values)', ok, JSON.stringify(s));
}

// C++ -> JS: environment
{
  const e = Proto.decodeEnv(Proto.fromHex(run('env')));
  const ok = close(e.tempC, -3.5, 0.006) && close(e.rhPct, 62.25, 0.006) && close(e.pressureHpa, 1002.7, 0.06) &&
    close(e.windMps, 2.34, 0.006) && close(e.headingDeg, 271.4, 0.06) && close(e.pitchDeg, -4.6, 0.06) &&
    e.densityAltM === 412 && e.bmeOk && e.imuOk && e.batteryPct === 87;
  check('env C++->JS (negative temp and pitch)', ok, JSON.stringify(e));
}

{
  const raw = Proto.fromHex(run('env'));
  check('env: battery flag bit set by the board', (raw[14] & 4) === 4 && raw[15] === 87);
  check('env: charging flag clear by default', Proto.decodeEnv(raw).charging === false);
  raw[14] |= 8; check('env: bit3 decodes as charging', Proto.decodeEnv(raw).charging === true); raw[14] &= ~8;
  raw[14] &= ~4; raw[15] = 0;
  check('env: older firmware (flag clear, byte 15 = 0) shows no battery, not 0 %', Proto.decodeEnv(raw).batteryPct === null);
  raw[14] |= 4; check('env: battery 0 with the flag set is a real 0 %', Proto.decodeEnv(raw).batteryPct === 0);
  raw[15] = 255; check('env: out-of-range value is treated as unknown', Proto.decodeEnv(raw).batteryPct === null);
}

{
  const sl = Proto.encodeRequest({ sleep: true, rangeM: 100, windClockDeg: 0, windSpeedMps: 0, slopeDeg: 0 });
  check('request: sleep sets flag bit6 (0x40) and nothing else', sl.length === 28 && sl[0] === 0x40, sl[0]);
  check('request: ordinary request leaves bit6 clear', (Proto.encodeRequest({ aeroJump: true, rangeM: 100, windClockDeg: 0, windSpeedMps: 0, slopeDeg: 0 })[0] & 0x40) === 0);
}

// Bad input must be rejected, not crash the solver.
{
  const bad = Proto.encodeConfig({ ...cfg, bc: -1 });
  let out; try { out = run('config', Proto.toHex(bad)); } catch (e) { out = 'REJECT'; }
  check('config with negative BC is rejected', out === 'REJECT', out);
  const badLat = Proto.encodeRequest({ sensorWind: true, imuSlope: true, rangeM: 500, windClockDeg: 0, windSpeedMps: 0, slopeDeg: 0, latitudeDeg: 120 });
  try { out = run('request', Proto.toHex(badLat)); } catch (e) { out = 'REJECT'; }
  check('request with latitude 120 is rejected', out === 'REJECT', out);
  const badReq = Proto.encodeRequest({ sensorWind: true, imuSlope: true, rangeM: NaN,
                                       windClockDeg: 0, windSpeedMps: 0, slopeDeg: 0 });
  try { out = run('request', Proto.toHex(badReq)); } catch (e) { out = 'REJECT'; }
  check('request with NaN range is rejected', out === 'REJECT', out);
  try { out = run('config', Proto.toHex(Proto.encodeConfig(cfg)).slice(0, 40)); } catch (e) { out = 'REJECT'; }
  check('truncated config packet is rejected', out === 'REJECT', out);
}

console.log(fails ? `\nFAILED (${fails})` : '\nALL PASSED');
process.exit(fails ? 1 : 0);
