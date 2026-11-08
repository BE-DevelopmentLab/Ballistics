// JS side of the BLE protocol. Must match core/protocol.h byte for byte.
// Works in the browser (global `Proto`) and in node (module.exports).
(function (root) {
  'use strict';

  const UUID = {
    service:  'b4a10000-1d5b-4c0e-9a3f-5e7d2c0a1b00',
    config:   'b4a10001-1d5b-4c0e-9a3f-5e7d2c0a1b00',
    request:  'b4a10002-1d5b-4c0e-9a3f-5e7d2c0a1b00',
    solution: 'b4a10003-1d5b-4c0e-9a3f-5e7d2c0a1b00',
    env:      'b4a10004-1d5b-4c0e-9a3f-5e7d2c0a1b00',
  };
  const VERSION = 1;
  const FLAG_SENSOR_WIND = 0x01;
  const FLAG_IMU_SLOPE = 0x02;

  function view(bytes) {
    const u8 = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
    return new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  }

  // cfg: {model:'G1'|'G7', bc, muzzleMps, sightHeightM, zeroRangeM,
  //       zeroTempC, zeroPressureHpa, zeroRhPct}
  function encodeConfig(cfg) {
    const b = new Uint8Array(32), v = view(b);
    v.setUint8(0, VERSION);
    v.setUint8(1, cfg.model === 'G7' ? 1 : 0);
    v.setFloat32(4, cfg.bc, true);
    v.setFloat32(8, cfg.muzzleMps, true);
    v.setFloat32(12, cfg.sightHeightM, true);
    v.setFloat32(16, cfg.zeroRangeM, true);
    v.setFloat32(20, cfg.zeroTempC, true);
    v.setFloat32(24, cfg.zeroPressureHpa, true);
    v.setFloat32(28, cfg.zeroRhPct, true);
    return b;
  }

  // req: {sensorWind:bool, imuSlope:bool, rangeM, windClockDeg, windSpeedMps, slopeDeg}
  function encodeRequest(req) {
    const b = new Uint8Array(20), v = view(b);
    v.setUint8(0, (req.sensorWind ? FLAG_SENSOR_WIND : 0) | (req.imuSlope ? FLAG_IMU_SLOPE : 0));
    v.setFloat32(4, req.rangeM, true);
    v.setFloat32(8, req.windClockDeg, true);
    v.setFloat32(12, req.windSpeedMps, true);
    v.setFloat32(16, req.slopeDeg, true);
    return b;
  }

  function decodeSolution(bytes) {
    const v = view(bytes);
    if (v.byteLength !== 18) throw new Error('solution packet must be 18 bytes, got ' + v.byteLength);
    return {
      ok: v.getUint8(0) === 1,
      elevationMrad: v.getInt16(2, true) / 100,
      windageMrad: v.getInt16(4, true) / 100,
      tofS: v.getUint16(6, true) / 1000,
      velocityMps: v.getUint16(8, true) / 10,
      dropM: v.getInt16(10, true) / 100,
      rangeM: v.getUint16(12, true) / 10,
      windMps: v.getUint16(14, true) / 100,
      slopeDeg: v.getInt8(16),
    };
  }

  function decodeEnv(bytes) {
    const v = view(bytes);
    if (v.byteLength !== 16) throw new Error('env packet must be 16 bytes, got ' + v.byteLength);
    const flags = v.getUint8(14);
    return {
      tempC: v.getInt16(0, true) / 100,
      rhPct: v.getUint16(2, true) / 100,
      pressureHpa: v.getUint16(4, true) / 10,
      windMps: v.getUint16(6, true) / 100,
      headingDeg: v.getUint16(8, true) / 10,
      pitchDeg: v.getInt16(10, true) / 10,
      densityAltM: v.getInt16(12, true),
      bmeOk: (flags & 1) !== 0,
      imuOk: (flags & 2) !== 0,
    };
  }

  const toHex = (b) => Array.from(b, (x) => x.toString(16).padStart(2, '0')).join('');
  const fromHex = (h) => Uint8Array.from(h.match(/../g) || [], (x) => parseInt(x, 16));

  const Proto = { UUID, encodeConfig, encodeRequest, decodeSolution, decodeEnv, toHex, fromHex };
  if (typeof module !== 'undefined' && module.exports) module.exports = Proto;
  else root.Proto = Proto;
})(typeof self !== 'undefined' ? self : this);
