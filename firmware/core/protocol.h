// BLE packet layouts shared by the ESP32 firmware and the phone page
// (web/protocol.js must match this file byte for byte).
//
// All multi-byte values are little-endian. Notify payloads are kept to
// 20 bytes or less so they work even if the MTU stays at the BLE default.
#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include "ballistics.h"

namespace protocol {

constexpr uint8_t kVersion = 2;

// 128-bit UUIDs (custom, not Bluetooth SIG assigned).
#define BC_UUID_SERVICE  "b4a10000-1d5b-4c0e-9a3f-5e7d2c0a1b00"
#define BC_UUID_CONFIG   "b4a10001-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // write
#define BC_UUID_REQUEST  "b4a10002-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // write
#define BC_UUID_SOLUTION "b4a10003-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // notify
#define BC_UUID_ENV      "b4a10004-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // notify
#define BC_UUID_CARD     "b4a10005-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // write (range card request)

// ---- little-endian helpers ----------------------------------------------
inline void put_u8(uint8_t* p, uint8_t v) { p[0] = v; }
inline void put_u16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
inline void put_i16(uint8_t* p, int16_t v) { put_u16(p, static_cast<uint16_t>(v)); }
inline uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline int16_t get_i16(const uint8_t* p) { return static_cast<int16_t>(get_u16(p)); }
inline void put_f32(uint8_t* p, float v) {
    uint32_t u; std::memcpy(&u, &v, 4);
    p[0] = u & 0xFF; p[1] = (u >> 8) & 0xFF; p[2] = (u >> 16) & 0xFF; p[3] = (u >> 24) & 0xFF;
}
inline float get_f32(const uint8_t* p) {
    uint32_t u = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                 (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    float v; std::memcpy(&v, &u, 4); return v;
}
inline int16_t clamp_i16(double v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(std::lround(v));
}
inline uint16_t clamp_u16(double v) {
    if (v > 65535.0) return 65535;
    if (v < 0.0) return 0;
    return static_cast<uint16_t>(std::lround(v));
}

// ---- CONFIG (phone -> board, 48 bytes) ------------------------------------
// 0 u8 version | 1 u8 model (0=G1,1=G7) | 2..3 reserved
// 4 f32 bc (lb/in^2) | 8 f32 muzzle velocity m/s | 12 f32 sight height m
// 16 f32 zero range m | 20 f32 zero temp C | 24 f32 zero pressure hPa | 28 f32 zero RH %
// 32 f32 bullet mass grains | 36 f32 diameter m | 40 f32 length m
// 44 f32 twist m per turn (+ right-hand, - left-hand). All four 0 = spin drift data not given.
constexpr size_t kConfigSize = 48;

struct Config {
    uint8_t version, model;
    float bc, muzzle_mps, sight_height_m, zero_range_m;
    float zero_temp_c, zero_pressure_hpa, zero_rh_pct;
    float mass_gr, diameter_m, length_m, twist_m;
    bool has_spin_data() const { return mass_gr > 0 && diameter_m > 0 && length_m > 0 && twist_m != 0; }
};

inline bool decode_config(const uint8_t* d, size_t n, Config& c) {
    if (n != kConfigSize) return false;
    c.version = d[0]; c.model = d[1];
    c.bc = get_f32(d + 4);          c.muzzle_mps = get_f32(d + 8);
    c.sight_height_m = get_f32(d + 12); c.zero_range_m = get_f32(d + 16);
    c.zero_temp_c = get_f32(d + 20);    c.zero_pressure_hpa = get_f32(d + 24);
    c.zero_rh_pct = get_f32(d + 28);
    c.mass_gr = get_f32(d + 32);        c.diameter_m = get_f32(d + 36);
    c.length_m = get_f32(d + 40);       c.twist_m = get_f32(d + 44);
    if (c.version != kVersion || c.model > 1) return false;
    // Reject nonsense so a bad packet can't send the solver into a loop.
    if (!(c.bc > 0.01f && c.bc < 2.0f)) return false;
    if (!(c.muzzle_mps > 100.0f && c.muzzle_mps < 1500.0f)) return false;
    if (!(c.sight_height_m > 0.0f && c.sight_height_m < 0.3f)) return false;
    if (!(c.zero_range_m >= 10.0f && c.zero_range_m < 1000.0f)) return false;
    if (!(c.zero_temp_c > -60.0f && c.zero_temp_c < 70.0f)) return false;
    if (!(c.zero_pressure_hpa > 300.0f && c.zero_pressure_hpa < 1100.0f)) return false;
    if (!(c.zero_rh_pct >= 0.0f && c.zero_rh_pct <= 100.0f)) return false;
    // Bullet data for spin drift: either all zero (not given) or all plausible.
    if (c.mass_gr == 0.0f && c.diameter_m == 0.0f && c.length_m == 0.0f && c.twist_m == 0.0f) return true;
    if (!(c.mass_gr >= 10.0f && c.mass_gr <= 1000.0f)) return false;
    if (!(c.diameter_m >= 0.0025f && c.diameter_m <= 0.02f)) return false;
    if (!(c.length_m >= 1.5f * c.diameter_m && c.length_m <= 8.0f * c.diameter_m)) return false;
    const float tw = c.twist_m < 0 ? -c.twist_m : c.twist_m;
    if (!(tw >= 5.0f * c.diameter_m && tw <= 150.0f * c.diameter_m)) return false;
    return true;
}

// ---- REQUEST (phone -> board, 28 bytes) -----------------------------------
// 0 u8 flags | 1..3 reserved
//   bit0 use measured wind speed | bit1 use IMU slope | bit2 apply Coriolis
//   bit3 apply spin drift | bit4 use IMU heading as firing direction (else azimuth below)
//   bit5 apply aerodynamic jump (added in place of a reserved bit; older boards ignore it)
//   bit6 sleep: put the board into deep sleep (one-shot, never combined with a real solve request;
//        older boards ignore it). The BOOT or RESET button wakes it.
// 4 f32 range m | 8 f32 wind from (clock deg, 0=head 90=right)
// 12 f32 wind speed m/s (used if bit0 clear) | 16 f32 slope deg (used if bit1 clear)
// 20 f32 latitude deg (north +) | 24 f32 azimuth deg (clockwise from north, used if bit4 clear)
constexpr size_t kRequestSize = 28;
constexpr uint8_t kFlagSensorWind = 0x01;
constexpr uint8_t kFlagImuSlope = 0x02;
constexpr uint8_t kFlagCoriolis = 0x04;
constexpr uint8_t kFlagSpinDrift = 0x08;
constexpr uint8_t kFlagImuAzimuth = 0x10;
constexpr uint8_t kFlagAeroJump = 0x20;
constexpr uint8_t kFlagSleep = 0x40;

struct Request {
    uint8_t flags;
    float range_m, wind_clock_deg, wind_speed_mps, slope_deg, latitude_deg, azimuth_deg;
};

inline bool decode_request(const uint8_t* d, size_t n, Request& r) {
    if (n != kRequestSize) return false;
    r.flags = d[0];
    r.range_m = get_f32(d + 4);        r.wind_clock_deg = get_f32(d + 8);
    r.wind_speed_mps = get_f32(d + 12); r.slope_deg = get_f32(d + 16);
    r.latitude_deg = get_f32(d + 20);   r.azimuth_deg = get_f32(d + 24);
    if (!(r.latitude_deg >= -90.0f && r.latitude_deg <= 90.0f)) return false;
    if (!(r.azimuth_deg >= -360.0f && r.azimuth_deg <= 720.0f)) return false;
    if (!(r.range_m >= 10.0f && r.range_m <= 2000.0f)) return false;
    if (!(r.wind_clock_deg >= -360.0f && r.wind_clock_deg <= 720.0f)) return false;
    if (!(r.wind_speed_mps >= 0.0f && r.wind_speed_mps <= 60.0f)) return false;
    if (!(r.slope_deg >= -89.0f && r.slope_deg <= 89.0f)) return false;
    return true;
}

// ---- RANGE CARD request (phone -> board, 32 bytes) --------------------------
// Asks for `count` solutions at start, start+step, ... The board answers with one
// SOLUTION notification per row (row = 1..count) instead of a single live solution.
// 0 u8 flags (same bits as REQUEST) | 1 u8 count (1..24) | 2..3 reserved
// 4 f32 start m | 8 f32 step m | 12 f32 wind from (clock deg) | 16 f32 wind speed m/s
// 20 f32 slope deg | 24 f32 latitude deg | 28 f32 azimuth deg
constexpr size_t kCardSize = 32;
constexpr uint8_t kMaxCardRows = 24;

struct CardRequest {
    uint8_t flags, count;
    float start_m, step_m, wind_clock_deg, wind_speed_mps, slope_deg, latitude_deg, azimuth_deg;
};

inline bool decode_card(const uint8_t* d, size_t n, CardRequest& c) {
    if (n != kCardSize) return false;
    c.flags = d[0]; c.count = d[1];
    c.start_m = get_f32(d + 4);        c.step_m = get_f32(d + 8);
    c.wind_clock_deg = get_f32(d + 12); c.wind_speed_mps = get_f32(d + 16);
    c.slope_deg = get_f32(d + 20);
    c.latitude_deg = get_f32(d + 24);   c.azimuth_deg = get_f32(d + 28);
    if (!(c.latitude_deg >= -90.0f && c.latitude_deg <= 90.0f)) return false;
    if (!(c.azimuth_deg >= -360.0f && c.azimuth_deg <= 720.0f)) return false;
    if (c.count < 1 || c.count > kMaxCardRows) return false;
    if (!(c.start_m >= 10.0f && c.start_m <= 2000.0f)) return false;
    if (!(c.step_m >= 1.0f && c.step_m <= 2000.0f)) return false;
    if (!(c.start_m + c.step_m * (c.count - 1) <= 2000.0f)) return false;
    if (!(c.wind_clock_deg >= -360.0f && c.wind_clock_deg <= 720.0f)) return false;
    if (!(c.wind_speed_mps >= 0.0f && c.wind_speed_mps <= 60.0f)) return false;
    if (!(c.slope_deg >= -89.0f && c.slope_deg <= 89.0f)) return false;
    return true;
}

// ---- SOLUTION (board -> phone notify, 20 bytes) ---------------------------
// 0 u8 ok | 1 u8 row (0 = live solution, 1..24 = range card row)
// 2 i16 elevation (0.01 mrad, + = up) | 4 i16 windage (0.01 mrad, + = right)
// 6 u16 time of flight (ms) | 8 u16 velocity (0.1 m/s) | 10 i16 drop (cm)
// 12 u16 range used (0.1 m) | 14 u16 wind used (0.01 m/s)
// 16 i8 slope used (deg) | 17 u8 reserved | 18 i16 spin drift part of windage (0.01 mrad)
constexpr size_t kSolutionSize = 20;

struct SolutionPacket {
    bool ok;
    double elevation_mrad, windage_mrad, tof_s, velocity_mps, drop_m;
    double range_m, wind_mps, slope_deg;
    uint8_t row = 0;
    double spin_drift_mrad = 0;
};

inline void encode_solution(const SolutionPacket& s, uint8_t* out) {
    std::memset(out, 0, kSolutionSize);
    out[0] = s.ok ? 1 : 0;
    out[1] = s.row;
    put_i16(out + 2, clamp_i16(s.elevation_mrad * 100.0));
    put_i16(out + 4, clamp_i16(s.windage_mrad * 100.0));
    put_u16(out + 6, clamp_u16(s.tof_s * 1000.0));
    put_u16(out + 8, clamp_u16(s.velocity_mps * 10.0));
    put_i16(out + 10, clamp_i16(s.drop_m * 100.0));
    put_u16(out + 12, clamp_u16(s.range_m * 10.0));
    put_u16(out + 14, clamp_u16(s.wind_mps * 100.0));
    long slope = std::lround(s.slope_deg);
    if (slope > 127) slope = 127;
    if (slope < -128) slope = -128;
    out[16] = static_cast<uint8_t>(static_cast<int8_t>(slope));
    put_i16(out + 18, clamp_i16(s.spin_drift_mrad * 100.0));
}

// ---- ENVIRONMENT (board -> phone notify, 16 bytes) ------------------------
// 0 i16 temp (0.01 C) | 2 u16 RH (0.01 %) | 4 u16 pressure (0.1 hPa)
// 6 u16 wind avg (0.01 m/s) | 8 u16 heading (0.1 deg) | 10 i16 pitch (0.1 deg)
// 12 i16 density altitude (m) | 14 u8 flags (bit0 BME ok, bit1 IMU ok, bit2 battery valid, bit3 charging) | 15 u8 battery % (0..100)
// Byte 15 used to be reserved, so the layout and version did not change; an older app just ignores it.
// Older firmware sends 0 there with bit2 clear, so the app must only trust byte 15 when bit2 is set.
constexpr size_t kEnvSize = 16;
constexpr uint8_t kEnvBmeOk = 0x01;
constexpr uint8_t kEnvImuOk = 0x02;
constexpr uint8_t kEnvBattOk = 0x04;
constexpr uint8_t kEnvCharging = 0x08;   // external power is charging (or topping off) the battery

struct EnvPacket {
    double temp_c, rh_pct, pressure_hpa, wind_mps, heading_deg, pitch_deg, density_alt_m;
    uint8_t flags;
    uint8_t battery_pct = 255;       // 255 = unknown
};

// One-cell LiPo state of charge (%) from the resting cell voltage in millivolts. Piecewise-linear
// through a typical discharge curve; it is an estimate (the voltage sags under load and warms/cools).
inline uint8_t battery_percent_from_mv(double mv) {
    static const double v[] = {3300, 3500, 3600, 3700, 3750, 3800, 3850, 3900, 3950, 4000, 4100, 4200};
    static const double p[] = {   0,    5,   10,   20,   30,   42,   55,   65,   75,   82,   93,  100};
    if (!(mv > 0)) return 255;
    if (mv <= v[0]) return 0;
    if (mv >= v[11]) return 100;
    for (int i = 1; i < 12; ++i)
        if (mv <= v[i]) return (uint8_t)(p[i - 1] + (p[i] - p[i - 1]) * (mv - v[i - 1]) / (v[i] - v[i - 1]) + 0.5);
    return 100;
}

inline void encode_env(const EnvPacket& e, uint8_t* out) {
    std::memset(out, 0, kEnvSize);
    put_i16(out + 0, clamp_i16(e.temp_c * 100.0));
    put_u16(out + 2, clamp_u16(e.rh_pct * 100.0));
    put_u16(out + 4, clamp_u16(e.pressure_hpa * 10.0));
    put_u16(out + 6, clamp_u16(e.wind_mps * 100.0));
    put_u16(out + 8, clamp_u16(e.heading_deg * 10.0));
    put_i16(out + 10, clamp_i16(e.pitch_deg * 10.0));
    put_i16(out + 12, clamp_i16(e.density_alt_m));
    const bool battKnown = e.battery_pct <= 100;
    out[14] = e.flags | (battKnown ? kEnvBattOk : 0);
    out[15] = battKnown ? e.battery_pct : 0;
}

// Density altitude (m) from air density, ICAO standard atmosphere.
inline double density_altitude_m(double rho) {
    // rho/rho0 = (1 - 2.25577e-5 h)^4.25588
    const double ratio = rho / 1.225;
    return (1.0 - std::pow(ratio, 1.0 / 4.25588)) / 2.25577e-5;
}

}  // namespace protocol
