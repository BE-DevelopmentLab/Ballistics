// Meridian (Celestial V2 rifle-mounted unit) Bluetooth packet layouts.
//
// Meridian runs two GATT services:
//   1. The Solstice service (b4a10000-...), byte for byte the same as Solstice (../../core/protocol.h), so the
//      existing phone app connects to Meridian and works unchanged (config, request, solution, env, card).
//   2. The Meridian service (b4a20000-...), defined here, for the camera spotter (Jetson) and the app:
//      laser range, rifle angles, atmospherics, GPS, corrections back from the spotter, and setup commands.
//
// tools/meridian_client.py (the spotter / laptop side) must match this file byte for byte;
// tests/test_protocol_py.sh checks that.
//
// All multi-byte values are little-endian. Every notify payload is 20 bytes so it fits the default BLE MTU.
#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include "protocol.h"   // Solstice: little-endian helpers and the shared packets

namespace meridian {

using protocol::put_u16; using protocol::get_u16; using protocol::put_i16; using protocol::get_i16;
using protocol::put_f32; using protocol::get_f32; using protocol::clamp_i16; using protocol::clamp_u16;

constexpr uint8_t kVersion = 1;

#define MR_UUID_SERVICE    "b4a20000-1d5b-4c0e-9a3f-5e7d2c0a1b00"
#define MR_UUID_SHOT       "b4a20001-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // notify/read: range, angles, atmosphere
#define MR_UUID_POSITION   "b4a20002-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // notify/read: GPS
#define MR_UUID_CORRECTION "b4a20003-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // write: spotter -> Meridian dial/hold
#define MR_UUID_COMMAND    "b4a20004-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // write: range, calibrate, settings
#define MR_UUID_MESSAGE    "b4a20005-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // write: short text for the screen
#define MR_UUID_STATUS     "b4a20006-1d5b-4c0e-9a3f-5e7d2c0a1b00"  // notify/read: setup and health

inline void put_i32(uint8_t* p, int32_t v) {
    const uint32_t u = static_cast<uint32_t>(v);
    p[0] = u & 0xFF; p[1] = (u >> 8) & 0xFF; p[2] = (u >> 16) & 0xFF; p[3] = (u >> 24) & 0xFF;
}
inline int32_t get_i32(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}
inline void put_u32(uint8_t* p, uint32_t v) { put_i32(p, static_cast<int32_t>(v)); }
inline uint32_t get_u32(const uint8_t* p) { return static_cast<uint32_t>(get_i32(p)); }
inline uint8_t clamp_u8(double v) { return v < 0 ? 0 : v > 255 ? 255 : static_cast<uint8_t>(std::lround(v)); }

// ---- SHOT (Meridian -> spotter/app notify, 20 bytes, 5 Hz and at once on a new range) --------------
// 0 u8 flags | 1 u8 battery % (255 = unknown)
// 2 u16 range (0.1 m, 0 = no range yet) | 4 u16 range age (0.1 s, 65535 = older)
// 6 i16 pitch / line-of-sight angle (0.01 deg, + = uphill) | 8 i16 cant (0.01 deg, + = rifle rolled right)
// 10 u16 bearing of the bore (0.01 deg, clockwise from TRUE north when flag kShotTrueNorth, else magnetic)
// 12 i16 temperature (0.01 C) | 14 u16 station pressure (0.1 hPa) | 16 u16 RH (0.01 %)
// 18 u8 range sequence (+1 for each new laser result, so the spotter can see a fresh range)
// 19 u8 bits 0-1 compass calibration (0 unreliable .. 3 high), bits 2-7 heading accuracy (deg, 63 = unknown)
constexpr size_t kShotSize = 20;
constexpr uint8_t kShotRangeValid = 0x01;   // bytes 2..5 hold a real laser range
constexpr uint8_t kShotImuOk = 0x02;
constexpr uint8_t kShotBmeOk = 0x04;
constexpr uint8_t kShotGpsFix = 0x08;
constexpr uint8_t kShotTrueNorth = 0x10;    // declination applied to the bearing
constexpr uint8_t kShotRanging = 0x20;      // laser busy (single shot pending or continuous mode)
constexpr uint8_t kShotCharging = 0x40;
constexpr uint8_t kShotLaserFault = 0x80;   // laser missing, timed out or self-check failed

struct Shot {
    uint8_t flags = 0;
    uint8_t battery_pct = 255;
    double range_m = 0, range_age_s = 0;
    double pitch_deg = 0, cant_deg = 0, bearing_deg = 0;
    double temp_c = 0, pressure_hpa = 0, rh_pct = 0;
    uint8_t range_seq = 0;
    uint8_t mag_cal = 0;              // 0..3
    double heading_acc_deg = -1;      // < 0 = unknown
};

inline double wrap360(double d) { d = std::fmod(d, 360.0); return d < 0 ? d + 360.0 : d; }

inline void encode_shot(const Shot& s, uint8_t* out) {
    std::memset(out, 0, kShotSize);
    out[0] = s.flags;
    out[1] = s.battery_pct;
    put_u16(out + 2, (s.flags & kShotRangeValid) ? clamp_u16(s.range_m * 10.0) : 0);
    put_u16(out + 4, (s.flags & kShotRangeValid) ? clamp_u16(s.range_age_s * 10.0) : 65535);
    put_i16(out + 6, clamp_i16(s.pitch_deg * 100.0));
    put_i16(out + 8, clamp_i16(s.cant_deg * 100.0));
    put_u16(out + 10, static_cast<uint16_t>(clamp_u16(wrap360(s.bearing_deg) * 100.0) % 36000));
    put_i16(out + 12, clamp_i16(s.temp_c * 100.0));
    put_u16(out + 14, clamp_u16(s.pressure_hpa * 10.0));
    put_u16(out + 16, clamp_u16(s.rh_pct * 100.0));
    out[18] = s.range_seq;
    uint8_t acc = s.heading_acc_deg < 0 ? 63 : (s.heading_acc_deg > 62 ? 62 : static_cast<uint8_t>(std::lround(s.heading_acc_deg)));
    out[19] = static_cast<uint8_t>((s.mag_cal & 0x03) | (acc << 2));
}

inline Shot decode_shot(const uint8_t* d) {
    Shot s;
    s.flags = d[0]; s.battery_pct = d[1];
    s.range_m = get_u16(d + 2) / 10.0; s.range_age_s = get_u16(d + 4) / 10.0;
    s.pitch_deg = get_i16(d + 6) / 100.0; s.cant_deg = get_i16(d + 8) / 100.0;
    s.bearing_deg = get_u16(d + 10) / 100.0;
    s.temp_c = get_i16(d + 12) / 100.0; s.pressure_hpa = get_u16(d + 14) / 10.0; s.rh_pct = get_u16(d + 16) / 100.0;
    s.range_seq = d[18];
    s.mag_cal = d[19] & 0x03;
    const uint8_t acc = d[19] >> 2;
    s.heading_acc_deg = acc == 63 ? -1 : acc;
    return s;
}

// ---- POSITION (Meridian -> spotter/app notify, 20 bytes, 1 Hz) ---------------------------------------
// 0 i32 latitude (1e-7 deg, north +) | 4 i32 longitude (1e-7 deg, east +)
// 8 i16 altitude above mean sea level (m) | 10 u8 fix quality (0 none, 1 GPS, 2 DGPS/SBAS) | 11 u8 satellites
// 12 u16 HDOP (0.01) | 14 u32 UTC time (seconds since 1970-01-01, 0 = unknown)
// 18 i16 magnetic declination in use (0.01 deg, east +)
constexpr size_t kPositionSize = 20;

struct Position {
    double lat_deg = 0, lon_deg = 0, alt_m = 0;
    uint8_t fix = 0, sats = 0;
    double hdop = 0;
    uint32_t utc = 0;
    double declination_deg = 0;
};

inline void encode_position(const Position& p, uint8_t* out) {
    std::memset(out, 0, kPositionSize);
    put_i32(out + 0, static_cast<int32_t>(std::llround(p.lat_deg * 1e7)));
    put_i32(out + 4, static_cast<int32_t>(std::llround(p.lon_deg * 1e7)));
    put_i16(out + 8, clamp_i16(p.alt_m));
    out[10] = p.fix; out[11] = p.sats;
    put_u16(out + 12, clamp_u16(p.hdop * 100.0));
    put_u32(out + 14, p.utc);
    put_i16(out + 18, clamp_i16(p.declination_deg * 100.0));
}

inline Position decode_position(const uint8_t* d) {
    Position p;
    p.lat_deg = get_i32(d) / 1e7; p.lon_deg = get_i32(d + 4) / 1e7;
    p.alt_m = get_i16(d + 8); p.fix = d[10]; p.sats = d[11];
    p.hdop = get_u16(d + 12) / 100.0; p.utc = get_u32(d + 14);
    p.declination_deg = get_i16(d + 18) / 100.0;
    return p;
}

// ---- CORRECTION (spotter -> Meridian write, 20 bytes) ----------------------------------------------
// What the shooter should dial or hold, worked out by the spotter. Meridian shows it until a newer one
// arrives or it goes stale.
// 0 u8 version (= kVersion) | 1 u8 flags
// 2 i16 elevation (0.01 mrad, + = up) | 4 i16 windage (0.01 mrad, + = right)
// 6 u16 range it was solved for (0.1 m) | 8 u16 time of flight (ms)
// 10 i16 lead for a moving target (0.01 mrad, + = right, 0 = none)
// 12 u8 hit chance % (255 = none) | 13 u8 target number (0 = none)
// 14 u16 wind used (0.01 m/s) | 16 i16 wind from (deg, 0 = head, 90 = from the right)
// 18 u16 sequence (the spotter counts up; Meridian echoes the last one in STATUS)
constexpr size_t kCorrectionSize = 20;
constexpr uint8_t kCorrValid = 0x01;        // clear = spotter has no solution (shows "--")
constexpr uint8_t kCorrMissAdjusted = 0x02; // includes a spotted miss correction
constexpr uint8_t kCorrHasLead = 0x04;
constexpr uint8_t kCorrClear = 0x80;        // clear the screen back to Meridian's own solution

struct Correction {
    uint8_t flags = 0;
    double elevation_mrad = 0, windage_mrad = 0, range_m = 0, tof_s = 0, lead_mrad = 0;
    uint8_t hit_pct = 255, target = 0;
    double wind_mps = 0, wind_from_deg = 0;
    uint16_t seq = 0;
};

inline void encode_correction(const Correction& c, uint8_t* out) {
    std::memset(out, 0, kCorrectionSize);
    out[0] = kVersion; out[1] = c.flags;
    put_i16(out + 2, clamp_i16(c.elevation_mrad * 100.0));
    put_i16(out + 4, clamp_i16(c.windage_mrad * 100.0));
    put_u16(out + 6, clamp_u16(c.range_m * 10.0));
    put_u16(out + 8, clamp_u16(c.tof_s * 1000.0));
    put_i16(out + 10, clamp_i16(c.lead_mrad * 100.0));
    out[12] = c.hit_pct; out[13] = c.target;
    put_u16(out + 14, clamp_u16(c.wind_mps * 100.0));
    put_i16(out + 16, clamp_i16(c.wind_from_deg));
    put_u16(out + 18, c.seq);
}

inline bool decode_correction(const uint8_t* d, size_t n, Correction& c) {
    if (n != kCorrectionSize || d[0] != kVersion) return false;
    c.flags = d[1];
    c.elevation_mrad = get_i16(d + 2) / 100.0; c.windage_mrad = get_i16(d + 4) / 100.0;
    c.range_m = get_u16(d + 6) / 10.0; c.tof_s = get_u16(d + 8) / 1000.0;
    c.lead_mrad = get_i16(d + 10) / 100.0;
    c.hit_pct = d[12]; c.target = d[13];
    c.wind_mps = get_u16(d + 14) / 100.0; c.wind_from_deg = get_i16(d + 16);
    c.seq = get_u16(d + 18);
    if (c.hit_pct > 100 && c.hit_pct != 255) return false;
    if (std::fabs(c.elevation_mrad) > 200 || std::fabs(c.windage_mrad) > 200) return false;
    return true;
}

// ---- COMMAND (spotter/app -> Meridian write, 1 to 20 bytes) ----------------------------------------
// 0 u8 command | 1.. arguments (f32 arguments at byte 1, u8 arguments at byte 1)
enum Cmd : uint8_t {
    kCmdRange = 1,             // fire one laser measurement
    kCmdContinuous = 2,        // u8 1 = continuous ranging on, 0 = off
    kCmdLevelZero = 3,         // the rifle is level right now (scope bubble): store pitch and cant zero
    kCmdClearLevel = 4,        // forget the level zero
    kCmdDeclination = 5,       // f32 magnetic declination, deg east + (from a chart or the spotter)
    kCmdAimBearing = 6,        // f32 the bore is pointing at this TRUE bearing now: store a heading offset
    kCmdMount = 7,             // u8 how the BNO085 is mounted (see orientation.h, 0..23)
    kCmdMagCalStart = 8,       // start a compass calibration (figure-8 motion, away from steel)
    kCmdMagCalSave = 9,        // save the calibration to the BNO085's flash
    kCmdUnits = 10,            // u8 0 = mil, 1 = MOA on Meridian's screen
    kCmdLaserOffset = 11,      // f32 metres added to every laser range (module-to-muzzle or calibration)
    kCmdTargetMode = 12,       // u8 1 = first target (steel in front of a berm), 2 = last target (through brush)
    kCmdSolsticeLink = 13,     // u8 1 = pull wind from Solstice over Bluetooth, 0 = don't
    kCmdSleep = 14,            // deep sleep; the MODE button wakes it
    kCmdClearHeading = 15,     // forget the heading offset from kCmdAimBearing
};

struct Command {
    uint8_t cmd = 0;
    float f = 0;     // f32 argument, if the command takes one
    uint8_t u = 0;   // u8 argument, if the command takes one
};

inline bool decode_command(const uint8_t* d, size_t n, Command& c) {
    if (n < 1 || n > 20) return false;
    c.cmd = d[0];
    switch (c.cmd) {
        case kCmdRange: case kCmdLevelZero: case kCmdClearLevel: case kCmdMagCalStart:
        case kCmdMagCalSave: case kCmdSleep: case kCmdClearHeading:
            return true;
        case kCmdContinuous: case kCmdSolsticeLink:
            if (n < 2 || d[1] > 1) return false;
            c.u = d[1]; return true;
        case kCmdUnits:
            if (n < 2 || d[1] > 1) return false;
            c.u = d[1]; return true;
        case kCmdTargetMode:
            if (n < 2 || d[1] < 1 || d[1] > 2) return false;
            c.u = d[1]; return true;
        case kCmdMount:
            if (n < 2 || d[1] > 23) return false;
            c.u = d[1]; return true;
        case kCmdDeclination:
            if (n < 5) return false;
            c.f = get_f32(d + 1);
            return std::fabs(c.f) <= 60.0f;
        case kCmdAimBearing:
            if (n < 5) return false;
            c.f = get_f32(d + 1);
            return c.f >= 0.0f && c.f < 360.0f;
        case kCmdLaserOffset:
            if (n < 5) return false;
            c.f = get_f32(d + 1);
            return std::fabs(c.f) <= 20.0f;
        default:
            return false;
    }
}

// ---- MESSAGE (spotter -> Meridian write, 0 to 20 bytes of printable ASCII) --------------------------
// Shown on the bottom line of the screen, e.g. "SEND IT", "HOLD", "TGT 3". Empty clears it.
constexpr size_t kMessageMax = 20;
inline bool decode_message(const uint8_t* d, size_t n, char* out /* kMessageMax + 1 */) {
    if (n > kMessageMax) return false;
    for (size_t i = 0; i < n; ++i) {
        if (d[i] < 0x20 || d[i] > 0x7E) return false;
        out[i] = static_cast<char>(d[i]);
    }
    out[n] = 0;
    return true;
}

// ---- STATUS (Meridian -> app/spotter notify and read, 20 bytes, 1 Hz and after each command) --------
// 0 u8 protocol version | 1 u8 flags | 2 u8 mount code | 3 u8 units (0 mil, 1 MOA)
// 4 i16 level zero: pitch offset (0.01 deg) | 6 i16 level zero: cant offset (0.01 deg)
// 8 i16 heading offset (0.01 deg) | 10 i16 declination (0.01 deg) | 12 i16 laser offset (cm)
// 14 u8 laser target mode (1 first, 2 last) | 15 u8 last command result (0 ok, 1 rejected, 2 failed)
// 16 u16 last correction sequence received | 18 u8 connected centrals | 19 u8 laser state (LaserState)
constexpr size_t kStatusSize = 20;
constexpr uint8_t kStatSolsticeLinked = 0x01;  // receiving wind from Solstice
constexpr uint8_t kStatMagCalRunning = 0x02;
constexpr uint8_t kStatLevelSet = 0x04;
constexpr uint8_t kStatHeadingSet = 0x08;
constexpr uint8_t kStatSolsticeWanted = 0x10;  // link to Solstice switched on
constexpr uint8_t kStatCorrFresh = 0x20;       // the screen shows a spotter correction

enum LaserState : uint8_t { kLaserAbsent = 0, kLaserIdle = 1, kLaserBusy = 2, kLaserContinuous = 3, kLaserFault = 4 };

struct Status {
    uint8_t flags = 0, mount = 0, units = 0;
    double pitch_zero = 0, cant_zero = 0, heading_offset = 0, declination = 0, laser_offset_m = 0;
    uint8_t target_mode = 1, last_result = 0;
    uint16_t last_corr_seq = 0;
    uint8_t centrals = 0, laser = 0;
};

inline void encode_status(const Status& s, uint8_t* out) {
    std::memset(out, 0, kStatusSize);
    out[0] = kVersion; out[1] = s.flags; out[2] = s.mount; out[3] = s.units;
    put_i16(out + 4, clamp_i16(s.pitch_zero * 100.0));
    put_i16(out + 6, clamp_i16(s.cant_zero * 100.0));
    put_i16(out + 8, clamp_i16(s.heading_offset * 100.0));
    put_i16(out + 10, clamp_i16(s.declination * 100.0));
    put_i16(out + 12, clamp_i16(s.laser_offset_m * 100.0));
    out[14] = s.target_mode; out[15] = s.last_result;
    put_u16(out + 16, s.last_corr_seq);
    out[18] = s.centrals; out[19] = s.laser;
}

// ---- Solstice ENV packet, as Meridian receives it when it pulls wind from Solstice ------------------
// Layout in ../../core/protocol.h (encode_env). Only the parts Meridian uses are decoded.
struct SolsticeEnv { double wind_mps = 0; bool ok = false; };
inline bool decode_solstice_env(const uint8_t* d, size_t n, SolsticeEnv& e) {
    if (n != protocol::kEnvSize) return false;
    e.wind_mps = get_u16(d + 6) / 100.0;
    e.ok = e.wind_mps < 60.0;
    return e.ok;
}

// Request flag bit7 on the Solstice-compatible request: use Meridian's laser range instead of the typed
// range (when a laser range exists). It was a reserved bit, so the current app (which never sets it) and
// Solstice (which ignores it) are unaffected.
constexpr uint8_t kFlagLaserRange = 0x80;

}  // namespace meridian
