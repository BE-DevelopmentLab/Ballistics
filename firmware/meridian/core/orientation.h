// Rifle angles from the BNO085 rotation vector.
//
// The BNO085 gives a quaternion that turns sensor-frame vectors into the world frame. This file turns it
// into what the shooter cares about: the bore's angle above level (pitch), how far the rifle is rolled
// (cant) and which way the bore points (bearing).
//
// ASSUMPTIONS to check on the bench once the BNO085 is on the rifle:
//   - World frame: Z is up (true for both ENU and the other right-handed frames the SH-2 can use). If the
//     pitch sign comes out backwards when the muzzle is raised, the frame is Z-down and kWorldZUp is wrong.
//   - North: bearing = atan2(x, y), i.e. world +Y is north (ENU). If the bearing is off by a constant
//     (90 or 180 deg), the AIM BEARING calibration absorbs it, but fix the formula once it is known.
//   - The mount code says which sensor axis points along the bore and which points up (default: breakout
//     flat on top of the rifle, silkscreen +Y arrow toward the muzzle, +Z up).
#pragma once

#include <cmath>
#include <cstdint>

namespace orient {

struct Vec { double x, y, z; };
struct Quat { double w, x, y, z; };

inline Vec cross(const Vec& a, const Vec& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double dot(const Vec& a, const Vec& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// v' = q v q*
inline Vec rotate(const Quat& q, const Vec& v) {
    const Vec u{q.x, q.y, q.z};
    const Vec t = cross(u, v);
    const Vec t2{2 * t.x, 2 * t.y, 2 * t.z};
    const Vec c = cross(u, t2);
    return {v.x + q.w * t2.x + c.x, v.y + q.w * t2.y + c.y, v.z + q.w * t2.z + c.z};
}

// Axis index: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z.
inline Vec axis(int a) {
    const double s = (a & 1) ? -1.0 : 1.0;
    switch (a >> 1) { case 0: return {s, 0, 0}; case 1: return {0, s, 0}; default: return {0, 0, s}; }
}

// Mount code 0..23 = forward axis (0..5) * 4 + which of the 4 perpendicular axes points up.
inline void mount_axes(uint8_t code, Vec& fwd, Vec& up) {
    if (code > 23) code = 0;
    const int f = code / 4, k = code % 4;
    int cand[4], n = 0;
    for (int a = 0; a < 6; ++a) if ((a >> 1) != (f >> 1)) cand[n++] = a;
    fwd = axis(f); up = axis(cand[k]);
}
inline uint8_t mount_code(int fwdAxis, int upAxis) {
    int n = 0;
    for (int a = 0; a < 6; ++a) {
        if ((a >> 1) == (fwdAxis >> 1)) continue;
        if (a == upAxis) return static_cast<uint8_t>(fwdAxis * 4 + n);
        ++n;
    }
    return 0;
}
constexpr uint8_t kDefaultMount = 2 * 4 + 2;   // +Y forward, +Z up (checked in the tests)

struct Calibration {
    uint8_t mount = kDefaultMount;
    double pitch_zero = 0, cant_zero = 0;     // raw readings stored when the rifle was level
    double heading_offset = 0;                // added to the raw bearing (from AIM BEARING)
    double declination = 0;                   // magnetic -> true, east +
    bool level_set = false, heading_set = false, declination_set = false;
};

struct Angles { double pitch_deg, cant_deg, bearing_deg, raw_pitch, raw_cant, raw_bearing; };

constexpr double kRad = 57.29577951308232;

inline double wrap360(double d) { d = std::fmod(d, 360.0); return d < 0 ? d + 360.0 : d; }
inline double wrap180(double d) { d = wrap360(d); return d > 180.0 ? d - 360.0 : d; }

inline Angles angles(const Quat& q, const Calibration& cal) {
    Vec fs, us;
    mount_axes(cal.mount, fs, us);
    const Vec f = rotate(q, fs), u = rotate(q, us);
    const Vec r = cross(f, u);                       // rifle's right-hand side
    double sp = f.z; if (sp > 1) sp = 1; if (sp < -1) sp = -1;
    Angles a;
    a.raw_pitch = std::asin(sp) * kRad;
    a.raw_cant = std::atan2(-r.z, u.z) * kRad;      // + = rolled right (right side down)
    a.raw_bearing = wrap360(std::atan2(f.x, f.y) * kRad);
    a.pitch_deg = a.raw_pitch - (cal.level_set ? cal.pitch_zero : 0);
    a.cant_deg = wrap180(a.raw_cant - (cal.level_set ? cal.cant_zero : 0));
    a.bearing_deg = wrap360(a.raw_bearing + cal.heading_offset + cal.declination);
    return a;
}

// The rifle is level now: store the raw readings as zero.
inline void set_level(Calibration& cal, const Angles& now) {
    cal.pitch_zero = now.raw_pitch; cal.cant_zero = now.raw_cant; cal.level_set = true;
}
// The bore points at this TRUE bearing now: store the offset that makes it read that.
inline void set_aim_bearing(Calibration& cal, const Angles& now, double true_bearing) {
    cal.heading_offset = wrap180(true_bearing - cal.declination - now.raw_bearing);
    cal.heading_set = true;
}

}  // namespace orient
