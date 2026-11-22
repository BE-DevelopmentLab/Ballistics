// Point-mass ballistic solver. Header-only, no heap, no exceptions.
// Builds for host (tests) and ESP32 (Arduino / ESP-IDF).
//
// Frame: x = along line of sight, y = perpendicular up, z = right.
// Slope is handled by rotating gravity in this frame.
// Units are SI internally; helpers convert at the edges.
#pragma once

#include <cmath>
#include "drag_tables.h"

#ifndef BALLISTICS_REAL
#define BALLISTICS_REAL float
#endif

namespace ballistics {

using real = BALLISTICS_REAL;

constexpr real kPi = static_cast<real>(3.14159265358979323846);
constexpr real kG = static_cast<real>(9.80665);
constexpr real kLbPerIn2ToKgPerM2 = static_cast<real>(703.0696);
constexpr real kFpsToMps = static_cast<real>(0.3048);
constexpr real kYdToM = static_cast<real>(0.9144);

enum class DragModel { G1, G7 };

struct Bullet {
    real bc;                // lb/in^2, for the chosen drag model
    DragModel model;
    real muzzle_velocity;   // m/s
    // Optional, for spin drift. Leave 0 to disable it.
    real mass_gr = 0;       // grains
    real diameter_in = 0;   // inches
    real length_in = 0;     // inches
    real twist_in = 0;      // inches per turn; positive = right-hand twist, negative = left
};

struct Atmosphere {
    real temp_c;            // air temperature
    real pressure_hpa;      // STATION pressure (not sea-level adjusted)
    real rh_pct;            // 0..100
};

struct Shot {
    real sight_height;      // m, bore below sight line
    real zero_range;        // m, zeroed on flat ground
    real wind_speed;        // m/s
    real wind_clock_deg;    // direction wind comes FROM: 0=headwind, 90=from right
    real slope_rad;         // line of sight angle above horizontal (uphill +)
    // Earth effects. Coriolis needs latitude and the compass direction you are shooting.
    bool coriolis = false;
    real latitude_deg = 0;  // north +
    real azimuth_deg = 0;   // direction of fire, clockwise from north
    bool spin_drift = false;
    bool aero_jump = false;   // Litz aerodynamic jump from crosswind (needs the spin-drift bullet data)
};

struct Solution {
    real elevation_mrad;    // come-up relative to zero (positive = up)
    real windage_mrad;      // correction (positive = dial/hold right)
    real tof;               // s
    real velocity;          // m/s at target
    real drop;              // m, bullet below muzzle-bore line (informational)
    bool ok;
    real spin_drift_mrad = 0;   // part of windage_mrad that is spin drift (already included)
    real aero_jump_mrad = 0;    // part of elevation_mrad that is aerodynamic jump (already included)
};

// ---- Atmosphere ---------------------------------------------------------

struct AirState {
    real density;           // kg/m^3
    real sound_speed;       // m/s
};

inline AirState air_state(const Atmosphere& a) {
    const real tk = a.temp_c + static_cast<real>(273.15);
    // Buck (1996) saturation vapour pressure, hPa
    const real es = static_cast<real>(6.1121) *
        std::exp((static_cast<real>(18.678) - a.temp_c / static_cast<real>(234.5)) *
                 (a.temp_c / (static_cast<real>(257.14) + a.temp_c)));
    real e = (a.rh_pct / static_cast<real>(100)) * es;
    if (e > a.pressure_hpa) e = a.pressure_hpa;
    const real pd = a.pressure_hpa - e;
    AirState s;
    s.density = (pd * static_cast<real>(100)) / (static_cast<real>(287.058) * tk) +
                (e * static_cast<real>(100)) / (static_cast<real>(461.495) * tk);
    // Virtual temperature approximation for speed of sound in moist air
    const real tv = tk / (static_cast<real>(1) - static_cast<real>(0.378) * e / a.pressure_hpa);
    s.sound_speed = static_cast<real>(20.0468) * std::sqrt(tv);
    return s;
}

// ---- Drag lookup --------------------------------------------------------

inline real drag_cd(DragModel m, real mach) {
    const DragPoint* t = (m == DragModel::G1) ? kG1 : kG7;
    const int n = (m == DragModel::G1) ? kG1Count : kG7Count;
    if (mach <= t[0].mach) return t[0].cd;
    if (mach >= t[n - 1].mach) return t[n - 1].cd;
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (t[mid].mach <= mach) lo = mid; else hi = mid;
    }
    const real f = (mach - t[lo].mach) / (t[hi].mach - t[lo].mach);
    return t[lo].cd + f * (t[hi].cd - t[lo].cd);
}

// ---- Integrator ---------------------------------------------------------

struct State {
    real x, y, z, vx, vy, vz;
};

struct Env {
    real bc_si;             // kg/m^2
    DragModel model;
    real rho;
    real c;
    real wx, wz;            // air velocity (wind) components, m/s
    real gx, gy;            // gravity in the LOS frame
    real ox = 0, oy = 0, oz = 0;   // Earth rotation (rad/s) in the LOS frame; all 0 = no Coriolis
};

inline void deriv(const Env& e, const State& s, State& d) {
    const real rx = s.vx - e.wx;
    const real ry = s.vy;
    const real rz = s.vz - e.wz;
    const real vr = std::sqrt(rx * rx + ry * ry + rz * rz);
    const real cd = drag_cd(e.model, vr / e.c);
    // a = -(pi/8) * rho * Cd * |v_rel| / BC * v_rel
    const real k = (kPi / static_cast<real>(8)) * e.rho * cd * vr / e.bc_si;
    d.x = s.vx;
    d.y = s.vy;
    d.z = s.vz;
    // Coriolis: a = -2 * (Omega x v), using ground velocity
    d.vx = -k * rx + e.gx - 2 * (e.oy * s.vz - e.oz * s.vy);
    d.vy = -k * ry + e.gy - 2 * (e.oz * s.vx - e.ox * s.vz);
    d.vz = -k * rz        - 2 * (e.ox * s.vy - e.oy * s.vx);
}

inline State advance(const State& s, const State& d, real h) {
    State r;
    r.x = s.x + d.x * h;   r.y = s.y + d.y * h;   r.z = s.z + d.z * h;
    r.vx = s.vx + d.vx * h; r.vy = s.vy + d.vy * h; r.vz = s.vz + d.vz * h;
    return r;
}

inline State rk4(const Env& e, const State& s, real h) {
    State k1, k2, k3, k4;
    deriv(e, s, k1);
    deriv(e, advance(s, k1, h / 2), k2);
    deriv(e, advance(s, k2, h / 2), k3);
    deriv(e, advance(s, k3, h), k4);
    State r;
    const real h6 = h / static_cast<real>(6);
    r.x  = s.x  + h6 * (k1.x  + 2 * k2.x  + 2 * k3.x  + k4.x);
    r.y  = s.y  + h6 * (k1.y  + 2 * k2.y  + 2 * k3.y  + k4.y);
    r.z  = s.z  + h6 * (k1.z  + 2 * k2.z  + 2 * k3.z  + k4.z);
    r.vx = s.vx + h6 * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx);
    r.vy = s.vy + h6 * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy);
    r.vz = s.vz + h6 * (k1.vz + 2 * k2.vz + 2 * k3.vz + k4.vz);
    return r;
}

struct Impact {
    real y, z, tof, speed;
    bool ok;
};

// Fly from the muzzle at launch angle `alpha` (rad, above the sight line)
// until x reaches `range`. Muzzle sits `sight_height` below the sight line.
inline Impact fly(const Env& e, const Bullet& b, real sight_height, real alpha,
                  real range, real dt = static_cast<real>(0.001)) {
    State s;
    s.x = 0; s.y = -sight_height; s.z = 0;
    s.vx = b.muzzle_velocity * std::cos(alpha);
    s.vy = b.muzzle_velocity * std::sin(alpha);
    s.vz = 0;
    real t = 0;
    const int kMaxSteps = 20000;
    for (int i = 0; i < kMaxSteps; ++i) {
        if (s.x >= range) break;
        const State prev = s;
        s = rk4(e, s, dt);
        const real tp = t;
        t += dt;
        if (s.vx <= static_cast<real>(1)) return {0, 0, t, 0, false};
        if (s.x >= range) {
            const real f = (range - prev.x) / (s.x - prev.x);
            Impact r;
            r.y = prev.y + f * (s.y - prev.y);
            r.z = prev.z + f * (s.z - prev.z);
            r.tof = tp + f * dt;
            const real vx = prev.vx + f * (s.vx - prev.vx);
            const real vy = prev.vy + f * (s.vy - prev.vy);
            const real vz = prev.vz + f * (s.vz - prev.vz);
            r.speed = std::sqrt(vx * vx + vy * vy + vz * vz);
            r.ok = true;
            return r;
        }
    }
    return {0, 0, t, 0, false};
}

// Find the launch angle so the bullet crosses the sight line at `range`.
inline bool solve_angle(const Env& e, const Bullet& b, real sight_height,
                        real range, real& alpha_out, Impact& at_range) {
    real a0 = sight_height / range + kG * range / (2 * b.muzzle_velocity * b.muzzle_velocity);
    real a1 = a0 * static_cast<real>(1.05) + static_cast<real>(1e-4);
    Impact i0 = fly(e, b, sight_height, a0, range);
    Impact i1 = fly(e, b, sight_height, a1, range);
    if (!i0.ok || !i1.ok) return false;
    for (int it = 0; it < 20; ++it) {
        if (std::fabs(i1.y) < static_cast<real>(1e-4)) break;
        const real denom = i1.y - i0.y;
        if (std::fabs(denom) < static_cast<real>(1e-9)) break;
        const real a2 = a1 - i1.y * (a1 - a0) / denom;
        a0 = a1; i0 = i1;
        a1 = a2;
        i1 = fly(e, b, sight_height, a1, range);
        if (!i1.ok) return false;
    }
    alpha_out = a1;
    at_range = i1;
    return true;
}

// Coriolis is smaller than other error sources inside this range, so it is only applied beyond it.
constexpr real kCoriolisMinRange = static_cast<real>(1000) * kYdToM;   // 1000 yd

constexpr real kEarthRate = static_cast<real>(7.2921159e-5);   // rad/s

inline Env make_env(const Bullet& b, const Atmosphere& atm, real wind_speed,
                    real wind_clock_deg, real slope_rad,
                    bool coriolis = false, real latitude_deg = 0, real azimuth_deg = 0) {
    const AirState air = air_state(atm);
    Env e;
    e.bc_si = b.bc * kLbPerIn2ToKgPerM2;
    e.model = b.model;
    e.rho = air.density;
    e.c = air.sound_speed;
    const real ang = wind_clock_deg * kPi / static_cast<real>(180);
    e.wx = -wind_speed * std::cos(ang);   // headwind (0 deg) pushes air toward shooter
    e.wz = -wind_speed * std::sin(ang);   // from right (90 deg) pushes air toward -z
    e.gx = -kG * std::sin(slope_rad);
    e.gy = -kG * std::cos(slope_rad);
    if (coriolis) {
        // Earth's rotation in local east/north/up is w*(0, cos(lat), sin(lat)).
        // Project onto: forward (horizontal, toward target), right, up; then tilt by the slope.
        const real d2r = kPi / static_cast<real>(180);
        const real lat = latitude_deg * d2r, az = azimuth_deg * d2r;
        const real w_f = kEarthRate * std::cos(lat) * std::cos(az);
        const real w_r = -kEarthRate * std::cos(lat) * std::sin(az);
        const real w_u = kEarthRate * std::sin(lat);
        const real cs = std::cos(slope_rad), sn = std::sin(slope_rad);
        e.ox = w_f * cs + w_u * sn;
        e.oy = -w_f * sn + w_u * cs;
        e.oz = w_r;
    }
    return e;
}

// ---- Spin drift (Litz) ----------------------------------------------------
// Miller stability factor, then drift = 1.25 (Sg + 1.2) t^1.83 inches (right for right-hand twist).
// Returns Sg, or 0 if the bullet data is incomplete.
inline real stability_factor(const Bullet& b, const Atmosphere& atm) {
    if (!(b.mass_gr > 0 && b.diameter_in > 0 && b.length_in > 0 && b.twist_in != 0)) return 0;
    const real d = b.diameter_in;
    const real t = std::fabs(b.twist_in) / d;        // calibres per turn
    const real l = b.length_in / d;                  // calibres
    real sg = static_cast<real>(30) * b.mass_gr / (t * t * d * d * d * l * (1 + l * l));
    const real fps = b.muzzle_velocity / kFpsToMps;
    sg *= std::cbrt(fps / static_cast<real>(2800));
    sg *= (atm.temp_c * static_cast<real>(1.8) + static_cast<real>(32) + static_cast<real>(460)) / static_cast<real>(519);
    sg *= static_cast<real>(29.92) / (atm.pressure_hpa / static_cast<real>(33.8639));
    return sg;
}

// Lateral spin drift in metres at time of flight `tof` (positive = to the right).
inline real spin_drift_m(const Bullet& b, const Atmosphere& atm, real tof) {
    const real sg = stability_factor(b, atm);
    if (!(sg > 0) || !(tof > 0)) return 0;
    const real inches = static_cast<real>(1.25) * (sg + static_cast<real>(1.2)) * std::pow(tof, static_cast<real>(1.83));
    return (b.twist_in > 0 ? 1 : -1) * inches * static_cast<real>(0.0254);
}

// ---- Aerodynamic jump (Litz) ----------------------------------------------
// Litz, Applied Ballistics for Long Range Shooting: a crosswind tips the spinning bullet's nose
// at the muzzle, which moves the impact vertically by
//     jump (MOA) = (0.01 Sg - 0.0024 L + 0.032) * crosswind (mph),   L = length in calibres.
// With a right-hand twist, wind from the right moves the impact DOWN and wind from the left moves
// it UP; a left-hand twist reverses this. Because it is an angle, the same MOA applies at every range.
constexpr real kMradPerMoa = static_cast<real>(0.290888);
constexpr real kMpsToMph = static_cast<real>(2.236936);

// Elevation correction in mrad (positive = dial up). `wind_clock_deg` is where the wind comes FROM.
inline real aero_jump_mrad(const Bullet& b, const Atmosphere& atm, real wind_speed, real wind_clock_deg) {
    const real sg = stability_factor(b, atm);
    if (!(sg > 0)) return 0;
    const real l = b.length_in / b.diameter_in;
    real y = static_cast<real>(0.01) * sg - static_cast<real>(0.0024) * l + static_cast<real>(0.032);
    if (y < 0) y = 0;                                   // the fit is only meaningful while positive
    // Crosswind from the right is positive.
    const real cross_mph = wind_speed * std::sin(wind_clock_deg * kPi / static_cast<real>(180)) * kMpsToMph;
    const real twist_sign = b.twist_in > 0 ? 1 : -1;
    // Right-hand twist + wind from right => impact low => correction up (+).
    return twist_sign * y * cross_mph * kMradPerMoa;
}

// Zero the rifle on flat ground; returns bore angle relative to sight line.
inline bool zero_angle(const Bullet& b, const Atmosphere& zero_atm, const Shot& shot,
                       real& alpha0) {
    const Env e = make_env(b, zero_atm, 0, 0, 0);
    Impact at;
    return solve_angle(e, b, shot.sight_height, shot.zero_range, alpha0, at);
}

// Full firing solution at `range` metres (along the line of sight).
// `zero_atm` are the conditions the rifle was zeroed in.
inline Solution solve(const Bullet& b, const Atmosphere& zero_atm,
                      const Atmosphere& atm, const Shot& shot, real range) {
    Solution out;
    out.elevation_mrad = out.windage_mrad = out.tof = out.velocity = out.drop = 0;
    out.ok = false;
    real alpha0;
    if (!zero_angle(b, zero_atm, shot, alpha0)) return out;

    // Solve elevation with the wind included (headwind/tailwind changes drop),
    // then read lateral drift from that same flight.
    const Env e_wind = make_env(b, atm, shot.wind_speed, shot.wind_clock_deg, shot.slope_rad,
                                shot.coriolis && range > kCoriolisMinRange, shot.latitude_deg, shot.azimuth_deg);
    real alpha_r;
    Impact at_wind;
    if (!solve_angle(e_wind, b, shot.sight_height, range, alpha_r, at_wind)) return out;

    const real aj = shot.aero_jump ? aero_jump_mrad(b, atm, shot.wind_speed, shot.wind_clock_deg) : 0;
    out.elevation_mrad = (alpha_r - alpha0) * static_cast<real>(1000) + aj;
    out.aero_jump_mrad = aj;
    const real sd = shot.spin_drift ? spin_drift_m(b, atm, at_wind.tof) : 0;
    out.windage_mrad = -((at_wind.z + sd) / range) * static_cast<real>(1000);
    out.spin_drift_mrad = -(sd / range) * static_cast<real>(1000);
    out.tof = at_wind.tof;
    out.velocity = at_wind.speed;
    out.drop = at_wind.y;
    out.ok = true;
    return out;
}

// ---- Truing -------------------------------------------------------------------
// Adjust the load to match what was seen on the range. Inputs are the come-up (mrad, as in
// Solution::elevation_mrad) that actually hit at a known range in known conditions.
//   1. true_muzzle_velocity: from one observed come-up at a mid range (300-600 yd), where drop is
//      mostly set by velocity, find the muzzle velocity that reproduces it.
//   2. true_bc: from one observed come-up past about 600 yd, with the trued velocity, find the
//      ballistic coefficient that reproduces it. A BC change is the same as a drop scale factor
//      here and keeps the existing config fields, so the trued load is just a new MV and BC.
// Each returns false (and leaves the bullet alone) if it cannot match the observation.
struct TruingShot {
    Atmosphere atm;       // conditions when the observed shot was fired
    Shot shot;            // sight height, zero range, wind, slope (switches as when observed)
    real range;           // m
    real observed_mrad;   // come-up that hit, mrad
};

namespace detail {
// Find x in [lo, hi] where f(x) = 0 using bisection plus secant steps. f must change sign.
template <typename F>
inline bool find_root(F f, real lo, real hi, real tol, real& x_out) {
    real flo = f(lo), fhi = f(hi);
    if (!std::isfinite(flo) || !std::isfinite(fhi) || flo * fhi > 0) return false;
    for (int it = 0; it < 60; ++it) {
        real x = hi - fhi * (hi - lo) / (fhi - flo);          // secant guess
        if (!(x > lo && x < hi)) x = (lo + hi) / 2;           // fall back to bisection
        const real fx = f(x);
        if (!std::isfinite(fx)) return false;
        if (std::fabs(fx) < tol || (hi - lo) < tol * static_cast<real>(1e-3)) { x_out = x; return true; }
        if ((fx < 0) == (flo < 0)) { lo = x; flo = fx; } else { hi = x; fhi = fx; }
        // Keep the bracket shrinking even when the secant hugs one end.
        const real mid = (lo + hi) / 2, fm = f(mid);
        if (!std::isfinite(fm)) return false;
        if ((fm < 0) == (flo < 0)) { lo = mid; flo = fm; } else { hi = mid; fhi = fm; }
    }
    x_out = (lo + hi) / 2;
    return true;
}

inline real come_up_error(const Bullet& b, const Atmosphere& zero_atm, const TruingShot& o) {
    const Solution s = solve(b, zero_atm, o.atm, o.shot, o.range);
    if (!s.ok) return static_cast<real>(NAN);
    return s.elevation_mrad - o.observed_mrad;
}
}  // namespace detail

// Come-up falls as velocity rises, so search between 60% and 140% of the current value.
inline bool true_muzzle_velocity(Bullet& b, const Atmosphere& zero_atm, const TruingShot& o,
                                 real tol_mrad = static_cast<real>(0.005)) {
    if (!(o.range > o.shot.zero_range) || !(o.observed_mrad > 0)) return false;
    Bullet t = b;
    auto f = [&](real mv) { t.muzzle_velocity = mv; return detail::come_up_error(t, zero_atm, o); };
    real mv;
    if (!detail::find_root(f, b.muzzle_velocity * static_cast<real>(0.6), b.muzzle_velocity * static_cast<real>(1.4), tol_mrad, mv)) return false;
    b.muzzle_velocity = mv;
    return true;
}

// Come-up falls as BC rises, so search between 40% and 250% of the current value.
inline bool true_bc(Bullet& b, const Atmosphere& zero_atm, const TruingShot& o,
                    real tol_mrad = static_cast<real>(0.005)) {
    if (!(o.range > o.shot.zero_range) || !(o.observed_mrad > 0)) return false;
    Bullet t = b;
    auto f = [&](real bc) { t.bc = bc; return detail::come_up_error(t, zero_atm, o); };
    real bc;
    if (!detail::find_root(f, b.bc * static_cast<real>(0.4), b.bc * static_cast<real>(2.5), tol_mrad, bc)) return false;
    b.bc = bc;
    return true;
}

// Both steps together. The BC change moves the mid-range point slightly, so alternate until
// both observations agree (a few passes is plenty).
inline bool true_load(Bullet& b, const Atmosphere& zero_atm, const TruingShot& mid, const TruingShot& far) {
    // MV and BC trade off against each other, so match each point tightly on every pass.
    const real tol = static_cast<real>(0.0005);
    Bullet t = b;
    for (int pass = 0; pass < 10; ++pass) {
        if (!true_muzzle_velocity(t, zero_atm, mid, tol)) return false;
        if (!true_bc(t, zero_atm, far, tol)) return false;
        const real e1 = detail::come_up_error(t, zero_atm, mid), e2 = detail::come_up_error(t, zero_atm, far);
        if (std::fabs(e1) < static_cast<real>(0.002) && std::fabs(e2) < static_cast<real>(0.002)) break;
    }
    b = t;
    return true;
}

}  // namespace ballistics
