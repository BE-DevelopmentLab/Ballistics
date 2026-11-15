// What Meridian's 240x240 screen says, worked out without any display code so it can be tested on a PC.
// firmware/src/main.cpp draws these strings; it makes no decisions of its own.
#pragma once

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>

namespace screen {

constexpr double kMoaPerMrad = 3.43774677;
constexpr double kCantWarnDeg = 1.0;        // cant bar turns red beyond this
constexpr double kCorrStaleS = 120.0;       // spotter correction older than this is no longer shown
constexpr double kRangeOldS = 30.0;         // range age shown once older than this

enum Source : uint8_t { kNone = 0, kSpotter = 1, kLocal = 2 };
enum Color : uint8_t { kWhite = 0, kGreen, kAmber, kRed, kGrey, kCyan };

struct Input {
    // Spotter correction (if any)
    bool corr_valid = false;   // a valid correction has been received
    double corr_age_s = 1e9;
    double corr_elev = 0, corr_wind = 0, corr_range = 0, corr_lead = 0;
    bool corr_has_lead = false;
    uint8_t corr_hit = 255, corr_target = 0;
    // Meridian's own solution
    bool local_ok = false;
    double local_elev = 0, local_wind = 0, local_range = 0;
    // Laser
    bool range_valid = false, ranging = false, laser_fault = false, laser_no_target = false;
    double range_m = 0, range_age_s = 0;
    // Rifle
    bool imu_ok = false;
    double cant_deg = 0, pitch_deg = 0, bearing_deg = 0;
    // Status
    uint8_t units = 0;           // 0 mil, 1 MOA
    uint8_t battery_pct = 255;
    bool charging = false;
    uint8_t centrals = 0;        // phone/spotter connections
    bool solstice = false;       // getting wind from Solstice
    const char* message = "";
};

struct Model {
    Source source = kNone;
    char title[16] = {};   Color title_color = kGrey;
    char elev[16] = {};    char wind[16] = {};   Color dial_color = kWhite;
    char range[24] = {};   Color range_color = kWhite;
    char extra[24] = {};   // lead / hit chance / target
    char angles[24] = {};  // pitch and bearing
    char status[24] = {};  // battery, links
    char message[24] = {};
    double cant_deg = 0;   bool cant_ok = false;  Color cant_color = kGreen;
};

inline void fmt_dir(char* out, size_t n, double mrad, uint8_t units, char pos, char neg) {
    const double v = units == 1 ? mrad * kMoaPerMrad : mrad;
    const double a = std::fabs(v);
    if (a < 0.05) std::snprintf(out, n, "0.0");
    else std::snprintf(out, n, "%c %.1f", v > 0 ? pos : neg, a);
}

inline Model build(const Input& in) {
    Model m;
    const bool corrShow = in.corr_valid && in.corr_age_s <= kCorrStaleS;
    const char* u = in.units == 1 ? "MOA" : "MIL";
    if (corrShow) {
        m.source = kSpotter;
        std::snprintf(m.title, sizeof m.title, "SPOTTER %s", u);
        m.title_color = kGreen;
        fmt_dir(m.elev, sizeof m.elev, in.corr_elev, in.units, 'U', 'D');
        fmt_dir(m.wind, sizeof m.wind, in.corr_wind, in.units, 'R', 'L');
        // A correction worked out for a different range than the last laser reading is suspect.
        const bool mismatch = in.range_valid && in.corr_range > 0 && std::fabs(in.corr_range - in.range_m) > 5.0;
        m.dial_color = (in.corr_age_s > 30.0 || mismatch) ? kAmber : kWhite;
        int k = 0;
        if (in.corr_has_lead) {
            char l[16]; fmt_dir(l, sizeof l, in.corr_lead, in.units, 'R', 'L');
            k += std::snprintf(m.extra + k, sizeof m.extra - k, "LEAD %s ", l);
        }
        if (in.corr_hit <= 100 && k < (int)sizeof m.extra) k += std::snprintf(m.extra + k, sizeof m.extra - k, "HIT %u%% ", in.corr_hit);
        if (in.corr_target && k < (int)sizeof m.extra) std::snprintf(m.extra + k, sizeof m.extra - k, "T%u", in.corr_target);
        if (mismatch && !in.corr_has_lead && in.corr_hit > 100) std::snprintf(m.extra, sizeof m.extra, "FOR %.0f m", in.corr_range);
    } else if (in.local_ok) {
        m.source = kLocal;
        std::snprintf(m.title, sizeof m.title, "MERIDIAN %s", u);
        m.title_color = kCyan;
        fmt_dir(m.elev, sizeof m.elev, in.local_elev, in.units, 'U', 'D');
        fmt_dir(m.wind, sizeof m.wind, in.local_wind, in.units, 'R', 'L');
    } else {
        std::snprintf(m.title, sizeof m.title, "NO SOLUTION");
        std::snprintf(m.elev, sizeof m.elev, "--");
        std::snprintf(m.wind, sizeof m.wind, "--");
        m.dial_color = kGrey;
    }

    if (in.ranging && !in.range_valid) { std::snprintf(m.range, sizeof m.range, "RANGING..."); m.range_color = kCyan; }
    else if (in.laser_fault) { std::snprintf(m.range, sizeof m.range, "LASER FAULT"); m.range_color = kRed; }
    else if (in.laser_no_target) { std::snprintf(m.range, sizeof m.range, "NO TARGET"); m.range_color = kAmber; }
    else if (in.range_valid) {
        if (in.range_age_s > kRangeOldS)
            std::snprintf(m.range, sizeof m.range, "%.0f m %.0fs", in.range_m, in.range_age_s);
        else
            std::snprintf(m.range, sizeof m.range, "%.0f m", in.range_m);
        m.range_color = in.range_age_s > kRangeOldS ? kAmber : kWhite;
    } else { std::snprintf(m.range, sizeof m.range, "NO RANGE"); m.range_color = kGrey; }

    if (in.imu_ok) {
        std::snprintf(m.angles, sizeof m.angles, "P %+.1f  BRG %03.0f", in.pitch_deg, std::fmod(std::round(in.bearing_deg), 360.0));
        m.cant_deg = in.cant_deg; m.cant_ok = true;
        m.cant_color = std::fabs(in.cant_deg) > kCantWarnDeg ? kRed : kGreen;
    } else {
        std::snprintf(m.angles, sizeof m.angles, "NO IMU");
        m.cant_color = kGrey;
    }

    char batt[8];
    if (in.battery_pct <= 100) std::snprintf(batt, sizeof batt, "%u%%%s", in.battery_pct, in.charging ? "+" : "");
    else std::snprintf(batt, sizeof batt, "--%%");
    std::snprintf(m.status, sizeof m.status, "%s BT%u%s", batt, in.centrals, in.solstice ? " WIND" : "");
    std::snprintf(m.message, sizeof m.message, "%s", in.message ? in.message : "");
    return m;
}

}  // namespace screen
