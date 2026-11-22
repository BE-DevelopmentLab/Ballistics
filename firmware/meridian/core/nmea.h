// Small NMEA 0183 parser for the PA1010D GPS (GGA and RMC sentences only).
// Fed one character at a time; checks the *hh checksum and ignores everything else.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace nmea {

struct Fix {
    bool valid = false;        // RMC status A and GGA quality > 0
    double lat_deg = 0, lon_deg = 0, alt_m = 0;
    uint8_t quality = 0, sats = 0;
    double hdop = 0;
    uint32_t utc = 0;          // Unix seconds, 0 until RMC has given a date
    uint32_t updates = 0;      // counts accepted GGA sentences
};

// Days from 1970-01-01 to y-m-d (proleptic Gregorian). From Howard Hinnant's days_from_civil.
inline int64_t days_from_civil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

// ddmm.mmmm + hemisphere -> signed degrees.
inline double to_degrees(const char* v, char hemi) {
    if (!v || !*v) return 0;
    const double raw = std::atof(v);
    const int deg = static_cast<int>(raw / 100);
    const double d = deg + (raw - deg * 100) / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -d : d;
}

class Parser {
  public:
    // Returns true when a sentence that changed the fix was accepted.
    bool feed(char c) {
        if (c == '$') { len_ = 0; in_ = true; return false; }
        if (!in_) return false;
        if (c == '\r' || c == '\n') { in_ = false; return finish(); }
        if (len_ >= sizeof(buf_) - 1) { in_ = false; return false; }
        buf_[len_++] = c;
        return false;
    }
    const Fix& fix() const { return fix_; }
    uint32_t badChecksums() const { return bad_; }

  private:
    static int hexv(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    }
    bool finish() {
        buf_[len_] = 0;
        char* star = std::strchr(buf_, '*');
        if (!star || star[1] == 0 || star[2] == 0) { ++bad_; return false; }
        uint8_t sum = 0;
        for (char* p = buf_; p < star; ++p) sum ^= static_cast<uint8_t>(*p);
        const int h = hexv(star[1]), l = hexv(star[2]);
        if (h < 0 || l < 0 || sum != (h << 4 | l)) { ++bad_; return false; }
        *star = 0;
        // Split on commas, keeping empty fields.
        char* f[24]; int n = 0;
        f[n++] = buf_;
        for (char* p = buf_; *p && n < 24; ++p) if (*p == ',') { *p = 0; f[n++] = p + 1; }
        if (std::strlen(f[0]) < 5) return false;
        const char* type = f[0] + 2;   // skip talker ID (GP, GN, ...)
        if (std::strcmp(type, "GGA") == 0 && n >= 10) return gga(f);
        if (std::strcmp(type, "RMC") == 0 && n >= 10) return rmc(f);
        return false;
    }
    bool gga(char** f) {
        // 1 time, 2 lat, 3 N/S, 4 lon, 5 E/W, 6 quality, 7 sats, 8 hdop, 9 alt
        fix_.quality = static_cast<uint8_t>(std::atoi(f[6]));
        fix_.sats = static_cast<uint8_t>(std::atoi(f[7]));
        fix_.hdop = std::atof(f[8]);
        if (fix_.quality > 0 && *f[2] && *f[4]) {
            fix_.lat_deg = to_degrees(f[2], *f[3]);
            fix_.lon_deg = to_degrees(f[4], *f[5]);
            fix_.alt_m = std::atof(f[9]);
        }
        fix_.valid = fix_.quality > 0 && rmcOk_;
        ++fix_.updates;
        return true;
    }
    bool rmc(char** f) {
        // 1 time hhmmss.ss, 2 status A/V, 3 lat, 4 N/S, 5 lon, 6 E/W, 7 sog, 8 cog, 9 date ddmmyy
        rmcOk_ = (*f[2] == 'A');
        const char* t = f[1]; const char* d = f[9];
        if (std::strlen(t) >= 6 && std::strlen(d) == 6) {
            auto two = [](const char* s) { return (s[0] - '0') * 10 + (s[1] - '0'); };
            const int yy = two(d + 4);
            const int64_t days = days_from_civil(yy < 80 ? 2000 + yy : 1900 + yy, two(d + 2), two(d));
            fix_.utc = static_cast<uint32_t>(days * 86400 + two(t) * 3600 + two(t + 2) * 60 + two(t + 4));
        }
        if (rmcOk_ && *f[3] && *f[5]) {
            fix_.lat_deg = to_degrees(f[3], *f[4]);
            fix_.lon_deg = to_degrees(f[5], *f[6]);
        }
        fix_.valid = rmcOk_ && fix_.quality > 0;
        return true;
    }
    char buf_[100] = {};
    size_t len_ = 0;
    bool in_ = false, rmcOk_ = false;
    Fix fix_;
    uint32_t bad_ = 0;
};

}  // namespace nmea
