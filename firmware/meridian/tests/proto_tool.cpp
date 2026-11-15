// Helper for test_protocol.py: prints C++-encoded packets as hex, and decodes hex given on the command line.
//   proto_tool encode            -> one line each: shot, position, status
//   proto_tool correction <hex>  -> decoded fields
//   proto_tool command <hex>     -> decoded fields
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "meridian_protocol.h"

static void hex(const char* name, const uint8_t* b, size_t n) {
    std::printf("%s ", name);
    for (size_t i = 0; i < n; ++i) std::printf("%02x", b[i]);
    std::printf("\n");
}
static std::vector<uint8_t> unhex(const char* s) {
    std::vector<uint8_t> v;
    for (size_t i = 0; s[i] && s[i + 1]; i += 2) { unsigned x; std::sscanf(s + i, "%2x", &x); v.push_back((uint8_t)x); }
    return v;
}
int main(int argc, char** argv) {
    if (argc >= 2 && !std::strcmp(argv[1], "encode")) {
        uint8_t b[20];
        meridian::Shot s; s.flags = 0x1B; s.battery_pct = 77; s.range_m = 612.3; s.range_age_s = 4.2; s.pitch_deg = -3.21;
        s.cant_deg = 1.57; s.bearing_deg = 271.43; s.temp_c = -4.25; s.pressure_hpa = 845.6; s.rh_pct = 33.3; s.range_seq = 200;
        s.mag_cal = 3; s.heading_acc_deg = 4;
        meridian::encode_shot(s, b); hex("shot", b, 20);
        meridian::Position p; p.lat_deg = 45.1234567; p.lon_deg = -111.7654321; p.alt_m = 1523; p.fix = 1; p.sats = 9;
        p.hdop = 0.87; p.utc = 1794772800; p.declination_deg = 11.25;
        meridian::encode_position(p, b); hex("position", b, 20);
        meridian::Status t; t.flags = 0x25; t.mount = 10; t.units = 1; t.pitch_zero = 0.81; t.cant_zero = -1.3; t.heading_offset = 93.5;
        t.declination = 11.5; t.laser_offset_m = 0.3; t.target_mode = 2; t.last_result = 0; t.last_corr_seq = 42; t.centrals = 2; t.laser = 1;
        meridian::encode_status(t, b); hex("status", b, 20);
        return 0;
    }
    if (argc >= 3 && !std::strcmp(argv[1], "correction")) {
        auto v = unhex(argv[2]); meridian::Correction c;
        if (!meridian::decode_correction(v.data(), v.size(), c)) { std::printf("invalid\n"); return 0; }
        std::printf("%u %.2f %.2f %.1f %.3f %.2f %u %u %.2f %.0f %u\n", c.flags, c.elevation_mrad, c.windage_mrad, c.range_m, c.tof_s,
                    c.lead_mrad, c.hit_pct, c.target, c.wind_mps, c.wind_from_deg, c.seq);
        return 0;
    }
    if (argc >= 3 && !std::strcmp(argv[1], "command")) {
        auto v = unhex(argv[2]); meridian::Command c;
        if (!meridian::decode_command(v.data(), v.size(), c)) { std::printf("invalid\n"); return 0; }
        std::printf("%u %.3f %u\n", c.cmd, c.f, c.u);
        return 0;
    }
    return 2;
}
