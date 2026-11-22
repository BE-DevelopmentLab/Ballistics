// Tiny CLI so the node test can cross-check the C++ and JS packet code.
//   proto_tool config <hex>      decode a config packet, print fields
//   proto_tool request <hex>     decode a request packet, print fields
//   proto_tool card <hex>        decode a range card request, print fields
//   proto_tool solution          print an encoded sample solution (hex)
//   proto_tool env               print an encoded sample env packet (hex)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include "protocol.h"

static std::vector<uint8_t> from_hex(const char* h) {
    std::vector<uint8_t> out;
    for (size_t i = 0; h[i] && h[i + 1]; i += 2) {
        char b[3] = {h[i], h[i + 1], 0};
        out.push_back(static_cast<uint8_t>(std::strtol(b, nullptr, 16)));
    }
    return out;
}
static void print_hex(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i) std::printf("%02x", p[i]);
    std::printf("\n");
}

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    const std::string cmd = argv[1];
    if (cmd == "config" && argc == 3) {
        auto d = from_hex(argv[2]);
        protocol::Config c;
        if (!protocol::decode_config(d.data(), d.size(), c)) { std::printf("REJECT\n"); return 1; }
        std::printf("%u %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", c.model, c.bc, c.muzzle_mps,
                    c.sight_height_m, c.zero_range_m, c.zero_temp_c, c.zero_pressure_hpa, c.zero_rh_pct,
                    c.mass_gr, c.diameter_m, c.length_m, c.twist_m);
        return 0;
    }
    if (cmd == "request" && argc == 3) {
        auto d = from_hex(argv[2]);
        protocol::Request r;
        if (!protocol::decode_request(d.data(), d.size(), r)) { std::printf("REJECT\n"); return 1; }
        std::printf("%u %.6f %.6f %.6f %.6f %.6f %.6f\n", r.flags, r.range_m, r.wind_clock_deg,
                    r.wind_speed_mps, r.slope_deg, r.latitude_deg, r.azimuth_deg);
        return 0;
    }
    if (cmd == "card" && argc == 3) {
        auto d = from_hex(argv[2]);
        protocol::CardRequest c;
        if (!protocol::decode_card(d.data(), d.size(), c)) { std::printf("REJECT\n"); return 1; }
        std::printf("%u %u %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", c.flags, c.count, c.start_m, c.step_m,
                    c.wind_clock_deg, c.wind_speed_mps, c.slope_deg, c.latitude_deg, c.azimuth_deg);
        return 0;
    }
    if (cmd == "solution") {
        protocol::SolutionPacket s{true, 6.15, -1.91, 1.066, 450.2, -3.21, 640.1, 4.47, -12, 7, -0.37};
        uint8_t buf[protocol::kSolutionSize];
        protocol::encode_solution(s, buf);
        print_hex(buf, sizeof buf);
        return 0;
    }
    if (cmd == "env") {
        protocol::EnvPacket e{-3.5, 62.25, 1002.7, 2.34, 271.4, -4.6, 412.0, 3, 87};
        uint8_t buf[protocol::kEnvSize];
        protocol::encode_env(e, buf);
        print_hex(buf, sizeof buf);
        return 0;
    }
    return 2;
}
