// Prints reference sensitivities from the real solver, as JSON lines, for tests/test_hitprob.js.
#include <cstdio>
#include "ballistics.h"
using namespace ballistics;
int main() {
    struct C { DragModel m; double bc, mv, range; } cases[] = {
        {DragModel::G7, 0.176, 780, 457.2}, {DragModel::G7, 0.176, 780, 731.5}, {DragModel::G7, 0.176, 780, 914.4},
        {DragModel::G7, 0.243, 800, 914.4}, {DragModel::G1, 0.475, 800, 548.6}, {DragModel::G7, 0.300, 870, 1200},
        {DragModel::G7, 0.176, 780, 182.9},
    };
    const Atmosphere atm{15, 1013.25, 50};
    for (const C& c : cases) {
        Bullet b{static_cast<real>(c.bc), c.m, static_cast<real>(c.mv)};
        Shot s{}; s.sight_height = 0.0381; s.zero_range = 91.44; s.wind_speed = 0; s.wind_clock_deg = 90; s.slope_rad = 0;
        auto el = [&](const Bullet& bb, real r) { return solve(bb, atm, atm, s, r).elevation_mrad; };
        const Solution base = solve(b, atm, atm, s, c.range);
        Bullet bp = b, bm = b; bp.muzzle_velocity += 5; bm.muzzle_velocity -= 5;
        Bullet bh = b, bl = b; bh.bc = static_cast<real>(c.bc * 1.01); bl.bc = static_cast<real>(c.bc * 0.99);
        Shot sw = s; sw.wind_speed = 1;
        const Solution w = solve(b, atm, atm, sw, c.range);
        std::printf("{\"mv\":%g,\"range\":%g,\"tof\":%.5f,\"vel\":%.3f,\"dR\":%.6f,\"dMv\":%.6f,\"dWind\":%.6f,\"dBc\":%.6f,\"bc\":%g,\"g1\":%d}\n",
            c.mv, c.range, (double)base.tof, (double)base.velocity,
            (double)((el(b, c.range + 5) - el(b, c.range - 5)) / 10), (double)((el(bp, c.range) - el(bm, c.range)) / 10),
            (double)w.windage_mrad, (double)((el(bh, c.range) - el(bl, c.range)) / 2), c.bc, c.m == DragModel::G1 ? 1 : 0);
    }
}
