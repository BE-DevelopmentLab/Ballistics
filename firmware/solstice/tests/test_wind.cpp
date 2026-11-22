// Checks core/wind_pulse.h: the analog impeller pulse detector.
// Signal levels match the bench (2026-11-16): DRV5055A1 flat on the Kestrel housing gives
// 55-71 mV peak-to-peak while spinning and 7-13 mV of swing at rest.
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include "../core/wind_pulse.h"

static int g_fail = 0;
static void check(bool ok, const char* name, double got, double want) {
    std::printf("[%s] %-60s got=%.3f want=%.3f\n", ok ? "PASS" : "FAIL", name, got, want);
    if (!ok) ++g_fail;
}
// Deterministic noise: roughly Gaussian (sum of 4 uniforms) with standard deviation `sd`, from a small LCG.
static uint32_t g_seed = 12345;
static float uni() { g_seed = g_seed * 1664525u + 1013904223u; return (g_seed >> 8) / 16777216.0f - 0.5f; }
static float noise(float sd) { return sd * 1.732f * (uni() + uni() + uni() + uni()); }

static const float FS = 2000.0f;   // samples per second (500 us)
static const float MID = 1650.0f;  // DRV5055 quiet output at 3.3 V
static double g_phase = 0;

// Runs `secs` of a sine at `rev_hz` revolutions/s, `pp_mv` peak-to-peak, around `mid_mv`.
static int run(wind::PulseDetector& d, float secs, float rev_hz, float pp_mv, float sd, float mid_mv = MID) {
    int pulses = 0;
    for (int i = 0; i < (int)(secs * FS); ++i) {
        g_phase += 2.0 * M_PI * rev_hz / FS;
        pulses += d.feed(mid_mv + 0.5f * pp_mv * (float)std::sin(g_phase) + noise(sd));
    }
    return pulses;
}

int main() {
    // 1. Bench signal (60 mVpp) with bench noise (sd 4 mV): 2 pulses per revolution.
    { wind::PulseDetector d; run(d, 1, 10, 60, 4); int p = run(d, 10, 10, 60, 4);
      check(std::abs(p - 200) <= 1, "10 rev/s, 60 mVpp, bench noise: 2 pulses/rev", p, 200); }
    // 2. Weakest seen while spinning (41 mVpp) with double the bench noise.
    { wind::PulseDetector d; run(d, 1, 10, 41, 8); int p = run(d, 10, 10, 41, 8);
      check(std::abs(p - 200) <= 2, "41 mVpp in 2x bench noise", p, 200); }
    // 3. Slow coast-down speed: 0.5 rev/s.
    { wind::PulseDetector d; run(d, 4, 0.5f, 60, 4); int p = run(d, 20, 0.5f, 60, 4);
      check(std::abs(p - 20) <= 1, "0.5 rev/s slow spin", p, 20); }
    // 4. Fast spin.
    { wind::PulseDetector d; run(d, 1, 200, 60, 4); int p = run(d, 5, 200, 60, 4);
      check(std::abs(p - 2000) <= 4, "200 rev/s fast spin", p, 2000); }
    // 5. Stopped impeller, noise only (2x bench), 10 minutes: no pulses.
    { wind::PulseDetector d; int p = run(d, 600, 0, 0, 8);
      check(p == 0, "stopped 10 min, 2x bench noise: no pulses", p, 0); }
    // 6. Spin then stop with the magnet parked off-centre: pulses end at the stop.
    { wind::PulseDetector d; run(d, 5, 10, 60, 4);
      int p = 0; for (int i = 0; i < (int)(60 * FS); ++i) p += d.feed(MID + 25.0f + noise(4));
      check(p <= 2, "after stop with magnet parked: at most 2 stray pulses", p, 0); }
    // 7. Offset drift (temperature): mid moves 150 mV over 20 s, still counts.
    { wind::PulseDetector d; run(d, 1, 5, 60, 4, 1600); int p = 0;
      for (int i = 0; i < (int)(20 * FS); ++i) { g_phase += 2 * M_PI * 5 / FS;
          p += d.feed(1600.0f + 150.0f * i / (20 * FS) + 30.0f * (float)std::sin(g_phase) + noise(4)); }
      check(std::abs(p - 200) <= 2, "baseline drifts 150 mV: still 2 pulses/rev", p, 200); }
    // 8. Rate change: fast then slow, counts follow.
    { wind::PulseDetector d; run(d, 2, 50, 60, 4); int p = run(d, 10, 3, 60, 4);
      check(std::abs(p - 60) <= 2, "50 rev/s then 3 rev/s: counts follow", p, 60); }
    // 9. Strong signal (a closer mount, 1 V pp) still counts exactly.
    { wind::PulseDetector d; run(d, 1, 20, 1000, 4); int p = run(d, 10, 20, 1000, 4);
      check(std::abs(p - 400) <= 1, "strong 1 Vpp signal: 2 pulses/rev", p, 400); }

    std::printf("%s (%d failed)\n", g_fail ? "FAIL" : "ALL PASS", g_fail);
    return g_fail ? 1 : 0;
}
