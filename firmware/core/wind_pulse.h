// Turns the analog output of a linear Hall sensor (DRV5055 / DRV5053) into impeller pulses.
//
// The impeller's diametric magnet makes the sensor output a wave around its quiet level as it spins.
// We track the wave's top and bottom with slowly decaying envelopes, then count a pulse every
// time the signal crosses from one side of the middle to the other, with hysteresis so noise
// near the middle can't double count. One revolution gives 2 pulses (up and down crossing),
// the same as the omnipolar DRV5033 it replaces, so WIND_MPS_PER_HZ keeps its meaning.
//
// Pure C++ with no Arduino calls, so it is unit tested on the PC (tests/test_wind.cpp).
#pragma once
#include <cstdint>

namespace wind {

struct PulseDetector {
    // Settings (millivolts and samples), sampled every 500 us. Tuned 2026-11-16 on the bench for the
    // DRV5055A1 mounted flat on the Kestrel housing: ~55-71 mV swing spinning, 7-13 mV at rest.
    float min_swing_mv = 30.0f;     // a wave smaller than this peak-to-peak is treated as no signal
    float hyst_frac = 0.25f;        // switch points at mid +/- this fraction of the swing...
    float min_band_mv = 12.0f;      // ...but never closer to the middle than this (noise margin)
    float decay = 0.9998f;          // envelope pull-in per sample (tau ~ 2.5 s at 2 kHz)
    int smooth = 5;                 // moving average length (2.5 ms): cuts noise ~2x, passes spins past 200 rev/s
    int min_gap = 2;                // samples between counted crossings (debounce)

    // State
    float hi = 0, lo = 0, avg = 0;
    float buf[16] = {0};
    int n = 0, idx = 0, since = 0;
    int side = 0;                   // +1 above the upper switch point, -1 below the lower, 0 unknown

    void reset() { n = 0; idx = 0; since = 0; side = 0; avg = hi = lo = 0; }

    float swing() const { return hi - lo; }

    // Feed one sample. Returns 1 when it completes a crossing (a pulse), else 0.
    int feed(float mv) {
        const int len = smooth < 1 ? 1 : (smooth > 16 ? 16 : smooth);
        if (n == 0) { for (int i = 0; i < len; ++i) buf[i] = mv; avg = hi = lo = mv; }
        avg += (mv - buf[idx]) / len;
        buf[idx] = mv;
        idx = (idx + 1) % len;
        ++n;
        const float x = avg;

        // Envelopes jump out to new extremes, then drift back toward the middle.
        const float mid0 = 0.5f * (hi + lo);
        hi = mid0 + (hi - mid0) * decay;
        lo = mid0 + (lo - mid0) * decay;
        if (x > hi) hi = x;
        if (x < lo) lo = x;

        if (since < 1000000) ++since;
        const float sw = hi - lo;
        if (sw < min_swing_mv) { side = 0; return 0; }
        const float mid = 0.5f * (hi + lo), band = hyst_frac * sw > min_band_mv ? hyst_frac * sw : min_band_mv;
        int now = side;
        if (x > mid + band) now = 1;
        else if (x < mid - band) now = -1;
        int pulse = 0;
        if (now != side) {
            if (side != 0 && since >= min_gap) { pulse = 1; since = 0; }
            side = now;
        }
        return pulse;
    }
};

}  // namespace wind
