// Host-side checks for the solver. Build twice (float / double) and compare.
//   g++ -std=c++17 -O2 -I../core -DBALLISTICS_REAL=double test_solver.cpp -o t_double
//   g++ -std=c++17 -O2 -I../core -DBALLISTICS_REAL=float  test_solver.cpp -o t_float
#include <cstdio>
#include <cmath>
#include <initializer_list>
#include "ballistics.h"

using namespace ballistics;

static int g_fail = 0;
static void check(bool ok, const char* name, double got, double want) {
    std::printf("[%s] %-46s got=%.5f want=%.5f\n", ok ? "PASS" : "FAIL", name, got, want);
    if (!ok) ++g_fail;
}
static bool near_rel(double a, double b, double rel) {
    return std::fabs(a - b) <= rel * std::fabs(b);
}

static Atmosphere std_atm() { return {15.0f, 1013.25f, 0.0f}; }

int main() {
    std::printf("real = %s (%zu bytes)\n\n", sizeof(real) == 8 ? "double" : "float", sizeof(real));

    // 1. Vacuum limit: huge BC => no drag => closed-form come-up.
    {
        Bullet b{1.0e9f, DragModel::G1, 800.0f};
        Shot s{0.04f, 100.0f, 0, 0, 0};
        const double h = s.sight_height, v = b.muzzle_velocity, g = kG;
        const double R = 500.0, R0 = s.zero_range;
        const double want = (h / R + g * R / (2 * v * v) - (h / R0 + g * R0 / (2 * v * v))) * 1000.0;
        Solution sol = solve(b, std_atm(), std_atm(), s, R);
        check(sol.ok && near_rel(sol.elevation_mrad, want, 0.01),
              "vacuum come-up 100m zero -> 500m (mrad)", sol.elevation_mrad, want);
        check(sol.ok && near_rel(sol.tof, R / v, 0.01), "vacuum time of flight (s)", sol.tof, R / v);
        check(sol.ok && std::fabs(sol.windage_mrad) < 1e-3, "vacuum crosswind => zero drift",
              sol.windage_mrad, 0.0);
    }

    // 2. Rifle hits point of aim at its own zero range.
    {
        Bullet b{0.462f, DragModel::G1, 2650.0f * kFpsToMps};
        Shot s{0.0381f, 100.0f * kYdToM, 0, 0, 0};
        Solution sol = solve(b, std_atm(), std_atm(), s, s.zero_range);
        check(sol.ok && std::fabs(sol.elevation_mrad) < 0.01, "come-up at zero range is 0 (mrad)",
              sol.elevation_mrad, 0.0);
    }

    // 3. Wind-lag rule: drift = w * (TOF - R/v0), exact when drag depends on speed only.
    {
        Bullet b{0.243f, DragModel::G7, 2600.0f * kFpsToMps};
        Shot s{0.0381f, 100.0f * kYdToM, 4.4704f /*10 mph*/, 90.0f, 0};
        const double R = 600.0;
        Solution sol = solve(b, std_atm(), std_atm(), s, R);
        const double drift_m = sol.windage_mrad / 1000.0 * R;   // correction magnitude
        const double want = s.wind_speed * (sol.tof - R / b.muzzle_velocity);
        check(sol.ok && near_rel(drift_m, want, 0.03), "wind drift matches lag-time rule (m)", drift_m, want);
        check(sol.windage_mrad > 0, "wind from right => dial right (positive)", sol.windage_mrad, 1.0);
    }

    // 4. Come-up grows with range.
    {
        Bullet b{0.462f, DragModel::G1, 2650.0f * kFpsToMps};
        Shot s{0.0381f, 100.0f * kYdToM, 0, 0, 0};
        double prev = -1e9;
        bool mono = true;
        for (int y = 200; y <= 1000; y += 100) {
            Solution sol = solve(b, std_atm(), std_atm(), s, y * kYdToM);
            if (!sol.ok || sol.elevation_mrad <= prev) mono = false;
            prev = sol.elevation_mrad;
        }
        check(mono, "come-up strictly increases 200..1000 yd", mono, 1);
    }

    // 5. Thinner air (high altitude, hot) => less come-up.
    {
        Bullet b{0.462f, DragModel::G1, 2650.0f * kFpsToMps};
        Shot s{0.0381f, 100.0f * kYdToM, 0, 0, 0};
        const real R = 800.0f * kYdToM;
        Solution sea = solve(b, std_atm(), std_atm(), s, R);
        Solution high = solve(b, std_atm(), Atmosphere{25.0f, 800.0f, 30.0f}, s, R);
        check(sea.ok && high.ok && high.elevation_mrad < sea.elevation_mrad,
              "thin air needs less come-up (mrad)", high.elevation_mrad, sea.elevation_mrad);
    }

    // 6. Steep uphill shot needs less come-up than flat (cosine effect).
    {
        Bullet b{0.462f, DragModel::G1, 2650.0f * kFpsToMps};
        Shot flat{0.0381f, 100.0f * kYdToM, 0, 0, 0};
        Shot up = flat; up.slope_rad = 30.0f * kPi / 180.0f;
        const real R = 700.0f * kYdToM;
        Solution a = solve(b, std_atm(), std_atm(), flat, R);
        Solution c = solve(b, std_atm(), std_atm(), up, R);
        check(a.ok && c.ok && c.elevation_mrad < a.elevation_mrad,
              "30 deg slope needs less come-up (mrad)", c.elevation_mrad, a.elevation_mrad);
    }

    // Reference-style table for eyeballing against JBM / a Kestrel.
    std::printf("\n.308 168gr, G1 0.462, 2650 fps, 100 yd zero, 1.5 in sight, std atmosphere,\n"
                "10 mph full-value wind from 3 o'clock\n");
    std::printf("%6s %9s %9s %9s %8s %8s\n", "yd", "up(mil)", "up(MOA)", "wind(mil)", "tof(s)", "v(fps)");
    {
        Bullet b{0.462f, DragModel::G1, 2650.0f * kFpsToMps};
        Shot s{0.0381f, 100.0f * kYdToM, 4.4704f, 90.0f, 0};
        for (int y = 100; y <= 1000; y += 100) {
            Solution sol = solve(b, std_atm(), std_atm(), s, y * kYdToM);
            std::printf("%6d %9.2f %9.2f %9.2f %8.3f %8.0f\n", y, (double)sol.elevation_mrad,
                        sol.elevation_mrad * 3.4377, (double)sol.windage_mrad, (double)sol.tof,
                        sol.velocity / kFpsToMps);
        }
    }

    // ---- spin drift ----
    {
        // .308 175 gr, 1:10, 2600 fps: Miller Sg ~1.6 at standard conditions
        Bullet b{0.243f, DragModel::G7, 2600.0f * kFpsToMps, 175.0f, 0.308f, 1.24f, 10.0f};
        const double sg = stability_factor(b, std_atm());
        check(sg > 1.8 && sg < 3.0, "Miller stability factor plausible (.308 175gr 1:10, ~1.5 at 1:12)", sg, 2.4);
        // hand-computed: Sg*(T+460)/519 etc. at 15 C / 1013.25 hPa
        const double t = 10.0 / 0.308, l = 1.24 / 0.308;
        double want = 30.0 * 175.0 / (t * t * 0.308 * 0.308 * 0.308 * l * (1 + l * l));
        want *= std::cbrt(2600.0 / 2800.0) * ((59.0 + 460.0) / 519.0) * (29.92 / (1013.25 / 33.8639));
        check(near_rel(sg, want, 1e-3), "Miller formula matches hand calculation", sg, want);
        Shot s{0.0381f, 100.0f * kYdToM, 0.0f, 0.0f, 0};
        const real R = 1000.0f * kYdToM;
        Solution off = solve(b, std_atm(), std_atm(), s, R);
        s.spin_drift = true;
        Solution on = solve(b, std_atm(), std_atm(), s, R);
        const double drift_in = (-(on.windage_mrad - off.windage_mrad) * 1e-3 * R) / 0.0254;
        const double lit = 1.25 * (sg + 1.2) * std::pow(off.tof, 1.83);
        check(off.ok && on.ok && near_rel(drift_in, lit, 0.01), "spin drift matches Litz formula (in)", drift_in, lit);
        check(drift_in > 6 && drift_in < 16, "1000 yd spin drift is a believable 6-16 in", drift_in, 11);
        check(on.windage_mrad < off.windage_mrad, "right-hand twist: hold left (windage negative)", on.windage_mrad, off.windage_mrad);
        check(std::fabs(on.spin_drift_mrad - (on.windage_mrad - off.windage_mrad)) < 1e-3, "spin_drift_mrad reports the added part", on.spin_drift_mrad, on.windage_mrad - off.windage_mrad);
        check(std::fabs(on.elevation_mrad - off.elevation_mrad) < 1e-4, "spin drift does not change elevation", on.elevation_mrad, off.elevation_mrad);
        Bullet lh = b; lh.twist_in = -10.0f;
        Solution l2 = solve(lh, std_atm(), std_atm(), s, R);
        check(l2.windage_mrad > off.windage_mrad, "left-hand twist drifts the other way", l2.windage_mrad, off.windage_mrad);
        Bullet none{0.243f, DragModel::G7, 2600.0f * kFpsToMps};
        Solution n2 = solve(none, std_atm(), std_atm(), s, R);
        check(std::fabs(n2.windage_mrad - off.windage_mrad) < 1e-5 && n2.spin_drift_mrad == 0, "no bullet data => no spin drift", n2.windage_mrad, off.windage_mrad);
    }

    // ---- Coriolis (very high BC so drag is negligible and the textbook formulas apply) ----
    {
        Bullet b{1000.0f, DragModel::G1, 800.0f};
        const double w = 7.2921159e-5;
        const real R = 1000.0f;
        auto run = [&](double lat, double az, bool on) {
            Shot s{0.0381f, 100.0f, 0.0f, 0.0f, 0};
            s.coriolis = on; s.latitude_deg = (real)lat; s.azimuth_deg = (real)az;
            return solve(b, std_atm(), std_atm(), s, R);
        };
        Solution base = run(45, 0, false);
        // Northern hemisphere deflects right by w*sin(lat)*R*T whatever the heading.
        for (double az : {0.0, 90.0, 217.0}) {
            Solution c = run(45, az, true);
            const double got = -(c.windage_mrad - base.windage_mrad) * 1e-3 * R;   // metres, right +
            const double want = w * std::sin(45 * 3.14159265358979323846 / 180) * R * base.tof;
            check(c.ok && near_rel(got, want, 0.03), "Coriolis horizontal: w sin(lat) R T (m, right)", got, want);
        }
        // Eotvos: firing east at the equator lifts the bullet by w*R*T, so you dial DOWN.
        Solution east = run(0, 90, true), eq0 = run(0, 90, false);
        const double dy = -(east.elevation_mrad - eq0.elevation_mrad) * 1e-3 * R;    // metres the bullet lands high
        check(near_rel(dy, w * R * eq0.tof, 0.05), "Eotvos east at equator: lands high by w R T (m)", dy, w * R * eq0.tof);
        Solution west = run(0, 270, true);
        check((west.elevation_mrad - eq0.elevation_mrad) > 0, "firing west at the equator => dial UP", west.elevation_mrad - eq0.elevation_mrad, 1);
        Solution south = run(-45, 0, true);
        check(-(south.windage_mrad - base.windage_mrad) < 0, "southern hemisphere deflects LEFT", -(south.windage_mrad - base.windage_mrad), -1);
        Solution north0 = run(0, 0, true);
        check(std::fabs(north0.windage_mrad - base.windage_mrad) < 0.01, "no horizontal Coriolis at the equator firing north", north0.windage_mrad - base.windage_mrad, 0);
        // Only applied beyond 1000 yd (914.4 m): at or below it the answer is identical to Coriolis off.
        {
            Shot sh{0.0381f, 100.0f, 0.0f, 0.0f, 0};
            sh.coriolis = true; sh.latitude_deg = 45; sh.azimuth_deg = 90;
            Shot off = sh; off.coriolis = false;
            Solution a = solve(b, std_atm(), std_atm(), sh, kCoriolisMinRange), a0 = solve(b, std_atm(), std_atm(), off, kCoriolisMinRange);
            Solution c = solve(b, std_atm(), std_atm(), sh, 920.0f), c0 = solve(b, std_atm(), std_atm(), off, 920.0f);
            check(a.windage_mrad == a0.windage_mrad && a.elevation_mrad == a0.elevation_mrad, "no Coriolis at exactly 1000 yd", a.windage_mrad, a0.windage_mrad);
            check(c.windage_mrad != c0.windage_mrad, "Coriolis applies just past 1000 yd", c.windage_mrad, c0.windage_mrad);
        }
        // uphill slope must not break it
        Shot s{0.0381f, 100.0f, 0.0f, 0.0f, 0.3f};
        s.coriolis = true; s.latitude_deg = 45; s.azimuth_deg = 0;
        Solution sl = solve(b, std_atm(), std_atm(), s, R);
        check(sl.ok, "Coriolis on a 17 degree slope still solves", sl.ok, 1);
    }


    // ---- spin drift: Litz power law and Miller atmosphere correction ----
    {
        // Owner's load: Barnes 77 gr .223 OTM BT, G7 0.176, 0.224 x 0.990 in, 1:8 twist.
        Bullet b{0.176f, DragModel::G7, 2600.0f * kFpsToMps, 77.0f, 0.224f, 0.990f, 8.0f};
        const double sg = stability_factor(b, std_atm());
        check(sg > 1.4 && sg < 2.2, "Miller Sg for 77 gr .223 at 1:8 is plausible", sg, 1.75);
        Atmosphere hot{35.0f, 1013.25f, 0.0f}, thin{15.0f, 850.0f, 0.0f};
        check(stability_factor(b, hot) > sg, "warmer air raises Sg (Miller temperature term)", stability_factor(b, hot), sg);
        check(stability_factor(b, thin) > sg, "thinner air raises Sg (Miller pressure term)", stability_factor(b, thin), sg);
        Bullet slow = b; slow.muzzle_velocity *= 0.9f;
        check(stability_factor(slow, std_atm()) < sg, "lower velocity lowers Sg (Miller velocity term)", stability_factor(slow, std_atm()), sg);
        // Drift grows with time of flight to the 1.83 power.
        const double d1 = spin_drift_m(b, std_atm(), 0.5f), d2 = spin_drift_m(b, std_atm(), 1.0f);
        check(near_rel(d2 / d1, std::pow(2.0, 1.83), 1e-3), "spin drift scales as TOF^1.83", d2 / d1, std::pow(2.0, 1.83));
        check(spin_drift_m(b, std_atm(), 0.0f) == 0, "no time of flight => no spin drift", spin_drift_m(b, std_atm(), 0.0f), 0);
    }

    // ---- aerodynamic jump (Litz) ----
    {
        Bullet b{0.243f, DragModel::G7, 2600.0f * kFpsToMps, 175.0f, 0.308f, 1.24f, 10.0f};
        const double sg = stability_factor(b, std_atm());
        const double y = 0.01 * sg - 0.0024 * (1.24 / 0.308) + 0.032;     // MOA per mph
        const double want = y * 10.0 * 0.290888;                           // 10 mph full value, mrad
        Shot s{0.0381f, 100.0f * kYdToM, 4.4704f /*10 mph*/, 90.0f /*from right*/, 0};
        const real R = 600.0f * kYdToM;
        Solution off = solve(b, std_atm(), std_atm(), s, R);
        s.aero_jump = true;
        Solution on = solve(b, std_atm(), std_atm(), s, R);
        check(on.ok && near_rel(on.aero_jump_mrad, want, 1e-3), "aero jump matches Litz formula (mrad)", on.aero_jump_mrad, want);
        check(on.aero_jump_mrad > 0, "RH twist, wind from right => impact low => dial up", on.aero_jump_mrad, 1);
        check(std::fabs((on.elevation_mrad - off.elevation_mrad) - on.aero_jump_mrad) < 1e-4, "aero jump is added to elevation", on.elevation_mrad - off.elevation_mrad, on.aero_jump_mrad);
        check(std::fabs(on.windage_mrad - off.windage_mrad) < 1e-5, "aero jump does not change windage", on.windage_mrad, off.windage_mrad);
        check(off.aero_jump_mrad == 0, "switch off => no aero jump", off.aero_jump_mrad, 0);
        Shot left = s; left.wind_clock_deg = 270.0f;
        Solution lw = solve(b, std_atm(), std_atm(), left, R);
        check(near_rel(lw.aero_jump_mrad, -want, 1e-3), "wind from left => dial down", lw.aero_jump_mrad, -want);
        Bullet lh = b; lh.twist_in = -10.0f;
        Solution lt = solve(lh, std_atm(), std_atm(), s, R);
        check(near_rel(lt.aero_jump_mrad, -want, 1e-3), "left-hand twist reverses aero jump", lt.aero_jump_mrad, -want);
        Shot head = s; head.wind_clock_deg = 0.0f;
        Solution hw = solve(b, std_atm(), std_atm(), head, R);
        check(std::fabs(hw.aero_jump_mrad) < 1e-5, "headwind => no aero jump", hw.aero_jump_mrad, 0);
        Shot half = s; half.wind_clock_deg = 30.0f;    // 1 o'clock: half value
        Solution hv = solve(b, std_atm(), std_atm(), half, R);
        check(near_rel(hv.aero_jump_mrad, want * 0.5, 1e-3), "1 o'clock wind => half value", hv.aero_jump_mrad, want * 0.5);
        Solution far = solve(b, std_atm(), std_atm(), s, 1000.0f * kYdToM);
        check(near_rel(far.aero_jump_mrad, on.aero_jump_mrad, 1e-5), "aero jump is the same angle at any range", far.aero_jump_mrad, on.aero_jump_mrad);
        Bullet none{0.243f, DragModel::G7, 2600.0f * kFpsToMps};
        Solution nb = solve(none, std_atm(), std_atm(), s, R);
        check(nb.aero_jump_mrad == 0, "no bullet data => no aero jump", nb.aero_jump_mrad, 0);
        check(want > 0.05 && want < 0.6, "10 mph aero jump is a believable 0.2-2 MOA", want / 0.290888, 0.7);
    }

    // ---- truing from observed come-ups (synthetic data) ----
    {
        // "True" load the rifle actually shoots, and a wrong starting guess from the box.
        const Bullet truth{0.176f, DragModel::G7, 2560.0f * kFpsToMps, 77.0f, 0.224f, 0.990f, 8.0f};
        const Atmosphere zero_atm = std_atm();
        const Atmosphere field{24.0f, 985.0f, 40.0f};
        Shot s{0.0445f /*1.75 in*/, 100.0f * kYdToM, 0, 0, 0};
        TruingShot mid{field, s, 400.0f * kYdToM, 0};
        TruingShot far{field, s, 800.0f * kYdToM, 0};
        mid.observed_mrad = solve(truth, zero_atm, field, s, mid.range).elevation_mrad;
        far.observed_mrad = solve(truth, zero_atm, field, s, far.range).elevation_mrad;

        Bullet mv_only{0.176f, DragModel::G7, 2675.0f * kFpsToMps, 77.0f, 0.224f, 0.990f, 8.0f};
        const bool ok1 = true_muzzle_velocity(mv_only, zero_atm, mid);
        check(ok1 && near_rel(mv_only.muzzle_velocity, truth.muzzle_velocity, 0.002), "truing MV from one 400 yd come-up recovers MV (m/s)", mv_only.muzzle_velocity, truth.muzzle_velocity);

        Bullet bc_only{0.200f, DragModel::G7, truth.muzzle_velocity, 77.0f, 0.224f, 0.990f, 8.0f};
        const bool ok2 = true_bc(bc_only, zero_atm, far);
        check(ok2 && near_rel(bc_only.bc, truth.bc, 0.01), "truing BC from one 800 yd come-up recovers BC", bc_only.bc, truth.bc);

        Bullet guess{0.200f, DragModel::G7, 2675.0f * kFpsToMps, 77.0f, 0.224f, 0.990f, 8.0f};
        const bool ok3 = true_load(guess, zero_atm, mid, far);
        check(ok3 && near_rel(guess.muzzle_velocity, truth.muzzle_velocity, 0.001), "true_load recovers MV with a wrong BC to start", guess.muzzle_velocity / kFpsToMps, truth.muzzle_velocity / kFpsToMps);
        check(ok3 && near_rel(guess.bc, truth.bc, 0.005), "true_load recovers BC with a wrong MV to start", guess.bc, truth.bc);
        const Solution chk = solve(guess, zero_atm, field, s, 600.0f * kYdToM);
        const Solution ref = solve(truth, zero_atm, field, s, 600.0f * kYdToM);
        check(std::fabs(chk.elevation_mrad - ref.elevation_mrad) < 0.02, "trued load matches truth at 600 yd (mrad)", chk.elevation_mrad, ref.elevation_mrad);

        Bullet keep = guess;
        TruingShot bad = mid; bad.observed_mrad = 200.0f;
        check(!true_muzzle_velocity(keep, zero_atm, bad) && keep.muzzle_velocity == guess.muzzle_velocity, "impossible observation rejected, load unchanged", keep.muzzle_velocity, guess.muzzle_velocity);
        TruingShot neg = mid; neg.observed_mrad = -1.0f;
        check(!true_bc(keep, zero_atm, neg) && keep.bc == guess.bc, "negative come-up rejected", keep.bc, guess.bc);
    }

    std::printf("\n%s (%d failure%s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
