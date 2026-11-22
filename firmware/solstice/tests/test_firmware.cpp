// Runs firmware/src/main.cpp on the PC against stand-in hardware headers (tests/stubs).
// Checks the range-card sweep: row count, order, ranges, pacing, values, restart and failure handling.
//   test_firmware           full checks
//   test_firmware nocfg     card request with no profile sent
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include "../firmware/src/main.cpp"

static int g_fail = 0;
static void check(bool ok, const char* name, double got = 0, double want = 0) {
    std::printf("[%s] %-52s got=%.4f want=%.4f\n", ok ? "PASS" : "FAIL", name, got, want);
    if (!ok) ++g_fail;
}
static std::vector<uint8_t> makeConfig(bool bullet = false) {
    std::vector<uint8_t> b(protocol::kConfigSize, 0);
    b[0] = protocol::kVersion; b[1] = 1;   // G7
    protocol::put_f32(&b[4], 0.243f);   protocol::put_f32(&b[8], 792.48f);
    protocol::put_f32(&b[12], 0.0381f); protocol::put_f32(&b[16], 91.44f);
    protocol::put_f32(&b[20], 15.0f);   protocol::put_f32(&b[24], 1013.25f); protocol::put_f32(&b[28], 50.0f);
    if (bullet) {   // .308 175 gr, 1:10 right-hand
        protocol::put_f32(&b[32], 175.0f);   protocol::put_f32(&b[36], 0.00782f);
        protocol::put_f32(&b[40], 0.0315f);  protocol::put_f32(&b[44], 0.254f);
    }
    return b;
}
static std::vector<uint8_t> makeCard(uint8_t count, float start, float step, float wind, uint8_t flags = 0,
                                     float lat = 0, float az = 0) {
    std::vector<uint8_t> b(protocol::kCardSize, 0);
    b[0] = flags; b[1] = count;                              // default: manual wind, manual slope
    protocol::put_f32(&b[24], lat); protocol::put_f32(&b[28], az);
    protocol::put_f32(&b[4], start); protocol::put_f32(&b[8], step);
    protocol::put_f32(&b[12], 90.0f); protocol::put_f32(&b[16], wind); protocol::put_f32(&b[20], 0.0f);
    return b;
}
struct Row { uint8_t row; bool ok; double up, wind, range; uint32_t ms; double spin; };
static std::vector<Row> rowsSince(size_t from) {
    std::vector<Row> out;
    for (size_t i = from; i < g_notifs().size(); ++i) {
        const Notif& n = g_notifs()[i];
        if (n.uuid != BC_UUID_SOLUTION || n.data.size() != protocol::kSolutionSize || n.data[1] == 0) continue;
        out.push_back({n.data[1], n.data[0] == 1, protocol::get_i16(&n.data[2]) / 100.0,
                       protocol::get_i16(&n.data[4]) / 100.0, protocol::get_u16(&n.data[12]) / 10.0, n.ms, protocol::get_i16(&n.data[18]) / 100.0});
    }
    return out;
}
static void spin(int iterations) { for (int i = 0; i < iterations; ++i) loop(); }
static std::vector<uint8_t> makeRequest(uint8_t flags, float range = 100.0f) {
    std::vector<uint8_t> b(protocol::kRequestSize, 0);
    b[0] = flags; protocol::put_f32(&b[4], range);
    return b;
}

int main(int argc, char** argv) {
    setup();
    check(g_bme_i2c_power_at_begin == HIGH, "I2C power (GPIO 7) is HIGH before BME280 starts", g_bme_i2c_power_at_begin, HIGH);
    check(g_adv_name() == "Solstice" && g_init_name() == "Solstice", "board advertises as Solstice", 0, 0);
    {
        const int a0 = g_adv_starts;
        stub_connect_central();           // the phone connects
        check(g_adv_starts == a0 + 1, "keeps advertising after the phone connects (so Meridian can link)", g_adv_starts - a0, 1);
        stub_connect_central(); stub_connect_central();   // phone + Meridian + one more = 3, the NimBLE limit
        check(g_adv_starts == a0 + 2, "stops advertising once all 3 connection slots are used", g_adv_starts - a0, 2);
        g_connected = 0;
    }
    if (argc > 1 && !std::strcmp(argv[1], "nocfg")) {
        stub_write(BC_UUID_CARD, makeCard(5, 100, 100, 0));
        spin(100);
        auto rows = rowsSince(0);
        check(rows.size() == 1 && rows[0].row == 1 && !rows[0].ok, "no profile -> single failed row 1", rows.size(), 1);
        std::printf("\n%s\n", g_fail ? "FAILED" : "ALL PASSED");
        return g_fail ? 1 : 0;
    }

    stub_write(BC_UUID_CONFIG, makeConfig());

    // 1. Normal sweep: 10 rows, 100 yd steps, 10 mph wind from the right.
    size_t mark = g_notifs().size();
    stub_write(BC_UUID_CARD, makeCard(10, 91.44f, 91.44f, 4.4704f));
    spin(400);                                   // 2 s of simulated time
    auto rows = rowsSince(mark);
    check(rows.size() == 10, "sweep sends exactly 10 rows", rows.size(), 10);
    bool ordered = true, allOk = true, rising = true, paced = true;
    for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].row != i + 1) ordered = false;
        if (!rows[i].ok) allOk = false;
        if (i && rows[i].up <= rows[i - 1].up) rising = false;
        if (i && rows[i].ms - rows[i - 1].ms < CARD_ROW_SPACING_MS) paced = false;
    }
    check(ordered, "rows arrive as 1..10 in order");
    check(allOk, "every row solved (ok)");
    check(rising, "come-up rises row by row");
    check(paced, "rows are spaced at least 40 ms apart");
    if (rows.size() == 10) {
        check(std::fabs(rows[0].range - 91.4) < 0.06, "row 1 range is 91.4 m", rows[0].range, 91.4);
        check(std::fabs(rows[9].range - 914.4) < 0.06, "row 10 range is 914.4 m", rows[9].range, 914.4);
        check(std::fabs(rows[0].up) < 0.011, "row 1 (zero range) needs no come-up", rows[0].up, 0);
        check(rows[9].wind > 0, "wind from the right gives positive windage", rows[9].wind, 1);

        // Cross-check against calling the solver directly with identical inputs.
        ballistics::Bullet b{0.243f, ballistics::DragModel::G7, 792.48f};
        ballistics::Atmosphere atm{15.0f, 1013.25f, 50.0f};
        ballistics::Shot shot{0.0381f, 91.44f, 4.4704f, 90.0f, 0};
        auto direct = ballistics::solve(b, atm, atm, shot, 91.44f + 91.44f * 4);
        check(std::fabs(rows[4].up - direct.elevation_mrad) < 0.011, "row 5 come-up matches direct solve (mrad)", rows[4].up, direct.elevation_mrad);
        check(std::fabs(rows[4].wind - direct.windage_mrad) < 0.011, "row 5 windage matches direct solve (mrad)", rows[4].wind, direct.windage_mrad);
    }

    // 2. A new request mid-sweep restarts it.
    mark = g_notifs().size();
    stub_write(BC_UUID_CARD, makeCard(10, 91.44f, 91.44f, 0));
    spin(25);                                    // ~125 ms: a few rows out
    size_t partial = rowsSince(mark).size();
    stub_write(BC_UUID_CARD, makeCard(3, 200.0f, 100.0f, 0));
    spin(200);
    auto all = rowsSince(mark);
    check(partial > 0 && partial < 10, "first sweep was interrupted part-way", partial, 5);
    std::vector<Row> tail(all.end() - std::min<size_t>(3, all.size()), all.end());
    check(tail.size() == 3 && tail[0].row == 1 && tail[2].row == 3 && std::fabs(tail[0].range - 200.0) < 0.06 &&
          std::fabs(tail[2].range - 400.0) < 0.06, "restart delivers the new 3-row sweep (200..400 m)",
          tail.size() ? tail[0].range : 0, 200);

    // 3. Invalid request: one failed row 1 and nothing else.
    mark = g_notifs().size();
    stub_write(BC_UUID_CARD, makeCard(0, 100, 100, 0));
    spin(100);
    rows = rowsSince(mark);
    check(rows.size() == 1 && rows[0].row == 1 && !rows[0].ok, "invalid card -> single failed row 1", rows.size(), 1);

    // 4. Spin drift and Coriolis through the real firmware path.
    stub_write(BC_UUID_CONFIG, makeConfig(true));
    {
        ballistics::Bullet b{0.243f, ballistics::DragModel::G7, 792.48f, 175.0f, 0.308f, 1.24f, 10.0f};
        ballistics::Atmosphere atm{15.0f, 1013.25f, 50.0f};
        const float R = 1000.0f;   // beyond 1000 yd, where Coriolis applies
        auto runCard = [&](uint8_t flags, float lat, float az) {
            size_t m = g_notifs().size();
            stub_write(BC_UUID_CARD, makeCard(1, R, 100, 0, flags, lat, az));
            spin(60);
            auto r = rowsSince(m);
            return r.empty() ? Row{0, false, 0, 0, 0, 0, 0} : r.back();
        };
        Row plain = runCard(0, 0, 0);
        Row sd = runCard(protocol::kFlagSpinDrift, 0, 0);
        ballistics::Shot shot{0.0381f, 91.44f, 0, 0, 0};
        auto d0 = ballistics::solve(b, atm, atm, shot, R);
        shot.spin_drift = true;
        auto d1 = ballistics::solve(b, atm, atm, shot, R);
        check(plain.ok && std::fabs(plain.wind - d0.windage_mrad) < 0.011 && plain.spin == 0, "flags off: no spin drift in windage", plain.wind, d0.windage_mrad);
        check(sd.ok && std::fabs(sd.wind - d1.windage_mrad) < 0.011, "spin drift flag: windage matches direct solve", sd.wind, d1.windage_mrad);
        check(std::fabs(sd.spin - d1.spin_drift_mrad) < 0.011 && sd.spin < 0, "spin_drift_mrad field is sent (negative = hold left)", sd.spin, d1.spin_drift_mrad);

        Row co = runCard(protocol::kFlagCoriolis, 45.0f, 0.0f);
        ballistics::Shot cs{0.0381f, 91.44f, 0, 0, 0};
        cs.coriolis = true; cs.latitude_deg = 45; cs.azimuth_deg = 0;
        auto dc = ballistics::solve(b, atm, atm, cs, R);
        check(co.ok && std::fabs(co.wind - dc.windage_mrad) < 0.011 && co.wind < plain.wind, "Coriolis flag: windage matches direct solve (hold left)", co.wind, dc.windage_mrad);

        Row near = runCard(protocol::kFlagCoriolis, 45.0f, 0.0f);   // same R, so reuse; check short range separately
        {
            size_t m = g_notifs().size();
            stub_write(BC_UUID_CARD, makeCard(1, 500.0f, 100, 0, protocol::kFlagCoriolis, 45.0f, 0.0f));
            spin(60);
            auto r1 = rowsSince(m);
            m = g_notifs().size();
            stub_write(BC_UUID_CARD, makeCard(1, 500.0f, 100, 0, 0, 0, 0));
            spin(60);
            auto r2 = rowsSince(m);
            check(!r1.empty() && !r2.empty() && r1.back().wind == r2.back().wind, "Coriolis flag has no effect at 500 m", r1.empty() ? 0 : r1.back().wind, r2.empty() ? 0 : r2.back().wind);
        }
        (void)near;
        // Aerodynamic jump: 10 mph from 3 o'clock, with and without the flag.
        auto runWindCard = [&](uint8_t flags) {
            size_t m = g_notifs().size();
            stub_write(BC_UUID_CARD, makeCard(1, R, 100, 4.4704f, flags));
            spin(60);
            auto r = rowsSince(m);
            return r.empty() ? Row{0, false, 0, 0, 0, 0, 0} : r.back();
        };
        Row noJump = runWindCard(0), jump = runWindCard(protocol::kFlagAeroJump);
        ballistics::Shot ws{0.0381f, 91.44f, 4.4704f, 90.0f, 0};
        ws.aero_jump = true;
        auto dj = ballistics::solve(b, atm, atm, ws, R);
        check(jump.ok && std::fabs((jump.up - noJump.up) - dj.aero_jump_mrad) < 0.011 && dj.aero_jump_mrad > 0,
              "aero jump flag adds Litz jump to elevation", jump.up - noJump.up, dj.aero_jump_mrad);
        check(std::fabs(jump.wind - noJump.wind) < 0.011, "aero jump flag leaves windage alone", jump.wind, noJump.wind);
        // Both off when the profile has no bullet data: spin flag is ignored, not an error.
        stub_write(BC_UUID_CONFIG, makeConfig(false));
        Row nob = runCard(protocol::kFlagSpinDrift, 0, 0);
        check(nob.ok && nob.spin == 0 && std::fabs(nob.wind - plain.wind) < 0.011, "spin flag without bullet data is ignored", nob.wind, plain.wind);
        Row nj0 = runWindCard(0), nj = runWindCard(protocol::kFlagAeroJump);
        check(nj.ok && std::fabs(nj.up - nj0.up) < 0.011, "aero jump flag without bullet data is ignored", nj.up, nj0.up);
    }

    // ---- sleep command ----
    g_sleepStarted = false;
    stub_write(BC_UUID_REQUEST, makeRequest(protocol::kFlagAeroJump)); delay(1000); spin(5);
    check(!g_sleepStarted, "sleep: an ordinary request does not sleep the board");
    stub_write(BC_UUID_REQUEST, makeRequest(protocol::kFlagSleep, 5.0f)); delay(1000); spin(5);   // range 5 m is invalid
    check(!g_sleepStarted, "sleep: an invalid request with the sleep bit is ignored");
    stub_write(BC_UUID_REQUEST, makeRequest(protocol::kFlagSleep));
    spin(2);
    check(!g_sleepStarted, "sleep: waits a moment so the write can be acknowledged");
    delay(500); spin(2);
    check(g_sleepStarted, "sleep: valid request with the sleep bit puts the board to sleep");
    g_sleepStarted = false; sleepRequested = false;

    // ---- battery percentage ----
    check(protocol::battery_percent_from_mv(4200) == 100 && protocol::battery_percent_from_mv(4500) == 100, "battery: full cell is 100 %");
    check(protocol::battery_percent_from_mv(3300) == 0 && protocol::battery_percent_from_mv(3000) == 0, "battery: 3.3 V or less is 0 %");
    check(protocol::battery_percent_from_mv(0) == 255, "battery: no reading is unknown (255)");
    check(protocol::battery_percent_from_mv(3800) == 42, "battery: 3.8 V is 42 %", protocol::battery_percent_from_mv(3800), 42);
    check(protocol::battery_percent_from_mv(3775) == 36, "battery: interpolates between points", protocol::battery_percent_from_mv(3775), 36);
    {
        double prev = -1; bool mono = true;
        for (int mv = 3000; mv <= 4300; mv += 5) { double q = protocol::battery_percent_from_mv(mv); if (q < prev) mono = false; prev = q; }
        check(mono, "battery: never goes down as voltage rises");
    }
    {   // MAX17048 fuel gauge: real state of charge and voltage go out in env byte 15
        fuelOk = true; g_fuel_present = true; g_fuel_v = 3.85f; g_fuel_pct = 71.4f;
        updateBattery();
        check(std::fabs(batt_mv - 3850) < 1 && batt_pct == 71, "gauge: voltage and rounded percent read", batt_pct, 71);
        g_notifs().clear(); publishEnv();
        check(g_notifs().back().data.size() == 16 && g_notifs().back().data[15] == 71, "gauge: env byte 15 carries gauge percent", g_notifs().back().data[15], 71);
        g_fuel_pct = 100.6f; updateBattery(); check(batt_pct == 100, "gauge: over-100 reading is clamped", batt_pct, 100);
        g_fuel_pct = -1.0f; updateBattery(); check(batt_pct == 0, "gauge: negative reading is clamped to 0", batt_pct, 0);
        // charging detection from the cell-voltage trend (the gauge's charge-rate register is flat on this board)
        g_fuel_rate = -4.2f; g_fuel_present = true; fuelOk = true;
        {   // rising ~1 mV/min (what the board really showed): charging after a few minutes, not at once
            ChargeDetector d;
            float v = 4100; bool early = false;
            for (int t = 1; t <= 600; ++t) { v += 1.0f / 60.0f; d.feed(v); if (t == 90 && d.charging) early = true; }
            check(!early && d.charging, "charging: +1 mV/min for 10 min turns it on, but not in the first minute and a half", d.charging, 1);
        }
        {   // steady discharge
            ChargeDetector d; float v = 3900;
            for (int t = 1; t <= 900; ++t) { v -= 0.8f / 60.0f; d.feed(v); }
            check(!d.charging, "charging: slowly falling voltage is not charging");
        }
        {   // flat voltage mid-range (battery idle, no charger)
            ChargeDetector d;
            for (int t = 1; t <= 900; ++t) d.feed(3850);
            check(!d.charging, "charging: flat 3.85 V is not charging");
        }
        {   // full cell on USB: flat near 4.2 V counts as plugged in, after the history fills
            ChargeDetector d;
            for (int t = 1; t <= 600; ++t) d.feed(4190);
            check(d.charging, "charging: flat 4.19 V (full cell on USB) counts as charging");
        }
        {   // full cell, USB pulled: a 30 mV drop switches it off straight away, and it stays off
            ChargeDetector d;
            for (int t = 1; t <= 600; ++t) d.feed(4190);
            for (int t = 1; t <= 45; ++t) d.feed(4160);
            check(!d.charging, "charging: 30 mV step down (USB pulled) turns it off within a minute");
            float v = 4160;
            for (int t = 1; t <= 900; ++t) { v -= 1.6f / 60.0f; d.feed(v); }
            check(!d.charging, "charging: stays off while the cell slowly discharges above 4.15 V");
        }
        {   // USB plugged in mid-discharge: 30 mV step up
            ChargeDetector d;
            for (int t = 1; t <= 300; ++t) d.feed(3900);
            for (int t = 1; t <= 45; ++t) d.feed(3935);
            check(d.charging, "charging: 35 mV step up (USB plugged in) turns it on within a minute");
        }
        {   // through the real update path: percent from the gauge, charging from the trend, flag in env byte 14
            fuelOk = true; g_fuel_present = true; g_fuel_pct = 80.0f; g_fuel_v = 3.9f; chargeDet = ChargeDetector();
            for (int t = 0; t < 700; ++t) { g_fuel_v += 0.00002f; updateBattery(); }   // +1.2 mV/min
            check(batt_charging, "charging: updateBattery follows the voltage trend", batt_charging, 1);
            g_notifs().clear(); publishEnv();
            check((g_notifs().back().data[14] & protocol::kEnvCharging) != 0, "charging: env flag bit3 set while charging", g_notifs().back().data[14], 8);
            for (int t = 0; t < 400; ++t) { g_fuel_v -= 0.00004f; updateBattery(); }
            check(!batt_charging, "charging: updateBattery clears it when the voltage falls", batt_charging, 0);
            g_notifs().clear(); publishEnv();
            check((g_notifs().back().data[14] & protocol::kEnvCharging) == 0, "charging: env flag bit3 clear when not charging");
        }
        g_fuel_v = 3.85f; g_fuel_pct = 71.4f; g_fuel_rate = -5.0f;
        g_fuel_present = false;                 // chip stops answering (NaN): fall back to the VBAT pin curve
        g_batt_mv = 1950; batt_mv = 0; updateBattery();
        check(batt_pct == 65, "gauge: NaN falls back to the pin curve", batt_pct, 65);
        fuelOk = false; g_fuel_present = true;  // chip not found at boot: pin curve is used even if it later appears
        g_fuel_v = 3.0f; g_fuel_pct = 5.0f; g_batt_mv = 1950; batt_mv = 0; updateBattery();
        check(batt_pct == 65, "no gauge at boot: pin curve is used", batt_pct, 65);
    }
    {   // the board reads the pin, doubles it, and sends the percentage in env byte 15
        g_batt_mv = 1950;                       // 3.90 V cell
        batt_mv = 0; updateBattery();
        check(std::fabs(batt_mv - 3900) < 1, "battery: pin reading doubled by the divider", batt_mv, 3900);
        g_notifs().clear(); publishEnv();
        const auto& n = g_notifs().back().data;
        check(n.size() == 16 && n[15] == 65, "battery: env byte 15 carries 65 %", n.size() == 16 ? n[15] : -1, 65);
        check(n.size() == 16 && (n[14] & protocol::kEnvBattOk) != 0, "battery: battery-valid flag set when known", n.size() == 16 ? n[14] : -1, 4);
        g_batt_mv = 100; batt_mv = 0; updateBattery();
        g_notifs().clear(); publishEnv();
        check((g_notifs().back().data[14] & protocol::kEnvBattOk) == 0, "battery: floating pin clears the battery-valid flag", g_notifs().back().data[14], 0);
        g_batt_mv = 3900;
    }

    std::printf("\n%s\n", g_fail ? "FAILED" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
