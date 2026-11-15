// Runs firmware/src/main.cpp on the PC against stand-in hardware (tests/stubs): a fake laser on Serial1,
// a scripted BNO085 and GPS, a recording screen, and a fake Solstice for the wind link.
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include "../firmware/src/main.cpp"

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name, double got = 0, double want = 0) {
    std::printf("[%s] %-62s got=%.4f want=%.4f\n", ok ? "PASS" : "FAIL", name, got, want);
    ok ? ++g_pass : ++g_fail;
}
static void spin(int ms) { const uint32_t end = millis() + ms; while ((int32_t)(millis() - end) < 0) loop(); }

static std::vector<uint8_t> makeConfig() {   // .308-ish G7 profile, same as Solstice's firmware test
    std::vector<uint8_t> b(protocol::kConfigSize, 0);
    b[0] = protocol::kVersion; b[1] = 1;
    protocol::put_f32(&b[4], 0.243f);   protocol::put_f32(&b[8], 792.48f);
    protocol::put_f32(&b[12], 0.0381f); protocol::put_f32(&b[16], 91.44f);
    protocol::put_f32(&b[20], 15.0f);   protocol::put_f32(&b[24], 1013.25f); protocol::put_f32(&b[28], 50.0f);
    return b;
}
static std::vector<uint8_t> makeRequest(uint8_t flags, float range, float clock = 90, float wind = 0) {
    std::vector<uint8_t> b(protocol::kRequestSize, 0);
    b[0] = flags; protocol::put_f32(&b[4], range); protocol::put_f32(&b[8], clock); protocol::put_f32(&b[12], wind);
    return b;
}
static void laserReply(float m) {
    const int whole = (int)m, tenth = (int)std::lround((m - whole) * 10);
    uint8_t f[] = {0xEE, 0x16, 0x06, 0x03, 0x02, 0x00, (uint8_t)(whole >> 8), (uint8_t)(whole & 0xFF), (uint8_t)tenth, 0};
    uint8_t s = 0; for (int i = 3; i < 9; ++i) s += f[i];
    f[9] = s;
    Serial1.rx.insert(Serial1.rx.end(), f, f + sizeof f);
}
static const Notif* last(const char* uuid) {
    for (size_t i = g_notifs().size(); i-- > 0;) if (g_notifs()[i].uuid == uuid) return &g_notifs()[i];
    return nullptr;
}
static std::string screenLine(int y) { auto it = g_tft_lines().find(y); return it == g_tft_lines().end() ? "" : it->second; }
static bool txHasCmd(uint8_t cmd) {
    for (size_t i = 0; i + 4 < Serial1.tx.size(); ++i)
        if (Serial1.tx[i] == 0xEE && Serial1.tx[i + 1] == 0x16 && Serial1.tx[i + 4] == cmd) return true;
    return false;
}
static void command(std::vector<uint8_t> b) { stub_write(MR_UUID_COMMAND, b); }
static void commandF(uint8_t c, float v) { std::vector<uint8_t> b(5); b[0] = c; protocol::put_f32(&b[1], v); command(b); }
static orient::Quat qaxis(double ax, double ay, double az, double deg) {
    const double h = deg / orient::kRad / 2;
    return {std::cos(h), std::sin(h) * ax, std::sin(h) * ay, std::sin(h) * az};
}
static void setQuat(const orient::Quat& q) { g_bno_quat[0] = q.w; g_bno_quat[1] = q.x; g_bno_quat[2] = q.y; g_bno_quat[3] = q.z; }

int main() {
    // A Solstice and another Meridian are nearby; only Solstice should be linked.
    g_nearby().push_back({{BC_UUID_SERVICE}, "Meridian", {"m"}});
    g_nearby().push_back({{BC_UUID_SERVICE}, "Solstice", {"s"}});

    setup();
    check(g_bme_i2c_power_at_begin == HIGH, "I2C power (GPIO 7) is HIGH before the sensors start");
    check(g_init_name() == "Meridian" && g_adv_name() == "Meridian", "advertises as Meridian");
    check(g_adv_data().bytes() <= 31 && g_scan_data().bytes() <= 31, "advert and scan response each fit 31 bytes",
          (double)std::max(g_adv_data().bytes(), g_scan_data().bytes()), 31);
    check(g_adv_data().uuids.size() == 1 && g_adv_data().uuids[0] == BC_UUID_SERVICE, "advert carries the Solstice service (phone app finds it)");
    check(g_scan_data().uuids.size() == 1 && g_scan_data().uuids[0] == MR_UUID_SERVICE, "scan response carries the Meridian service");
    check(g_services().size() == 2, "two GATT services", g_services().size(), 2);
    check(Serial1.baud == 115200 && Serial1.rxPin == 38 && Serial1.txPin == 39, "laser serial: 115200 on RX 38 / TX 39");
    check(g_pin_level[LASER_EN_PIN] == HIGH && g_pin_level[TFT_BL] == HIGH, "laser power and backlight on");
    check(g_gps_cmds.size() == 2, "GPS told to send RMC+GGA at 1 Hz", g_gps_cmds.size(), 2);

    spin(1100);
    check(txHasCmd(lrf::kCmdSelfCheck), "laser self-check sent after start-up");
    check(txHasCmd(lrf::kCmdTargetMode), "laser target mode sent");
    const uint8_t sc[] = {0xEE, 0x16, 0x06, 0x03, 0x01, 0xFF, 0x00, 0xF7, 0xFF, 0xF9};
    Serial1.rx.insert(Serial1.rx.end(), sc, sc + sizeof sc);
    spin(50);
    check(laser.present() && !laserFault(), "laser answered self-check: present and healthy");
    check(last(MR_UUID_SHOT) && last(MR_UUID_POSITION) && last(MR_UUID_STATUS) && last(BC_UUID_ENV), "shot, position, status and env all notified");

    // Solstice link.
    check(g_sol_connects == 1 && solsticeLinked, "linked to Solstice (not the other Meridian)", g_sol_connects, 1);
    protocol::EnvPacket se{}; se.wind_mps = 4.0; se.temp_c = 10; se.rh_pct = 50; se.pressure_hpa = 1000;
    std::vector<uint8_t> seb(protocol::kEnvSize); protocol::encode_env(se, seb.data());
    stub_solstice_notify(seb);
    spin(1000);
    stub_solstice_notify(seb);
    spin(20);
    const Notif* env = last(BC_UUID_ENV);
    check(env && std::fabs(protocol::get_u16(&env->data[6]) / 100.0 - 4.0) < 0.01, "app env packet passes Solstice's wind through",
          env ? protocol::get_u16(&env->data[6]) / 100.0 : -1, 4.0);
    check(env && (env->data[14] & protocol::kEnvImuOk), "app env packet says the IMU is present");

    // Range: command, then reply.
    Serial1.tx.clear();
    command({meridian::kCmdRange});
    spin(20);
    check(txHasCmd(lrf::kCmdSingle), "range command fires the laser");
    const size_t before = g_notifs().size();
    laserReply(612.3f);
    loop();
    bool shotNow = false;
    for (size_t i = before; i < g_notifs().size(); ++i) if (g_notifs()[i].uuid == MR_UUID_SHOT) shotNow = true;
    const meridian::Shot sh = meridian::decode_shot(last(MR_UUID_SHOT)->data.data());
    check(shotNow && (sh.flags & meridian::kShotRangeValid) && std::fabs(sh.range_m - 612.3) < 0.051 && sh.range_seq == 1,
          "new range goes out at once: 612.3 m, seq 1", sh.range_m, 612.3);
    // Range button.
    Serial1.tx.clear();
    g_pin_input[BTN_RANGE] = LOW; spin(100); g_pin_input[BTN_RANGE] = HIGH; spin(100);
    check(txHasCmd(lrf::kCmdSingle), "RANGE button press fires the laser");
    laserReply(612.3f); spin(20);

    // Meridian's own solution on screen once the app has sent the rifle profile.
    stub_write(BC_UUID_CONFIG, makeConfig());
    stub_write(BC_UUID_REQUEST, makeRequest(0, 300, 90, 0));
    spin(1100);
    {
        Bullet b{0.243f, DragModel::G7, 792.48f};
        Atmosphere a{15.0f, 1013.25f, 50.0f};
        Shot s{}; s.slope_rad = 0; s.sight_height = 0.0381f; s.zero_range = 91.44f; s.wind_speed = 4.0f; s.wind_clock_deg = 90;
        const Solution d = solve(b, a, a, s, 612.3f);
        check(localOk && std::fabs(localElev - d.elevation_mrad) < 0.01 && std::fabs(localWind - d.windage_mrad) < 0.01,
              "screen solution = solver at the laser range with Solstice wind", localElev, d.elevation_mrad);
        char want[16]; screen::fmt_dir(want, sizeof want, d.elevation_mrad, 0, 'U', 'D');
        check(screenLine(4) == "MERIDIAN MIL" && screenLine(40) == want, "screen shows MERIDIAN MIL and the come-up");
        check(screenLine(136) == "612 m", "screen shows the range");
    }
    // The app's own live solution: typed range unless bit7, sensor wind only with bit0.
    stub_write(BC_UUID_REQUEST, makeRequest(0, 300, 90, 0)); spin(20);
    protocol::SolutionPacket chk{};
    auto sol = [&]() { const Notif* n = last(BC_UUID_SOLUTION); return n ? n->data : std::vector<uint8_t>(); };
    std::vector<uint8_t> s1 = sol();
    check(!s1.empty() && s1[0] == 1 && std::fabs(protocol::get_u16(&s1[12]) / 10.0 - 300) < 0.06 && protocol::get_u16(&s1[14]) == 0,
          "app request: typed 300 m, typed wind 0 (like Solstice)");
    stub_write(BC_UUID_REQUEST, makeRequest(meridian::kFlagLaserRange | protocol::kFlagSensorWind, 300)); spin(20);
    s1 = sol();
    check(std::fabs(protocol::get_u16(&s1[12]) / 10.0 - 612.3) < 0.06 && std::fabs(protocol::get_u16(&s1[14]) / 100.0 - 4.0) < 0.01,
          "app request bit7 + bit0: laser range and Solstice wind", protocol::get_u16(&s1[12]) / 10.0, 612.3);
    (void)chk;

    // Range card still works (the app's range card tab).
    std::vector<uint8_t> card(protocol::kCardSize, 0);
    card[1] = 5; protocol::put_f32(&card[4], 100); protocol::put_f32(&card[8], 100);
    const size_t c0 = g_notifs().size();
    stub_write(BC_UUID_CARD, card);
    spin(400);
    int rows = 0;
    for (size_t i = c0; i < g_notifs().size(); ++i) if (g_notifs()[i].uuid == BC_UUID_SOLUTION && g_notifs()[i].data[1] > 0) ++rows;
    check(rows == 5, "range card: 5 rows", rows, 5);

    // Spotter correction takes over the screen.
    meridian::Correction c; c.flags = meridian::kCorrValid; c.elevation_mrad = 5.02; c.windage_mrad = -1.26; c.range_m = 612.3;
    c.hit_pct = 71; c.seq = 42;
    std::vector<uint8_t> cb(meridian::kCorrectionSize); meridian::encode_correction(c, cb.data());
    stub_write(MR_UUID_CORRECTION, cb);
    stub_write(MR_UUID_MESSAGE, {'S', 'E', 'N', 'D', ' ', 'I', 'T'});
    spin(1100);
    check(screenLine(4) == "SPOTTER MIL" && screenLine(40) == "U 5.0" && screenLine(88) == "L 1.3", "screen shows the spotter's U 5.0 / L 1.3");
    check(screenLine(164) == "HIT 71% " && screenLine(224) == "SEND IT", "screen shows hit chance and the spotter's message");
    const Notif* st = last(MR_UUID_STATUS);
    check(st && protocol::get_u16(&st->data[16]) == 42 && (st->data[1] & meridian::kStatCorrFresh), "status echoes correction #42");
    spin(121000);
    check(screenLine(4) == "MERIDIAN MIL", "after 2 minutes the stale correction gives way to Meridian's own");

    // Level zero: sensor is tilted 2 deg nose-up and rolled 1 deg when the rifle is level.
    setQuat(qaxis(1, 0, 0, 2));
    spin(100);
    check(std::fabs(ang.pitch_deg - 2) < 0.01, "tilted sensor reads 2 deg pitch before zeroing", ang.pitch_deg, 2);
    command({meridian::kCmdLevelZero});
    spin(100);
    check(std::fabs(ang.pitch_deg) < 0.01 && cal.level_set, "LEVEL ZERO: now reads 0", ang.pitch_deg, 0);
    commandF(meridian::kCmdDeclination, 11.5f);
    commandF(meridian::kCmdAimBearing, 270.0f);
    spin(300);
    check(std::fabs(ang.bearing_deg - 270) < 0.01, "AIM BEARING 270 with declination set: reads 270", ang.bearing_deg, 270);
    const meridian::Shot sh2 = meridian::decode_shot(last(MR_UUID_SHOT)->data.data());
    check(sh2.flags & meridian::kShotTrueNorth, "shot packet marks the bearing as true north");
    // Settings survive a reboot.
    cal = orient::Calibration{};
    loadSettings();
    check(cal.level_set && std::fabs(cal.declination - 11.5) < 1e-4 && cal.heading_set, "calibration reloads after power-off");
    command({meridian::kCmdUnits, 1}); spin(300);
    check(screenLine(4) == "MERIDIAN MOA", "UNITS MOA switches the screen");
    command({meridian::kCmdMagCalStart}); spin(20);
    command({meridian::kCmdMagCalSave}); spin(20);
    check(g_bno_calconfig == (SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG) && g_bno_dcd_saves == 1, "compass calibration start and save reach the BNO085");
    command({99}); spin(20);
    check(lastCmdResult == 1, "unknown command reported as rejected");

    // GPS.
    g_gps_text = "$GNGGA,010203.00,4530.0000,N,11145.0000,W,1,09,0.80,1500.0,M,0,M,,*";
    { uint8_t s = 0; for (size_t i = 1; i < g_gps_text.size() - 1; ++i) s ^= (uint8_t)g_gps_text[i]; char t[8]; std::snprintf(t, sizeof t, "%02X\r\n", s); g_gps_text += t; }
    std::string rmc = "GNRMC,010203.00,A,4530.0000,N,11145.0000,W,0.0,0.0,151126,,,A";
    { uint8_t s = 0; for (char ch : rmc) s ^= (uint8_t)ch; char t[8]; std::snprintf(t, sizeof t, "*%02X\r\n", s); g_gps_text += "$" + rmc + t; }
    spin(1200);
    const meridian::Position pos = meridian::decode_position(last(MR_UUID_POSITION)->data.data());
    check(pos.fix == 1 && std::fabs(pos.lat_deg - 45.5) < 1e-6 && std::fabs(pos.lon_deg + 111.75) < 1e-6 && pos.sats == 9,
          "GPS fix goes out in the position packet", pos.lat_deg, 45.5);

    // Solstice drops: wind no longer used.
    g_client().disconnect();
    spin(6000);
    check(std::isnan(liveWind()), "Solstice dropping out stops its wind being used");
    check(g_sol_connects >= 2, "Meridian looks for Solstice again", g_sol_connects, 2);

    // Laser silent -> fault shown.
    command({meridian::kCmdRange}); spin(1700);
    check(laserFault() && screenLine(136) == "LASER FAULT", "laser that doesn't answer shows LASER FAULT");

    // Second central keeps advertising going.
    const int adv0 = g_adv_starts;
    stub_connect_central();
    check(g_adv_starts == adv0 + 1, "after a connection Meridian keeps advertising for the second device");

    // Sleep from a long MODE press.
    g_pin_input[BTN_MODE] = LOW; spin(3200); g_pin_input[BTN_MODE] = HIGH; spin(100);
    check(g_sleepStarted, "holding MODE 3 s puts Meridian to sleep");

    std::printf("firmware: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
