// PC tests for Meridian's hardware-free code: packets, laser frames, GPS sentences, rifle angles, screen text.
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include "meridian_protocol.h"
#include "rangefinder.h"
#include "nmea.h"
#include "orientation.h"
#include "screen.h"

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name, double got = 0, double want = 0) {
    std::printf("[%s] %-60s got=%.4f want=%.4f\n", ok ? "PASS" : "FAIL", name, got, want);
    ok ? ++g_pass : ++g_fail;
}
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ---------------------------------------------------------------------------------------------------------
static void test_protocol() {
    meridian::Shot s;
    s.flags = meridian::kShotRangeValid | meridian::kShotImuOk | meridian::kShotTrueNorth;
    s.battery_pct = 77; s.range_m = 612.3; s.range_age_s = 4.2; s.pitch_deg = -3.21; s.cant_deg = 1.57;
    s.bearing_deg = 359.996; s.temp_c = -4.25; s.pressure_hpa = 845.6; s.rh_pct = 33.3; s.range_seq = 200;
    s.mag_cal = 3; s.heading_acc_deg = 4.4;
    uint8_t b[20];
    meridian::encode_shot(s, b);
    const meridian::Shot d = meridian::decode_shot(b);
    check(d.flags == s.flags && d.battery_pct == 77, "shot: flags and battery round-trip");
    check(near(d.range_m, 612.3, 0.051) && near(d.range_age_s, 4.2, 0.051), "shot: range 0.1 m, age 0.1 s", d.range_m, 612.3);
    check(near(d.pitch_deg, -3.21, 0.006) && near(d.cant_deg, 1.57, 0.006), "shot: pitch and cant 0.01 deg", d.pitch_deg, -3.21);
    check(d.bearing_deg >= 0 && d.bearing_deg < 360 && (near(d.bearing_deg, 0, 0.01)), "shot: bearing 359.996 wraps to 0", d.bearing_deg, 0);
    check(near(d.temp_c, -4.25, 0.006) && near(d.pressure_hpa, 845.6, 0.051) && near(d.rh_pct, 33.3, 0.006), "shot: atmosphere round-trip");
    check(d.range_seq == 200 && d.mag_cal == 3 && d.heading_acc_deg == 4, "shot: seq, compass cal and accuracy", d.heading_acc_deg, 4);
    s.flags = 0; meridian::encode_shot(s, b);
    check(protocol::get_u16(b + 2) == 0 && protocol::get_u16(b + 4) == 65535, "shot: no range -> 0 m and age 65535");
    s.heading_acc_deg = -1; meridian::encode_shot(s, b);
    check(meridian::decode_shot(b).heading_acc_deg < 0, "shot: unknown heading accuracy stays unknown");

    meridian::Position p; p.lat_deg = 45.1234567; p.lon_deg = -111.7654321; p.alt_m = 1523.4; p.fix = 1; p.sats = 9;
    p.hdop = 0.87; p.utc = 1794772800; p.declination_deg = 11.25;
    meridian::encode_position(p, b);
    const meridian::Position q = meridian::decode_position(b);
    check(near(q.lat_deg, 45.1234567, 1e-7) && near(q.lon_deg, -111.7654321, 1e-7), "position: lat/lon to 1e-7 deg", q.lon_deg, -111.7654321);
    check(q.alt_m == 1523 && q.fix == 1 && q.sats == 9 && near(q.hdop, 0.87, 0.006) && q.utc == 1794772800 && near(q.declination_deg, 11.25, 0.006),
          "position: altitude, fix, sats, HDOP, time, declination");

    meridian::Correction c; c.flags = meridian::kCorrValid | meridian::kCorrHasLead; c.elevation_mrad = 7.43; c.windage_mrad = -1.27;
    c.range_m = 803.5; c.tof_s = 1.234; c.lead_mrad = 2.5; c.hit_pct = 64; c.target = 3; c.wind_mps = 4.47; c.wind_from_deg = 270; c.seq = 513;
    meridian::encode_correction(c, b);
    meridian::Correction e;
    check(meridian::decode_correction(b, 20, e), "correction: decodes");
    check(near(e.elevation_mrad, 7.43, 0.006) && near(e.windage_mrad, -1.27, 0.006) && near(e.range_m, 803.5, 0.051) &&
          near(e.tof_s, 1.234, 0.0006) && near(e.lead_mrad, 2.5, 0.006) && e.hit_pct == 64 && e.target == 3 &&
          near(e.wind_mps, 4.47, 0.006) && e.wind_from_deg == 270 && e.seq == 513, "correction: every field round-trips");
    check(!meridian::decode_correction(b, 19, e), "correction: wrong length rejected");
    b[0] = 9; check(!meridian::decode_correction(b, 20, e), "correction: wrong version rejected");
    meridian::encode_correction(c, b); b[12] = 150; check(!meridian::decode_correction(b, 20, e), "correction: hit chance over 100 rejected");

    meridian::Command k;
    const uint8_t r1[] = {meridian::kCmdRange};
    check(meridian::decode_command(r1, 1, k) && k.cmd == meridian::kCmdRange, "command: range");
    uint8_t dc[5] = {meridian::kCmdDeclination}; protocol::put_f32(dc + 1, -12.5f);
    check(meridian::decode_command(dc, 5, k) && near(k.f, -12.5, 1e-6), "command: declination carries f32", k.f, -12.5);
    protocol::put_f32(dc + 1, 75.0f); check(!meridian::decode_command(dc, 5, k), "command: declination over 60 rejected");
    const uint8_t m1[] = {meridian::kCmdMount, 24}; check(!meridian::decode_command(m1, 2, k), "command: mount code 24 rejected");
    const uint8_t t1[] = {meridian::kCmdTargetMode, 2}; check(meridian::decode_command(t1, 2, k) && k.u == 2, "command: last-target mode");
    const uint8_t x1[] = {99}; check(!meridian::decode_command(x1, 1, k), "command: unknown rejected");

    char msg[21];
    const uint8_t ok[] = {'S', 'E', 'N', 'D', ' ', 'I', 'T'};
    check(meridian::decode_message(ok, 7, msg) && std::string(msg) == "SEND IT", "message: plain text accepted");
    const uint8_t bad[] = {'A', 0x07};
    check(!meridian::decode_message(bad, 2, msg), "message: control characters rejected");

    // Solstice env as Meridian reads it: build it with Solstice's own encoder.
    protocol::EnvPacket env{}; env.wind_mps = 3.21; env.temp_c = 20; env.rh_pct = 40; env.pressure_hpa = 900;
    uint8_t eb[protocol::kEnvSize]; protocol::encode_env(env, eb);
    meridian::SolsticeEnv se;
    check(meridian::decode_solstice_env(eb, sizeof eb, se) && near(se.wind_mps, 3.21, 0.006), "Solstice env: wind decoded from Solstice's encoder", se.wind_mps, 3.21);
    check(!meridian::decode_solstice_env(eb, 15, se), "Solstice env: wrong length rejected");
}

// ---------------------------------------------------------------------------------------------------------
static void test_laser() {
    uint8_t f[12];
    // Command frames from the LRF0310C manual.
    const uint8_t single[] = {0xEE, 0x16, 0x02, 0x03, 0x02, 0x05};
    size_t n = lrf::build(lrf::kCmdSingle, nullptr, 0, f);
    check(n == 6 && !std::memcmp(f, single, 6), "laser: single-range frame matches manual (EE 16 02 03 02 05)");
    const uint8_t stop[] = {0xEE, 0x16, 0x02, 0x03, 0x05, 0x08};
    n = lrf::build(lrf::kCmdStop, nullptr, 0, f);
    check(n == 6 && !std::memcmp(f, stop, 6), "laser: stop frame matches manual");
    const uint8_t first[] = {0xEE, 0x16, 0x03, 0x03, 0x03, 0x01, 0x07};
    const uint8_t p1 = lrf::kTargetFirst;
    n = lrf::build(lrf::kCmdTargetMode, &p1, 1, f);
    check(n == 7 && !std::memcmp(f, first, 7), "laser: first-target frame matches manual");
    const uint8_t hz5[] = {0xEE, 0x16, 0x04, 0x03, 0xA1, 0x05, 0x00, 0xA9};
    const uint8_t p5[] = {0x05, 0x00};
    n = lrf::build(lrf::kCmdFrequency, p5, 2, f);
    check(n == 8 && !std::memcmp(f, hz5, 8), "laser: 5 Hz frame matches manual");

    // Reply parsing: 612.3 m, single target. Checksum = 03+02+00+02+64+03 = 0x6E.
    lrf::Parser ps; lrf::Frame fr; lrf::Result r;
    const uint8_t reply[] = {0x55, 0xEE, 0xEE, 0x16, 0x06, 0x03, 0x02, 0x00, 0x02, 0x64, 0x03, 0x6E};
    int frames = 0;
    for (uint8_t b : reply) if (ps.feed(b, fr)) ++frames;
    check(frames == 1 && lrf::decode_range(fr, r) && r.kind == lrf::Result::kRange && near(r.range_m, 612.3, 0.01),
          "laser: reply after junk bytes gives 612.3 m", r.range_m, 612.3);
    const uint8_t oor[] = {0xEE, 0x16, 0x06, 0x03, 0x02, 0x04, 0x00, 0x00, 0x00, 0x09};   // manual's example
    frames = 0; for (uint8_t b : oor) if (ps.feed(b, fr)) ++frames;
    check(frames == 1 && lrf::decode_range(fr, r) && r.kind == lrf::Result::kNoTarget, "laser: manual's out-of-range example -> no target");
    uint8_t badsum[sizeof reply]; std::memcpy(badsum, reply, sizeof reply); badsum[11] ^= 1;
    frames = 0; for (uint8_t b : badsum) if (ps.feed(b, fr)) ++frames;
    check(frames == 0 && ps.badFrames() > 0, "laser: bad checksum is dropped");

    // Driver against a fake port.
    struct Port {
        std::vector<uint8_t> rx, tx; size_t at = 0;
        int available() { return (int)(rx.size() - at); }
        int read() { return at < rx.size() ? rx[at++] : -1; }
        size_t write(const uint8_t* d, size_t k) { tx.insert(tx.end(), d, d + k); return k; }
    } port;
    static int powerLevel = -1, powerCalls = 0;
    lrf::ErdiRangeFinder<Port> drv(port, [](bool on) { powerLevel = on; ++powerCalls; });
    auto cmds = [&]() { std::vector<uint8_t> c; for (size_t i = 0; i + 4 < port.tx.size(); ++i) if (port.tx[i] == 0xEE && port.tx[i + 1] == 0x16) c.push_back(port.tx[i + 4]); return c; };
    drv.targetMode(lrf::kTargetLast);
    drv.begin(0);
    check(powerLevel == 1 && drv.powered(), "laser driver: begin() switches POWER_ON high");
    check(!drv.poll(100, r) && port.tx.empty(), "laser driver: nothing sent during the 0.3 s start-up");
    drv.poll(450, r);
    auto c0 = cmds();
    check(c0.size() == 2 && c0[0] == lrf::kCmdSelfCheck && c0[1] == lrf::kCmdTargetMode && port.tx[11] == lrf::kTargetLast,
          "laser driver: after start-up sends self-check, then the saved target mode (last)");
    const uint8_t sc[] = {0xEE, 0x16, 0x06, 0x03, 0x01, 0xFF, 0x00, 0xF7, 0xFF, 0xF9};      // manual's example
    port.rx.assign(sc, sc + sizeof sc); port.at = 0;
    drv.poll(460, r);
    check(drv.present() && drv.selfCheckOk(), "laser driver: manual's self-check reply reads as healthy");
    port.tx.clear(); drv.single(1000); drv.poll(1000, r);
    check(drv.busy() && port.tx.size() == 6 && port.tx[4] == lrf::kCmdSingle, "laser driver: warm laser fires at once");
    port.rx.insert(port.rx.end(), reply, reply + sizeof reply);
    const bool got = drv.poll(1050, r);
    check(got && r.kind == lrf::Result::kRange && std::fabs(r.range_m - 612.3) < 0.01 && !drv.busy(), "laser driver: reply ends the shot with 612.3 m", r.range_m, 612.3);
    drv.single(2000); drv.poll(2000, r);
    check(!drv.poll(3000, r), "laser driver: no timeout before 1.5 s");
    check(drv.poll(3600, r) && r.kind == lrf::Result::kTimeout && !drv.busy(), "laser driver: silent module times out");
    drv.poll(3600 + 59000, r);
    check(drv.powered(), "laser driver: still on before a minute idle");
    drv.poll(3600 + 61000, r);
    check(!drv.powered() && powerLevel == 0, "laser driver: switched off after a minute idle (10 mW instead of 0.8 W)");
    port.tx.clear(); drv.single(70000); drv.poll(70000, r);
    check(drv.powered() && port.tx.empty(), "laser driver: cold shot powers up and waits");
    drv.poll(70400, r);
    auto c1 = cmds();
    check(c1.size() == 2 && c1[0] == lrf::kCmdTargetMode && c1[1] == lrf::kCmdSingle, "laser driver: then re-sends target mode and fires");
    check(!drv.poll(71800, r), "laser driver: reply timeout counts from when the shot went out");
    drv.poll(72000, r);
    port.tx.clear(); drv.continuous(true, 73000); drv.poll(73000, r);
    auto c2 = cmds();
    check(drv.isContinuous() && c2.size() == 2 && c2[0] == lrf::kCmdFrequency && c2[1] == lrf::kCmdContinuous,
          "laser driver: continuous sends 1 Hz then start");
    drv.poll(73000 + 120000, r);
    check(drv.powered(), "laser driver: stays on while in continuous mode");
    port.tx.clear(); drv.continuous(false, 200000);
    drv.poll(200000, r);
    check(!drv.isContinuous() && port.tx.size() == 6 && port.tx[4] == lrf::kCmdStop, "laser driver: continuous off sends stop");
    const uint8_t err[] = {0xEE, 0x16, 0x06, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x09};
    port.rx.insert(port.rx.end(), err, err + sizeof err);
    check(drv.poll(200010, r) && r.kind == lrf::Result::kFault, "laser driver: unsolicited ranging-error frame (0x06) is a fault");
    const uint8_t sc2[] = {0xEE, 0x16, 0x06, 0x03, 0x01, 0x00, 0x00, 0xF7, 0x00, 0xFB};   // 5V6 power bit clear
    port.rx.insert(port.rx.end(), sc2, sc2 + sizeof sc2);
    check(drv.poll(200020, r) && r.kind == lrf::Result::kFault && !drv.selfCheckOk(), "laser driver: self-check with 5V6 power abnormal is a fault");
}

// ---------------------------------------------------------------------------------------------------------
static std::string nmea_line(const std::string& body) {
    uint8_t s = 0; for (char c : body) s ^= (uint8_t)c;
    char t[8]; std::snprintf(t, sizeof t, "*%02X\r\n", s);
    return "$" + body + t;
}
static void test_nmea() {
    nmea::Parser p;
    auto feed = [&](const std::string& s) { for (char c : s) p.feed(c); };
    // Sentences with checksums from a published example (the 1998 NMEA reference sentences).
    feed("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
    check(p.fix().quality == 1 && p.fix().sats == 8 && near(p.fix().alt_m, 545.4, 1e-6), "GPS: GGA quality, sats, altitude");
    check(near(p.fix().lat_deg, 48.1173, 1e-4) && near(p.fix().lon_deg, 11.516667, 1e-5), "GPS: GGA lat/lon ddmm.mmm -> degrees", p.fix().lat_deg, 48.1173);
    check(!p.fix().valid, "GPS: not valid until RMC says A");
    feed("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    check(p.fix().valid, "GPS: valid after RMC status A");
    check(p.fix().utc == 764426119u, "GPS: RMC time 1994-03-23 12:35:19 -> Unix seconds", p.fix().utc, 764426119u);
    feed(nmea_line("GNGGA,010203.00,4530.0000,S,11145.0000,W,2,11,0.75,1500.0,M,0,M,,"));
    feed(nmea_line("GNRMC,010203.00,A,4530.0000,S,11145.0000,W,0.0,0.0,151126,,,A"));
    check(near(p.fix().lat_deg, -45.5, 1e-9) && near(p.fix().lon_deg, -111.75, 1e-9) && p.fix().quality == 2,
          "GPS: south/west hemispheres are negative (GN talker)", p.fix().lon_deg, -111.75);
    check(p.fix().utc == 1794704523u, "GPS: 2026-11-15 01:02:03 UTC", p.fix().utc, 1794704523u);
    const uint32_t before = p.fix().updates;
    feed("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48\r\n");
    check(p.fix().updates == before && p.badChecksums() == 1, "GPS: bad checksum ignored");
    feed(nmea_line("GPRMC,123600,V,,,,,,,151126,,,N"));
    check(!p.fix().valid, "GPS: RMC status V drops the fix");
}

// ---------------------------------------------------------------------------------------------------------
static orient::Quat qaxis(double ax, double ay, double az, double deg) {
    const double h = deg / orient::kRad / 2, n = std::sqrt(ax * ax + ay * ay + az * az);
    return {std::cos(h), std::sin(h) * ax / n, std::sin(h) * ay / n, std::sin(h) * az / n};
}
static orient::Quat qmul(const orient::Quat& a, const orient::Quat& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
// Rifle pose in the world (ENU), default mount (+Y forward, +Z up): yaw about up (bearing clockwise),
// then pitch about the rifle's right axis, then cant about the bore.
static orient::Quat pose(double bearing, double pitch, double cant) {
    const orient::Quat yaw = qaxis(0, 0, 1, -bearing);         // clockwise from north = negative about +Z
    const orient::Quat pit = qaxis(1, 0, 0, pitch);            // +X is the rifle's right (sensor frame)
    const orient::Quat rol = qaxis(0, 1, 0, cant);             // + about the bore (+Y) = right side down
    return qmul(yaw, qmul(pit, rol));
}
static void test_orientation() {
    orient::Vec f, u;
    orient::mount_axes(orient::kDefaultMount, f, u);
    check(f.y == 1 && u.z == 1, "mount: default is +Y forward, +Z up");
    check(orient::mount_code(2, 4) == orient::kDefaultMount, "mount: code from axes matches default");
    bool allOk = true;
    for (uint8_t c = 0; c < 24; ++c) { orient::mount_axes(c, f, u); if (orient::dot(f, u) != 0) allOk = false; }
    check(allOk, "mount: all 24 codes give perpendicular forward/up");

    orient::Calibration cal;
    orient::Angles a = orient::angles(pose(0, 0, 0), cal);
    check(near(a.pitch_deg, 0, 1e-9) && near(a.cant_deg, 0, 1e-9) && near(a.bearing_deg, 0, 1e-9), "angles: level, pointing north");
    a = orient::angles(pose(90, 0, 0), cal);
    check(near(a.bearing_deg, 90, 1e-6), "angles: pointing east reads 90", a.bearing_deg, 90);
    a = orient::angles(pose(225, 10, 0), cal);
    check(near(a.bearing_deg, 225, 1e-6) && near(a.pitch_deg, 10, 1e-6), "angles: 225 deg, 10 deg uphill", a.pitch_deg, 10);
    a = orient::angles(pose(30, -5, 3), cal);
    check(near(a.cant_deg, 3, 1e-6) && near(a.pitch_deg, -5, 1e-6) && near(a.bearing_deg, 30, 1e-6), "angles: 3 deg cant right, 5 downhill", a.cant_deg, 3);
    a = orient::angles(pose(30, 0, -2), cal);
    check(near(a.cant_deg, -2, 1e-6), "angles: cant left is negative", a.cant_deg, -2);
    a = orient::angles(pose(100, 20, 15), cal);
    check(near(a.bearing_deg, 100, 1e-6) && near(a.pitch_deg, 20, 1e-6) && near(a.cant_deg, 15, 1e-6), "angles: bearing/pitch unaffected by big cant");

    // Sensor mounted differently: +X toward the muzzle, -Z up (upside down under the rail).
    orient::Calibration c2; c2.mount = orient::mount_code(0, 5);
    // The same rifle pose; the sensor body is rotated relative to the rifle, so build the sensor quaternion
    // as rifle pose * (sensor-to-rifle). Sensor +X -> rifle forward (+Y), sensor -Z -> rifle up (+Z):
    // rotate 180 about X then -90 about Z maps those (checked by the expected angles below).
    const orient::Quat mountQ = qmul(qaxis(0, 0, 1, 90), qaxis(1, 0, 0, 180));
    a = orient::angles(qmul(pose(60, 7, -4), mountQ), c2);
    check(near(a.bearing_deg, 60, 1e-6) && near(a.pitch_deg, 7, 1e-6) && near(a.cant_deg, -4, 1e-6), "angles: other mount reads the same rifle angles", a.bearing_deg, 60);

    // Level zero and aim-bearing calibration.
    orient::Calibration c3;
    const orient::Quat crooked = pose(10, 0.8, -1.3);      // sensor sits 0.8 up and 1.3 rolled when rifle is level
    orient::set_level(c3, orient::angles(crooked, c3));
    a = orient::angles(crooked, c3);
    check(near(a.pitch_deg, 0, 1e-9) && near(a.cant_deg, 0, 1e-9), "level zero: the level pose reads 0/0");
    c3.declination = 11.0; c3.declination_set = true;
    orient::set_aim_bearing(c3, orient::angles(crooked, c3), 95.0);
    a = orient::angles(crooked, c3);
    check(near(a.bearing_deg, 95, 1e-6), "aim bearing: reads the bearing it was told", a.bearing_deg, 95);
    a = orient::angles(pose(40, 0.8, -1.3), c3);
    check(near(a.bearing_deg, 125, 1e-6), "aim bearing: swinging 30 deg right reads 125", a.bearing_deg, 125);
}

// ---------------------------------------------------------------------------------------------------------
static void test_screen() {
    screen::Input in;
    screen::Model m = screen::build(in);
    check(m.source == screen::kNone && std::string(m.elev) == "--" && std::string(m.range) == "NO RANGE", "screen: nothing yet");
    in.local_ok = true; in.local_elev = 4.34; in.local_wind = -0.81; in.range_valid = true; in.range_m = 612.4; in.range_age_s = 2;
    m = screen::build(in);
    check(m.source == screen::kLocal && std::string(m.elev) == "U 4.3" && std::string(m.wind) == "L 0.8" && std::string(m.range) == "612 m",
          "screen: Meridian's own solution in mil");
    in.units = 1; m = screen::build(in);
    check(std::string(m.elev) == "U 14.9" && std::string(m.title) == "MERIDIAN MOA", "screen: MOA (4.34 mil = 14.9 MOA)");
    in.units = 0;
    in.corr_valid = true; in.corr_age_s = 3; in.corr_elev = 5.02; in.corr_wind = 1.26; in.corr_range = 612; in.corr_hit = 71;
    m = screen::build(in);
    check(m.source == screen::kSpotter && std::string(m.elev) == "U 5.0" && std::string(m.wind) == "R 1.3" &&
          std::string(m.extra) == "HIT 71% " && m.dial_color == screen::kWhite, "screen: spotter correction wins, with hit chance");
    in.corr_range = 700; m = screen::build(in);
    check(m.dial_color == screen::kAmber, "screen: correction for another range shows amber");
    in.corr_range = 612; in.corr_age_s = 200; m = screen::build(in);
    check(m.source == screen::kLocal, "screen: stale correction falls back to Meridian's solution");
    in.range_age_s = 45; m = screen::build(in);
    check(std::string(m.range) == "612 m 45s" && m.range_color == screen::kAmber, "screen: old range shows its age");
    in.ranging = true; in.range_valid = false; m = screen::build(in);
    check(std::string(m.range) == "RANGING...", "screen: ranging in progress");
    in.ranging = false; in.laser_fault = true; m = screen::build(in);
    check(std::string(m.range) == "LASER FAULT" && m.range_color == screen::kRed, "screen: laser fault");
    in.imu_ok = true; in.cant_deg = 1.6; in.pitch_deg = -2.04; in.bearing_deg = 271.4; m = screen::build(in);
    check(m.cant_color == screen::kRed && std::string(m.angles) == "P -2.0  BRG 271", "screen: cant over 1 deg is red, angles line");
    in.cant_deg = 0.4; m = screen::build(in);
    check(m.cant_color == screen::kGreen, "screen: small cant is green");
    in.battery_pct = 82; in.charging = true; in.centrals = 2; in.solstice = true; in.message = "SEND IT"; m = screen::build(in);
    check(std::string(m.status) == "82%+ BT2 WIND" && std::string(m.message) == "SEND IT", "screen: status and message lines");
    bool fits = std::strlen(m.elev) * 30 <= 230 && std::strlen(m.range) * 18 <= 230 && std::strlen(m.angles) * 12 <= 240;
    check(fits, "screen: lines fit 240 px at their text sizes");
}

int main() {
    test_protocol();
    test_laser();
    test_nmea();
    test_orientation();
    test_screen();
    std::printf("core: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
