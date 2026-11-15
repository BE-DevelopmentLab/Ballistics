// Meridian (Celestial project, V2 rifle-mounted unit) firmware for the Adafruit Feather ESP32-S3
// (8 MB flash, no PSRAM).
//
// UNTESTED ON HARDWARE: written before the parts arrived. The packet, laser, GPS, angle and screen logic
// (core/ and ../core/) is tested on a PC, and this file runs on the PC against stand-in hardware
// (tests/test_firmware.cpp). Everything that touches real hardware still needs a first run on Meridian.
//
// What Meridian does:
//   - Measures: laser range (ERDI LRF0306C on Serial1), atmosphere (BME280), rifle pitch, cant and bearing
//     (BNO085), position and time (PA1010D GPS on I2C).
//   - Sends that to the camera spotter and the phone over Bluetooth (Meridian service, meridian_protocol.h).
//   - Shows the dial/hold the spotter sends back on its 1.3" 240x240 screen. With no spotter, it shows its
//     own solution from the Solstice solver (needs the rifle profile from the phone app).
//   - Also offers the Solstice service, so today's phone app connects to Meridian as if it were Solstice.
//   - Pulls live wind from Solstice over Bluetooth (Meridian connects to Solstice as a client).
//
// Libraries: NimBLE-Arduino 2.x, Adafruit BME280, BNO08x, GPS, ST7735/ST7789 + GFX, MAX1704X, LC709203F.

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MAX1704X.h>
#include <Adafruit_LC709203F.h>
#if __has_include(<esp_sleep.h>)
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#endif

#include "ballistics.h"          // Solstice solver (shared, unchanged)
#include "protocol.h"            // Solstice packets (shared, unchanged)
#include "meridian_protocol.h"
#include "rangefinder.h"
#include "nmea.h"
#include "orientation.h"
#include "screen.h"

// ---- Hardware settings (check against the wiring) ----------------------------------------------------
#define BLE_NAME "Meridian"
#define SOLSTICE_NAME "Solstice"        // the handheld Meridian pulls wind from
#ifndef I2C_POWER
#define I2C_POWER 7                     // Feather ESP32-S3: STEMMA QT / I2C power switch, must be HIGH
#endif
#define BME_ADDR 0x77                   // 0x76 if the jumper is cut
#define GPS_ADDR 0x10                   // PA1010D on I2C
#define BNO_ADDR 0x4A                   // Adafruit BNO085 default

// Laser (ERDI LRF0306C): 3.3 V serial on the Feather's RX/TX pins (Serial1). POWER_ON driven by LASER_EN.
#define LASER_BAUD 115200               // LRF0306C factory default
#define LASER_EN_PIN 6                  // D6 -> module pin 1 POWER_ON (HIGH = on). If POWER_ON is tied high, set -1
                                        // (the laser then draws up to 0.8 W in standby instead of 10 mW).
#ifndef RX
#define RX 38
#define TX 39
#endif

// Screen: Adafruit 1.3" 240x240 ST7789 on the hardware SPI pins.
#define TFT_CS 10                       // D10
#define TFT_DC 9                        // D9
#define TFT_RST -1                      // tied to the Feather's RESET (or wire it to a pin and change this)
#define TFT_BL 5                        // D5 backlight (HIGH = on)
#define TFT_ROTATION 2                  // 0..3; pick so the text reads upright as mounted

// Buttons to ground (internal pull-ups).
#define BTN_RANGE 11                    // D11: press = range, hold 1 s = continuous ranging on/off
#define BTN_MODE 12                     // D12: press = next page, hold 3 s = sleep (it also wakes Meridian)
#define LONG_RANGE_MS 1000
#define LONG_MODE_MS 3000

#define SHOT_NOTIFY_MS 200              // 5 Hz shot packet
#define DRAW_MS 200                     // 5 Hz screen refresh (only changed lines are redrawn)
#define SOLSTICE_SCAN_EVERY_MS 20000    // look for Solstice this often while not linked
#define SOLSTICE_SCAN_MS 4000
#define SOLSTICE_WIND_STALE_MS 5000     // Solstice wind older than this is not used

using namespace ballistics;

// ---- Hardware objects ---------------------------------------------------------------------------------
static Adafruit_BME280 bme;
static Adafruit_BNO08x bno(-1);
static Adafruit_GPS gps(&Wire);
static Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
static Adafruit_MAX17048 maxGauge;
static Adafruit_LC709203F lcGauge;      // older Feather ESP32-S3 boards have this gauge instead
static Preferences prefs;
static void laserPower(bool on) { digitalWrite(LASER_EN_PIN, on ? HIGH : LOW); }
static lrf::ErdiRangeFinder<HardwareSerial> laser(Serial1, LASER_EN_PIN >= 0 ? laserPower : nullptr);
static nmea::Parser nmeaParser;

static bool bmeOk = false, imuOk = false, gpsOk = false, maxOk = false, lcOk = false;

// ---- Measured state ------------------------------------------------------------------------------------
static float temp_c = 15, rh_pct = 50, press_hpa = 1013.25f;
static orient::Calibration cal;
static orient::Angles ang{0, 0, 0, 0, 0, 0};
static bool angValid = false;
static uint8_t magCal = 0;             // 0..3 from the BNO085
static float headingAccDeg = -1;
static bool magCalRunning = false;

static bool rangeValid = false;
static float range_m = 0;
static uint32_t rangeAtMs = 0;
static uint8_t rangeSeq = 0;
static uint32_t noTargetUntil = 0, laserFaultUntil = 0;
static float laserOffset_m = 0;
static uint8_t targetMode = lrf::kTargetFirst;

static uint8_t batt_pct = 255;
static bool batt_charging = false;

static uint8_t units = 0;               // 0 mil, 1 MOA (Meridian's screen only)

// Charging from the cell-voltage trend (copied from Solstice's main.cpp, where it was tuned on the board:
// the MAX17048's charge-rate register reads flat on this board family). Call feed() once a second.
struct ChargeDetector {
    static constexpr int kEvery = 30, kHist = 10;
    float ema = 0, hist[kHist] = {};
    int n = 0;
    uint32_t ticks = 0;
    bool charging = false;
    void feed(float mv) {
        ema = ema > 0 ? ema + 0.2f * (mv - ema) : mv;
        if (++ticks % kEvery) return;
        for (int i = kHist - 1; i > 0; --i) hist[i] = hist[i - 1];
        hist[0] = ema;
        if (n < kHist) ++n;
        if (n < 2) return;
        const float step = hist[0] - hist[1];
        const float d2 = n > 4 ? hist[0] - hist[4] : 0.0f;
        const float d5 = n >= kHist ? hist[0] - hist[kHist - 1] : 0.0f;
        const bool topped = n >= kHist && ema >= 4150.0f && d5 >= -1.5f;
        const bool off = step <= -20.0f || d2 <= -4.0f || d5 <= -3.0f;
        const bool on = step >= 20.0f || d2 >= 4.0f || d5 >= 3.0f || topped;
        if (off) charging = false;
        else if (on) charging = true;
    }
};
static ChargeDetector chargeDet;

// ---- Shared with the Bluetooth task (guarded by mux) ---------------------------------------------------
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static protocol::Config cfg;
static bool cfgValid = false, cfgChanged = false;
static protocol::Request req;
static bool reqValid = false, reqChanged = false;
static protocol::CardRequest cardReq;
static bool cardPending = false, cardBad = false;
static meridian::Correction corr;
static bool corrValid = false;
static uint32_t corrAtMs = 0;
static uint16_t lastCorrSeq = 0;
static char message[meridian::kMessageMax + 1] = "";
static meridian::Command cmdQueue[4];
static uint8_t cmdHead = 0, cmdCount = 0;
static float solsticeWind = 0;
static uint32_t solsticeWindAtMs = 0;
static bool solsticeLinked = false;

static volatile bool sleepRequested = false;
static volatile uint32_t sleepAtMs = 0;

// Range card state (loop only)
static bool cardActive = false;
static uint8_t cardNext = 0;
static uint32_t cardLastMs = 0;
static protocol::Config cardCfg;
static protocol::CardRequest cardRun;
#define CARD_ROW_SPACING_MS 40

static bool solsticeWanted = true;
static uint8_t lastCmdResult = 0;

// ---- Settings that survive power-off ------------------------------------------------------------------
static void loadSettings() {
    prefs.begin("meridian", true);
    cal.mount = prefs.getUChar("mount", orient::kDefaultMount);
    cal.level_set = prefs.getBool("lvl", false);
    cal.pitch_zero = prefs.getFloat("p0", 0);
    cal.cant_zero = prefs.getFloat("c0", 0);
    cal.heading_set = prefs.getBool("hset", false);
    cal.heading_offset = prefs.getFloat("hoff", 0);
    cal.declination_set = prefs.getBool("dset", false);
    cal.declination = prefs.getFloat("decl", 0);
    units = prefs.getUChar("units", 0);
    laserOffset_m = prefs.getFloat("loff", 0);
    targetMode = prefs.getUChar("tmode", lrf::kTargetFirst);
    solsticeWanted = prefs.getBool("sol", true);
    prefs.end();
}
static void saveSettings() {
    prefs.begin("meridian", false);
    prefs.putUChar("mount", cal.mount);
    prefs.putBool("lvl", cal.level_set);
    prefs.putFloat("p0", cal.pitch_zero);
    prefs.putFloat("c0", cal.cant_zero);
    prefs.putBool("hset", cal.heading_set);
    prefs.putFloat("hoff", cal.heading_offset);
    prefs.putBool("dset", cal.declination_set);
    prefs.putFloat("decl", cal.declination);
    prefs.putUChar("units", units);
    prefs.putFloat("loff", laserOffset_m);
    prefs.putUChar("tmode", targetMode);
    prefs.putBool("sol", solsticeWanted);
    prefs.end();
}

// ---- Sleep ---------------------------------------------------------------------------------------------
#if __has_include(<esp_sleep.h>)
static void deepSleepNow() {
    laser.off();
    digitalWrite(TFT_BL, LOW);
    tft.enableSleep(true);
    digitalWrite(I2C_POWER, LOW);                          // BME280, BNO085, GPS off
    gpio_hold_en((gpio_num_t)I2C_POWER);
    if (LASER_EN_PIN >= 0) gpio_hold_en((gpio_num_t)LASER_EN_PIN);
    gpio_hold_en((gpio_num_t)TFT_BL);
    gpio_deep_sleep_hold_en();
    rtc_gpio_pullup_en((gpio_num_t)BTN_MODE);              // keep the button input high while asleep
    rtc_gpio_pulldown_dis((gpio_num_t)BTN_MODE);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_MODE, 0);
    esp_deep_sleep_start();
}
#else
static bool g_sleepStarted = false;                         // host tests record instead of sleeping
static void deepSleepNow() { g_sleepStarted = true; }
#endif

// ---- Battery -------------------------------------------------------------------------------------------
static void updateBattery() {
    float v = NAN, p = NAN;
    if (maxOk) { v = maxGauge.cellVoltage(); p = maxGauge.cellPercent(); }
    else if (lcOk) { v = lcGauge.cellVoltage(); p = lcGauge.cellPercent(); }
    if (std::isnan(v) || std::isnan(p) || v < 2.0f) { batt_pct = 255; batt_charging = false; return; }
    batt_pct = (uint8_t)(p < 0 ? 0 : p > 100 ? 100 : p + 0.5f);
    chargeDet.feed(v * 1000.0f);
    batt_charging = chargeDet.charging;
}

// ---- IMU -----------------------------------------------------------------------------------------------
static void enableImuReports() { bno.enableReport(SH2_ROTATION_VECTOR, 50000); }   // 20 Hz, magnetometer-fused

static void updateImu() {
    if (!imuOk) return;
    if (bno.wasReset()) enableImuReports();
    sh2_SensorValue_t v;
    while (bno.getSensorEvent(&v)) {
        if (v.sensorId != SH2_ROTATION_VECTOR) continue;
        const orient::Quat q{v.un.rotationVector.real, v.un.rotationVector.i, v.un.rotationVector.j, v.un.rotationVector.k};
        ang = orient::angles(q, cal);
        angValid = true;
        magCal = v.status & 0x03;
        headingAccDeg = v.un.rotationVector.accuracy * 57.29578f;
    }
}

// ---- GPS -----------------------------------------------------------------------------------------------
static void updateGps() {
    if (!gpsOk) return;
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if (now - lastMs < 100) return;                 // the PA1010D buffers; 10 polls a second is plenty
    lastMs = now;
    for (int i = 0; i < 128; ++i) {                  // at most 4 I2C reads of 32 bytes
        const char c = gps.read();
        if (c) nmeaParser.feed(c);
    }
}

// ---- Laser ---------------------------------------------------------------------------------------------
static void notifyShot();
static void requestLocalSolve();

static void updateLaser() {
    lrf::Result r;
    const uint32_t now = millis();
    if (!laser.poll(now, r)) return;
    switch (r.kind) {
        case lrf::Result::kRange:
            range_m = r.range_m + laserOffset_m;
            rangeValid = true; rangeAtMs = now; ++rangeSeq;
            noTargetUntil = laserFaultUntil = 0;
            Serial.printf("laser %.1f m (echo %u)\n", range_m, r.echo);
            notifyShot();
            requestLocalSolve();
            break;
        case lrf::Result::kNoTarget:
            noTargetUntil = now + 3000;
            Serial.println("laser: no target");
            break;
        case lrf::Result::kTimeout:
        case lrf::Result::kFault:
            laserFaultUntil = now + 5000;
            Serial.printf("laser: %s\n", r.kind == lrf::Result::kTimeout ? "no reply" : "fault");
            break;
        default: break;
    }
}

static bool laserFault() { return (int32_t)(laserFaultUntil - millis()) > 0 || (laser.present() && !laser.selfCheckOk()); }

// ---- Bluetooth: server side ------------------------------------------------------------------------------
static NimBLEServer* server = nullptr;
static NimBLECharacteristic *chSolution = nullptr, *chEnv = nullptr;
static NimBLECharacteristic *chShot = nullptr, *chPos = nullptr, *chStatus = nullptr;

class ServerCb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo&) override {
        // Keep advertising so the phone AND the spotter can both connect.
        if (s->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS - 1) NimBLEDevice::startAdvertising();
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override { NimBLEDevice::startAdvertising(); }
};

class ConfigCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        const NimBLEAttValue v = c->getValue();
        protocol::Config n;
        const bool ok = protocol::decode_config(v.data(), v.size(), n);
        portENTER_CRITICAL(&mux);
        if (ok) { cfg = n; cfgValid = true; } else { cfgValid = false; }
        cfgChanged = true;
        portEXIT_CRITICAL(&mux);
    }
};
class RequestCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        const NimBLEAttValue v = c->getValue();
        protocol::Request n;
        const bool ok = protocol::decode_request(v.data(), v.size(), n);
        if (ok && (n.flags & protocol::kFlagSleep)) { sleepAtMs = millis() + 400; sleepRequested = true; return; }
        portENTER_CRITICAL(&mux);
        if (ok) { req = n; reqValid = true; } else { reqValid = false; }
        reqChanged = true;
        portEXIT_CRITICAL(&mux);
    }
};
class CardCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        const NimBLEAttValue v = c->getValue();
        protocol::CardRequest n;
        const bool ok = protocol::decode_card(v.data(), v.size(), n);
        portENTER_CRITICAL(&mux);
        if (ok) { cardReq = n; cardBad = false; } else { cardBad = true; }
        cardPending = true;
        portEXIT_CRITICAL(&mux);
    }
};
class CorrectionCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        const NimBLEAttValue v = c->getValue();
        meridian::Correction n;
        if (!meridian::decode_correction(v.data(), v.size(), n)) return;
        portENTER_CRITICAL(&mux);
        if (n.flags & meridian::kCorrClear) corrValid = false;
        else { corr = n; corrValid = (n.flags & meridian::kCorrValid) != 0; corrAtMs = millis(); }
        lastCorrSeq = n.seq;
        portEXIT_CRITICAL(&mux);
    }
};
class CommandCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        const NimBLEAttValue v = c->getValue();
        meridian::Command n;
        const bool ok = meridian::decode_command(v.data(), v.size(), n);
        portENTER_CRITICAL(&mux);
        if (!ok) lastCmdResult = 1;
        else if (cmdCount < 4) { cmdQueue[(cmdHead + cmdCount) % 4] = n; ++cmdCount; }
        portEXIT_CRITICAL(&mux);
    }
};
class MessageCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        const NimBLEAttValue v = c->getValue();
        char tmp[meridian::kMessageMax + 1];
        if (!meridian::decode_message(v.data(), v.size(), tmp)) return;
        portENTER_CRITICAL(&mux);
        std::memcpy(message, tmp, sizeof message);
        portEXIT_CRITICAL(&mux);
    }
};

static void setupBle() {
    NimBLEDevice::init(BLE_NAME);
    server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCb());
    server->advertiseOnDisconnect(true);

    // 1. Solstice-compatible service: today's phone app works with Meridian unchanged.
    NimBLEService* sol = server->createService(BC_UUID_SERVICE);
    sol->createCharacteristic(BC_UUID_CONFIG, NIMBLE_PROPERTY::WRITE)->setCallbacks(new ConfigCb());
    sol->createCharacteristic(BC_UUID_REQUEST, NIMBLE_PROPERTY::WRITE)->setCallbacks(new RequestCb());
    sol->createCharacteristic(BC_UUID_CARD, NIMBLE_PROPERTY::WRITE)->setCallbacks(new CardCb());
    chSolution = sol->createCharacteristic(BC_UUID_SOLUTION, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    chEnv = sol->createCharacteristic(BC_UUID_ENV, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    sol->start();

    // 2. Meridian service: for the spotter (and a future Meridian page in the app).
    NimBLEService* mr = server->createService(MR_UUID_SERVICE);
    chShot = mr->createCharacteristic(MR_UUID_SHOT, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    chPos = mr->createCharacteristic(MR_UUID_POSITION, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    mr->createCharacteristic(MR_UUID_CORRECTION, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)->setCallbacks(new CorrectionCb());
    mr->createCharacteristic(MR_UUID_COMMAND, NIMBLE_PROPERTY::WRITE)->setCallbacks(new CommandCb());
    mr->createCharacteristic(MR_UUID_MESSAGE, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)->setCallbacks(new MessageCb());
    chStatus = mr->createCharacteristic(MR_UUID_STATUS, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    mr->start();

    // Advertising packet: the Solstice service UUID (so the phone app's device chooser lists Meridian).
    // Scan response: the name and the Meridian service UUID (so the spotter can find it). 31 bytes each.
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData advData, scanData;
    advData.setFlags(0x06);                          // general discoverable, no classic Bluetooth
    advData.addServiceUUID(BC_UUID_SERVICE);
    scanData.setName(BLE_NAME);
    scanData.addServiceUUID(MR_UUID_SERVICE);
    adv->setAdvertisementData(advData);
    adv->setScanResponseData(scanData);
    adv->enableScanResponse(true);
    adv->start();
}

// ---- Bluetooth: client side (wind from Solstice) --------------------------------------------------------
// Meridian connects to Solstice like the phone does and subscribes to its env notifications.
// NOTE: Solstice stops advertising once something is connected to it. If the phone is already connected
// to Solstice, Meridian can't find it (and vice versa). The phone can use Meridian instead: Meridian
// passes Solstice's wind through in its own env packet.
static NimBLEClient* solClient = nullptr;
static NimBLEAddress solAddr;
static bool solFound = false;
static uint32_t lastScanMs = 0;

class SolScanCb : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* d) override {
        if (!d->isAdvertisingService(NimBLEUUID(BC_UUID_SERVICE))) return;
        if (d->getName() != SOLSTICE_NAME) return;   // other Meridians advertise the same service
        solAddr = d->getAddress();
        solFound = true;
        NimBLEDevice::getScan()->stop();
    }
};
static SolScanCb solScanCb;

class SolClientCb : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient*, int) override {
        portENTER_CRITICAL(&mux);
        solsticeLinked = false;
        portEXIT_CRITICAL(&mux);
    }
};
static SolClientCb solClientCb;

static void onSolsticeEnv(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    meridian::SolsticeEnv e;
    if (!meridian::decode_solstice_env(data, len, e)) return;
    portENTER_CRITICAL(&mux);
    solsticeWind = e.wind_mps; solsticeWindAtMs = millis();
    portEXIT_CRITICAL(&mux);
}

static void serviceSolstice() {
    const uint32_t now = millis();
    if (!solsticeWanted) {
        if (solClient && solClient->isConnected()) solClient->disconnect();
        return;
    }
    if (solClient && solClient->isConnected()) return;
    if (solFound) {
        solFound = false;
        if (!solClient) {
            solClient = NimBLEDevice::createClient();
            solClient->setClientCallbacks(&solClientCb, false);
            solClient->setConnectTimeout(3000);
        }
        // Blocks for up to the connect timeout. Only happens when Solstice was just found.
        if (!solClient->connect(solAddr)) { Serial.println("Solstice: connect failed"); return; }
        NimBLERemoteService* svc = solClient->getService(BC_UUID_SERVICE);
        NimBLERemoteCharacteristic* env = svc ? svc->getCharacteristic(BC_UUID_ENV) : nullptr;
        if (!env || !env->subscribe(true, onSolsticeEnv)) {
            Serial.println("Solstice: no env characteristic");
            solClient->disconnect();
            return;
        }
        portENTER_CRITICAL(&mux);
        solsticeLinked = true;
        portEXIT_CRITICAL(&mux);
        Serial.println("Solstice: linked, receiving wind");
        return;
    }
    NimBLEScan* scan = NimBLEDevice::getScan();
    if (!scan->isScanning() && (lastScanMs == 0 || now - lastScanMs >= SOLSTICE_SCAN_EVERY_MS)) {
        lastScanMs = now ? now : 1;
        scan->setScanCallbacks(&solScanCb, false);
        scan->setActiveScan(true);                   // Solstice's name is in its scan response
        scan->start(SOLSTICE_SCAN_MS, false, true);
    }
}

// Live wind from Solstice, or NAN if there is none.
static float liveWind() {
    float w; uint32_t at; bool linked;
    portENTER_CRITICAL(&mux);
    w = solsticeWind; at = solsticeWindAtMs; linked = solsticeLinked;
    portEXIT_CRITICAL(&mux);
    if (!linked || at == 0 || millis() - at > SOLSTICE_WIND_STALE_MS) return NAN;
    return w;
}

// ---- Solving ----------------------------------------------------------------------------------------------
static uint16_t corrSeqLogged = 0;
static bool localOk = false;
static float localElev = 0, localWind = 0, localRange = 0;
static bool localSolveWanted = false;
static void requestLocalSolve() { localSolveWanted = true; }

static float rangeAgeS() { return rangeValid ? (millis() - rangeAtMs) / 1000.0f : 0; }

static protocol::SolutionPacket solveOne(const protocol::Config& c, uint8_t flags, float clockDeg, float manualWind,
                                         float manualSlope, float latDeg, float manualAz, float range, uint8_t row) {
    protocol::SolutionPacket out{};
    out.row = row;
    if ((flags & meridian::kFlagLaserRange) && rangeValid) range = range_m;
    out.range_m = range;

    Bullet b{c.bc, c.model == 1 ? DragModel::G7 : DragModel::G1, c.muzzle_mps};
    if (c.has_spin_data()) {
        b.mass_gr = c.mass_gr; b.diameter_in = c.diameter_m / 0.0254f;
        b.length_in = c.length_m / 0.0254f; b.twist_in = c.twist_m / 0.0254f;
    }
    Atmosphere zeroAtm{c.zero_temp_c, c.zero_pressure_hpa, c.zero_rh_pct};
    Atmosphere nowAtm = bmeOk ? Atmosphere{temp_c, press_hpa, rh_pct} : zeroAtm;

    const float live = liveWind();
    const float windUsed = ((flags & protocol::kFlagSensorWind) && !std::isnan(live)) ? live : manualWind;
    const float slopeUsed = ((flags & protocol::kFlagImuSlope) && angValid) ? (float)ang.pitch_deg : manualSlope;
    const float azUsed = ((flags & protocol::kFlagImuAzimuth) && angValid) ? (float)ang.bearing_deg : manualAz;
    const nmea::Fix& fx = nmeaParser.fix();
    const float latUsed = fx.valid ? (float)fx.lat_deg : latDeg;

    Shot shot;
    shot.sight_height = c.sight_height_m;
    shot.zero_range = c.zero_range_m;
    shot.wind_speed = windUsed;
    shot.wind_clock_deg = clockDeg;
    shot.slope_rad = slopeUsed * (3.14159265f / 180.0f);
    shot.coriolis = (flags & protocol::kFlagCoriolis) != 0;
    shot.latitude_deg = latUsed;
    shot.azimuth_deg = azUsed;
    shot.spin_drift = (flags & protocol::kFlagSpinDrift) != 0 && c.has_spin_data();
    shot.aero_jump = (flags & protocol::kFlagAeroJump) != 0 && c.has_spin_data();

    const Solution sol = solve(b, zeroAtm, nowAtm, shot, range);
    out.ok = sol.ok;
    out.elevation_mrad = sol.elevation_mrad; out.windage_mrad = sol.windage_mrad;
    out.tof_s = sol.tof; out.velocity_mps = sol.velocity; out.drop_m = sol.drop;
    out.spin_drift_mrad = sol.spin_drift_mrad;
    out.wind_mps = windUsed; out.slope_deg = slopeUsed;
    return out;
}

static void publishSolution(const protocol::SolutionPacket& s) {
    uint8_t buf[protocol::kSolutionSize];
    protocol::encode_solution(s, buf);
    chSolution->setValue(buf, sizeof buf);
    chSolution->notify();
}

// The phone app's live solution: exactly what Solstice would answer, with Meridian's sensors behind the flags.
static void solveForApp() {
    protocol::Config c; protocol::Request r; bool cv, rv;
    portENTER_CRITICAL(&mux);
    c = cfg; cv = cfgValid; r = req; rv = reqValid;
    portEXIT_CRITICAL(&mux);
    if (!cv || !rv) { protocol::SolutionPacket none{}; none.ok = false; publishSolution(none); return; }
    publishSolution(solveOne(c, r.flags, r.wind_clock_deg, r.wind_speed_mps, r.slope_deg, r.latitude_deg, r.azimuth_deg, r.range_m, 0));
}

// Meridian's own screen solution: always uses the laser range, the BNO085 angles, GPS latitude and
// Solstice wind when they exist. Wind direction and the spin/Coriolis/jump switches come from the app's
// last request. Needs the rifle profile (config) from the app.
static void solveForScreen() {
    protocol::Config c; protocol::Request r; bool cv, rv;
    portENTER_CRITICAL(&mux);
    c = cfg; cv = cfgValid; r = req; rv = reqValid;
    portEXIT_CRITICAL(&mux);
    if (!cv || (!rangeValid && !rv)) { localOk = false; return; }
    if (!rv) { r = protocol::Request{}; r.range_m = range_m; }
    const uint8_t flags = (r.flags & (protocol::kFlagCoriolis | protocol::kFlagSpinDrift | protocol::kFlagAeroJump)) |
                          protocol::kFlagSensorWind | protocol::kFlagImuSlope | protocol::kFlagImuAzimuth |
                          meridian::kFlagLaserRange;
    const protocol::SolutionPacket s = solveOne(c, flags, r.wind_clock_deg, r.wind_speed_mps, r.slope_deg,
                                                r.latitude_deg, r.azimuth_deg, r.range_m, 0);
    localOk = s.ok;
    localElev = s.elevation_mrad; localWind = s.windage_mrad; localRange = s.range_m;
}

static void serviceCard() {
    bool pending, bad;
    protocol::CardRequest cr;
    portENTER_CRITICAL(&mux);
    pending = cardPending; bad = cardBad; cr = cardReq;
    cardPending = false;
    portEXIT_CRITICAL(&mux);
    if (pending) {
        protocol::Config c; bool cv;
        portENTER_CRITICAL(&mux);
        c = cfg; cv = cfgValid;
        portEXIT_CRITICAL(&mux);
        if (bad || !cv) {
            protocol::SolutionPacket fail{}; fail.ok = false; fail.row = 1;
            publishSolution(fail);
            cardActive = false;
            return;
        }
        cardCfg = c; cardRun = cr; cardNext = 0; cardActive = true; cardLastMs = 0;
    }
    if (!cardActive) return;
    const uint32_t now = millis();
    if (cardLastMs != 0 && now - cardLastMs < CARD_ROW_SPACING_MS) return;
    cardLastMs = now ? now : 1;
    const float range = cardRun.start_m + cardRun.step_m * cardNext;
    // Card rows never use the laser range (bit7 cleared): each row is its own range.
    publishSolution(solveOne(cardCfg, cardRun.flags & ~meridian::kFlagLaserRange, cardRun.wind_clock_deg,
                             cardRun.wind_speed_mps, cardRun.slope_deg, cardRun.latitude_deg, cardRun.azimuth_deg,
                             range, cardNext + 1));
    if (++cardNext >= cardRun.count) cardActive = false;
}

// ---- Outgoing packets -------------------------------------------------------------------------------------
static uint8_t shotFlags() {
    uint8_t f = 0;
    if (rangeValid) f |= meridian::kShotRangeValid;
    if (angValid) f |= meridian::kShotImuOk;
    if (bmeOk) f |= meridian::kShotBmeOk;
    if (nmeaParser.fix().valid) f |= meridian::kShotGpsFix;
    if (cal.declination_set || cal.heading_set) f |= meridian::kShotTrueNorth;
    if (laser.busy()) f |= meridian::kShotRanging;
    if (batt_charging) f |= meridian::kShotCharging;
    if (laserFault()) f |= meridian::kShotLaserFault;
    return f;
}

static void notifyShot() {
    meridian::Shot s;
    s.flags = shotFlags();
    s.battery_pct = batt_pct;
    s.range_m = range_m; s.range_age_s = rangeAgeS();
    s.pitch_deg = ang.pitch_deg; s.cant_deg = ang.cant_deg; s.bearing_deg = ang.bearing_deg;
    s.temp_c = temp_c; s.pressure_hpa = press_hpa; s.rh_pct = rh_pct;
    s.range_seq = rangeSeq;
    s.mag_cal = magCal;
    s.heading_acc_deg = angValid ? headingAccDeg : -1;
    uint8_t buf[meridian::kShotSize];
    meridian::encode_shot(s, buf);
    chShot->setValue(buf, sizeof buf);
    chShot->notify();
}

static void notifyPosition() {
    const nmea::Fix& fx = nmeaParser.fix();
    meridian::Position p;
    p.lat_deg = fx.lat_deg; p.lon_deg = fx.lon_deg; p.alt_m = fx.alt_m;
    p.fix = fx.valid ? fx.quality : 0; p.sats = fx.sats; p.hdop = fx.hdop; p.utc = fx.utc;
    p.declination_deg = cal.declination;
    uint8_t buf[meridian::kPositionSize];
    meridian::encode_position(p, buf);
    chPos->setValue(buf, sizeof buf);
    chPos->notify();
}

static bool corrFresh() {
    bool v; uint32_t at;
    portENTER_CRITICAL(&mux);
    v = corrValid; at = corrAtMs;
    portEXIT_CRITICAL(&mux);
    return v && (millis() - at) / 1000.0 <= screen::kCorrStaleS;
}

static uint8_t laserState() {
    if (!laser.present()) return meridian::kLaserAbsent;
    if (laserFault()) return meridian::kLaserFault;
    if (laser.isContinuous()) return meridian::kLaserContinuous;
    return laser.busy() ? meridian::kLaserBusy : meridian::kLaserIdle;
}

static void notifyStatus() {
    meridian::Status s;
    bool linked; uint16_t seq; uint8_t res;
    portENTER_CRITICAL(&mux);
    linked = solsticeLinked; seq = lastCorrSeq; res = lastCmdResult;
    portEXIT_CRITICAL(&mux);
    s.flags = (linked ? meridian::kStatSolsticeLinked : 0) | (magCalRunning ? meridian::kStatMagCalRunning : 0) |
              (cal.level_set ? meridian::kStatLevelSet : 0) | (cal.heading_set ? meridian::kStatHeadingSet : 0) |
              (solsticeWanted ? meridian::kStatSolsticeWanted : 0) | (corrFresh() ? meridian::kStatCorrFresh : 0);
    s.mount = cal.mount; s.units = units;
    s.pitch_zero = cal.level_set ? cal.pitch_zero : 0; s.cant_zero = cal.level_set ? cal.cant_zero : 0;
    s.heading_offset = cal.heading_offset; s.declination = cal.declination; s.laser_offset_m = laserOffset_m;
    s.target_mode = targetMode; s.last_result = res; s.last_corr_seq = seq;
    s.centrals = server ? server->getConnectedCount() : 0;
    s.laser = laserState();
    uint8_t buf[meridian::kStatusSize];
    meridian::encode_status(s, buf);
    chStatus->setValue(buf, sizeof buf);
    chStatus->notify();
}

// The Solstice-compatible env packet for the phone app: Meridian's sensors, wind passed through from Solstice.
static void notifyEnv() {
    protocol::EnvPacket e{};
    const AirState air = air_state(Atmosphere{temp_c, press_hpa, rh_pct});
    const float w = liveWind();
    e.temp_c = temp_c; e.rh_pct = rh_pct; e.pressure_hpa = press_hpa;
    e.wind_mps = std::isnan(w) ? 0 : w;
    e.heading_deg = ang.bearing_deg; e.pitch_deg = ang.pitch_deg;
    e.density_alt_m = bmeOk ? protocol::density_altitude_m(air.density) : 0;
    e.flags = (bmeOk ? protocol::kEnvBmeOk : 0) | (angValid ? protocol::kEnvImuOk : 0);
    e.battery_pct = batt_pct;
    if (batt_charging) e.flags |= protocol::kEnvCharging;
    uint8_t buf[protocol::kEnvSize];
    protocol::encode_env(e, buf);
    chEnv->setValue(buf, sizeof buf);
    chEnv->notify();
}

// ---- Commands (from the phone or spotter; run in loop, not in the Bluetooth task) ---------------------------
static void startRange() {
    if (laserFault() && !laser.present()) return;
    laser.single(millis());
    noTargetUntil = 0;                            // the old range stays on screen until the new one arrives
}

static uint8_t runCommand(const meridian::Command& c) {
    using namespace meridian;
    switch (c.cmd) {
        case kCmdRange: startRange(); return 0;
        case kCmdContinuous: laser.continuous(c.u == 1, millis()); return 0;
        case kCmdLevelZero:
            if (!angValid) return 2;
            orient::set_level(cal, ang); saveSettings(); return 0;
        case kCmdClearLevel: cal.level_set = false; saveSettings(); return 0;
        case kCmdDeclination: cal.declination = c.f; cal.declination_set = true; saveSettings(); return 0;
        case kCmdAimBearing:
            if (!angValid) return 2;
            orient::set_aim_bearing(cal, ang, c.f); saveSettings(); return 0;
        case kCmdClearHeading: cal.heading_set = false; cal.heading_offset = 0; saveSettings(); return 0;
        case kCmdMount: cal.mount = c.u; cal.level_set = false; saveSettings(); return 0;  // old level zero is meaningless
        case kCmdMagCalStart:
            if (!imuOk) return 2;
            sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
            magCalRunning = true; return 0;
        case kCmdMagCalSave:
            if (!imuOk) return 2;
            if (sh2_saveDcdNow() != 0) return 2;
            magCalRunning = false; return 0;
        case kCmdUnits: units = c.u; saveSettings(); return 0;
        case kCmdLaserOffset: laserOffset_m = c.f; saveSettings(); return 0;
        case kCmdTargetMode: targetMode = c.u; laser.targetMode(targetMode); saveSettings(); return 0;
        case kCmdSolsticeLink: solsticeWanted = c.u == 1; saveSettings(); return 0;
        case kCmdSleep: sleepAtMs = millis() + 400; sleepRequested = true; return 0;
        default: return 1;
    }
}

static void serviceCommands() {
    for (;;) {
        meridian::Command c; bool have = false;
        portENTER_CRITICAL(&mux);
        if (cmdCount) { c = cmdQueue[cmdHead]; cmdHead = (cmdHead + 1) % 4; --cmdCount; have = true; }
        portEXIT_CRITICAL(&mux);
        if (!have) return;
        const uint8_t res = runCommand(c);
        Serial.printf("command %u -> %u\n", c.cmd, res);
        portENTER_CRITICAL(&mux);
        lastCmdResult = res;
        portEXIT_CRITICAL(&mux);
        notifyStatus();
        requestLocalSolve();
    }
}

// ---- Buttons ------------------------------------------------------------------------------------------------
static uint8_t page = 0;              // 0 = corrections, 1 = sensors
static bool screenDirty = true;

struct Button {
    int pin; uint32_t holdMs;
    bool down = false, longFired = false; uint32_t downAt = 0, lastChange = 0;
    // Returns 1 on a short press (on release), 2 when held past holdMs, else 0.
    int poll(uint32_t now) {
        const bool pressed = digitalRead(pin) == LOW;
        if (pressed != down && now - lastChange > 30) {          // 30 ms debounce
            lastChange = now; down = pressed;
            if (pressed) { downAt = now; longFired = false; }
            else if (!longFired) return 1;
        }
        if (down && !longFired && now - downAt >= holdMs) { longFired = true; return 2; }
        return 0;
    }
};
static Button btnRange{BTN_RANGE, LONG_RANGE_MS}, btnMode{BTN_MODE, LONG_MODE_MS};

static void serviceButtons() {
    const uint32_t now = millis();
    const int r = btnRange.poll(now);
    if (r == 1) startRange();
    if (r == 2) laser.continuous(!laser.isContinuous(), now);
    const int m = btnMode.poll(now);
    if (m == 1) { page = (page + 1) % 2; screenDirty = true; }
    if (m == 2) { sleepAtMs = now + 50; sleepRequested = true; }
}

// ---- Screen ------------------------------------------------------------------------------------------------
static uint16_t rgb(screen::Color c) {
    switch (c) {
        case screen::kGreen: return 0x07E0;
        case screen::kAmber: return 0xFD20;
        case screen::kRed: return 0xF800;
        case screen::kGrey: return 0x7BEF;
        case screen::kCyan: return 0x07FF;
        default: return 0xFFFF;
    }
}

static screen::Model shown;           // what is on the glass now

static void drawLine(const char* now, char* was, size_t wasSize, int16_t x, int16_t y, int16_t h, uint8_t size,
                     uint16_t color, bool force) {
    if (!force && std::strcmp(now, was) == 0) return;
    tft.fillRect(0, y, 240, h, 0x0000);
    tft.setTextSize(size);
    tft.setTextColor(color);
    tft.setCursor(x, y);
    tft.print(now);
    std::snprintf(was, wasSize, "%s", now);
}

static screen::Input screenInput() {
    screen::Input in;
    meridian::Correction c; bool cv; uint32_t at; char msg[sizeof message];
    portENTER_CRITICAL(&mux);
    c = corr; cv = corrValid; at = corrAtMs; std::memcpy(msg, message, sizeof msg);
    portEXIT_CRITICAL(&mux);
    static char msgCopy[sizeof message];
    std::memcpy(msgCopy, msg, sizeof msgCopy);
    const uint32_t now = millis();
    in.corr_valid = cv; in.corr_age_s = cv ? (now - at) / 1000.0 : 1e9;
    in.corr_elev = c.elevation_mrad; in.corr_wind = c.windage_mrad; in.corr_range = c.range_m;
    in.corr_lead = c.lead_mrad; in.corr_has_lead = (c.flags & meridian::kCorrHasLead) != 0;
    in.corr_hit = c.hit_pct; in.corr_target = c.target;
    in.local_ok = localOk; in.local_elev = localElev; in.local_wind = localWind; in.local_range = localRange;
    in.range_valid = rangeValid; in.range_m = range_m; in.range_age_s = rangeAgeS();
    in.ranging = laser.busy() && !laser.isContinuous();
    in.laser_fault = laserFault();
    in.laser_no_target = (int32_t)(noTargetUntil - now) > 0;
    in.imu_ok = angValid; in.cant_deg = ang.cant_deg; in.pitch_deg = ang.pitch_deg; in.bearing_deg = ang.bearing_deg;
    in.units = units; in.battery_pct = batt_pct; in.charging = batt_charging;
    in.centrals = server ? server->getConnectedCount() : 0;
    in.solstice = !std::isnan(liveWind());
    in.message = msgCopy;
    return in;
}

static void drawCantBar(const screen::Model& m, bool force) {
    static int lastX = -1000; static uint16_t lastColor = 0;
    const int y = 184, w = 200, x0 = 20;
    double c = m.cant_ok ? m.cant_deg : 0;
    if (c > 5) c = 5;
    if (c < -5) c = -5;
    const int x = x0 + w / 2 + (int)std::lround(c * (w / 2) / 5.0);
    const uint16_t col = rgb(m.cant_color);
    if (!force && x == lastX && col == lastColor) return;
    lastX = x; lastColor = col;
    tft.fillRect(0, y, 240, 16, 0x0000);
    tft.drawRect(x0, y + 2, w, 12, 0x7BEF);
    tft.drawFastVLine(x0 + w / 2, y, 16, 0xFFFF);
    if (m.cant_ok) tft.fillRect(x - 4, y + 3, 9, 10, col);
}

static void drawMainPage(bool force) {
    const screen::Model m = screen::build(screenInput());
    drawLine(m.title, shown.title, sizeof shown.title, 0, 4, 18, 2, rgb(m.title_color), force);
    drawLine(m.status, shown.status, sizeof shown.status, 0, 24, 10, 1, 0xFFFF, force);
    drawLine(m.elev, shown.elev, sizeof shown.elev, 10, 40, 42, 5, rgb(m.dial_color), force || m.dial_color != shown.dial_color);
    drawLine(m.wind, shown.wind, sizeof shown.wind, 10, 88, 42, 5, rgb(m.dial_color), force || m.dial_color != shown.dial_color);
    shown.dial_color = m.dial_color;
    drawLine(m.range, shown.range, sizeof shown.range, 10, 136, 24, 3, rgb(m.range_color), force || m.range_color != shown.range_color);
    shown.range_color = m.range_color;
    drawLine(m.extra, shown.extra, sizeof shown.extra, 10, 164, 18, 2, 0xFFE0, force);
    drawCantBar(m, force);
    drawLine(m.angles, shown.angles, sizeof shown.angles, 10, 204, 16, 2, 0xFFFF, force);
    drawLine(m.message, shown.message, sizeof shown.message, 0, 224, 16, 2, 0x07E0, force);
}

static void drawSensorPage() {
    // Redrawn whole once a second; it is a setup page, not used while shooting.
    static uint32_t last = 0;
    const uint32_t now = millis();
    if (!screenDirty && now - last < 1000) return;
    last = now;
    const nmea::Fix& fx = nmeaParser.fix();
    char l[12][28];
    std::snprintf(l[0], 28, "SENSORS");
    std::snprintf(l[1], 28, "T %.1fC RH %.0f%%", temp_c, rh_pct);
    std::snprintf(l[2], 28, "P %.1f hPa", press_hpa);
    std::snprintf(l[3], 28, "GPS %s sat %u", fx.valid ? "fix" : "none", fx.sats);
    std::snprintf(l[4], 28, "%.5f %.5f", fx.lat_deg, fx.lon_deg);
    std::snprintf(l[5], 28, "IMU %s cal %u/3", imuOk ? "ok" : "MISSING", magCal);
    std::snprintf(l[6], 28, "P%+.1f C%+.1f B%03.0f", ang.pitch_deg, ang.cant_deg, ang.bearing_deg);
    std::snprintf(l[7], 28, "LASER %s", laser.present() ? (laserFault() ? "FAULT" : "ok") : "silent");
    const float w = liveWind();
    if (std::isnan(w)) std::snprintf(l[8], 28, "WIND no Solstice");
    else std::snprintf(l[8], 28, "WIND %.1f m/s", w);
    std::snprintf(l[9], 28, "LVL %s HDG %s", cal.level_set ? "set" : "--", cal.heading_set ? "set" : "--");
    std::snprintf(l[10], 28, "DECL %+.1f MNT %u", cal.declination, cal.mount);
    std::snprintf(l[11], 28, "BATT %u%%%s", batt_pct <= 100 ? batt_pct : 0, batt_charging ? " chg" : "");
    tft.fillScreen(0x0000);
    tft.setTextSize(2);
    for (int i = 0; i < 12; ++i) {
        tft.setTextColor(i == 0 ? 0x07FF : 0xFFFF);
        tft.setCursor(0, i * 20);
        tft.print(l[i]);
    }
}

static void drawScreen() {
    static uint8_t lastPage = 255;
    const bool force = screenDirty || page != lastPage;
    if (page != lastPage) { tft.fillScreen(0x0000); shown = screen::Model{}; }
    lastPage = page;
    if (page == 0) drawMainPage(force); else drawSensorPage();
    screenDirty = false;
}

// ---- Arduino entry points ------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);
    pinMode(I2C_POWER, OUTPUT);
    digitalWrite(I2C_POWER, HIGH);                // STEMMA QT power: BME280, BNO085, GPS
    if (LASER_EN_PIN >= 0) { pinMode(LASER_EN_PIN, OUTPUT); digitalWrite(LASER_EN_PIN, LOW); }   // the driver powers it
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    pinMode(BTN_RANGE, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);
    delay(10);

    loadSettings();

    tft.init(240, 240);
    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(0x0000);
    tft.setTextSize(3); tft.setTextColor(0x07FF); tft.setCursor(30, 100); tft.print("MERIDIAN");

    Wire.begin();
    Wire.setClock(400000);
    bmeOk = bme.begin(BME_ADDR);
    maxOk = maxGauge.begin();
    if (!maxOk) {
        lcOk = lcGauge.begin();
        if (lcOk) lcGauge.setPackSize(LC709203F_APA_1000MAH);   // closest setting to the 1200 mAh cell
    }
    imuOk = bno.begin_I2C(BNO_ADDR);
    if (imuOk) enableImuReports();
    gpsOk = gps.begin(GPS_ADDR);
    if (gpsOk) {
        gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);          // only the two sentences Meridian parses
        gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    }

    Serial1.begin(LASER_BAUD, SERIAL_8N1, RX, TX);
    laser.targetMode(targetMode);                         // remembered; sent once the laser has started
    laser.begin(millis());                                // power on, self-check, then off after a minute idle

    Serial.printf("BME280: %s\n", bmeOk ? "ok" : "NOT FOUND");
    Serial.printf("BNO085: %s\n", imuOk ? "ok" : "NOT FOUND");
    Serial.printf("GPS (PA1010D): %s\n", gpsOk ? "ok" : "NOT FOUND");
    Serial.printf("Battery gauge: %s\n", maxOk ? "MAX17048" : lcOk ? "LC709203F" : "NOT FOUND");

    setupBle();
    Serial.println("Advertising as " BLE_NAME);
    screenDirty = true;
    tft.fillScreen(0x0000);
}

void loop() {
    if (sleepRequested && (int32_t)(millis() - sleepAtMs) >= 0) {
        Serial.println("Sleeping (MODE button wakes Meridian)");
        delay(50);
        deepSleepNow();
        sleepRequested = false;                  // only reached on the host stub
    }
    updateLaser();
    updateImu();
    updateGps();
    serviceButtons();
    serviceCommands();
    serviceSolstice();

    const uint32_t now = millis();
    static uint32_t lastSlowMs = 0, lastShotMs = 0, lastDrawMs = 0;

    bool appSolve = false;
    portENTER_CRITICAL(&mux);
    if (cfgChanged || reqChanged) { appSolve = true; cfgChanged = reqChanged = false; }
    portEXIT_CRITICAL(&mux);

    if (now - lastSlowMs >= 1000) {              // 1 Hz: atmosphere, battery, env, position, status, re-solve
        lastSlowMs = now;
        if (bmeOk) {
            temp_c = bme.readTemperature();
            rh_pct = bme.readHumidity();
            press_hpa = bme.readPressure() / 100.0f;   // station pressure, which the solver wants
        }
        updateBattery();
        notifyEnv();
        notifyPosition();
        notifyStatus();
        appSolve = true;
        localSolveWanted = true;
        static uint8_t logTick = 0;
        if (++logTick >= 5) {
            logTick = 0;
            const nmea::Fix& fx = nmeaParser.fix();
            Serial.printf("env T=%.1f C RH=%.1f %% P=%.1f hPa | pitch=%.1f cant=%.1f brg=%.0f cal=%u | gps=%s sats=%u | "
                          "range=%.1f m | batt=%u%% | links=%u solstice=%s\n",
                          temp_c, rh_pct, press_hpa, ang.pitch_deg, ang.cant_deg, ang.bearing_deg, magCal,
                          fx.valid ? "fix" : "none", fx.sats, rangeValid ? range_m : 0.0f, batt_pct,
                          server ? server->getConnectedCount() : 0, std::isnan(liveWind()) ? "no" : "yes");
        }
    }
    if (appSolve) solveForApp();
    if (localSolveWanted) { localSolveWanted = false; solveForScreen(); }
    serviceCard();

    if (now - lastShotMs >= SHOT_NOTIFY_MS) { lastShotMs = now; notifyShot(); }
    {
        uint16_t seq;
        portENTER_CRITICAL(&mux);
        seq = lastCorrSeq;
        portEXIT_CRITICAL(&mux);
        if (seq != corrSeqLogged) { corrSeqLogged = seq; Serial.printf("correction #%u from spotter\n", seq); }
    }
    if (now - lastDrawMs >= DRAW_MS) { lastDrawMs = now; drawScreen(); }
    delay(2);
}
