// Solstice (Celestial project, V1 handheld) ballistic computer firmware for ESP32-S3.
//
// UNTESTED: written without a toolchain or hardware. The solver and packet code
// it calls (core/) are tested on a PC; everything touching real hardware here
// (BLE stack, sensors, pulse input) still needs a first run on the board.
//
// Roles: BME280 -> temp/humidity/station pressure, BNO085 -> heading/pitch,
// impeller read by a DRV5055 analog Hall sensor on A5 -> wind speed, BLE -> the phone is the screen.
//
// Libraries: NimBLE-Arduino 2.x, Adafruit BME280, Adafruit BNO08x.

#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_MAX1704X.h>
#if __has_include(<esp_sleep.h>)
#include <esp_sleep.h>
#include <driver/gpio.h>
#endif

#include "ballistics.h"
#include "protocol.h"
#include "wind_pulse.h"
#if __has_include(<esp_timer.h>)
#include <esp_timer.h>
#endif
#if __has_include(<esp_system.h>)
#include <esp_system.h>
static int bootResetReason() { return (int)esp_reset_reason(); }   // 1 power-on, 3 software, 4 panic, 5-7 watchdog
#else
static int bootResetReason() { return 0; }
#endif

// ---- Hardware settings (check these against your wiring) --------------------
#define BLE_NAME "Solstice"         // Bluetooth advertised name (V1 handheld of the Celestial project)
// Wind sensor. WIND_ANALOG 1: DRV5055A1 linear Hall sensor on A5 (GPIO 8, an ADC1 pin), no pull-up,
// sampled every WIND_SAMPLE_US and turned into pulses by core/wind_pulse.h (2 pulses per revolution).
// WIND_ANALOG 0: the original DRV5033 Hall switch on A0 (GPIO 18) with a 10K pull-up, counted on
// falling edges. The DRV5033AJ proved too insensitive for the Kestrel impeller's magnet (2026-11-12).
#ifndef WIND_ANALOG
#define WIND_ANALOG 1
#endif
#define WIND_ADC_PIN 8              // Feather silkscreen A5
#define WIND_SAMPLE_US 500          // 2 kHz
#define WIND_PIN 18                 // Feather silkscreen A0, digital switch mode only
#define WIND_MIN_PULSE_US 1500      // ignore pulses closer together than this (debounce)
#define BME_ADDR 0x77               // Adafruit BME280 breakout default; 0x76 if you cut the jumper
// The Feather ESP32-S3 switches power to its STEMMA QT / I2C pull-ups with GPIO 7. It must be
// HIGH before any I2C device answers. The board package names it NEOPIXEL_I2C_POWER.
#ifndef NEOPIXEL_I2C_POWER
#define NEOPIXEL_I2C_POWER 7
#endif

// Battery: this Feather has a MAX17048 fuel gauge (silkscreen "MAX17048 Monitor") on the I2C bus,
// address 0x36. It gives the real state of charge and cell voltage. If the chip is not found, fall
// back to the VBAT sense, a 1/2 divider on A13 (GPIO 12), and a voltage-to-percent curve.
// The fallback pin is UNVERIFIED: check it against a multimeter before trusting it.
#define BATT_PIN 12
#define BATT_DIVIDER 2.0f

// Sleep: the app's Sleep button sends a request with flag bit6. The board then drops the I2C power rail and
// goes into deep sleep. The BOOT button (GPIO 0, active low) or RESET wakes it. A sleeping board still charges.
#define WAKE_PIN 0

// CALIBRATION: metres/second per pulse-per-second. This is a placeholder. You must
// calibrate your impeller (e.g. against a known anemometer or from a car on a calm
// day) and replace it. Wind numbers are NOT trustworthy until you do.
#define WIND_MPS_PER_HZ 0.50f
#define WIND_START_HZ 0.5f          // below this the impeller is considered stalled -> 0

// ---- State ---------------------------------------------------------------------
using namespace ballistics;

static Adafruit_BME280 bme;
static Adafruit_BNO08x bno(-1);
static Adafruit_MAX17048 fuel;
static bool bmeOk = false, imuOk = false, fuelOk = false;

static volatile uint32_t pulseCount = 0;
static volatile uint32_t lastPulseUs = 0;

static float heading_deg = 0, pitch_deg = 0;
static float temp_c = 15, rh_pct = 50, press_hpa = 1013.25f;
static float wind_mps = 0;
static float batt_mv = 0;            // cell voltage, 0 = not read yet
static uint8_t batt_pct = 255;       // 0..100, 255 = unknown
static bool batt_charging = false;   // external power is charging the cell (needs the gauge)
static float batt_rate = NAN;        // gauge charge rate in %/hour, NAN = none (serial log only; it is flat on this board)

// Charging from the cell-voltage trend. The MAX17048's charge-rate register reads a flat -4.2 %/h on this
// board even while charging, so use the voltage instead: call feed() once a second. A reading is kept every
// 30 s (smoothed over ~5 s) and the last 5 minutes are compared. Rising = charging, falling = not. A full cell
// on USB sits flat near 4.2 V, so a high, not-falling voltage also counts as charging. A step of 20 mV in 30 s
// (USB plugged in or pulled out) switches it quickly. The thresholds are estimates, so tune them on the board.
struct ChargeDetector {
    static constexpr int kEvery = 30, kHist = 10;          // one point per 30 s, 10 points = 4.5 min span
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
        const float step = hist[0] - hist[1];                              // over 30 s
        const float d2 = n > 4 ? hist[0] - hist[4] : 0.0f;                 // over 2 min
        const float d5 = n >= kHist ? hist[0] - hist[kHist - 1] : 0.0f;    // over 4.5 min
        const bool topped = n >= kHist && ema >= 4150.0f && d5 >= -1.5f;   // full and not discharging
        const bool off = step <= -20.0f || d2 <= -4.0f || d5 <= -3.0f;
        const bool on = step >= 20.0f || d2 >= 4.0f || d5 >= 3.0f || topped;
        if (off) charging = false;
        else if (on) charging = true;                                      // otherwise keep the last answer
    }
};
static ChargeDetector chargeDet;
static volatile bool sleepRequested = false;
static volatile uint32_t sleepAtMs = 0;

#if __has_include(<esp_sleep.h>)
static void deepSleepNow() {
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);               // switch off the I2C rail (BME280)
    gpio_hold_en((gpio_num_t)NEOPIXEL_I2C_POWER);        // keep it off while asleep
    gpio_deep_sleep_hold_en();
    esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_PIN, 0);   // BOOT button pulls GPIO 0 low
    esp_deep_sleep_start();
}
#else
static bool g_sleepStarted = false;                       // host tests: record instead of sleeping
static void deepSleepNow() { g_sleepStarted = true; }
#endif

static void updateBattery() {
    if (fuelOk) {
        const float v = fuel.cellVoltage(), p = fuel.cellPercent();
        if (!std::isnan(v) && !std::isnan(p) && v > 2.0f) {
            batt_mv = v * 1000.0f;
            batt_pct = (uint8_t)(p < 0 ? 0 : p > 100 ? 100 : p + 0.5f);   // the gauge can read a little over 100
            batt_rate = fuel.chargeRate();                  // diagnostic only
            chargeDet.feed(batt_mv);                        // charging comes from the voltage trend
            batt_charging = chargeDet.charging;
            return;
        }
    }
    batt_charging = false;                                                           // the pin fallback cannot tell
    batt_rate = NAN;
    // Fallback (no gauge, or it stopped answering): VBAT pin through the divider.
    const float mv = analogReadMilliVolts(BATT_PIN) * BATT_DIVIDER;
    if (mv < 2000.0f || mv > 5500.0f) { batt_mv = 0; batt_pct = 255; return; }   // pin floating or not a battery
    batt_mv = batt_mv > 0 ? batt_mv * 0.9f + mv * 0.1f : mv;                      // smooth out load ripple
    batt_pct = protocol::battery_percent_from_mv(batt_mv);
}

static NimBLECharacteristic *chSolution = nullptr, *chEnv = nullptr;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static protocol::Config cfg;
static bool cfgValid = false, cfgChanged = false;
static protocol::Request req;
static bool reqValid = false, reqChanged = false;

// Range card: the BLE callback only stores the request; loop() computes and streams the rows.
static protocol::CardRequest cardReq;
static bool cardPending = false;          // set by BLE callback, consumed by loop()
static bool cardBad = false;              // request failed validation
static bool cardActive = false;           // currently streaming rows
static uint8_t cardNext = 0;              // next row index to send (0-based)
static uint32_t cardLastMs = 0;
static protocol::Config cardCfg;          // config snapshot taken when the sweep starts
static bool cardCfgValid = false;
#define CARD_ROW_SPACING_MS 40            // pace notifications so the BLE stack never drops rows

// ---- Wind impeller --------------------------------------------------------------
// Analog mode: an esp_timer callback (runs in the esp_timer task, not an ISR) reads the sensor.
static wind::PulseDetector windDet;
[[maybe_unused]] static void windSample(void*) {
    if (windDet.feed((float)analogReadMilliVolts(WIND_ADC_PIN))) pulseCount++;
}
[[maybe_unused]] static void IRAM_ATTR onWindPulse() {
    const uint32_t now = micros();
    if (now - lastPulseUs > WIND_MIN_PULSE_US) {
        pulseCount++;
        lastPulseUs = now;
    }
}

// 12 samples x 250 ms = 3 s rolling average, like the Kestrel's display.
static float hzRing[12] = {0};
static int hzIdx = 0;
static uint32_t lastPulseSnapshot = 0, lastWindMs = 0;

static void updateWind() {
    const uint32_t nowMs = millis();
    if (nowMs - lastWindMs < 250) return;
    const uint32_t dt = nowMs - lastWindMs;
    lastWindMs = nowMs;
    const uint32_t c = pulseCount;
    const float hz = (c - lastPulseSnapshot) * 1000.0f / dt;
    lastPulseSnapshot = c;
    hzRing[hzIdx] = hz;
    hzIdx = (hzIdx + 1) % 12;
    float sum = 0;
    for (float v : hzRing) sum += v;
    const float avg = sum / 12.0f;
    wind_mps = (avg < WIND_START_HZ) ? 0.0f : avg * WIND_MPS_PER_HZ;
}

// ---- IMU --------------------------------------------------------------------------
static void enableImuReports() { bno.enableReport(SH2_ROTATION_VECTOR, 50000); }  // 20 Hz

static void updateImu() {
    if (!imuOk) return;
    if (bno.wasReset()) enableImuReports();
    sh2_SensorValue_t v;
    while (bno.getSensorEvent(&v)) {
        if (v.sensorId != SH2_ROTATION_VECTOR) continue;
        const float qw = v.un.rotationVector.real, qx = v.un.rotationVector.i;
        const float qy = v.un.rotationVector.j, qz = v.un.rotationVector.k;
        // Yaw (heading) and pitch from the quaternion. Axis conventions depend on how the
        // board is mounted; verify heading against a phone compass and pitch by tilting.
        float yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 57.29578f;
        heading_deg = fmodf(360.0f - yaw + 360.0f, 360.0f);
        float s = 2.0f * (qw * qy - qz * qx);
        if (s > 1) s = 1;
        if (s < -1) s = -1;
        pitch_deg = asinf(s) * 57.29578f;
    }
}

// ---- BLE ---------------------------------------------------------------------------
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
        if (ok && (n.flags & protocol::kFlagSleep)) {       // one-shot sleep command: not a solve request
            sleepAtMs = millis() + 400;                      // give the write response time to go out first
            sleepRequested = true;
            return;
        }
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

// Keep advertising after a connection, so the phone AND Meridian (which pulls Solstice's wind) can both connect.
// NimBLE stops advertising on each new connection; restart it while there is room for another.
#ifndef CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 3
#endif
class ServerCb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo&) override {
        if (s->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS) NimBLEDevice::startAdvertising();
    }
};

static void setupBle() {
    NimBLEDevice::init(BLE_NAME);
    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCb());
    server->advertiseOnDisconnect(true);
    NimBLEService* svc = server->createService(BC_UUID_SERVICE);

    auto* config = svc->createCharacteristic(BC_UUID_CONFIG, NIMBLE_PROPERTY::WRITE);
    config->setCallbacks(new ConfigCb());
    auto* request = svc->createCharacteristic(BC_UUID_REQUEST, NIMBLE_PROPERTY::WRITE);
    request->setCallbacks(new RequestCb());
    auto* card = svc->createCharacteristic(BC_UUID_CARD, NIMBLE_PROPERTY::WRITE);
    card->setCallbacks(new CardCb());
    chSolution = svc->createCharacteristic(BC_UUID_SOLUTION, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    chEnv = svc->createCharacteristic(BC_UUID_ENV, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BC_UUID_SERVICE);   // lets the phone page filter on our service
    adv->enableScanResponse(true);
    adv->setName(BLE_NAME);          // NimBLE 2.x does not advertise the name unless told to
    adv->start();
}

// ---- Solving -----------------------------------------------------------------------
static void publishSolution(const protocol::SolutionPacket& s) {
    uint8_t buf[protocol::kSolutionSize];
    protocol::encode_solution(s, buf);
    chSolution->setValue(buf, sizeof buf);
    chSolution->notify();
}

// Solve one range with the current conditions. `row` is 0 for the live solution or 1..N for a card row.
static protocol::SolutionPacket solveOne(const protocol::Config& c, uint8_t flags, float clockDeg,
                                         float manualWind, float manualSlope, float latDeg, float manualAz,
                                         float range, uint8_t row) {
    protocol::SolutionPacket out{};
    out.row = row;
    out.range_m = range;

    Bullet b{c.bc, c.model == 1 ? DragModel::G7 : DragModel::G1, c.muzzle_mps};
    if (c.has_spin_data()) {
        b.mass_gr = c.mass_gr;
        b.diameter_in = c.diameter_m / 0.0254f;
        b.length_in = c.length_m / 0.0254f;
        b.twist_in = c.twist_m / 0.0254f;
    }
    Atmosphere zeroAtm{c.zero_temp_c, c.zero_pressure_hpa, c.zero_rh_pct};
    // If the BME280 is missing, fall back to the zero conditions so we still give an answer.
    Atmosphere nowAtm = bmeOk ? Atmosphere{temp_c, press_hpa, rh_pct} : zeroAtm;

    const float windUsed = (flags & protocol::kFlagSensorWind) ? wind_mps : manualWind;
    const float slopeUsed = ((flags & protocol::kFlagImuSlope) && imuOk) ? pitch_deg : manualSlope;

    Shot shot;
    shot.sight_height = c.sight_height_m;
    shot.zero_range = c.zero_range_m;
    shot.wind_speed = windUsed;
    shot.wind_clock_deg = clockDeg;
    shot.slope_rad = slopeUsed * (3.14159265f / 180.0f);
    // Firing direction for Coriolis: the IMU heading (magnetic; a few degrees of declination
    // error is negligible here) or the number typed on the phone.
    const float azUsed = ((flags & protocol::kFlagImuAzimuth) && imuOk) ? heading_deg : manualAz;
    shot.coriolis = (flags & protocol::kFlagCoriolis) != 0;
    shot.latitude_deg = latDeg;
    shot.azimuth_deg = azUsed;
    shot.spin_drift = (flags & protocol::kFlagSpinDrift) != 0 && c.has_spin_data();
    shot.aero_jump = (flags & protocol::kFlagAeroJump) != 0 && c.has_spin_data();

    const uint32_t t0 = micros();
    const Solution sol = solve(b, zeroAtm, nowAtm, shot, range);
    Serial.printf("solve row=%u %u us ok=%d up=%.2f mrad\n", (unsigned)row, (unsigned)(micros() - t0),
                  sol.ok, sol.elevation_mrad);

    out.ok = sol.ok;
    out.elevation_mrad = sol.elevation_mrad;
    out.windage_mrad = sol.windage_mrad;
    out.tof_s = sol.tof;
    out.velocity_mps = sol.velocity;
    out.drop_m = sol.drop;
    out.spin_drift_mrad = sol.spin_drift_mrad;
    out.wind_mps = windUsed;
    out.slope_deg = slopeUsed;
    return out;
}

static void solveAndPublish() {
    protocol::Config c; protocol::Request r; bool cv, rv;
    portENTER_CRITICAL(&mux);
    c = cfg; cv = cfgValid; r = req; rv = reqValid;
    portEXIT_CRITICAL(&mux);

    if (!cv || !rv) { protocol::SolutionPacket none{}; none.ok = false; publishSolution(none); return; }
    publishSolution(solveOne(c, r.flags, r.wind_clock_deg, r.wind_speed_mps, r.slope_deg, r.latitude_deg, r.azimuth_deg, r.range_m, 0));
}

// Starts a sweep when the phone asks for one, then sends one row per CARD_ROW_SPACING_MS.
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
            // Tell the phone right away: one failed row 1 ends its wait.
            protocol::SolutionPacket fail{};
            fail.ok = false; fail.row = 1;
            publishSolution(fail);
            cardActive = false;
            return;
        }
        cardCfg = c; cardCfgValid = true;
        cardReq = cr;
        cardNext = 0;
        cardActive = true;
        cardLastMs = 0;                       // send the first row immediately
    }

    if (!cardActive) return;
    const uint32_t now = millis();
    if (cardLastMs != 0 && now - cardLastMs < CARD_ROW_SPACING_MS) return;
    cardLastMs = now ? now : 1;

    const float range = cardReq.start_m + cardReq.step_m * cardNext;
    publishSolution(solveOne(cardCfg, cardReq.flags, cardReq.wind_clock_deg, cardReq.wind_speed_mps,
                             cardReq.slope_deg, cardReq.latitude_deg, cardReq.azimuth_deg, range, cardNext + 1));
    if (++cardNext >= cardReq.count) cardActive = false;
}

static void publishEnv() {
    protocol::EnvPacket e{};
    const AirState air = air_state(Atmosphere{temp_c, press_hpa, rh_pct});
    e.temp_c = temp_c; e.rh_pct = rh_pct; e.pressure_hpa = press_hpa;
    e.wind_mps = wind_mps; e.heading_deg = heading_deg; e.pitch_deg = pitch_deg;
    e.density_alt_m = bmeOk ? protocol::density_altitude_m(air.density) : 0;
    e.flags = (bmeOk ? protocol::kEnvBmeOk : 0) | (imuOk ? protocol::kEnvImuOk : 0);
    e.battery_pct = batt_pct;
    if (batt_charging) e.flags |= protocol::kEnvCharging;
    uint8_t buf[protocol::kEnvSize];
    protocol::encode_env(e, buf);
    chEnv->setValue(buf, sizeof buf);
    chEnv->notify();
}

// ---- Arduino entry points --------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);   // power the I2C bus (BME280)
    delay(10);                                // let the sensor power up
    Wire.begin();
    bmeOk = bme.begin(BME_ADDR);
    fuelOk = fuel.begin();
    Serial.printf("MAX17048 battery gauge: %s\n", fuelOk ? "ok" : "NOT FOUND (using VBAT pin)");
    Serial.printf("BME280: %s\n", bmeOk ? "ok" : "NOT FOUND");
    imuOk = bno.begin_I2C();
    if (imuOk) enableImuReports();
    Serial.printf("BNO085: %s\n", imuOk ? "ok" : "NOT FOUND");

#if WIND_ANALOG
#if __has_include(<esp_timer.h>)
    pinMode(WIND_ADC_PIN, INPUT);
    analogSetPinAttenuation(WIND_ADC_PIN, ADC_11db);   // full 0-3.1 V range for the DRV5055
    esp_timer_create_args_t targs = {};
    targs.callback = &windSample;
    targs.name = "wind";
    esp_timer_handle_t windTimer = nullptr;
    if (esp_timer_create(&targs, &windTimer) == ESP_OK) esp_timer_start_periodic(windTimer, WIND_SAMPLE_US);
    else Serial.println("Wind sampler: timer NOT started");
#endif
#else
    pinMode(WIND_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WIND_PIN), onWindPulse, FALLING);
#endif
    lastWindMs = millis();

    setupBle();
    Serial.println("Advertising as " BLE_NAME);
}

void loop() {
    if (sleepRequested && (int32_t)(millis() - sleepAtMs) >= 0) {
        Serial.println("Sleeping (BOOT or RESET wakes the board)");
        delay(50);
        deepSleepNow();
        sleepRequested = false;                              // only reached on the host stub
    }
    updateWind();
    updateImu();

    static uint32_t lastSlowMs = 0;
    const uint32_t now = millis();

    bool doSolve = false;
    portENTER_CRITICAL(&mux);
    if (cfgChanged || reqChanged) { doSolve = true; cfgChanged = reqChanged = false; }
    portEXIT_CRITICAL(&mux);

    if (now - lastSlowMs >= 1000) {          // 1 Hz: read atmosphere, push env, re-solve
        lastSlowMs = now;
        if (bmeOk) {
            temp_c = bme.readTemperature();
            rh_pct = bme.readHumidity();
            press_hpa = bme.readPressure() / 100.0f;   // station pressure, which is what the solver wants
        }
        updateBattery();
        publishEnv();
        static uint8_t envLogTick = 0;        // every 5 s, so a late serial monitor still sees status
        if (++envLogTick >= 5) {
            envLogTick = 0;
            Serial.printf("env bme=%s T=%.1f C RH=%.1f %% P=%.1f hPa wind=%.2f m/s pulses=%lu swing=%.0fmV batt=%.0f mV %u%% (%s%s) rate=%.1f %%/h up=%lus reset=%d\n",
                          bmeOk ? "ok" : "NOT FOUND", temp_c, rh_pct, press_hpa, wind_mps,
                          (unsigned long)pulseCount, windDet.swing(), batt_mv,
                          (unsigned)batt_pct, fuelOk ? "gauge" : "pin", batt_charging ? ", charging" : "", batt_rate,
                          (unsigned long)(now / 1000), bootResetReason());
        }
        doSolve = true;                      // conditions changed, so refresh the answer
    }
    if (doSolve) solveAndPublish();
    serviceCard();
    delay(5);
}
