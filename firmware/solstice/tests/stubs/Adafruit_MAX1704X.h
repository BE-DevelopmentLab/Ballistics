#pragma once
#include <cmath>
#include "Arduino.h"
#include "Wire.h"
// Stand-in for the MAX17048 fuel gauge. Tests set these to simulate the chip.
inline bool  g_fuel_present = true;
inline float g_fuel_v = 3.90f, g_fuel_pct = 64.0f, g_fuel_rate = -5.0f;   // rate in %/hour, + = charging
struct Adafruit_MAX17048 {
    bool begin() { return g_fuel_present; }
    float cellVoltage() { return g_fuel_present ? g_fuel_v : NAN; }
    float cellPercent() { return g_fuel_present ? g_fuel_pct : NAN; }
    float chargeRate() { return g_fuel_present ? g_fuel_rate : NAN; }
};
