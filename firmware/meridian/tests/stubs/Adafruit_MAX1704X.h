#pragma once
#include <cmath>
inline bool  g_fuel_present = true;
inline float g_fuel_v = 3.90f, g_fuel_pct = 64.0f;
struct Adafruit_MAX17048 {
    bool begin() { return g_fuel_present; }
    float cellVoltage() { return g_fuel_present ? g_fuel_v : NAN; }
    float cellPercent() { return g_fuel_present ? g_fuel_pct : NAN; }
};
