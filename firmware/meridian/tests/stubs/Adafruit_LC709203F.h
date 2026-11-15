#pragma once
#include <cmath>
#define LC709203F_APA_1000MAH 0x19
inline bool g_lc_present = false;
struct Adafruit_LC709203F {
    bool begin() { return g_lc_present; }
    bool setPackSize(int) { return true; }
    float cellVoltage() { return 3.8f; }
    float cellPercent() { return 50.0f; }
};
