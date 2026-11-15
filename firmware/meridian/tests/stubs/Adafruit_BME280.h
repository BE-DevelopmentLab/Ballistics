#pragma once
#include "Arduino.h"
inline int g_bme_i2c_power_at_begin = -1;   // GPIO 7 level when begin() ran
// Fixed conditions equal to the standard atmosphere, so results match a plain solve().
struct Adafruit_BME280 {
    bool begin(int) { g_bme_i2c_power_at_begin = g_pin_level[7]; return true; }
    float readTemperature() { return 15.0f; }
    float readHumidity() { return 50.0f; }
    float readPressure() { return 101325.0f; }
};
