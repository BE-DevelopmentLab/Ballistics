// Host stand-ins for the Arduino core: a simulated clock and no-op hardware calls.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#define IRAM_ATTR
#define INPUT_PULLUP 2
#define OUTPUT 3
#define LOW 0
#define HIGH 1
#define FALLING 2
inline uint32_t g_micros = 1000000;               // simulated time, advances only via delay()
inline uint32_t millis() { return g_micros / 1000; }
inline uint32_t micros() { return g_micros; }
inline void delay(uint32_t ms) { g_micros += ms * 1000; }
inline void pinMode(int, int) {}
inline int g_pin_level[64] = {0};            // last digitalWrite per pin, for checks
inline void digitalWrite(int p, int v) { if (p >= 0 && p < 64) g_pin_level[p] = v; }
inline uint32_t g_batt_mv = 3900;              // simulated battery pin, in millivolts at the pin
inline uint32_t analogReadMilliVolts(int) { return g_batt_mv; }
inline int digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int, void (*)(), int) {}
struct SerialStub { void begin(int) {} void println(const char*) {} int printf(const char*, ...) { return 0; } };
inline SerialStub Serial;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL(portMUX_TYPE*) {}
