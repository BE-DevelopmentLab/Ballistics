// Host stand-ins for the Arduino core: a simulated clock, pins, and Serial1 wired to a fake laser.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <deque>
#include <vector>
#define IRAM_ATTR
#define INPUT_PULLUP 2
#define OUTPUT 3
#define LOW 0
#define HIGH 1
#define FALLING 2
#define SERIAL_8N1 0x800001c
inline uint32_t g_micros = 1000000;               // simulated time, advances only via delay()
inline uint32_t millis() { return g_micros / 1000; }
inline uint32_t micros() { return g_micros; }
inline void delay(uint32_t ms) { g_micros += ms * 1000; }
inline int g_pin_mode[64] = {0};
inline void pinMode(int p, int m) { if (p >= 0 && p < 64) g_pin_mode[p] = m; }
inline int g_pin_level[64] = {0};            // last digitalWrite per pin, for checks
inline int g_pin_input[64];                  // what digitalRead returns (tests press buttons here)
inline bool g_pin_input_init = [] { for (int& v : g_pin_input) v = HIGH; return true; }();
inline void digitalWrite(int p, int v) { if (p >= 0 && p < 64) g_pin_level[p] = v; }
inline int digitalRead(int p) { return (p >= 0 && p < 64) ? g_pin_input[p] : HIGH; }
struct SerialStub { void begin(int) {} void println(const char*) {} int printf(const char*, ...) { return 0; } };
inline SerialStub Serial;
// Serial1: the laser port. Bytes the firmware writes go to tx; tests push replies into rx.
struct HardwareSerial {
    std::deque<uint8_t> rx; std::vector<uint8_t> tx; uint32_t baud = 0; int rxPin = -1, txPin = -1;
    void begin(uint32_t b, uint32_t, int r, int t) { baud = b; rxPin = r; txPin = t; }
    int available() { return (int)rx.size(); }
    int read() { if (rx.empty()) return -1; const int c = rx.front(); rx.pop_front(); return c; }
    size_t write(const uint8_t* d, size_t n) { tx.insert(tx.end(), d, d + n); return n; }
};
inline HardwareSerial Serial1;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL(portMUX_TYPE*) {}
static const uint8_t TX = 39, RX = 38;
#define I2C_POWER 7
