// Host stand-in for ESP32 Preferences (NVS): one in-memory store that survives "reboots" in a test.
#pragma once
#include <map>
#include <string>
inline std::map<std::string, double>& g_prefs() { static std::map<std::string, double> m; return m; }
struct Preferences {
    std::string ns;
    bool begin(const char* n, bool) { ns = n; return true; }
    void end() {}
    double get(const char* k, double d) { auto it = g_prefs().find(ns + "/" + k); return it == g_prefs().end() ? d : it->second; }
    void put(const char* k, double v) { g_prefs()[ns + "/" + k] = v; }
    uint8_t getUChar(const char* k, uint8_t d) { return (uint8_t)get(k, d); }
    bool getBool(const char* k, bool d) { return get(k, d) != 0; }
    float getFloat(const char* k, float d) { return (float)get(k, d); }
    size_t putUChar(const char* k, uint8_t v) { put(k, v); return 1; }
    size_t putBool(const char* k, bool v) { put(k, v); return 1; }
    size_t putFloat(const char* k, float v) { put(k, v); return 4; }
};
