// Host stand-in for ESP32 Preferences (NVS): one in-memory store that survives "reboots" in a test.
#pragma once
#include <map>
#include <string>
#include <vector>
#include <cstring>
inline std::map<std::string, double>& g_prefs() { static std::map<std::string, double> m; return m; }
inline std::map<std::string, std::vector<uint8_t>>& g_prefs_bytes() { static std::map<std::string, std::vector<uint8_t>> m; return m; }
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
    size_t putBytes(const char* k, const void* v, size_t n) { auto& b = g_prefs_bytes()[ns + "/" + k]; b.assign((const uint8_t*)v, (const uint8_t*)v + n); return n; }
    size_t getBytes(const char* k, void* out, size_t max) {
        auto it = g_prefs_bytes().find(ns + "/" + k);
        if (it == g_prefs_bytes().end() || it->second.size() > max) return 0;
        std::memcpy(out, it->second.data(), it->second.size()); return it->second.size();
    }
};
