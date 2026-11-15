// Host stand-in for Adafruit_GPS over I2C: tests append NMEA text to g_gps_text.
#pragma once
#include <string>
#include <vector>
#include "Wire.h"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"
inline std::string g_gps_text;
inline std::vector<std::string> g_gps_cmds;
struct Adafruit_GPS {
    size_t pos = 0;
    Adafruit_GPS(TwoWire*) {}
    bool begin(uint32_t) { return true; }
    void sendCommand(const char* c) { g_gps_cmds.push_back(c); }
    char read() { if (pos < g_gps_text.size()) return g_gps_text[pos++]; return 0; }
};
