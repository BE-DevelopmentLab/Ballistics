// Host stand-in for the ST7789 screen: keeps the text printed since the last fillScreen/fillRect per line.
#pragma once
#include <cstdint>
#include <map>
#include <string>
inline std::map<int, std::string>& g_tft_lines() { static std::map<int, std::string> m; return m; }   // y -> text
inline int g_tft_fills = 0, g_tft_sleep = 0;
struct Adafruit_ST7789 {
    int cy = 0;
    Adafruit_ST7789(int, int, int) {}
    void init(int, int) {}
    void setRotation(int) {}
    void fillScreen(uint16_t) { g_tft_lines().clear(); ++g_tft_fills; }
    void fillRect(int, int y, int, int h, uint16_t) {
        for (auto it = g_tft_lines().begin(); it != g_tft_lines().end();) it = (it->first >= y && it->first < y + h) ? g_tft_lines().erase(it) : std::next(it);
    }
    void drawRect(int, int, int, int, uint16_t) {}
    void drawFastVLine(int, int, int, uint16_t) {}
    void setTextSize(int) {}
    void setTextColor(uint16_t) {}
    void setCursor(int, int y) { cy = y; }
    void print(const char* s) { g_tft_lines()[cy] += s; }
    void enableSleep(bool) { ++g_tft_sleep; }
};
