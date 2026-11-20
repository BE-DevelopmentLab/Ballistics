// Host stand-in for NimBLE-Arduino 2.x that records every notification with its time, and lets tests
// play a Solstice the client side can find, connect to and get wind notifications from.
// These mirror MY reading of the library API (checked against NimBLE-Arduino 2.5.2 headers), so they
// catch logic bugs in main.cpp, not mismatches with the real library.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <functional>
#include <string>
#include <vector>
#include "Arduino.h"

#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 3   // the ESP32 Arduino core sdkconfig sets 3
struct NimBLEConnInfo {};
struct NimBLEAttValue {
    std::vector<uint8_t> v;
    const uint8_t* data() const { return v.data(); }
    size_t size() const { return v.size(); }
};
struct NIMBLE_PROPERTY { enum { READ = 1, WRITE = 2, NOTIFY = 4, WRITE_NR = 8 }; };
struct NimBLEUUID { std::string s; NimBLEUUID(const char* u) : s(u) {} };
struct NimBLEAddress { std::string a; };

struct NimBLECharacteristic;
struct NimBLECharacteristicCallbacks {
    virtual ~NimBLECharacteristicCallbacks() {}
    virtual void onWrite(NimBLECharacteristic*, NimBLEConnInfo&) {}
};
struct Notif { std::string uuid; std::vector<uint8_t> data; uint32_t ms; };
inline std::vector<Notif>& g_notifs() { static std::vector<Notif> n; return n; }
inline std::vector<NimBLECharacteristic*>& g_chars() { static std::vector<NimBLECharacteristic*> c; return c; }

struct NimBLECharacteristic {
    std::string uuid; uint32_t props = 0;
    std::vector<uint8_t> value;
    NimBLECharacteristicCallbacks* cb = nullptr;
    NimBLEAttValue getValue() const { return NimBLEAttValue{value}; }
    void setValue(const uint8_t* d, size_t n) { value.assign(d, d + n); }
    bool notify() { g_notifs().push_back({uuid, value, millis()}); return true; }
    void setCallbacks(NimBLECharacteristicCallbacks* c) { cb = c; }
};
inline std::vector<std::string>& g_services() { static std::vector<std::string> s; return s; }
struct NimBLEService {
    NimBLECharacteristic* createCharacteristic(const char* uuid, uint32_t p) {
        auto* c = new NimBLECharacteristic(); c->uuid = uuid; c->props = p; g_chars().push_back(c); return c;
    }
    bool start() { return true; }
};
struct NimBLEServer;
struct NimBLEServerCallbacks {
    virtual ~NimBLEServerCallbacks() {}
    virtual void onConnect(NimBLEServer*, NimBLEConnInfo&) {}
    virtual void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) {}
};
inline int g_adv_starts = 0;
inline uint8_t g_connected = 0;
struct NimBLEServer {
    NimBLEServerCallbacks* cb = nullptr;
    NimBLEService* createService(const char* u) { g_services().push_back(u); return new NimBLEService(); }
    void advertiseOnDisconnect(bool) {}
    void setCallbacks(NimBLEServerCallbacks* c) { cb = c; }
    uint8_t getConnectedCount() const { return g_connected; }
};
struct NimBLEAdvertisementData {
    std::vector<std::string> uuids; std::string name; uint8_t flags = 0;
    bool setFlags(uint8_t f) { flags = f; return true; }
    bool addServiceUUID(const char* u) { uuids.push_back(u); return true; }
    bool setName(const std::string& n, bool = true) { name = n; return true; }
    // Bytes this would take in a packet: flags 3, each 128-bit UUID list 18, name 2 + len.
    size_t bytes() const { return (flags ? 3 : 0) + (uuids.empty() ? 0 : 2 + 16 * uuids.size()) + (name.empty() ? 0 : 2 + name.size()); }
};
inline std::string& g_adv_name() { static std::string s; return s; }
inline std::string& g_init_name() { static std::string s; return s; }
inline NimBLEAdvertisementData& g_adv_data() { static NimBLEAdvertisementData d; return d; }
inline NimBLEAdvertisementData& g_scan_data() { static NimBLEAdvertisementData d; return d; }
struct NimBLEAdvertising {
    bool setAdvertisementData(const NimBLEAdvertisementData& d) { g_adv_data() = d; return true; }
    bool setScanResponseData(const NimBLEAdvertisementData& d) { g_scan_data() = d; g_adv_name() = d.name; return true; }
    void enableScanResponse(bool) {}
    bool start() { ++g_adv_starts; return true; }
};

// ---- client side: a fake Solstice -------------------------------------------------------------------
struct NimBLERemoteCharacteristic;
typedef std::function<void(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool)> notify_callback;
struct NimBLERemoteCharacteristic {
    std::string uuid; mutable notify_callback cb;
    bool subscribe(bool, const notify_callback c = nullptr, bool = true) const { cb = c; return true; }
};
inline NimBLERemoteCharacteristic& g_sol_env() { static NimBLERemoteCharacteristic c{"b4a10004-1d5b-4c0e-9a3f-5e7d2c0a1b00", nullptr}; return c; }
struct NimBLERemoteService {
    NimBLERemoteCharacteristic* getCharacteristic(const char* u) const { return std::string(u) == g_sol_env().uuid ? &g_sol_env() : nullptr; }
};
struct NimBLEClient;
struct NimBLEClientCallbacks {
    virtual ~NimBLEClientCallbacks() {}
    virtual void onDisconnect(NimBLEClient*, int) {}
};
inline bool g_sol_connectable = true;
inline int g_sol_connects = 0;
struct NimBLEClient {
    bool conn = false; NimBLEClientCallbacks* cb = nullptr;
    void setClientCallbacks(NimBLEClientCallbacks* c, bool) { cb = c; }
    void setConnectTimeout(uint32_t) {}
    bool connect(const NimBLEAddress&) { ++g_sol_connects; conn = g_sol_connectable; return conn; }
    bool isConnected() const { return conn; }
    bool disconnect() { if (conn) { conn = false; if (cb) cb->onDisconnect(this, 0); } return true; }
    NimBLERemoteService* getService(const char* u) { static NimBLERemoteService s; return std::string(u) == "b4a10000-1d5b-4c0e-9a3f-5e7d2c0a1b00" ? &s : nullptr; }
};
inline NimBLEClient& g_client() { static NimBLEClient c; return c; }
struct NimBLEAdvertisedDevice {
    std::vector<std::string> uuids; std::string name; NimBLEAddress addr;
    bool isAdvertisingService(const NimBLEUUID& u) const { for (auto& x : uuids) if (x == u.s) return true; return false; }
    std::string getName() const { return name; }
    NimBLEAddress getAddress() const { return addr; }
};
struct NimBLEScanCallbacks {
    virtual ~NimBLEScanCallbacks() {}
    virtual void onResult(const NimBLEAdvertisedDevice*) {}
};
inline std::vector<NimBLEAdvertisedDevice>& g_nearby() { static std::vector<NimBLEAdvertisedDevice> d; return d; }
inline int g_scans = 0;
struct NimBLEScan {
    NimBLEScanCallbacks* cb = nullptr;
    void setScanCallbacks(NimBLEScanCallbacks* c, bool) { cb = c; }
    void setActiveScan(bool) {}
    bool isScanning() { return false; }                 // the fake scan finishes at once
    bool stop() { return true; }
    bool start(uint32_t, bool, bool) { ++g_scans; for (auto& d : g_nearby()) if (cb) cb->onResult(&d); return true; }
};
struct NimBLEDevice {
    static bool init(const char* n) { g_init_name() = n; return true; }
    static NimBLEServer* createServer() { static NimBLEServer s; return &s; }
    static NimBLEAdvertising* getAdvertising() { static NimBLEAdvertising a; return &a; }
    static bool startAdvertising() { ++g_adv_starts; return true; }
    static NimBLEScan* getScan() { static NimBLEScan s; return &s; }
    static NimBLEClient* createClient() { return &g_client(); }
};

// Test helpers
inline void stub_write(const char* uuid, const std::vector<uint8_t>& bytes) {
    for (auto* c : g_chars()) {
        if (c->uuid == uuid) { c->value = bytes; NimBLEConnInfo ci; if (c->cb) c->cb->onWrite(c, ci); return; }
    }
}
inline void stub_connect_central() {
    ++g_connected; NimBLEConnInfo ci; auto* s = NimBLEDevice::createServer(); if (s->cb) s->cb->onConnect(s, ci);
}
inline void stub_solstice_notify(const std::vector<uint8_t>& env) {
    std::vector<uint8_t> b = env;
    if (g_sol_env().cb) g_sol_env().cb(&g_sol_env(), b.data(), b.size(), true);
}
