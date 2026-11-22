// Host stand-in for NimBLE-Arduino that records every notification with its time.
// These mirror MY assumptions about the library API, so they catch logic bugs in main.cpp,
// not mismatches with the real library.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>
#include <vector>
#include "Arduino.h"

struct NimBLEConnInfo {};
struct NimBLEAttValue {
    std::vector<uint8_t> v;
    const uint8_t* data() const { return v.data(); }
    size_t size() const { return v.size(); }
};
struct NIMBLE_PROPERTY { enum { READ = 1, WRITE = 2, NOTIFY = 4 }; };

struct NimBLECharacteristic;
struct NimBLECharacteristicCallbacks {
    virtual ~NimBLECharacteristicCallbacks() {}
    virtual void onWrite(NimBLECharacteristic*, NimBLEConnInfo&) {}
};
struct Notif { std::string uuid; std::vector<uint8_t> data; uint32_t ms; };
inline std::vector<Notif>& g_notifs() { static std::vector<Notif> n; return n; }
struct NimBLECharacteristic;
inline std::vector<NimBLECharacteristic*>& g_chars() { static std::vector<NimBLECharacteristic*> c; return c; }

struct NimBLECharacteristic {
    std::string uuid;
    std::vector<uint8_t> value;
    NimBLECharacteristicCallbacks* cb = nullptr;
    NimBLEAttValue getValue() const { return NimBLEAttValue{value}; }
    void setValue(const uint8_t* d, size_t n) { value.assign(d, d + n); }
    bool notify() { g_notifs().push_back({uuid, value, millis()}); return true; }
    void setCallbacks(NimBLECharacteristicCallbacks* c) { cb = c; }
};
struct NimBLEService {
    NimBLECharacteristic* createCharacteristic(const char* uuid, uint32_t) {
        auto* c = new NimBLECharacteristic(); c->uuid = uuid; g_chars().push_back(c); return c;
    }
    bool start() { return true; }
};
inline std::string& g_adv_name() { static std::string s; return s; }   // last advertised name
inline std::string& g_init_name() { static std::string s; return s; }  // name given to NimBLEDevice::init
struct NimBLEServer;
struct NimBLEServerCallbacks {
    virtual ~NimBLEServerCallbacks() {}
    virtual void onConnect(NimBLEServer*, NimBLEConnInfo&) {}
};
inline int g_adv_starts = 0;          // advertising (re)starts
inline uint8_t g_connected = 0;       // simulated connected centrals
struct NimBLEServer {
    NimBLEServerCallbacks* cb = nullptr;
    NimBLEService* createService(const char*) { return new NimBLEService(); }
    void advertiseOnDisconnect(bool) {}
    void setCallbacks(NimBLEServerCallbacks* c) { cb = c; }
    uint8_t getConnectedCount() const { return g_connected; }
};
struct NimBLEAdvertising {
    bool addServiceUUID(const char*) { return true; }
    void enableScanResponse(bool) {}
    bool setName(const char* n) { g_adv_name() = n; return true; }
    bool start() { ++g_adv_starts; return true; }
};
struct NimBLEDevice {
    static bool init(const char* n) { g_init_name() = n; return true; }
    static NimBLEServer* createServer() { static NimBLEServer s; return &s; }
    static NimBLEAdvertising* getAdvertising() { static NimBLEAdvertising a; return &a; }
    static bool startAdvertising() { ++g_adv_starts; return true; }
};

// Test helper: pretend the phone wrote `bytes` to the characteristic with this UUID.
inline void stub_write(const char* uuid, const std::vector<uint8_t>& bytes) {
    for (auto* c : g_chars()) {
        if (c->uuid == uuid) { c->value = bytes; NimBLEConnInfo ci; if (c->cb) c->cb->onWrite(c, ci); return; }
    }
}

// Test helper: a central (phone or Meridian) connects.
inline void stub_connect_central() {
    ++g_connected; NimBLEConnInfo ci; auto* s = NimBLEDevice::createServer(); if (s->cb) s->cb->onConnect(s, ci);
}
