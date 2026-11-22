#pragma once
#include <cstdint>
#define SH2_ROTATION_VECTOR 5
struct sh2_RotationVectorWAcc_t { float real, i, j, k; };
struct sh2_SensorValue_t { uint8_t sensorId; union { sh2_RotationVectorWAcc_t rotationVector; } un; };
struct Adafruit_BNO08x {
    Adafruit_BNO08x(int) {}
    bool begin_I2C() { return false; }   // IMU absent in these tests
    bool enableReport(int, uint32_t) { return true; }
    bool wasReset() { return false; }
    bool getSensorEvent(sh2_SensorValue_t*) { return false; }
};
