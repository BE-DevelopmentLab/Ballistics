// Host stand-in for the BNO085: tests set g_bno_quat and the sensor reports it at every poll.
#pragma once
#include <cstdint>
#define SH2_ROTATION_VECTOR 5
#define SH2_CAL_ACCEL (0x01)
#define SH2_CAL_GYRO  (0x02)
#define SH2_CAL_MAG   (0x04)
struct sh2_RotationVectorWAcc_t { float i, j, k, real, accuracy; };
struct sh2_SensorValue_t { uint8_t sensorId; uint8_t sequence; uint8_t status; union { sh2_RotationVectorWAcc_t rotationVector; } un; };
inline bool g_bno_present = true;
inline float g_bno_quat[4] = {1, 0, 0, 0};         // w, x, y, z
inline uint8_t g_bno_status = 3;
inline int g_bno_calconfig = -1, g_bno_dcd_saves = 0;
inline int sh2_setCalConfig(uint8_t s) { g_bno_calconfig = s; return 0; }
inline int sh2_saveDcdNow() { ++g_bno_dcd_saves; return 0; }
struct Adafruit_BNO08x {
    bool sent = false;
    Adafruit_BNO08x(int) {}
    bool begin_I2C(uint8_t = 0x4A) { return g_bno_present; }
    bool enableReport(int, uint32_t) { return true; }
    bool wasReset() { return false; }
    bool getSensorEvent(sh2_SensorValue_t* v) {
        if (sent) { sent = false; return false; }     // one event per updateImu() call
        sent = true;
        v->sensorId = SH2_ROTATION_VECTOR; v->status = g_bno_status;
        v->un.rotationVector.real = g_bno_quat[0]; v->un.rotationVector.i = g_bno_quat[1];
        v->un.rotationVector.j = g_bno_quat[2]; v->un.rotationVector.k = g_bno_quat[3];
        v->un.rotationVector.accuracy = 0.05f;
        return true;
    }
};
