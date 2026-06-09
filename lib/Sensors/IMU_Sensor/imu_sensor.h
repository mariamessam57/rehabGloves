#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H
#include <Wire.h>
#include "../../Filters/Filters.h"
#include "SystemTypes.h"
#include "Config.h"

// ================================================================
//  IMU WRAPPER — MPU6050 register-level driver + Kalman per gyro axis
// ================================================================
class IMUWrapper {
public:
    IMUWrapper();

    bool begin();               // returns false if IMU not found
    void sample();              // call every sensor period

    IMUData getData() const { return _data; }
    bool    isConnected() const { return _connected; }

private:
    KalmanFilter1D _kf[3];     // one per gyro axis
    IMUData        _data;
    bool           _connected = false;

    static constexpr float GYRO_SCALE  = 65.5f;   // ±500 deg/s
    static constexpr float ACCEL_SCALE = 8192.0f;  // ±4g

    uint32_t _last_update_ms = 0;

    void _detectAnomalies();
};

#endif