#include "imu_sensor.h"
#include <math.h>

IMUWrapper::IMUWrapper() {
    for (int i = 0; i < 3; i++)
        _kf[i] = KalmanFilter1D(KALMAN_Q, KALMAN_R);
}

bool IMUWrapper::begin() {
    _mpu.initialize();
    _connected = _mpu.testConnection();
    if (!_connected) return false;

    // ±500 deg/s, ±4g
    _mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    _mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    _mpu.setDLPFMode(MPU6050_DLPF_BW_42);

    // Warm up Kalman
    for (int i = 0; i < 3; i++) _kf[i].init(0.0f);

    _last_update_ms = millis();
    _data.stuck = false;
    return true;
}

void IMUWrapper::sample() {
    if (!_connected) {
        _data.stuck = true;
        return;
    }

    int16_t ax, ay, az, gx, gy, gz;
    _mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float raw_g[3] = {
        (float)gx / GYRO_SCALE,
        (float)gy / GYRO_SCALE,
        (float)gz / GYRO_SCALE
    };

    // Spike check before Kalman
    float mag_raw = sqrtf(raw_g[0]*raw_g[0] +
                          raw_g[1]*raw_g[1] +
                          raw_g[2]*raw_g[2]);

    if (mag_raw > IMU_SPIKE_DEGS) {
        _data.spike = true;
        // Do not update Kalman on spike
    } else {
        _data.spike = false;
        for (int i = 0; i < 3; i++) {
            _data.gyro[i] = _kf[i].update(raw_g[i]);
        }
    }

    _data.accel[0] = (float)ax / ACCEL_SCALE;
    _data.accel[1] = (float)ay / ACCEL_SCALE;
    _data.accel[2] = (float)az / ACCEL_SCALE;

    _data.gyro_mag = sqrtf(_data.gyro[0]*_data.gyro[0] +
                           _data.gyro[1]*_data.gyro[1] +
                           _data.gyro[2]*_data.gyro[2]);

    _last_update_ms = millis();
    _data.last_ms   = _last_update_ms;
    _detectAnomalies();
}

void IMUWrapper::_detectAnomalies() {
    uint32_t now = millis();
    _data.stuck = ((now - _last_update_ms) > 200U);
}