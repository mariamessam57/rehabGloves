#include "imu_sensor.h"
#include <Arduino.h>
#include <math.h>

static const uint8_t MPU6050_ADDR        = 0x68;
static const uint8_t MPU6050_WHO_AM_I    = 0x75;
static const uint8_t MPU6050_PWR_MGMT_1  = 0x6B;
static const uint8_t MPU6050_SMPLRT_DIV  = 0x19;
static const uint8_t MPU6050_CONFIG      = 0x1A;
static const uint8_t MPU6050_GYRO_CONFIG = 0x1B;
static const uint8_t MPU6050_ACCEL_CONFIG= 0x1C;
static const uint8_t MPU6050_ACCEL_XOUT_H= 0x3B;

static bool mpu6050_writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool mpu6050_readRegs(uint8_t reg, uint8_t* buffer, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)MPU6050_ADDR, (int)len, (int)true) != len) return false;
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        buffer[i] = Wire.read();
    }
    return true;
}

static bool mpu6050_init() {
    uint8_t who = 0;
    if (!mpu6050_readRegs(MPU6050_WHO_AM_I, &who, 1)) return false;
    if (who != MPU6050_ADDR) return false;

    if (!mpu6050_writeReg(MPU6050_PWR_MGMT_1, 0x00)) return false;
    if (!mpu6050_writeReg(MPU6050_SMPLRT_DIV, 0x00)) return false;
    if (!mpu6050_writeReg(MPU6050_CONFIG, 0x03)) return false;
    if (!mpu6050_writeReg(MPU6050_GYRO_CONFIG, 0x08)) return false;
    if (!mpu6050_writeReg(MPU6050_ACCEL_CONFIG, 0x08)) return false;
    return true;
}

static bool mpu6050_readRaw(int16_t* ax, int16_t* ay, int16_t* az,
                            int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buf[14];
    if (!mpu6050_readRegs(MPU6050_ACCEL_XOUT_H, buf, 14)) return false;

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
    return true;
}

static bool mpu6050_readAccelGyro(int16_t* ax, int16_t* ay, int16_t* az,
                                  int16_t* gx, int16_t* gy, int16_t* gz) {
    return mpu6050_readRaw(ax, ay, az, gx, gy, gz);
}

static void mpu6050_computeAngle(const float raw_g[3], float out[3]) {
    for (int i = 0; i < 3; i++) out[i] = raw_g[i];
}

IMUWrapper::IMUWrapper() {
    for (int i = 0; i < 3; i++)
        _kf[i] = KalmanFilter1D(KALMAN_Q, KALMAN_R);
}

bool IMUWrapper::begin() {
    _connected = mpu6050_init();
    if (!_connected) return false;

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
    if (!mpu6050_readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz)) {
        _data.stuck = true;
        return;
    }

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

    mpu6050_computeAngle(raw_g, _data.gyro);

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