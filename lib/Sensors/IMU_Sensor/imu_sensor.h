#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include "SystemTypes.h"

class IMUWrapper
{
private:
    static constexpr uint8_t MPU_ADDR = 0x68;

    int16_t GyX, GyY, GyZ;

    SimpleKalmanFilter kalmanX;
    SimpleKalmanFilter kalmanY;
    SimpleKalmanFilter kalmanZ;

    IMUData data;

    float angleX;
    float angleY;
    float angleZ;

    unsigned long lastTime;
    float dt;

    float DEADZONE;

public:
    IMUWrapper();

    bool begin();

    void sample();

    IMUData getData() const;
};

#endif