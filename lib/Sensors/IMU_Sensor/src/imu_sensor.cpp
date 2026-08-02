#include "imu_sensor.h"
#include "systemstate/System_State.h"
#include "config.h"

IMUWrapper::IMUWrapper()
    : kalmanX(2, 2, 0.01),
      kalmanY(2, 2, 0.01),
      kalmanZ(2, 2, 0.01)
{
    angleX = 0;
    angleY = 0;
    angleZ = 0;

    lastTime = 0;
    dt = 0;

    DEADZONE = 0.05;
}
static bool recoverI2C()
{
    Wire.end();
    delay(2);
    Wire.begin(PIN_SDA, PIN_SCL);
    return true;
}

bool IMUWrapper::begin()
{
    I2CLockGuard lock;
    if (!lock.ok())
        return false;

    Wire.begin(PIN_SDA, PIN_SCL);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    if (Wire.endTransmission(true) != 0)
        return false;

    lastTime = millis();

    return true;
}

void IMUWrapper::sample()
{
    I2CLockGuard lock;
    if (!lock.ok())
        return;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    if (Wire.endTransmission(false) != 0)
        return;

    uint8_t retries = 0;
    const uint8_t MAX_RETRIES = 6;
    while (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6)
    {
        if (++retries >= MAX_RETRIES)
        {
            data.stuck = true;
            data.spike = false;
            recoverI2C();
            return;
        }

        // try to recover the bus and retry
        recoverI2C();
        delay(5);
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        if (Wire.endTransmission(false) != 0)
            continue;
    }

    if (Wire.available() < 6)
        return;

    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    float gx = GyX / 131.0f;
    float gy = GyY / 131.0f;
    float gz = GyZ / 131.0f;

    data.gyro[0] = kalmanX.updateEstimate(gx);
    data.gyro[1] = kalmanY.updateEstimate(gy);
    data.gyro[2] = kalmanZ.updateEstimate(gz);

    if (abs(data.gyro[0]) < DEADZONE)
        data.gyro[0] = 0;
    if (abs(data.gyro[1]) < DEADZONE)
        data.gyro[1] = 0;
    if (abs(data.gyro[2]) < DEADZONE)
        data.gyro[2] = 0;

    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    angleX += data.gyro[0] * dt;
    angleY += data.gyro[1] * dt;
    angleZ += data.gyro[2] * dt;

    angleX *= 0.999f;
    angleY *= 0.999f;
    angleZ *= 0.999f;

    // ربط الـ SystemTypes.h
    data.pitch = angleX;
    data.roll = angleY;

    data.gyro_mag = sqrt(
        data.gyro[0] * data.gyro[0] +
        data.gyro[1] * data.gyro[1] +
        data.gyro[2] * data.gyro[2]);

    data.last_ms = millis();

    data.stuck = false;
    data.spike = false;
}

IMUData IMUWrapper::getData() const
{
    return data;
}