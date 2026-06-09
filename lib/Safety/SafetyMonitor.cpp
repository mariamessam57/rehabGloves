#include "SafetyMonitor.h"
#include <math.h>
#include <Arduino.h>

SafetyMonitor::Report SafetyMonitor::checkFlex(const SensorSnapshot& snap)
{
    if (!snap.calib_complete)
        return Report(true, nullptr);

    for (int f = 0; f < NUM_FINGERS; f++) {

        float n = snap.flex[f].normalized;

        if (n < -FLEX_SAFETY_MARGIN ||
            n > (1.0f + FLEX_SAFETY_MARGIN))
        {
            return Report(false, "FLEX_OUT_OF_RANGE");
        }
    }

    return Report(true, nullptr);
}

SafetyMonitor::Report SafetyMonitor::checkIMU(const SensorSnapshot& snap)
{
    if (snap.imu.stuck)
        return Report(false, "IMU_STUCK");

    if (snap.imu.spike)
        return Report(false, "IMU_SPIKE");

    return Report(true, nullptr);
}

SafetyMonitor::Report SafetyMonitor::checkMotorStall(
    const SensorSnapshot& snap,
    const MotorState motors[NUM_FINGERS])
{
    uint32_t now = millis();

    for (int f = 0; f < NUM_FINGERS; f++) {

        const MotorState& m = motors[f];

        if (m.enabled && m.target > PWM_DUTY_MIN) {

            if (fabsf(snap.flex[f].velocity) < STALL_VEL_THRESH) {

                if (_stall_start[f] == 0)
                    _stall_start[f] = now;

                if ((now - _stall_start[f]) >= STALL_TIMEOUT_MS) {
                    return Report(false, "MOTOR_STALL");
                }

            } else {
                _stall_start[f] = 0;
            }

        } else {
            _stall_start[f] = 0;
        }
    }

    return Report(true, nullptr);
}

SafetyMonitor::Report SafetyMonitor::checkIMUFlexCorrelation(const SensorSnapshot& snap)
{
    if (!snap.calib_complete) return Report(true, nullptr);

    if (snap.imu.gyro_mag > IMU_FLEX_CORR_GYRO_MIN) {
        bool any_flex_moving = false;
        for (int f = 0; f < NUM_FINGERS; f++) {
            if (fabsf(snap.flex[f].velocity) > IMU_FLEX_CORR_VEL_MIN) {
                any_flex_moving = true;
                break;
            }
        }
        if (!any_flex_moving) {
            if (_imu_flex_conflict_start == 0)
                _imu_flex_conflict_start = millis();
            if ((millis() - _imu_flex_conflict_start) >= IMU_FLEX_CONFLICT_MS)
                return Report(false, "IMU_FLEX_CONFLICT");
        } else {
            _imu_flex_conflict_start = 0;
        }
    } else {
        _imu_flex_conflict_start = 0;
    }
    return Report(true, nullptr);
}
