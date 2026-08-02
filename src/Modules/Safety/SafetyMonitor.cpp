#include "SafetyMonitor.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>

// ================================================================
// FLEX SAFETY CHECK
// ================================================================
SafetyMonitor::Report SafetyMonitor::checkFlex(const SensorSnapshot &snap)
{
    if (!snap.calib_complete)
        return Report(true, nullptr);

    float n = snap.flex[MASTER_FLEX_IDX].normalized;

    if (n < -FLEX_SAFETY_MARGIN || n > (1.0f + FLEX_SAFETY_MARGIN))
    {
        return Report(false, "FLEX_OUT_OF_RANGE");
    }

    return Report(true, nullptr);
}

// ================================================================
// IMU SAFETY CHECK
// ================================================================
SafetyMonitor::Report SafetyMonitor::checkIMU(const SensorSnapshot &snap)
{
    if (snap.imu.stuck)
        return Report(false, "IMU_STUCK");

    if (snap.imu.spike)
        return Report(false, "IMU_SPIKE");

    return Report(true, nullptr);
}

// ================================================================
// IMU - FLEX CORRELATION CHECK
// ================================================================
SafetyMonitor::Report SafetyMonitor::checkIMUFlexCorrelation(const SensorSnapshot &snap)
{
    if (!snap.calib_complete)
        return Report(true, nullptr);

    if (snap.imu.gyro_mag > IMU_FLEX_CORR_GYRO_MIN)
    {

        bool any_flex_moving = false;

        if (fabsf(snap.flex[MASTER_FLEX_IDX].velocity) > IMU_FLEX_CORR_VEL_MIN)
        {
            any_flex_moving = true;
        }

        if (!any_flex_moving)
        {

            if (_imu_flex_conflict_start == 0)
                _imu_flex_conflict_start = millis();

            if ((millis() - _imu_flex_conflict_start) >= IMU_FLEX_CONFLICT_MS)
                return Report(false, "IMU_FLEX_CONFLICT");
        }
        else
        {
            _imu_flex_conflict_start = 0;
        }
    }
    else
    {
        _imu_flex_conflict_start = 0;
    }

    return Report(true, nullptr);
}