#pragma once

#include "SystemTypes.h"
#include "Config.h"

// ================================================================
// SAFETY MONITOR
// ================================================================

class SafetyMonitor {
public:

    struct Report {
        bool ok;
        const char* reason;

        Report(bool status = true, const char* msg = nullptr)
            : ok(status), reason(msg) {}
    };

    Report checkFlex(const SensorSnapshot& snap);

    Report checkIMU(const SensorSnapshot& snap);

    /*Report checkMotorStall(
        const SensorSnapshot& snap,
        const MotorState motors[NUM_FINGERS]);*/

    // NEW (was missing)
    Report checkIMUFlexCorrelation(const SensorSnapshot& snap);

private:

    uint32_t _stall_start[NUM_FINGERS] = {0};

    // NEW (used in cpp but missing in header)
    uint32_t _imu_flex_conflict_start = 0;
};