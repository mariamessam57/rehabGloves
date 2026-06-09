#pragma once

#include "SystemTypes.h"
#include "Config.h"

// ================================================================
//  SAFETY MONITOR — pure logic, no hardware access
//  Evaluated by SafetyTask every PERIOD_SAFETY_MS
// ================================================================

class SafetyMonitor {
public:

    struct Report {

        bool ok;
        const char* reason;

        Report(bool status = true, const char* msg = nullptr)
            : ok(status), reason(msg) {}
    };

    // All checks return a Report; if ok==false → trigger ESTOP

    Report checkFlex(const SensorSnapshot& snap);

    Report checkIMU(
        const SensorSnapshot& snap);

    Report checkMotorStall(
        const SensorSnapshot& snap,
        const MotorState motors[NUM_FINGERS]);

private:

    uint32_t _stall_start[NUM_FINGERS] = {0};
};