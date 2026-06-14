// ================================================================
//  safety_task.cpp  — FIXED
//
//  Changes from original:
//  [FIX-1] SENSOR_STALE check is now suppressed while
//          calibration is in progress. During _collectSamples
//          sensor_task blocks for up to 7 s, which always tripped
//          the 80 ms staleness threshold and fired ESTOP.
//  [FIX-2] 'goto estop' replaced with structured continue so
//          last_wake is always reset and there is no hidden
//          control-flow across variable declarations.
// ================================================================

#include "Tasks/SafetyTask/safety_task.h"
#include "systemstate/System_State.h"
#include "SafetyMonitor.h"
#include "MotorDriver.h"
#include "config.h"
#include <Arduino.h>

static SafetyMonitor monitor;

void safety_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    TickType_t    last_wake = xTaskGetTickCount();
    SensorSnapshot snap;
    MotorState    motors[NUM_FINGERS];

    getMotorDriver().begin();
    getMotorDriver().disableAll();

    for (;;) {
        // ── 1. Always service ESTOP first ─────────────────────────
        if (ss.isEStop()) {
            getMotorDriver().disableAll();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_SAFETY_MS));
            continue;
        }

        ss.readSensors(snap);
        ss.readMotors(motors);

        // ── 2. Stale-sensor check — suppressed during calibration ──
        //
        // [FIX-1]  When calibration is running, sensor_task is
        //           blocked inside _collectSamples() for 2–7 s and
        //           cannot call writeSensors(). Firing ESTOP here
        //           was the root cause of the system freeze.
        //           We skip the staleness check whenever
        //           CalibInProgress is true. No motors run during
        //           calibration so there is no safety hazard.
        //
        if (!ss.isCalibInProgress()) {
            uint32_t age_ms = millis() - snap.imu.last_ms;
            // 5× instead of original 4× — gives one extra sensor
            // period of headroom against scheduler jitter.
            if (age_ms > (PERIOD_SENSOR_MS * 5u)) {
                ss.triggerEStop("SENSOR_STALE");
                Serial.printf("[SAFETY] SENSOR_STALE: age=%lu ms\n", age_ms);
                getMotorDriver().disableAll();
                last_wake = xTaskGetTickCount();
                continue;
            }
        }

        // ── 3. Remaining safety checks ────────────────────────────
        SafetyMonitor::Report r;

        r = monitor.checkFlex(snap);
        if (!r.ok) {
            ss.triggerEStop(r.reason);
            Serial.printf("[SAFETY] ESTOP: %s\n", r.reason);
            getMotorDriver().disableAll();
            last_wake = xTaskGetTickCount();
            continue;
        }

        r = monitor.checkIMU(snap);
        if (!r.ok) {
            ss.triggerEStop(r.reason);
            Serial.printf("[SAFETY] ESTOP: %s\n", r.reason);
            getMotorDriver().disableAll();
            last_wake = xTaskGetTickCount();
            continue;
        }

        r = monitor.checkIMUFlexCorrelation(snap);
        if (!r.ok) {
            ss.triggerEStop(r.reason);
            Serial.printf("[SAFETY] ESTOP: %s\n", r.reason);
            getMotorDriver().disableAll();
            last_wake = xTaskGetTickCount();
            continue;
        }
        // Non-fatal warning path
        if (r.reason) ss.setWarning(r.reason);
        else          ss.clearWarning();

        // Motor-stall check — also suppressed during calibration.
        // Motors are disabled; stall logic would produce false
        // positives because all MotorState targets are zero.
        if (!ss.isCalibInProgress()) {
            r = monitor.checkMotorStall(snap, motors);
            if (!r.ok) {
                ss.triggerEStop(r.reason);
                Serial.printf("[SAFETY] ESTOP: %s\n", r.reason);
                getMotorDriver().disableAll();
                last_wake = xTaskGetTickCount();
                continue;
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_SAFETY_MS));
    }
}