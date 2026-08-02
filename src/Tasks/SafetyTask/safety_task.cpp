#include "Tasks/SafetyTask/safety_task.h"
#include "systemstate/System_State.h"
#include "SafetyMonitor.h"
#include "MotorDriver.h"
#include "config.h"
#include <Arduino.h>

static SafetyMonitor monitor;

static MotorDriver kill_driver;
static bool kill_driver_ready = false;

// ================================================================
// FLEX stability filter
// ================================================================
static const uint8_t FLEX_FAIL_CONFIRM = 3;
static uint8_t flex_fail_counter = 0;

// ================================================================
void safety_task(void* pvParam)
{
    SharedState& ss = SharedState::get();

    kill_driver.begin();
    kill_driver_ready = true;
    kill_driver.stopAll();

    TickType_t last_wake = xTaskGetTickCount();

    SensorSnapshot snap;
    MotorState motors[NUM_FINGERS];

    for (;;)
    {
        // ========================================================
        // ESTOP HANDLING
        // ========================================================
        if (ss.isEStop())
        {
            if (kill_driver_ready)
                kill_driver.disableAll();

            last_wake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(PERIOD_SAFETY_MS));
            continue;
        }

        ss.readSensors(snap);
        ss.readMotors(motors);

        SafetyMonitor::Report r;

        // ========================================================
        // FLEX CHECK
        // ========================================================
        r = monitor.checkFlex(snap);

        if (!r.ok)
            flex_fail_counter++;
        else
            flex_fail_counter = 0;

        if (flex_fail_counter >= FLEX_FAIL_CONFIRM)
        {
            ss.triggerEStop(r.reason);
            goto estop;
        }

        // ========================================================
        // IMU CHECK
        // ========================================================
        r = monitor.checkIMU(snap);

        if (!r.ok)
        {
            ss.triggerEStop(r.reason);
            goto estop;
        }

        // ========================================================
        // NORMAL LOOP DELAY
        // ========================================================
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_SAFETY_MS));
        continue;

estop:

        if (kill_driver_ready)
            kill_driver.disableAll();

        Serial.println("[SAFETY] *** EMERGENCY STOP ***");

        last_wake = xTaskGetTickCount();
    }
}