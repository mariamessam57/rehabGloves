#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "systemstate/System_State.h"
#include "config.h"

// Tasks
#include "Tasks/SensorTask/sensor_task.h"
#include "Tasks/ControlTask/control_task.h"
#include "Tasks/SafetyTask/safety_task.h"
#include "Tasks/InputTask/input_task.h"

// Calibration
#include "Calibration.h"

// ================================================================
// GLOBAL FLAGS (LINKER FIX)
// ================================================================
volatile bool calib_done = false;

// ================================================================
// FLEX PINS
// ================================================================
const uint8_t flex_pins[NUM_FINGERS] = {
    PIN_FLEX_0,
    PIN_FLEX_1,
    PIN_FLEX_2
};

// ================================================================
// BOOT CALIBRATION TASK
// ================================================================
void bootCalibrationTask(void* pv)
{
    Serial.println("\n🔥 BOOT CALIBRATION START");

    SharedState& ss = SharedState::get();
    ss.setMode(SystemMode::CALIBRATING);

    FlexCalib calib[NUM_FINGERS];
    CalibrationSystem calibration;

    bool ok = calibration.runCalibration(
        flex_pins,
        calib,
        nullptr
    );

    if (ok) {
        Serial.println("[BOOT] Calibration SUCCESS → saving");

        calibration.save(calib);

        ss.setCalibComplete(true);
        ss.setCalibPhase(CalibPhase::DONE);

        Serial.println("[BOOT] CALIBRATION DONE ✅");
    } 
    else {
        Serial.println("[BOOT] Calibration FAILED ❌");

        ss.setCalibComplete(false);
        ss.setMode(SystemMode::SAFE_LOCK);
    }

    calib_done = true;

    ss.setMode(SystemMode::PASSIVE);

    vTaskDelete(NULL);
}
// ================================================================
// SETUP
// ================================================================
void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(200);

    Serial.println("\n=== Rehab Glove Firmware Boot ===");

    SharedState::get().init();

    // ============================================================
    // BOOT CALIBRATION TASK
    // ============================================================
    xTaskCreatePinnedToCore(
        bootCalibrationTask,
        "BootCalib",
        9000,
        nullptr,
        5,
        nullptr,
        1
    );

    // ============================================================
    // SYSTEM TASKS
    // ============================================================

    xTaskCreatePinnedToCore(
        safety_task,
        "Safety",
        STACK_SAFETY,
        nullptr,
        PRI_SAFETY,
        nullptr,
        0
    );

    xTaskCreatePinnedToCore(
        sensor_task,
        "Sensor",
        STACK_SENSOR,
        nullptr,
        PRI_SENSOR,
        nullptr,
        1
    );

    xTaskCreatePinnedToCore(
        control_task,
        "Control",
        STACK_CONTROL,
        nullptr,
        PRI_CONTROL,
        nullptr,
        0
    );

    xTaskCreatePinnedToCore(
        input_task,
        "Input",
        STACK_INPUT,
        nullptr,
        PRI_INPUT,
        nullptr,
        0
    );

    Serial.println("[MAIN] All tasks created. Scheduler running.");
}

// ================================================================
// LOOP (NOT USED)
// ================================================================
void loop()
{
    vTaskDelay(portMAX_DELAY);
}