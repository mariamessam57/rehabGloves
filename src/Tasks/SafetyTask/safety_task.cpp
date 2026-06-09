#include "Tasks/SafetyTask/safety_task.h"
#include "systemstate/System_State.h"
#include "SafetyMonitor.h"
#include "MotorDriver.h"
#include "config.h"
#include <Arduino.h>

static SafetyMonitor monitor;

// Separate hardware-kill driver instance for safety task
// (safety must never wait on control task's driver object)
static MotorDriver kill_driver;
static bool        kill_driver_ready = false;

static void buzzerInit() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}

static void buzzerPulse() {
    digitalWrite(PIN_BUZZER, HIGH);
    vTaskDelay(pdMS_TO_TICKS(150));
    digitalWrite(PIN_BUZZER, LOW);
    vTaskDelay(pdMS_TO_TICKS(150));
}

// ================================================================
void safety_task(void* pvParam) {
    SharedState& ss = SharedState::get();
    buzzerInit();

    // Initialize kill driver independently
    kill_driver.begin();
    kill_driver_ready = true;
    kill_driver.stopAll();

    TickType_t last_wake = xTaskGetTickCount();
    SensorSnapshot snap;
    MotorState     motors[NUM_FINGERS];

    for (;;) {
        // If already in ESTOP, just pulse buzzer and loop
        if (ss.isEStop()) {
            if (kill_driver_ready) kill_driver.disableAll();
            buzzerPulse();
            continue;  // no DelayUntil, buzzer timing controls loop
        }

        ss.readSensors(snap);
        ss.readMotors(motors);

        SafetyMonitor::Report r;

        r = monitor.checkFlex(snap);
        if (!r.ok) { ss.triggerEStop(r.reason); goto estop; }

        r = monitor.checkIMU(snap);
        if (!r.ok) { ss.triggerEStop(r.reason); goto estop; }

        r = monitor.checkMotorStall(snap, motors);
        if (!r.ok) { ss.triggerEStop(r.reason); goto estop; }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_SAFETY_MS));
        continue;

    estop:
        if (kill_driver_ready) kill_driver.disableAll();
        Serial.println("[SAFETY] *** EMERGENCY STOP ***");
        last_wake = xTaskGetTickCount(); // reset timing after estop
    }
}