#include "Tasks/ControlTask/control_task.h"
#include "Tasks/ControlTask/control_modes.h"
#include "systemstate/System_State.h"
#include "MotorDriver.h"
#include "config.h"
#include <Arduino.h>
static MotorDriver driver;

static constexpr bool MOTOR_SERIAL_LOGS = false;

static const char* modeName(SystemMode mode) {
    switch (mode) {
        case SystemMode::SAFE_LOCK:    return "SAFE_LOCK";
        case SystemMode::CALIBRATING:  return "CALIBRATING";
        case SystemMode::MANUAL_CALIB:  return "MANUAL_CALIB";
        case SystemMode::PASSIVE:      return "PASSIVE";
        case SystemMode::ASSISTIVE:    return "ASSISTIVE";
        case SystemMode::RESISTANCE:   return "RESISTANCE";
        case SystemMode::ESTOP:        return "ESTOP";
        default:                       return "UNKNOWN";
    }
}

static const char* dirName(MotorDir dir) {
    switch (dir) {
        case MotorDir::FORWARD: return "FORWARD";
        case MotorDir::REVERSE: return "REVERSE";
        case MotorDir::STOP:
        default:                return "STOP";
    }
}

// ================================================================
void control_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    // Wait for calibration before doing anything
    xEventGroupWaitBits(ss.events, EVT_CALIB_DONE,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    driver.begin();

    TickType_t    last_wake = xTaskGetTickCount();
    SensorSnapshot snap;
    MotorState     motors[NUM_FINGERS] = {};
    MotorState     prev_motors[NUM_FINGERS] = {};
    SystemMode     prev_mode = SystemMode::SAFE_LOCK;

    for (;;) {
        // ESTOP guard
        if (ss.isEStop()) {
            driver.disableAll();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        ss.readSensors(snap);
        SystemMode mode = ss.getMode();

        if (mode != prev_mode) {
            Serial.printf("[CONTROL] Mode now %s\n", modeName(mode));
        }

        if (mode == SystemMode::ASSISTIVE && prev_mode != SystemMode::ASSISTIVE) {
            resetAssistState();
            ss.clearWarning();
            Serial.println("[CONTROL] Assistive state armed for FSR-first entry");
        }

        if (prev_mode == SystemMode::ASSISTIVE && mode != SystemMode::ASSISTIVE) {
            resetAssistState();
            ss.clearWarning();
            Serial.println("[CONTROL] Assistive state reset");
        }
        prev_mode = mode;

        switch (mode) {
            case SystemMode::PASSIVE:    modePassive   (snap, motors); break;
            case SystemMode::ASSISTIVE:  modeAssistive(snap, motors, ss); break;
            case SystemMode::RESISTANCE: modeResistance(snap, motors); break;
            default:
                for (int f = 0; f < NUM_FINGERS; f++) motors[f].enabled = false;
                driver.stopAll();
                break;
        }

        // Apply ramping to each motor
        for (int f = 0; f < NUM_FINGERS; f++) {
            driver.applyRamp(motors[f], f);
            if (MOTOR_SERIAL_LOGS &&
                (motors[f].enabled != prev_motors[f].enabled ||
                 motors[f].dir != prev_motors[f].dir ||
                 motors[f].target != prev_motors[f].target ||
                 motors[f].current != prev_motors[f].current))
            {
                Serial.printf(
                    "[MOTOR] F%d mode=%s enabled=%d dir=%s target=%u current=%u\n",
                    f,
                    modeName(mode),
                    motors[f].enabled ? 1 : 0,
                    dirName(motors[f].dir),
                    motors[f].target,
                    motors[f].current);
            }
            prev_motors[f] = motors[f];
        }

        // Publish motor states for safety monitor
        ss.writeMotors(motors);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_CONTROL_MS));
    }
}