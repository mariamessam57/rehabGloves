#include "Tasks/InputTask/input_task.h"
#include "systemstate/System_State.h"
#include "Calibration.h"
#include "config.h"
#include <Keypad.h>
#include <Arduino.h>

// ── Keypad layout ─────────────────────────────────────────────
static const char KEYS[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2'},
    {'3', '4'}
};

static uint8_t ROW_PINS[KEYPAD_ROWS] = { PIN_ROW0, PIN_ROW1 };
static uint8_t COL_PINS[KEYPAD_COLS] = { PIN_COL0, PIN_COL1 };

static Keypad keypad = Keypad(
    makeKeymap(KEYS),
    ROW_PINS, COL_PINS,
    KEYPAD_ROWS, KEYPAD_COLS
);

// ================================================================
void input_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    keypad.setDebounceTime(80);  // ms

    TickType_t last_wake = xTaskGetTickCount();
    static bool        session_active = false;
    static bool        session_paused = false;
    static SystemMode  saved_mode     = SystemMode::SAFE_LOCK;
    static bool        calib_prompted = false;

    for (;;) {
        SystemMode current_mode = ss.getMode();

        if (current_mode == SystemMode::PASSIVE ||
            current_mode == SystemMode::ASSISTIVE ||
            current_mode == SystemMode::RESISTANCE)
        {
            session_active = true;
        } else if (!session_paused) {
            session_active = false;
        }

        if (current_mode == SystemMode::CALIBRATING &&
            ss.getCalibPhase() == CalibPhase::IDLE) {
            if (!calib_prompted) {
                Serial.println("[INPUT] Calibration prompt: MOVE? 1:Y 2:N");
                calib_prompted = true;
            }
        } else {
            calib_prompted = false;
        }

        char key = keypad.getKey();

        if (key != NO_KEY) {
            Serial.printf("[INPUT] Key pressed: %c\n", key);

            if (current_mode == SystemMode::CALIBRATING) {
                if (ss.getCalibPhase() == CalibPhase::IDLE) {
                    if (key == '1' || key == '2') {
                        bool manual = (key == '2');
                        ss.setCalibManualMode(manual);
                        ss.setCalibPhase(CalibPhase::OPEN_HAND);
                        ss.clearWarning();
                        Serial.printf("[INPUT] Calibration start: %s\n",
                            manual ? "MANUAL" : "AUTO");
                        Serial.println("[INPUT] Opening hand for 5 seconds...");
                        calib_prompted = false;
                    }
                } else if (key == '4') {
                    session_active = false;
                    session_paused = false;
                    ss.clearWarning();
                    ss.triggerEStop("MANUAL_OVERRIDE");
                    ss.setMode(SystemMode::SAFE_LOCK);
                    Serial.println("[INPUT] Global emergency stop triggered.");
                }
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
                continue;
            }

            if (session_active) {
                switch (key) {
                    case '1':  // PAUSE / RESUME
                        if (!session_paused) {
                            saved_mode = current_mode;
                            session_paused = true;
                            ss.setMode(SystemMode::SAFE_LOCK);
                            ss.setWarning("PAUSED");
                            Serial.println("[INPUT] Session paused.");
                        } else {
                            session_paused = false;
                            ss.clearWarning();
                            ss.setMode(saved_mode);
                            Serial.println("[INPUT] Session resumed.");
                        }
                        break;

                    case '2':  // RESTART CALIBRATION FLOW
                        if (!ss.isEStop()) {
                            ss.requestRecalibration();
                            ss.setMode(SystemMode::CALIBRATING);
                            ss.setCalibPhase(CalibPhase::IDLE);
                            ss.setCalibComplete(false);
                            ss.setCalibManualMode(false);
                            session_active = false;
                            session_paused = false;
                            ss.clearWarning();
                            Serial.println("[INPUT] Calibration restart requested.");
                        }
                        break;

                    case '3':  // EXIT SESSION
                        session_active = false;
                        session_paused = false;
                        ss.clearWarning();
                        ss.setMode(SystemMode::SAFE_LOCK);
                        Serial.println("[INPUT] Session exit requested.");
                        break;

                    case '4':  // GLOBAL EMERGENCY STOP
                        session_active = false;
                        session_paused = false;
                        ss.clearWarning();
                        ss.triggerEStop("MANUAL_OVERRIDE");
                        ss.setMode(SystemMode::SAFE_LOCK);
                        Serial.println("[INPUT] Global emergency stop triggered.");
                        break;
                }
            } else {
                switch (key) {
                    case '1':  // Passive Mode
                        if (ss.isCalibComplete() && !ss.isEStop())
                            ss.setMode(SystemMode::PASSIVE);
                        break;

                    case '2':  // Assistive Mode
                        if (ss.isCalibComplete() && !ss.isEStop())
                            ss.setMode(SystemMode::ASSISTIVE);
                        break;

                    case '3':  // Resistance Mode
                        if (ss.isCalibComplete() && !ss.isEStop())
                            ss.setMode(SystemMode::RESISTANCE);
                        break;

                    case '4':  // GLOBAL OVERRIDE: Emergency stop + return to menu
                        session_active = false;
                        session_paused = false;
                        ss.clearWarning();
                        ss.triggerEStop("MANUAL_OVERRIDE");
                        ss.setMode(SystemMode::SAFE_LOCK);
                        Serial.println("[INPUT] Global emergency stop triggered.");
                        break;
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
    }
}