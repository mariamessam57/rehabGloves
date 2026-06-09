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

    for (;;) {
        char key = keypad.getKey();

        if (key != NO_KEY) {
            Serial.printf("[INPUT] Key pressed: %c\n", key);

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

                case '4':  // ESTOP + Reset + Recalibration
                    Serial.println("[INPUT] Manual ESTOP + Recalibration triggered.");
                    // Clear saved calibration so SensorTask re-runs it
                    {
                        CalibrationSystem tmp;
                        tmp.clear();
                    }
                    ss.requestRecalibration();
                    ss.clearEStop();
                    break;
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
    }
}