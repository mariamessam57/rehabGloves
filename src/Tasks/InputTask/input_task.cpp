#include "Tasks/InputTask/input_task.h"
#include "systemstate/System_State.h"
#include "Calibration.h"
#include "config.h"
#include <Keypad.h>
#include <Arduino.h>

// ─────────────────────────────────────────────
// KEYPAD CONFIG
// ─────────────────────────────────────────────
static const char KEYS[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2'},
    {'3', '4'}
};

static uint8_t ROW_PINS[KEYPAD_ROWS] = { PIN_ROW0, PIN_ROW1 };
static uint8_t COL_PINS[KEYPAD_COLS] = { PIN_COL0, PIN_COL1 };

static Keypad keypad(
    makeKeymap(KEYS),
    ROW_PINS,
    COL_PINS,
    KEYPAD_ROWS,
    KEYPAD_COLS
);

// ─────────────────────────────────────────────
// MENU
// ─────────────────────────────────────────────
static void printMenu() {
    Serial.println("\n========== MODE MENU ==========");
    Serial.println("1 - ESTOP + Recalibration");
    Serial.println("2 - ASSISTIVE MODE");
    Serial.println("3 - RESISTANCE MODE");
    Serial.println("4 - PASSIVE MODE");
    Serial.println("C - CLEAR CALIBRATION");
    Serial.println("M - SHOW MENU");
    Serial.println("================================\n");
}

// ─────────────────────────────────────────────
// ACTIONS
// ─────────────────────────────────────────────
static void doRecalibration(SharedState& ss) {
    Serial.println("[SYSTEM] ESTOP + RECALIBRATION");

    CalibrationSystem tmp;
    tmp.clear();

    ss.requestRecalibration();
    ss.clearEStop();
}

static void clearCalibration(SharedState& ss) {
    Serial.println("[SYSTEM] CLEAR + RECALIBRATION");

    CalibrationSystem tmp;
    tmp.clear();

    ss.requestRecalibration();
    ss.clearEStop();
}

// ─────────────────────────────────────────────
// FIX: MODE SAFE WITH DEBUG (IMPORTANT)
// ─────────────────────────────────────────────
static void setModeSafe(SharedState& ss, SystemMode mode, const char* name) {

    bool calib = ss.isCalibComplete();
    bool estop = ss.isEStop();

    Serial.printf("[MODE CHECK] %s | CALIB=%d ESTOP=%d\n",
                  name, calib, estop);

    if (!calib) {
        Serial.println("[MODE BLOCKED] Calibration not complete");
        return;
    }

    if (estop) {
        Serial.println("[MODE BLOCKED] ESTOP active");
        return;
    }

    Serial.printf("[SYSTEM] MODE → %s\n", name);
    ss.setMode(mode);
}

// ─────────────────────────────────────────────
// SERIAL INPUT
// ─────────────────────────────────────────────
static void handleSerialInput(SharedState& ss) {

    if (!Serial.available()) return;

    char cmd = Serial.read();

    if (cmd == '\n' || cmd == '\r') return;

    Serial.printf("[SERIAL] CMD: %c\n", cmd);

    switch (cmd) {

        case '1':
            doRecalibration(ss);
            break;

        case '2':
            setModeSafe(ss, SystemMode::ASSISTIVE, "ASSISTIVE");
            break;

        case '3':
            setModeSafe(ss, SystemMode::RESISTANCE, "RESISTANCE");
            break;

        case '4':
            setModeSafe(ss, SystemMode::PASSIVE, "PASSIVE");
            break;

        case 'c':
        case 'C':
            clearCalibration(ss);
            break;

        case 'm':
        case 'M':
            printMenu();
            break;

        default:
            Serial.println("[SERIAL] UNKNOWN COMMAND");
            break;
    }
}

// ─────────────────────────────────────────────
// INPUT TASK
// ─────────────────────────────────────────────
void input_task(void* pvParam) {

    SharedState& ss = SharedState::get();

    keypad.setDebounceTime(80);

    Serial.println("\n🔥 INPUT TASK STARTED");
    printMenu();

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {

        // ================= KEYPAD =================
        char key = keypad.getKey();

        if (key != NO_KEY) {

            Serial.printf("[KEYPAD] KEY: %c\n", key);

            switch (key) {

                case '1':
                    doRecalibration(ss);
                    break;

                case '2':
                    setModeSafe(ss, SystemMode::ASSISTIVE, "ASSISTIVE");
                    break;

                case '3':
                    setModeSafe(ss, SystemMode::RESISTANCE, "RESISTANCE");
                    break;

                case '4':
                    setModeSafe(ss, SystemMode::PASSIVE, "PASSIVE");
                    break;

                default:
                    Serial.println("[KEYPAD] INVALID KEY");
                    break;
            }
        }

        // ================= SERIAL =================
        handleSerialInput(ss);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
    }
}