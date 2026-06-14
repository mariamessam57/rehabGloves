// ================================================================
//  input_task.cpp  — REDESIGNED
//
//  New UI flow:
//
//  SAFE_LOCK (main menu):
//    1 → PASSIVE    (requires calib)
//    2 → ASSISTIVE  (requires calib)
//    3 → RESISTANCE (requires calib)
//    4 → CALIBRATING (calib entry)
//
//  CALIBRATING / phase==IDLE (calib entry question):
//    1 → Auto calibration  (mode stays CALIBRATING, phase→OPEN_HAND)
//    2 → Manual calibration (mode→MANUAL_CALIB, step→WARN_OPEN)
//    4 → ESTOP
//
//  CALIBRATING / phase==OPEN_HAND or CLOSE_HAND (auto, in progress):
//    4 → ESTOP only
//
//  MANUAL_CALIB / step==WAIT_OPEN_CONFIRM or WAIT_CLOSE_CONFIRM:
//    1 → confirm position (setManualCalibConfirmed)
//    2 → more movement   (setManualCalibMore)
//    4 → ESTOP
//
//  PASSIVE / ASSISTIVE / RESISTANCE (session active):
//    1 → Pause / Resume
//    2 → Restart calibration
//    3 → Exit to main menu
//    4 → ESTOP
// ================================================================

#include "Tasks/InputTask/input_task.h"
#include "systemstate/System_State.h"
#include "config.h"
#include <Arduino.h>

#define NO_KEY '\0'

static struct {
    uint32_t last_press_ms[4] = {0, 0, 0, 0};
    bool     pressed[4]       = {false, false, false, false};
    bool     prev_pressed[4]  = {false, false, false, false};
} btn;

static char read_button() {
    uint32_t now = millis();

    bool raw[4] = {
        digitalRead(PIN_BUTTON_1) == LOW,
        digitalRead(PIN_BUTTON_2) == LOW,
        digitalRead(PIN_BUTTON_3) == LOW,
        digitalRead(PIN_BUTTON_4) == LOW
    };

    for (int i = 0; i < 4; i++) {
        if (raw[i] != btn.pressed[i]) {
            if ((now - btn.last_press_ms[i]) >= BUTTON_DEBOUNCE_MS) {
                btn.pressed[i]       = raw[i];
                btn.last_press_ms[i] = now;
            }
        }
    }

    char triggered = NO_KEY;
    for (int i = 0; i < 4; i++) {
        if (btn.pressed[i] && !btn.prev_pressed[i]) {
            btn.prev_pressed[i] = true;
            triggered = ('1' + i);
        } else if (!btn.pressed[i] && btn.prev_pressed[i]) {
            btn.prev_pressed[i] = false;
        }
    }
    return triggered;
}

// ================================================================
void input_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
    pinMode(PIN_BUTTON_3, INPUT_PULLUP);
    pinMode(PIN_BUTTON_4, INPUT_PULLUP);

    TickType_t    last_wake      = xTaskGetTickCount();
    static bool   session_active = false;
    static bool   session_paused = false;
    static SystemMode saved_mode = SystemMode::SAFE_LOCK;

    for (;;) {
        SystemMode mode = ss.getMode();

        // Track session state
        if (mode == SystemMode::PASSIVE   ||
            mode == SystemMode::ASSISTIVE ||
            mode == SystemMode::RESISTANCE) {
            session_active = true;
        } else if (mode == SystemMode::SAFE_LOCK && !session_paused) {
            session_active = false;
        }

        char key = read_button();
        if (key == NO_KEY) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
            continue;
        }

        Serial.printf("[INPUT] Key: %c  Mode: %d\n", key, (int)mode);

        // ──────────────────────────────── ESTOP (always key 4) ───
        // Handled per-branch below for clarity, but key 4 is ESTOP
        // in every mode except SAFE_LOCK where it enters calibration.

        // ─────────────────────────────────────── SAFE_LOCK (menu) ─
        if (mode == SystemMode::SAFE_LOCK && !session_paused) {
            switch (key) {
                case '1':
                    if (ss.isCalibComplete() && !ss.isEStop()) {
                        ss.setMode(SystemMode::PASSIVE);
                        Serial.println("[INPUT] → PASSIVE");
                    } else {
                        ss.setWarning("NEED CALIB");
                    }
                    break;
                case '2':
                    if (ss.isCalibComplete() && !ss.isEStop()) {
                        ss.setMode(SystemMode::ASSISTIVE);
                        Serial.println("[INPUT] → ASSISTIVE");
                    } else {
                        ss.setWarning("NEED CALIB");
                    }
                    break;
                case '3':
                    if (ss.isCalibComplete() && !ss.isEStop()) {
                        ss.setMode(SystemMode::RESISTANCE);
                        Serial.println("[INPUT] → RESISTANCE");
                    } else {
                        ss.setWarning("NEED CALIB");
                    }
                    break;
                case '4':
                    // Enter calibration
                    ss.requestRecalibration();
                    ss.setMode(SystemMode::CALIBRATING);
                    ss.setCalibPhase(CalibPhase::IDLE);
                    ss.setCalibComplete(false);
                    ss.setCalibManualMode(false);
                    ss.setManualCalibStep(ManualCalibStep::IDLE);
                    ss.clearWarning();
                    Serial.println("[INPUT] → CALIBRATING (entry)");
                    break;
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
            continue;
        }

        // ──────────────────────────── CALIBRATING entry (phase==IDLE) ─
        if (mode == SystemMode::CALIBRATING
            && ss.getCalibPhase() == CalibPhase::IDLE) {
            switch (key) {
                case '1':   // Auto calibration
                    ss.setCalibManualMode(false);
                    ss.setCalibPhase(CalibPhase::OPEN_HAND);
                    ss.clearWarning();
                    Serial.println("[INPUT] Auto calib selected → OPEN_HAND");
                    break;
                case '2':   // Manual calibration
                    ss.setCalibManualMode(true);
                    ss.setMode(SystemMode::MANUAL_CALIB);
                    ss.setManualCalibStep(ManualCalibStep::WARN_OPEN);
                    ss.setManualCountdown(3);   // 3 2 1 warning
                    ss.clearWarning();
                    Serial.println("[INPUT] Manual calib selected → WARN_OPEN");
                    break;
                case '4':
                    ss.triggerEStop("MANUAL_OVERRIDE");
                    break;
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
            continue;
        }

        // ─────────────────── CALIBRATING (auto, in progress) ─────
        if (mode == SystemMode::CALIBRATING
            && ss.getCalibPhase() != CalibPhase::IDLE
            && ss.getCalibPhase() != CalibPhase::DONE) {
            if (key == '4') {
                ss.triggerEStop("MANUAL_OVERRIDE");
            }
            // No other inputs valid during auto calibration
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
            continue;
        }

        // ────────────────────────────────── MANUAL_CALIB buttons ─
        if (mode == SystemMode::MANUAL_CALIB) {
            ManualCalibStep step = ss.getManualCalibStep();

            switch (key) {
                case '1':
                    // Confirm current position (open or close)
                    if (step == ManualCalibStep::WAIT_OPEN_CONFIRM ||
                        step == ManualCalibStep::WAIT_CLOSE_CONFIRM) {
                        ss.setManualCalibConfirmed(true);
                        Serial.println("[INPUT] Manual calib: position confirmed");
                    }
                    break;
                case '2':
                    // More movement in current direction
                    if (step == ManualCalibStep::WAIT_OPEN_CONFIRM  ||
                        step == ManualCalibStep::MOVING_OPEN        ||
                        step == ManualCalibStep::WAIT_CLOSE_CONFIRM ||
                        step == ManualCalibStep::MOVING_CLOSE) {
                        ss.setManualCalibMore(true);
                        Serial.println("[INPUT] Manual calib: more movement requested");
                    }
                    break;
                case '4':
                    ss.triggerEStop("MANUAL_OVERRIDE");
                    break;
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
            continue;
        }

        // ──────────────────────────────────── SESSION ACTIVE ──────
        if (session_active) {
            switch (key) {
                case '1':   // Pause / Resume
                    if (!session_paused) {
                        saved_mode     = mode;
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
                case '2':   // Restart calibration
                    ss.requestRecalibration();
                    ss.setMode(SystemMode::CALIBRATING);
                    ss.setCalibPhase(CalibPhase::IDLE);
                    ss.setCalibComplete(false);
                    ss.setCalibManualMode(false);
                    ss.setManualCalibStep(ManualCalibStep::IDLE);
                    session_active = false;
                    session_paused = false;
                    ss.clearWarning();
                    Serial.println("[INPUT] Calibration restart.");
                    break;
                case '3':   // Exit session
                    session_active = false;
                    session_paused = false;
                    ss.clearWarning();
                    ss.setMode(SystemMode::SAFE_LOCK);
                    Serial.println("[INPUT] Session exit.");
                    break;
                case '4':   // ESTOP
                    session_active = false;
                    session_paused = false;
                    ss.clearWarning();
                    ss.triggerEStop("MANUAL_OVERRIDE");
                    break;
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
    }
}