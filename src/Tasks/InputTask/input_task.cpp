// ================================================================
//  input_task.cpp  — PURE EVENT PRODUCER
//
//  Architectural contract (enforced by code structure):
//
//  ✓ Reads physical GPIO buttons with debouncing.
//  ✓ Reads current SystemMode from SharedState (READ-ONLY).
//  ✓ Maps each button press to exactly one EventGroup bit.
//  ✓ Sets that bit via ss.setEventBits() and returns.
//
//  ✗ NEVER calls setMode(), setCalibPhase(), setManualCalibStep()
//    or any other state-mutating SharedState method.
//  ✗ NEVER interprets what a button event means for the FSM.
//  ✗ NEVER makes decisions about valid/invalid transitions.
//  ✗ NEVER tracks session state (paused/active).
//
//  The mapping table:
//
//  Mode             Key   Event emitted
//  ─────────────────────────────────────────────────────────────
//  SAFE_LOCK         1    EVT_START_PASSIVE
//  SAFE_LOCK         2    EVT_START_ASSISTIVE
//  SAFE_LOCK         3    EVT_START_RESISTANCE
//  SAFE_LOCK         4    EVT_START_CALIB
//  ─────────────────────────────────────────────────────────────
//  CALIBRATING       1    EVT_CALIB_AUTO
//  CALIBRATING       2    EVT_CALIB_MANUAL
//  CALIBRATING       4    EVT_GLOBAL_ESTOP
//  ─────────────────────────────────────────────────────────────
//  MANUAL_CALIB      1    EVT_BTN_CONFIRM
//  MANUAL_CALIB      2    EVT_BTN_MORE
//  MANUAL_CALIB      4    EVT_GLOBAL_ESTOP
//  ─────────────────────────────────────────────────────────────
//  PASSIVE /
//  ASSISTIVE /
//  RESISTANCE        1    EVT_SESSION_PAUSE
//                    2    EVT_SESSION_RECALIB
//                    3    EVT_SESSION_EXIT
//                    4    EVT_GLOBAL_ESTOP
//  ─────────────────────────────────────────────────────────────
//
//  control_task decides what each event means for the FSM.
//  input_task does not know and does not care.
// ================================================================

#include "Tasks/InputTask/input_task.h"
#include "systemstate/System_State.h"
#include "FSM/fsm_events.h"
#include "config.h"
#include <Arduino.h>

// ── Debounce state ────────────────────────────────────────────────
static struct {
    TickType_t last_tick[4];
    bool       pressed  [4];
    bool       prev     [4];
} btn = {};

// ── Rising-edge detection with RTOS-tick debounce ─────────────────
// Returns '1'..'4' on the first tick a button is seen pressed,
// or '\0' if no new press this call.
static char read_button_event() {
    const TickType_t now = xTaskGetTickCount();
    const TickType_t debounce_ticks = pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS);

    const bool raw[4] = {
        digitalRead(PIN_BUTTON_1) == LOW,
        digitalRead(PIN_BUTTON_2) == LOW,
        digitalRead(PIN_BUTTON_3) == LOW,
        digitalRead(PIN_BUTTON_4) == LOW
    };

    for (int i = 0; i < 4; i++) {
        if (raw[i] != btn.pressed[i] &&
            (now - btn.last_tick[i]) >= debounce_ticks) {
            btn.pressed  [i] = raw[i];
            btn.last_tick[i] = now;
        }
    }

    char triggered = '\0';
    for (int i = 0; i < 4; i++) {
        if (btn.pressed[i] && !btn.prev[i]) {
            btn.prev[i] = true;
            triggered   = (char)('1' + i);
        } else if (!btn.pressed[i] && btn.prev[i]) {
            btn.prev[i] = false;
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

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        const char key = read_button_event();

        if (key != '\0') {
            // Read mode once — READ ONLY, no write ever
            const SystemMode mode = ss.getMode();

            Serial.printf("[INPUT] key=%c mode=%d\n", key, (int)mode);

            EventBits_t evt = 0;  // event to emit; 0 = nothing

            // ── SAFE_LOCK: main menu ──────────────────────────────
            if (mode == SystemMode::SAFE_LOCK) {
                switch (key) {
                    case '1': evt = EVT_START_PASSIVE;    break;
                    case '2': evt = EVT_START_ASSISTIVE;  break;
                    case '3': evt = EVT_START_RESISTANCE; break;
                    case '4': evt = EVT_START_CALIB;      break;
                }
            }

            // ── CALIBRATING: type selection ───────────────────────
            else if (mode == SystemMode::CALIBRATING) {
                switch (key) {
                    case '1': evt = EVT_CALIB_AUTO;    break;
                    case '2': evt = EVT_CALIB_MANUAL;  break;
                    case '4': evt = EVT_GLOBAL_ESTOP;  break;
                    // keys 3 ignored in calib entry
                }
            }

            // ── MANUAL_CALIB: position confirmation ───────────────
            else if (mode == SystemMode::MANUAL_CALIB) {
                switch (key) {
                    case '1': evt = EVT_BTN_CONFIRM;   break;
                    case '2': evt = EVT_BTN_MORE;      break;
                    case '4': evt = EVT_GLOBAL_ESTOP;  break;
                }
            }

            // ── Session modes: therapy controls ───────────────────
            else if (mode == SystemMode::PASSIVE   ||
                     mode == SystemMode::ASSISTIVE ||
                     mode == SystemMode::RESISTANCE) {
                switch (key) {
                    case '1': evt = EVT_SESSION_PAUSE;   break;
                    case '2': evt = EVT_SESSION_RECALIB; break;
                    case '3': evt = EVT_SESSION_EXIT;    break;
                    case '4': evt = EVT_GLOBAL_ESTOP;    break;
                }
            }

            // ── ESTOP / unknown: no events emitted ───────────────
            // (system is frozen; only power cycle recovers)

            if (evt != 0) {
                ss.setEventBits(evt);
                Serial.printf("[INPUT] EVT=0x%04X emitted\n", (unsigned)evt);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
    }
}