#include "Tasks/InputTask/input_task.h"
#include "systemstate/System_State.h"
#include "Calibration.h"
#include "config.h"
#include <Arduino.h>

// ── Define NO_KEY sentinel value ───────────────────────────────────────
#define NO_KEY '\0'

// ── Button state tracking for debouncing ──────────────────────────────
static struct {
    uint32_t last_press_ms[4];  // Last press time for each button
    bool     pressed[4];         // Current pressed state (debounced)
    bool     prev_pressed[4];    // Previous state to detect rising edge
} button_state = {{0, 0, 0, 0}, {false, false, false, false}, {false, false, false, false}};

// ── Read button state with robust non-blocking debouncing ───────────────
static char read_button(void) {
    uint32_t now_ms = millis();
    
    // Read raw GPIO states (LOW = pressed, HIGH = released due to PULLUP)
    bool btn1_raw = (digitalRead(PIN_BUTTON_1) == LOW);
    bool btn2_raw = (digitalRead(PIN_BUTTON_2) == LOW);
    bool btn3_raw = (digitalRead(PIN_BUTTON_3) == LOW);
    bool btn4_raw = (digitalRead(PIN_BUTTON_4) == LOW);
    
    bool raw_states[4] = {btn1_raw, btn2_raw, btn3_raw, btn4_raw};
    
    // 1. Debounce logic
    for (int i = 0; i < 4; i++) {
        if (raw_states[i] != button_state.pressed[i]) {
            if (now_ms - button_state.last_press_ms[i] >= BUTTON_DEBOUNCE_MS) {
                button_state.pressed[i] = raw_states[i];
                button_state.last_press_ms[i] = now_ms;
            }
        }
    }
    
    char triggered_key = NO_KEY; // متغير مؤقت لحفظ الزر المكتشف
    
    // 2. تحديث كامل مصفوفة الحالات دون الخروج المفاجئ من الـ Loop لضمان استقرار الحواف
    for (int i = 0; i < 4; i++) {
        if (button_state.pressed[i] && !button_state.prev_pressed[i]) {
            button_state.prev_pressed[i] = true;
            triggered_key = ('1' + i); // تسجيل الضغطة الحالية
        }
        else if (!button_state.pressed[i] && button_state.prev_pressed[i]) {
            button_state.prev_pressed[i] = false; // تحديث حالة التحرير (Release) بأمان
        }
    }
    
    return triggered_key;
}

// ================================================================
void input_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    // Initialize button pins as INPUT_PULLUP (common pin tied to GND on PCB)
    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
    pinMode(PIN_BUTTON_3, INPUT_PULLUP);
    pinMode(PIN_BUTTON_4, INPUT_PULLUP);

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

        char key = read_button();

        if (key != NO_KEY) {
            Serial.printf("[INPUT] Key pressed: %c\n", key);

            // ─── منطق التحكم أثناء وضع المعايرة النشط ──────────────────────────
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

            // ─── منطق التحكم أثناء تشغيل إحدى الجلسات العلاجية ───────────────────
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
                // ─── التوزيع الجديد عند شاشة البداية (Safe Lock) ─────────────────
                switch (key) {
                    case '1':  // زر 1 ──> مود 1 (Passive)
                        if (ss.isCalibComplete() && !ss.isEStop()) {
                            ss.setMode(SystemMode::PASSIVE);
                            Serial.println("[INPUT] Mode 1: Passive Activated.");
                        } else {
                            ss.setWarning("NEED CALIB");
                            Serial.println("[INPUT] Cannot start Passive. Calibration required!");
                        }
                        break;

                    case '2':  // زر 2 ──> مود 2 (Assistive)
                        if (ss.isCalibComplete() && !ss.isEStop()) {
                            ss.setMode(SystemMode::ASSISTIVE);
                            Serial.println("[INPUT] Mode 2: Assistive Activated.");
                        } else {
                            ss.setWarning("NEED CALIB");
                            Serial.println("[INPUT] Cannot start Assistive. Calibration required!");
                        }
                        break;

                    case '3':  // زر 3 ──> مود 3 (Resistance)
                        if (ss.isCalibComplete() && !ss.isEStop()) {
                            ss.setMode(SystemMode::RESISTANCE);
                            Serial.println("[INPUT] Mode 3: Resistance Activated.");
                        } else {
                            ss.setWarning("NEED CALIB");
                            Serial.println("[INPUT] Cannot start Resistance. Calibration required!");
                        }
                        break;

                    case '4':  // زر 4 ──> الدخول إلى وضع المعايرة (Calibration) فوراً
                        ss.requestRecalibration();
                        ss.setMode(SystemMode::CALIBRATING);
                        ss.setCalibPhase(CalibPhase::IDLE);
                        ss.setCalibComplete(false);
                        ss.setCalibManualMode(false);
                        ss.clearWarning();
                        Serial.println("[INPUT] Key 4: Calibration Mode Initiated.");
                        break;
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_INPUT_MS));
    }
}