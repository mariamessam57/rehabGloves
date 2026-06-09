#include "Tasks/ControlTask/control_task.h"
#include "systemstate/System_State.h"
#include "MotorDriver.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>

struct AssistState {
    bool         assessment_done  = false;
    bool         fsr_reached      = false;
    float        patient_force    = 0.0f;
    uint8_t      force_level      = 0;
    float        motor_fraction   = 1.0f;
    uint32_t     assess_start_ms  = 0;
    float        step_target      = 0.0f;
    bool         step_active      = false;
    uint32_t     step_end_ms      = 0;
    uint32_t     stall_start_ms   = 0;
    MotionIntent last_intent      = MotionIntent::NONE;  // ← الإضافة
};
static AssistState assist_st;
static MotorDriver driver;

// ── Helpers ─────────────────────────────────────────────────────
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline uint8_t dutyFromFraction(float f) {
    return (uint8_t)clampf(f * (float)PWM_DUTY_MAX, (float)PWM_DUTY_MIN, (float)PWM_DUTY_MAX);
}

// ── Intent detection ─────────────────────────────────────────────
static MotionIntent detectIntent(const SensorSnapshot& snap) {
    if (snap.imu.gyro_mag < INTENT_GYRO_THRESH) return MotionIntent::NONE;

    float avg_vel = 0.0f;
    for (int f = 0; f < NUM_FINGERS; f++)
        avg_vel += snap.flex[f].velocity;
    avg_vel /= (float)NUM_FINGERS;

    if (fabsf(avg_vel) < INTENT_VEL_THRESH) return MotionIntent::NONE;

    // Gyro Y dominant axis for grasp motion
    return (avg_vel > 0.0f) ? MotionIntent::CLOSING : MotionIntent::OPENING;
}

// ── MODE 1: PASSIVE ──────────────────────────────────────────────
static void modePassive(const SensorSnapshot& snap, MotorState out[NUM_FINGERS]) {
    uint32_t now    = millis();
    float    phase  = (float)(now % PASSIVE_PERIOD_MS) / (float)PASSIVE_PERIOD_MS;
    float    target = 0.5f * (1.0f - cosf(2.0f * M_PI * phase));

    for (int f = 0; f < NUM_FINGERS; f++) {
        float error = target - snap.flex[f].normalized;
        if (fabsf(error) < 0.03f) {
            out[f].enabled = false;
            out[f].target  = 0;
            out[f].dir     = MotorDir::STOP;
        } else if (error > 0.0f) {
            out[f].enabled = true;
            out[f].dir     = MotorDir::REVERSE;
            out[f].target  = dutyFromFraction(fabsf(error));
        } else {
            out[f].enabled = true;
            out[f].dir     = MotorDir::FORWARD;
            out[f].target  = dutyFromFraction(fabsf(error));
        }
    }
}

// ── MODE 2: ASSISTIVE ────────────────────────────────────────────
static uint8_t computeForceLevel(float fsr_norm) {
    if (fsr_norm < 0.25f) return 0;
    if (fsr_norm < 0.50f) return 1;
    if (fsr_norm < 0.75f) return 2;
    return 3;
}

static void modeAssistive(const SensorSnapshot& snap,
                           MotorState out[NUM_FINGERS],
                           SharedState& ss)
{
    uint32_t now = millis();

    float avg_norm = 0.0f, avg_vel = 0.0f;
    for (int f = 0; f < NUM_FINGERS; f++) {
        avg_norm += snap.flex[f].normalized;
        avg_vel  += snap.flex[f].velocity;
    }
    avg_norm /= NUM_FINGERS;
    avg_vel  /= NUM_FINGERS;

    // ── مرحلة التقييم ──────────────────────────────────────────
    if (!assist_st.assessment_done) {
        if (assist_st.assess_start_ms == 0)
            assist_st.assess_start_ms = now;

        for (int f = 0; f < NUM_FINGERS; f++) {
            out[f].enabled = false;
            out[f].target  = 0;
            out[f].dir     = MotorDir::STOP;
        }

        if (snap.fsr.normalized > FSR_TOUCHED_THRESH) {
            assist_st.fsr_reached     = true;
            assist_st.patient_force   = snap.fsr.normalized;
            assist_st.force_level     = computeForceLevel(snap.fsr.normalized);
            assist_st.motor_fraction  = 1.0f - (assist_st.force_level * 0.25f);
            assist_st.assessment_done = true;
            Serial.printf("[ASSIST] FSR touched. Level=%d Motor=%.0f%%\n",
                assist_st.force_level, assist_st.motor_fraction * 100.0f);
            return;
        }

        if ((now - assist_st.assess_start_ms) >= ASSIST_ASSESS_MS) {
            if (avg_norm < ASSIST_FLEX_THRESHOLD) {
                Serial.println("[ASSIST] Weak effort → back to Mode 1");
                ss.setMode(SystemMode::PASSIVE);
                assist_st = AssistState{};
            } else {
                assist_st.motor_fraction  = 1.0f;
                assist_st.assessment_done = true;
                Serial.println("[ASSIST] No FSR but good flex → full assist");
            }
        }
        return;
    }

    // ── مرحلة المساعدة التدريجية ───────────────────────────────
    uint8_t motor_duty = (uint8_t)clampf(
        assist_st.motor_fraction * (float)PWM_DUTY_MAX,
        (float)PWM_DUTY_MIN, (float)PWM_DUTY_MAX);

    bool patient_moving = fabsf(avg_vel) > ASSIST_STALL_VEL;

    if (assist_st.step_active) {
        if (now < assist_st.step_end_ms) {
            float error = assist_st.step_target - avg_norm;
            for (int f = 0; f < NUM_FINGERS; f++) {
                if (fabsf(error) > 0.02f) {
                    out[f].enabled = true;
                    out[f].dir     = (error > 0.0f) ? MotorDir::REVERSE : MotorDir::FORWARD;
                    out[f].target  = motor_duty;
                } else {
                    out[f].enabled = false;
                    out[f].target  = 0;
                    out[f].dir     = MotorDir::STOP;
                }
            }
        } else {
            assist_st.step_active    = false;
            assist_st.stall_start_ms = 0;
            for (int f = 0; f < NUM_FINGERS; f++) {
                out[f].enabled = false;
                out[f].target  = 0;
                out[f].dir     = MotorDir::STOP;
            }
        }
        return;
    }

    if (patient_moving) {
        assist_st.stall_start_ms = 0;
        MotionIntent intent = detectIntent(snap);
        if (intent != MotionIntent::NONE)
            assist_st.last_intent = intent;  // ← الإضافة
        for (int f = 0; f < NUM_FINGERS; f++) {
            if (intent == MotionIntent::CLOSING) {
                out[f].enabled = true;
                out[f].dir     = MotorDir::REVERSE;
                out[f].target  = motor_duty;
            } else if (intent == MotionIntent::OPENING) {
                out[f].enabled = true;
                out[f].dir     = MotorDir::FORWARD;
                out[f].target  = motor_duty;
            } else {
                out[f].enabled = false;
                out[f].target  = 0;
                out[f].dir     = MotorDir::STOP;
            }
        }
        return;
    }

    if (assist_st.stall_start_ms == 0)
        assist_st.stall_start_ms = now;

// بعد التعديل
    if ((now - assist_st.stall_start_ms) >= ASSIST_STALL_MS) {
        // حدد اتجاه الـ step بناءً على آخر حركة للمريض
        float step_dir = (assist_st.last_intent == MotionIntent::OPENING) 
                         ? -ASSIST_STEP_NORM 
                         :  ASSIST_STEP_NORM;  // default: closing
        assist_st.step_target    = clampf(avg_norm + step_dir, 0.0f, 1.0f);
        assist_st.step_active    = true;
        assist_st.step_end_ms    = now + ASSIST_STEP_MS;
        assist_st.stall_start_ms = 0;
        Serial.printf("[ASSIST] Step → target=%.3f dir=%s\n",
            assist_st.step_target,
            (step_dir < 0) ? "OPEN" : "CLOSE");
    }else {
        for (int f = 0; f < NUM_FINGERS; f++) {
            out[f].enabled = false;
            out[f].target  = 0;
            out[f].dir     = MotorDir::STOP;
        }
    }
}

// ── MODE 3: RESISTANCE ───────────────────────────────────────────
static void modeResistance(const SensorSnapshot& snap, MotorState out[NUM_FINGERS]) {
    MotionIntent intent    = detectIntent(snap);
    float        fsr_norm  = snap.fsr.normalized;
    uint8_t      resist_pw = dutyFromFraction(fsr_norm);

    for (int f = 0; f < NUM_FINGERS; f++) {
        // Safety: respect flex limits before applying resistance
        float norm = snap.flex[f].normalized;
        bool  at_limit = (norm < 0.03f && intent == MotionIntent::OPENING) ||
                         (norm > 0.97f && intent == MotionIntent::CLOSING);

        if (at_limit) {
            out[f].enabled = false;
            out[f].target  = 0;
            out[f].dir     = MotorDir::STOP;
            continue;
        }

        switch (intent) {
            case MotionIntent::CLOSING:
                // resist = apply opening torque
                out[f].enabled = true;
                out[f].dir     = MotorDir::FORWARD;
                out[f].target  = resist_pw;
                break;
            case MotionIntent::OPENING:
                // resist = apply closing torque
                out[f].enabled = true;
                out[f].dir     = MotorDir::REVERSE;
                out[f].target  = resist_pw;
                break;
            default:
                out[f].enabled = false;
                out[f].target  = 0;
                out[f].dir     = MotorDir::STOP;
                break;
        }
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

    for (;;) {
        // ESTOP guard
        if (ss.isEStop()) {
            driver.disableAll();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        static SystemMode prev_mode = SystemMode::SAFE_LOCK;
        ss.readSensors(snap);
        SystemMode mode = ss.getMode();

        if (prev_mode == SystemMode::ASSISTIVE && mode != SystemMode::ASSISTIVE) {
            assist_st = AssistState{};
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
        }

        // Publish motor states for safety monitor
        ss.writeMotors(motors);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_CONTROL_MS));
    }
}