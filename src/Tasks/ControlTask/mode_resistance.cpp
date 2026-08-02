#include "Tasks/ControlTask/control_modes.h"
#include "config.h"
#include <Arduino.h>

// ─────────────────────────────
// SYSTEM STATE
// ─────────────────────────────
enum SystemState
{
    CALIBRATION,
    ACTIVE
};

static SystemState system_state = CALIBRATION;

// ─────────────────────────────
// MOTION STATE
// ─────────────────────────────
enum MotionState
{
    TO_TOP,
    TO_BOTTOM
};

static MotionState state = TO_TOP;

// ─────────────────────────────
// CALIBRATION (FSR ONLY FIRST 5 SEC)
// ─────────────────────────────
static uint32_t start_ms = 0;
static float fsr_sum = 0.0f;
static uint32_t fsr_count = 0;

static float fsr_base = 0.0f;

// ✔ PWM ثابت بعد المعايرة
static uint8_t locked_pwm = 120;

// ─────────────────────────────
// CONTROL
// ─────────────────────────────
static bool first_target_selected = false;
static bool holding = false;
static uint32_t hold_start = 0;

// ─────────────────────────────
// ARRIVAL
// ─────────────────────────────
static uint8_t reach_counter = 0;

// ─────────────────────────────
// CONSTANTS
// ─────────────────────────────
const float TARGET_TOP = 90.0f;
const float TARGET_BOTTOM = 6.0f;

const uint8_t REQUIRED_SAMPLES = 4;
const uint32_t HOLD_TIME_MS = 5000;

// ─────────────────────────────
void modeResistance(const SensorSnapshot &snap,
                    MotorState out[NUM_FINGERS])
{
    uint32_t now = millis();

    float fsr = snap.fsr.raw;
    float flex = snap.flex[MASTER_FLEX_IDX].normalized;
    float angle = flex * 90.0f;

    // ─────────────────────────────
    // INIT
    // ─────────────────────────────
    if (start_ms == 0)
    {
        start_ms = now;
        Serial.println("[SYSTEM] CALIBRATION MODE");
    }

    // ─────────────────────────────
    // CALIBRATION (ONLY FSR FOR 5 SEC)
    // ─────────────────────────────
    if (system_state == CALIBRATION)
    {
        fsr_sum += fsr;
        fsr_count++;

        for (int i = 0; i < NUM_FINGERS; i++)
        {
            out[i].enabled = false;
            out[i].target = 0;
            out[i].target_pwm = 0;
            out[i].dir = MotorDir::STOP;
        }

        if (now - start_ms >= 5000)
        {
            fsr_base = fsr_sum / (float)fsr_count;

            // ✔ نحدد PWM مرة واحدة فقط
            locked_pwm = (uint8_t)constrain(
                120 + fsr_base * 0.01f,
                80,
                255
            );

            system_state = ACTIVE;

            Serial.println("[FSR DONE]");
            Serial.print("BASE=");
            Serial.println(fsr_base);
            Serial.print("PWM LOCKED=");
            Serial.println(locked_pwm);
        }

        return;
    }

    // ─────────────────────────────
    // FIRST TARGET
    // ─────────────────────────────
    if (!first_target_selected)
    {
        state = (angle > 50.0f) ? TO_BOTTOM : TO_TOP;
        first_target_selected = true;
    }

    // ─────────────────────────────
    // HOLD
    // ─────────────────────────────
    if (holding)
    {
        for (int i = 0; i < NUM_FINGERS; i++)
        {
            out[i].enabled = false;
            out[i].target = 0;
            out[i].target_pwm = 0;
            out[i].dir = MotorDir::STOP;
        }

        if (now - hold_start >= HOLD_TIME_MS)
        {
            holding = false;
            state = (state == TO_TOP) ? TO_BOTTOM : TO_TOP;
            reach_counter = 0;

            Serial.println("[STATE FLIP]");
        }

        return;
    }

    // ─────────────────────────────
    // TARGET
    // ─────────────────────────────
    float target = (state == TO_TOP) ? TARGET_TOP : TARGET_BOTTOM;

    MotorDir dir = (state == TO_TOP)
        ? MotorDir::FORWARD
        : MotorDir::REVERSE;

    // ─────────────────────────────
    // ARRIVAL (4 READINGS ONLY)
    // ─────────────────────────────
    bool inside_target = (state == TO_TOP)
        ? (angle >= TARGET_TOP)
        : (angle <= TARGET_BOTTOM);

    if (inside_target)
        reach_counter++;
    else
        reach_counter = 0;

    if (reach_counter >= REQUIRED_SAMPLES)
    {
        holding = true;
        hold_start = now;
        reach_counter = 0;

        Serial.println("[HOLD START]");
        Serial.print("ANGLE=");
        Serial.println(angle);

        return;
    }

    // ─────────────────────────────
    // PWM ثابت بعد أول 5 ثواني (FSR خارج اللعبة هنا)
    // ─────────────────────────────
    uint8_t pwm = locked_pwm;

    for (int i = 0; i < NUM_FINGERS; i++)
    {
        out[i].enabled = true;
        out[i].target = pwm;
        out[i].target_pwm = pwm;
        out[i].dir = dir;
    }

    // ─────────────────────────────
    // DEBUG
    // ─────────────────────────────
    static uint32_t last_print = 0;

    if (now - last_print > 100)
    {
        last_print = now;

        Serial.print("angle=");
        Serial.print(angle);

        Serial.print(" target=");
        Serial.print(target);

        Serial.print(" fsr=");
        Serial.print(fsr);

        Serial.print(" pwm=");
        Serial.print(pwm);

        Serial.print(" reach=");
        Serial.println(reach_counter);
    }
}