#pragma once

#include "systemstate/System_State.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi
                                    : v;
}

static inline uint8_t dutyFromFraction(float f)
{
    return (uint8_t)clampf(f * (float)PWM_DUTY_MAX, (float)PWM_DUTY_MIN, (float)PWM_DUTY_MAX);
}

static inline MotionIntent detectIntent(const SensorSnapshot &snap)
{
    if (snap.imu.gyro_mag < INTENT_GYRO_THRESH)
        return MotionIntent::NONE;

    float vel = snap.flex[MASTER_FLEX_IDX].velocity;

    if (fabsf(vel) < INTENT_VEL_THRESH)
        return MotionIntent::NONE;

    return (vel > 0.0f) ? MotionIntent::CLOSING : MotionIntent::OPENING;
}

void resetAssistState();
void modePassive(const SensorSnapshot &snap, MotorState out[NUM_FINGERS]);
void modeAssistive(const SensorSnapshot &snap, MotorState out[NUM_FINGERS], SharedState &ss);
void modeResistance(const SensorSnapshot &snap, MotorState out[NUM_FINGERS]);