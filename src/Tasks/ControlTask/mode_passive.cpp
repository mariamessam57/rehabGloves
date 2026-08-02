#include "Tasks/ControlTask/control_modes.h"
#include "config.h"
#include <Arduino.h>

void modePassive(const SensorSnapshot &snap, MotorState out[NUM_FINGERS])
{
    uint32_t now = millis();
    float phase = (float)(now % PASSIVE_PERIOD_MS) / (float)PASSIVE_PERIOD_MS;
    float target = 0.5f * (1.0f - cosf(2.0f * M_PI * phase));
    float master_norm = snap.flex[MASTER_FLEX_IDX].normalized;
    float error = target - master_norm;

    for (int f = 0; f < NUM_FINGERS; f++)
    {
        if (fabsf(error) < 0.03f)
        {
            out[f].enabled = false;
            out[f].target = 0;
            out[f].target_pwm = 0;
            out[f].dir = MotorDir::STOP;
        }
        else if (error > 0.0f)
        {
            out[f].enabled = true;
            out[f].dir = MotorDir::REVERSE;
            uint8_t duty = dutyFromFraction(fabsf(error));
            out[f].target = duty;
            out[f].target_pwm = duty;
        }
        else
        {
            out[f].enabled = true;
            out[f].dir = MotorDir::FORWARD;
            uint8_t duty = dutyFromFraction(fabsf(error));
            out[f].target = duty;
            out[f].target_pwm = duty;
        }
    }

    // ── Serial debug (200 ms throttle) ──────────────────────────
    static uint32_t last_print = 0;
    if (now - last_print < 200)
        return;
    last_print = now;

    float phase_pct = phase * 100.0f;

    Serial.println(F("=== PASSIVE ==="));
    Serial.print(F("  phase="));
    Serial.print(phase_pct, 1);
    Serial.print(F("%"));
    Serial.print(F("  target="));
    Serial.println(target, 3);

    Serial.print(F("  IMU  gyro=["));
    Serial.print(snap.imu.gyro[0], 2);
    Serial.print(F(", "));
    Serial.print(snap.imu.gyro[1], 2);
    Serial.print(F(", "));
    Serial.print(snap.imu.gyro[2], 2);
    Serial.print(F("]"));
    Serial.print(F("  mag="));
    Serial.print(snap.imu.gyro_mag, 2);
    Serial.print(F("  pitch="));
    Serial.print(snap.imu.pitch, 2);
    Serial.print(F("  roll="));
    Serial.println(snap.imu.roll, 2);

    Serial.print(F("  FSR  raw="));
    Serial.print(snap.fsr.raw, 1);
    Serial.print(F("  filt="));
    Serial.print(snap.fsr.filtered, 3);
    Serial.print(F("  norm="));
    Serial.println(snap.fsr.normalized, 3);

    const char *dirStr[] = {"STOP", "FWD", "REV"};
    int master = MASTER_FLEX_IDX;
    float err = target - snap.flex[master].normalized;
    Serial.print(F("  F"));
    Serial.print(master);
    Serial.print(F("  raw="));
    Serial.print(snap.flex[master].raw, 1);
    Serial.print(F("  filt="));
    Serial.print(snap.flex[master].filtered, 3);
    Serial.print(F("  norm="));
    Serial.print(snap.flex[master].normalized, 3);
    Serial.print(F("  ang="));
    Serial.print(snap.flex[master].angle, 1);
    Serial.print(F("  vel="));
    Serial.print(snap.flex[master].velocity, 4);
    Serial.print(F("  err="));
    Serial.print(err, 3);
    Serial.print(F("  en="));
    Serial.print(out[master].enabled ? "Y" : "N");
    Serial.print(F("  dir="));
    Serial.print(dirStr[(uint8_t)out[master].dir]);
    Serial.print(F("  pwm="));
    Serial.println(out[master].target);
}
