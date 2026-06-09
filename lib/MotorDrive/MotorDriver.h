#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include <Arduino.h>
#include "SystemTypes.h"
#include "Config.h"

// ================================================================
//  MOTOR DRIVER — DRV8833 single motor channel
//  Uses ESP32 LEDC for 20 kHz PWM
//  Safety: never sets IN1=HIGH and IN2=HIGH simultaneously
// ================================================================
class MotorChannel {
public:
    MotorChannel(uint8_t in1_pin, uint8_t in2_pin,
                 uint8_t ledc_ch1, uint8_t ledc_ch2);

    void begin();
    void set(MotorDir dir, uint8_t duty);
    void stop();

private:
    uint8_t _in1, _in2;
    uint8_t _ch1, _ch2;

    void _setRaw(uint32_t duty1, uint32_t duty2);
};

// ─────────────────────────────────────────────────────────────────

class MotorDriver {
public:
    MotorDriver();
    void begin();

    /**
     * Apply a MotorState (with ramping) to finger [index].
     * Mutates state.current toward state.target.
     */
    void applyRamp(MotorState& state, int finger_index);
    void stopAll();
    void disableAll();   // hard off (safety)

private:
    MotorChannel _ch[NUM_FINGERS];
};
#endif