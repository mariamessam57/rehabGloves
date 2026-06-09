#include "MotorDriver.h"
#include "driver/ledc.h"

// ── LEDC channel assignments ─────────────────────────────────────
// Finger 0: LEDC ch0 (IN1), ch1 (IN2)
// Finger 1: LEDC ch2 (IN1), ch3 (IN2)
// Finger 2: LEDC ch4 (IN1), ch5 (IN2)

static const uint8_t LEDC_CH_IN1[3] = { 0, 2, 4 };
static const uint8_t LEDC_CH_IN2[3] = { 1, 3, 5 };

// ================================================================
//  MotorChannel
// ================================================================
MotorChannel::MotorChannel(uint8_t in1_pin, uint8_t in2_pin,
                           uint8_t ledc_ch1, uint8_t ledc_ch2)
    : _in1(in1_pin), _in2(in2_pin), _ch1(ledc_ch1), _ch2(ledc_ch2) {}

void MotorChannel::begin() {
    // LEDC timer is configured once in MotorDriver::begin()
    ledcAttachPin(_in1, _ch1);
    ledcAttachPin(_in2, _ch2);
    stop();
}

void MotorChannel::set(MotorDir dir, uint8_t duty) {
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

    switch (dir) {
        case MotorDir::FORWARD:  _setRaw(duty, 0);    break;
        case MotorDir::REVERSE:  _setRaw(0,    duty); break;
        case MotorDir::STOP:
        default:                 _setRaw(0, 0);       break;
    }
}

void MotorChannel::stop() {
    _setRaw(0, 0);
}

void MotorChannel::_setRaw(uint32_t duty1, uint32_t duty2) {
    // Safety: never allow both high at the same time
    if (duty1 > 0 && duty2 > 0) { duty2 = 0; }
    ledcWrite(_ch1, duty1);
    ledcWrite(_ch2, duty2);
}

// ================================================================
//  MotorDriver
// ================================================================
MotorDriver::MotorDriver()
    : _ch {
        MotorChannel(PIN_M0_IN1, PIN_M0_IN2, LEDC_CH_IN1[0], LEDC_CH_IN2[0]),
        MotorChannel(PIN_M1_IN1, PIN_M1_IN2, LEDC_CH_IN1[1], LEDC_CH_IN2[1]),
        MotorChannel(PIN_M2_IN1, PIN_M2_IN2, LEDC_CH_IN1[2], LEDC_CH_IN2[2])
    }
{}

void MotorDriver::begin() {
    static bool ledc_initialized = false;

    if (!ledc_initialized) {
        // Configure one shared LEDC timer for all 6 channels
        ledc_timer_config_t timer = {
            .speed_mode      = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION_BITS,
            .timer_num       = LEDC_TIMER_0,
            .freq_hz         = PWM_FREQ_HZ,
            .clk_cfg         = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timer);
        ledc_initialized = true;
    }

    for (int i = 0; i < NUM_FINGERS; i++) {
        _ch[i].begin();
    }
}

void MotorDriver::applyRamp(MotorState& state, int finger_index) {
    if (finger_index < 0 || finger_index >= NUM_FINGERS) return;

    if (!state.enabled) {
        state.current = 0;
        _ch[finger_index].stop();
        return;
    }

    // Ramp current toward target
    if (state.current < state.target) {
        uint8_t next = state.current + PWM_RAMP_STEP;
        state.current = (next > state.target) ? state.target : next;
    } else if (state.current > state.target) {
        if (state.current <= PWM_RAMP_STEP) state.current = 0;
        else state.current -= PWM_RAMP_STEP;
    }

    _ch[finger_index].set(state.dir, state.current);
}

void MotorDriver::stopAll() {
    for (int i = 0; i < NUM_FINGERS; i++) _ch[i].stop();
}

void MotorDriver::disableAll() {
    stopAll();
}