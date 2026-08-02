#include "MotorDriver.h"
#include "driver/ledc.h"

static const uint8_t LEDC_CH_IN1[3] = {0, 2, 4};
static const uint8_t LEDC_CH_IN2[3] = {1, 3, 5};

MotorChannel::MotorChannel(uint8_t in1_pin, uint8_t in2_pin,
                           uint8_t ledc_ch1, uint8_t ledc_ch2)
    : _in1(in1_pin), _in2(in2_pin), _ch1(ledc_ch1), _ch2(ledc_ch2) {}

// ================================================================
void MotorChannel::begin()
{

    // IMPORTANT: attach pins to PWM channels
    ledcAttachPin(_in1, _ch1);
    ledcAttachPin(_in2, _ch2);

    stop();
}

// ================================================================
void MotorChannel::set(MotorDir dir, uint8_t duty)
{

    if (duty > PWM_DUTY_MAX)
        duty = PWM_DUTY_MAX;

    switch (dir)
    {

    case MotorDir::FORWARD:
        _setRaw(duty, 0);
        break;

    case MotorDir::REVERSE:
        _setRaw(0, duty);
        break;

    case MotorDir::STOP:
    default:
        _setRaw(0, 0);
        break;
    }
}

// ================================================================
void MotorChannel::stop()
{
    _setRaw(0, 0);
}

// ================================================================
void MotorChannel::_setRaw(uint32_t duty1, uint32_t duty2)
{

    if (duty1 > 0 && duty2 > 0)
        duty2 = 0;

#ifdef MOTOR_DEBUG
    Serial.printf("[MOTOR_DBG] pin1=%u ch=%u d1=%u  pin2=%u ch=%u d2=%u\n",
                  _in1, _ch1, (uint32_t)duty1, _in2, _ch2, (uint32_t)duty2);
#endif

    ledcWrite(_ch1, duty1);
    ledcWrite(_ch2, duty2);
}

// ================================================================
MotorDriver::MotorDriver()
    : _ch{
          MotorChannel(PIN_M0_IN1, PIN_M0_IN2, LEDC_CH_IN1[0], LEDC_CH_IN2[0]),
          MotorChannel(PIN_M1_IN1, PIN_M1_IN2, LEDC_CH_IN1[1], LEDC_CH_IN2[1]),
          MotorChannel(PIN_M2_IN1, PIN_M2_IN2, LEDC_CH_IN1[2], LEDC_CH_IN2[2])}
{
}

// ================================================================
void MotorDriver::begin()
{

    // FIX: REQUIRED in ESP32 Core 2.x for stable multi-channel PWM
    for (int i = 0; i < NUM_FINGERS; i++)
    {

        ledcSetup(LEDC_CH_IN1[i], PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
        ledcSetup(LEDC_CH_IN2[i], PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    }

    for (int i = 0; i < NUM_FINGERS; i++)
    {
        _ch[i].begin();
    }
}

// ================================================================
void MotorDriver::applyRamp(MotorState &state, int finger_index)
{

    if (finger_index < 0 || finger_index >= NUM_FINGERS)
        return;

    if (!state.enabled)
    {
        state.current = 0;
        state.current_pwm = 0;
        _ch[finger_index].stop();
        return;
    }

    static uint8_t last_target[NUM_FINGERS] = {0};

    // Always prefer the new PWM fields (target_pwm is explicitly set by all modes)
    uint8_t target = state.target_pwm;

    // smooth limiting
    if (target > last_target[finger_index])
    {
        if (target - last_target[finger_index] > 25)
            target = last_target[finger_index] + 25;
    }
    else
    {
        if (last_target[finger_index] - target > 25)
            target = last_target[finger_index] - 25;
    }

    last_target[finger_index] = target;
    state.target = target;

    // ramp logic
    if (state.current < state.target)
    {
        uint8_t next = state.current + PWM_RAMP_STEP;
        state.current = (next > state.target) ? state.target : next;
    }
    else if (state.current > state.target)
    {
        state.current = (state.current <= PWM_RAMP_STEP)
                            ? 0
                            : state.current - PWM_RAMP_STEP;
    }

    // keep both legacy and new fields in sync
    state.current_pwm = state.current;
    state.current = state.current_pwm;

    _ch[finger_index].set(state.dir, state.current_pwm);
}

// ================================================================
void MotorDriver::stopAll()
{
    for (int i = 0; i < NUM_FINGERS; i++)
        _ch[i].stop();
}

// ================================================================
void MotorDriver::disableAll()
{
    stopAll();
}