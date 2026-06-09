#include "FlexSensor.h"

FlexSensor::FlexSensor(uint8_t adc_pin, float ema_alpha)
    : _pin(adc_pin), _ema(ema_alpha) {}

void FlexSensor::begin() {
    pinMode(_pin, INPUT);
    _ema.reset((float)analogRead(_pin));
}

void FlexSensor::sample() {
    uint32_t now = millis();

    float raw      = (float)analogRead(_pin);
    float filtered = _ema.update(raw);

    // Velocity
    float dt = (_data.last_ms > 0)
        ? (float)(now - _data.last_ms) / 1000.0f
        : 0.02f;
    float norm = _computeNormalized(filtered);

    if (dt > 0.001f) {
        _data.velocity = (norm - _data.normalized) / dt;
    }

    _data.raw        = raw;
    _data.filtered   = filtered;
    _data.normalized = norm;
    _data.last_ms    = now;
}

float FlexSensor::_computeNormalized(float filtered) const {
    float range = _calib.max_raw - _calib.min_raw;
    if (range < 1.0f) return 0.0f;

    float n = (filtered - _calib.min_raw) / range;
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return n;
}