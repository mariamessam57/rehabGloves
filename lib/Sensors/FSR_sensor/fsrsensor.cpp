#include "FSRSensor.h"

FSRSensor::FSRSensor(uint8_t pin, float ema_alpha)
    : _pin(pin), _ema(ema_alpha) {}

void FSRSensor::begin() {
    pinMode(_pin, INPUT);
    _ema.reset((float)analogRead(_pin));
}

void FSRSensor::sample() {
    float raw      = (float)analogRead(_pin);
    _data.raw      = raw;
    _data.filtered = _ema.update(raw);
    _data.normalized = constrain(_data.filtered / 4095.0f, 0.0f, 1.0f);
}