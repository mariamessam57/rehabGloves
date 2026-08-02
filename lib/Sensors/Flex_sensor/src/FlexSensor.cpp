#include "FlexSensor.h"
#include <string.h>

FlexSensor::FlexSensor(uint8_t pin, float alpha)
    : _pin(pin), _alpha(alpha)
{
}

void FlexSensor::begin()
{
    pinMode(_pin, INPUT);

    memset(_samples, 0, sizeof(_samples));

    _idx = 0;
    _count = 0;

    _ema = 0;
    _lastValue = 0;
}

float FlexSensor::readRaw()
{
    return analogRead(_pin);
}

float FlexSensor::movingAverage(float v)
{
    _samples[_idx] = v;
    _idx = (_idx + 1) % WINDOW;

    if (_count < WINDOW)
        _count++;

    float sum = 0;

    for (int i = 0; i < _count; i++)
        sum += _samples[i];

    return sum / _count;
}

float FlexSensor::ema(float v)
{
    if (_ema == 0)
        _ema = v;
    else
        _ema = _alpha * v + (1.0f - _alpha) * _ema;

    return _ema;
}

float FlexSensor::readFiltered()
{
    float raw = readRaw();
    float ma = movingAverage(raw);
    float filtered = ema(ma);

    _lastValue = filtered;
    return filtered;
}

float FlexSensor::getAngle() const
{
    if (!_calib.valid)
        return 0;

    float range = _calib.max_raw - _calib.min_raw;

    if (range < 1)
        return 0;

    float n = (_lastValue - _calib.min_raw) / range;

    n = constrain(n, 0.0f, 1.0f);

    return n * 90.0f;
}

// ================================================================
// BACKWARD COMPATIBILITY
// ================================================================
void FlexSensor::setCalib(const FlexCalib& c)
{
    _calib = c;
}

FlexCalib FlexSensor::getCalib() const
{
    return _calib;
}

void FlexSensor::sample()
{
    readFiltered();
}

FlexData FlexSensor::getData() const
{
    FlexData d;

    d.raw = _lastValue;
    d.filtered = _lastValue;

    if (_calib.valid) {
        float range = _calib.max_raw - _calib.min_raw;

        if (range > 0.001f) {
            float n = (_lastValue - _calib.min_raw) / range;
            n = constrain(n, 0.0f, 1.0f);

            d.normalized = n;
            d.angle = n * 90.0f;
        }
    }

    return d;
}