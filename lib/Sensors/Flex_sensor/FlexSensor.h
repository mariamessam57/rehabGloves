#ifndef FLEX_SENSOR_H
#define FLEX_SENSOR_H

#include <Arduino.h>
#include "SystemTypes.h"

class FlexSensor {
public:
    FlexSensor(uint8_t pin, float alpha = 0.15f);

    void begin();

    float readRaw();
    float readFiltered();

    float getAngle() const;

    // ================= BACKWARD COMPATIBILITY =================
    void setCalib(const FlexCalib& c);
    FlexCalib getCalib() const;

    void sample();
    FlexData getData() const;

private:
    uint8_t _pin;

    static constexpr uint8_t WINDOW = 10;
    float _samples[WINDOW];
    uint8_t _idx = 0;
    uint8_t _count = 0;

    float _ema = 0;
    float _alpha;

    FlexCalib _calib;

    float _lastValue = 0;

    float movingAverage(float v);
    float ema(float v);
};

#endif