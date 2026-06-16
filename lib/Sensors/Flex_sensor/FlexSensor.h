#ifndef FLEX_SENSOR_H
#define FLEX_SENSOR_H
#include <Arduino.h>
#include "../Filters/Filters.h"
#include "../include/SystemTypes.h"

class FlexSensor {
public:
    explicit FlexSensor(uint8_t adc_pin, float ema_alpha = 0.15f);

    void  begin();
    void  sample();                    // call every sensor period

    // Returns latest processed data
    FlexData  getData()  const { return _data; }
    FlexCalib getCalib() const { return _calib; }

    void  setCalib(const FlexCalib& c) { _calib = c; }

    // Raw ADC [0–4095]
    float rawADC() const { return _data.raw; }

private:
    uint8_t      _pin;
    EMAFilter    _ema;
    FlexData     _data;
    FlexCalib    _calib;

    float _computeNormalized(float filtered) const;
};

#endif