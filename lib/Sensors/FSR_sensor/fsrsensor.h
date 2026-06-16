#ifndef FSR_SENSOR_H
#define FSR_SENSOR_H

#include <Arduino.h>
#include "../../Filters/Filters.h"
#include "../include/SystemTypes.h"

class FSRSensor {
public:
    explicit FSRSensor(uint8_t pin, float ema_alpha = 0.12f);
    void    begin();
    void    sample();
    FSRData getData() const { return _data; }

private:
    uint8_t   _pin;
    EMAFilter _ema;
    FSRData   _data;
};

#endif