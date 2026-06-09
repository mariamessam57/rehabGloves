#include "Filters.h"
#include <math.h>
#include <string.h>


EMAFilter::EMAFilter(float alpha)
    : _alpha(alpha), _state(0.0f), _initialized(false) {}

void EMAFilter::setAlpha(float alpha) { _alpha = alpha; }

void EMAFilter::reset(float value) {
    _state       = value;
    _initialized = true;
}

float EMAFilter::update(float measurement) {
    if (!_initialized) {
        _state       = measurement;
        _initialized = true;
    } else {
        _state = _alpha * measurement + (1.0f - _alpha) * _state;
    }
    return _state;
}

// ================================================================
//  KALMAN 1-D
// ================================================================
KalmanFilter1D::KalmanFilter1D(float q, float r)
    : _q(q), _r(r), _x(0.0f), _p(1.0f) {}

void KalmanFilter1D::init(float initial_value) {
    _x = initial_value;
    _p = 1.0f;
}

float KalmanFilter1D::update(float measurement) {
    // Predict
    _p += _q;
    // Kalman gain
    float K = _p / (_p + _r);
    // Update
    _x = _x + K * (measurement - _x);
    _p = (1.0f - K) * _p;
    return _x;
}

// ================================================================
//  SIGMA FILTER
// ================================================================
float SigmaFilter::compute(float* buf, int len, float sigma_thresh) {
    if (len <= 0) return 0.0f;

    // Mean
    float sum = 0.0f;
    for (int i = 0; i < len; i++) sum += buf[i];
    float mean = sum / (float)len;

    // Std dev
    float var = 0.0f;
    for (int i = 0; i < len; i++) {
        float d = buf[i] - mean;
        var += d * d;
    }
    float std = sqrtf(var / (float)len);

    // Re-average without outliers
    float clean_sum = 0.0f;
    int   clean_cnt = 0;
    for (int i = 0; i < len; i++) {
        if (fabsf(buf[i] - mean) <= sigma_thresh * std) {
            clean_sum += buf[i];
            clean_cnt++;
        }
    }
    return (clean_cnt > 0) ? (clean_sum / (float)clean_cnt) : mean;
}