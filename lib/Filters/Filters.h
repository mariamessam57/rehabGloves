#ifndef _FILTERS_H_
#define _FILTERS_H_



class EMAFilter {
public:
    explicit EMAFilter(float alpha = 0.15f);
    void  setAlpha(float alpha);
    float update(float measurement);
    void  reset(float value = 0.0f);
    float value() const { return _state; }

private:
    float _alpha;
    float _state;
    bool  _initialized;
};

// ─────────────────────────────────────────────────────────────────

class KalmanFilter1D {
public:
    KalmanFilter1D(float q = 0.001f, float r = 0.03f);
    void  init(float initial_value);
    float update(float measurement);
    float value() const { return _x; }

private:
    float _q;   // process noise
    float _r;   // measurement noise
    float _x;   // state estimate
    float _p;   // error covariance
};

// ─────────────────────────────────────────────────────────────────

namespace SigmaFilter {
    /**
     * Returns mean of samples after removing outliers
     * beyond sigma_thresh standard deviations.
     */
    float compute(float* buf, int len, float sigma_thresh = 2.0f);
}
#endif