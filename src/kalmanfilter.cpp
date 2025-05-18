#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(float q, float r, float x0) {
    _q = q;
    _r = r;
    _p = 1.0f;
    _x = x0;
}

float KalmanFilter::update(float measurement) {
    // Prediction update
    _p += _q;
    // Measurement update
    float k = _p / (_p + _r);
    _x += k * (measurement - _x);
    _p *= (1.0f - k);
    return _x;
}

void KalmanFilter::reset(float value) {
    _x = value;
    _p = 1.0f;
}