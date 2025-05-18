#pragma once

class KalmanFilter {
public:
    KalmanFilter(float q, float r, float x0 = 0.0f);
    float update(float measurement);
    void reset(float value = 0.0f);
private:

    float _x, _p, _q, _r;
};
