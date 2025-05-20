#include "kalmanfilter.h"
#include <cmath>  // C++ version of math.h

KalmanFilter::KalmanFilter(float mea_e, float est_e, float q, float initial_value) {
    _err_measure = mea_e;
    _err_estimate = est_e;
    _q = q;
    _current_estimate = initial_value;
    _last_estimate = initial_value;
}

float KalmanFilter::updateEstimate(float measurement) {
    _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
    _current_estimate = _last_estimate + _kalman_gain * (measurement - _last_estimate);
    _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabsf(_last_estimate - _current_estimate) * _q;
    _last_estimate = _current_estimate;
    
    return _current_estimate;
}

void KalmanFilter::setMeasurementError(float mea_e) {
    _err_measure = mea_e;
}

void KalmanFilter::setEstimationError(float est_e) {
    _err_estimate = est_e;
}

void KalmanFilter::setProcessNoise(float q) {
    _q = q;
}

float KalmanFilter::getLatestEstimate() const {
    return _current_estimate;
}