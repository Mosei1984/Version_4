#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
public:
    KalmanFilter(float mea_e, float est_e, float q, float initial_value = 0.0f);
    float updateEstimate(float measurement);
    void setMeasurementError(float mea_e);
    void setEstimationError(float est_e);
    void setProcessNoise(float q);
    float getLatestEstimate() const;

private:
    float _err_measure;       // Measurement error
    float _err_estimate;      // Estimation error
    float _q;                 // Process noise
    float _current_estimate;  // Current estimate
    float _last_estimate;     // Last estimate
    float _kalman_gain;       // Kalman gain
};

#endif // KALMANFILTER_H