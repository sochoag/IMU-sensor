#ifndef KalmanFilter_h
#define KalmanFilter_h

class KalmanFilter
{

public:
    KalmanFilter(float mea_e, float est_e, float q);
    float updateEstimate(float mea);
    void setMeasurementError(float mea_e);
    void setEstimateError(float est_e);
    void setProcessNoise(float q);
    float getKalmanGain();
    float getEstimateError();

private:
    float err_measure;
    float err_estimate;
    float q;
    float current_estimate = 0;
    float last_estimate = 0;
    float kalman_gain = 0;
};

#endif