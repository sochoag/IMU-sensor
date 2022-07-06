#include "Arduino.h"
#include "KalmanFilter.h"
#include <math.h>

KalmanFilter::KalmanFilter(float mea_e, float est_e, float q)
{
    this->err_measure = mea_e;
    this->err_estimate = est_e;
    this->q = q;
}

float KalmanFilter::updateEstimate(float mea)
{
    this->kalman_gain = this->err_estimate / (this->err_estimate + this->err_measure);
    this->current_estimate = this->last_estimate + this->kalman_gain * (mea - this->last_estimate);
    this->err_estimate = (1.0 - this->kalman_gain) * this->err_estimate + fabs(this->last_estimate - this->current_estimate) * this->q;
    this->last_estimate = this->current_estimate;
    
    return current_estimate;
}

void KalmanFilter::setMeasurementError(float mea_e)
{
    this->err_measure = mea_e;
}

void KalmanFilter::setEstimateError(float est_e)
{
    this->err_estimate = est_e;
}

void KalmanFilter::setProcessNoise(float q)
{
    this->q = q;
}

float KalmanFilter::getKalmanGain()
{
    return this->kalman_gain;
}

float KalmanFilter::getEstimateError()
{
    return this->err_estimate;
}