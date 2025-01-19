#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdio.h>

// Определение структуры KalmanFilter
typedef struct {
    float q;
    float r;
    float x;
    float p;
    float k;
} KalmanFilter;

// Объявления функций
void KalmanFilter_Init(KalmanFilter *kf, float q, float r, float p, float initial_value);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);
void FilterSensorData(const float accel[3], const float gyro[3], float filtered_accel[3], float filtered_gyro[3]);

#endif // KALMAN_FILTER_H