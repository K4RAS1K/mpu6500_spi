#include "kalman_filter.h"

// Инициализация фильтра Калмана
void KalmanFilter_Init(KalmanFilter *kf, float q, float r, float p, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->p = p;
    kf->x = initial_value;
}

// Обновление фильтра Калмана
float KalmanFilter_Update(KalmanFilter *kf, float measurement) {
    // Предсказание
    kf->p = kf->p + kf->q;

    // Обновление
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;

    return kf->x;
}

// Функция для фильтрации данных акселерометра и гироскопа
void FilterSensorData(
    const float accel[3], // Сырые данные акселерометра (x, y, z)
    const float gyro[3],  // Сырые данные гироскопа (x, y, z)
    float filtered_accel[3], // Отфильтрованные данные акселерометра (x, y, z)
    float filtered_gyro[3]   // Отфильтрованные данные гироскопа (x, y, z)
) {
    // Инициализация фильтров Калмана для акселерометра
    static KalmanFilter kf_accel[3] = {0};

    // Инициализация фильтров Калмана для гироскопа
    static KalmanFilter kf_gyro[3] = {0};

    // Инициализация фильтров (выполняется только один раз)
    static int is_initialized = 0;
    if (!is_initialized) {
        for (int i = 0; i < 3; i++) {
            KalmanFilter_Init(&kf_accel[i], 0.1, 0.1, 1.0, accel[i]);
            KalmanFilter_Init(&kf_gyro[i], 0.1, 0.1, 1.0, gyro[i]);
        }
        is_initialized = 1;
    }

    // Обновление фильтров Калмана для акселерометра
    for (int i = 0; i < 3; i++) {
        filtered_accel[i] = KalmanFilter_Update(&kf_accel[i], accel[i]);
    }

    // Обновление фильтров Калмана для гироскопа
    for (int i = 0; i < 3; i++) {
        filtered_gyro[i] = KalmanFilter_Update(&kf_gyro[i], gyro[i]);
    }
}