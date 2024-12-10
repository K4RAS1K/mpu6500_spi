#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Структура для хранения состояния фильтра Калмана
typedef struct {
    double x;  // Оценка состояния
    double P;  // Оценка ковариации
    double Q;  // Процессный шум
    double R;  // Шум измерения
    double K;  // Коэффициент Калмана
} KalmanFilter;

// Функция для инициализации фильтра Калмана
void kalman_init(KalmanFilter *kf, double initial_state, double process_noise, double measurement_noise) {
    kf->x = initial_state;
    kf->P = 1.0;  // Начальная оценка ковариации
    kf->Q = process_noise;
    kf->R = measurement_noise;
    kf->K = 0.0;
}

// Функция для обновления фильтра Калмана
double kalman_update(KalmanFilter *kf, double measurement) {
    // Прогноз
    kf->x = kf->x;  // Предсказание состояния (в данном случае не меняется)
    kf->P = kf->P + kf->Q;  // Обновление ковариации

    // Обновление
    kf->K = kf->P / (kf->P + kf->R);  // Расчет коэффициента Калмана
    kf->x = kf->x + kf->K * (measurement - kf->x);  // Обновление оценки состояния
    kf->P = (1 - kf->K) * kf->P;  // Обновление ковариации

    return kf->x;
}

int main() {
    // Пример массива с неотфильтрованными значениями
    double measurements[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    int num_measurements = sizeof(measurements) / sizeof(measurements[0]);

    // Инициализация фильтра Калмана
    KalmanFilter kf;
    kalman_init(&kf, measurements[0], 0.01, 0.1);  // Начальное состояние, шум процесса, шум измерения

    // Применение фильтра Калмана к измерениям
    double filtered_values[num_measurements];
    for (int i = 0; i < num_measurements; i++) {
        filtered_values[i] = kalman_update(&kf, measurements[i]);
    }

    // Вывод отфильтрованных значений
    printf("Filtered values:\n");
    for (int i = 0; i < num_measurements; i++) {
        printf("%.2f ", filtered_values[i]);
    }
    printf("\n");

    return 0;
}