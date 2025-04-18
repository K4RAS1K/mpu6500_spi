#include "MovingAverage.h"
#include <stdlib.h> 

void MovingAverage_Init(MovingAverage *ma, uint16_t window_size) {
    ma->window_size = window_size;
    ma->buffer = (float*)malloc(window_size * sizeof(float));
    ma->index = 0;
    ma->sum = 0.0f;
    ma->count = 0;

    for (uint16_t i = 0; i < window_size; i++) {
        ma->buffer[i] = 0.0f;
    }
}

float MovingAverage_Update(MovingAverage *ma, float new_value) {
    if (ma->count < ma->window_size) {
        ma->sum += new_value;
        ma->buffer[ma->index] = new_value;
        ma->index = (ma->index + 1) % ma->window_size;
        ma->count++;
        return ma->sum / ma->count;
    } else {
        ma->sum -= ma->buffer[ma->index];
        ma->sum += new_value;
        ma->buffer[ma->index] = new_value;
        ma->index = (ma->index + 1) % ma->window_size;
        return ma->sum / ma->window_size;
    }
}

void MovingAverage_Free(MovingAverage *ma) {
    if (ma->buffer != NULL) {
        free(ma->buffer);
        ma->buffer = NULL;
    }
}