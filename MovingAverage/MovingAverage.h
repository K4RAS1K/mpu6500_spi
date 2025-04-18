#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>

typedef struct {
    float *buffer;      
    uint16_t window_size; 
    uint16_t index;   
    float sum;        
    uint16_t count;    
} MovingAverage;

void MovingAverage_Init(MovingAverage *ma, uint16_t window_size);

float MovingAverage_Update(MovingAverage *ma, float new_value);

void MovingAverage_Free(MovingAverage *ma);

#endif // MOVING_AVERAGE_H