#include <stdio.h>
#include "HighPassFilter.h"

void HighPassFilter_Init(HighPassFilter *filter, float alpha) {
    filter->alpha = alpha;
    filter->prev_x = 0.0f;
    filter->prev_y = 0.0f;
}

float HighPassFilter_Update(HighPassFilter *filter, float input) {
    float output = filter->alpha * (filter->prev_y + input - filter->prev_x);
    filter->prev_y = output;
    filter->prev_x = input;
    return output;
}
