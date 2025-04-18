#ifndef MAIN_HIGHPASSFILTER_HIGHPASSFILTER_H_
#define MAIN_HIGHPASSFILTER_HIGHPASSFILTER_H_

typedef struct {
    float alpha; 
    float prev_x;
    float prev_y;
} HighPassFilter;

void HighPassFilter_Init(HighPassFilter *filter, float alpha);

float HighPassFilter_Update(HighPassFilter *filter, float input);

#endif /* MAIN_HIGHPASSFILTER_HIGHPASSFILTER_H_ */
