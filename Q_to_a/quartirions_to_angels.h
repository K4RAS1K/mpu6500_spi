#ifndef QUARTIRIONS_TO_ANGELS_H_
#define QUARTIRIONS_TO_ANGELS_H_
#include <stdio.h>
#include <math.h>

#define PI M_PI
#define R2D 180.00f / PI
#define D2R PI / 180.00f

struct Quaternion {
	float w;
	float x;
	float y;
	float z;
};

struct EulerAngles {
    float roll, pitch, yaw;
};

void computeAngles(struct Quaternion q, struct EulerAngles *angles);

#endif /* MAIN_HIGHPASSFILTER_HIGHPASSFILTER_H_ */
