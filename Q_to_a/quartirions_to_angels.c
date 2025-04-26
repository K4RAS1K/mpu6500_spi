#include "quartirions_to_angels.h"

struct Quaternion ToQuaternion(float roll, float pitch, float yaw) // roll (x), pitch (y), yaw (z), angles are in radians
{
    // Abbreviations for the various angular functions

    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    struct Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
void ToEulerAngles(struct Quaternion q, struct EulerAngles *angles) 
{

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles->roll = atan2(sinr_cosp, cosr_cosp) * R2D;

    // pitch (y-axis rotation)
    float sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    float cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles->pitch = (2 * atan2(sinp, cosp) - M_PI / 2) * R2D;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles->yaw = atan2(siny_cosp, cosy_cosp) * R2D;
}

void computeAngles(struct Quaternion q, struct EulerAngles *angles)
{
    float q0 = q.w;
    float q1 = q.x;
    float q2 = q.y;
    float q3 = q.z;
	angles->roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * R2D;
	angles->pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * R2D;
	if (fabs(-2.0f * (q1*q3 - q0*q2)) >= 1)
		angles->pitch = copysign(M_PI / 2, -2.0f * (q1*q3 - q0*q2)) * R2D; // use 90 degrees if out of range
	angles->yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * R2D;
}