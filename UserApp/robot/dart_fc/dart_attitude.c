#include "dart_attitude.h"
#include "dart_config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG (180.0f / (float)M_PI)

static float lpf(float prev, float x, float alpha)
{
    return prev + alpha * (x - prev);
}

void dart_attitude_init(dart_attitude_t *a)
{
    a->roll_deg = 0.0f;
    a->pitch_deg = 0.0f;
    a->yaw_deg = 0.0f;
    a->gx_dps = 0.0f;
    a->gy_dps = 0.0f;
    a->gz_dps = 0.0f;
    a->valid = 0;
}

void dart_attitude_update(dart_attitude_t *a, const float gyro[3], const float acc[3], float dt)
{
    float ax = acc[0];
    float ay = acc[1];
    float az = acc[2];
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    float roll_acc_deg;
    float pitch_acc_deg;
    int have_acc;

    a->gx_dps = lpf(a->gx_dps, gyro[0], CFG_GYRO_LPF_ALPHA);
    a->gy_dps = lpf(a->gy_dps, gyro[1], CFG_GYRO_LPF_ALPHA);
    a->gz_dps = lpf(a->gz_dps, gyro[2], CFG_GYRO_LPF_ALPHA);

    a->roll_deg  += a->gx_dps * dt;
    a->pitch_deg += a->gy_dps * dt;
    a->yaw_deg   += a->gz_dps * dt;

    have_acc = (norm > 0.7f && norm < 1.3f);
    if (have_acc) {
        ax /= norm;
        ay /= norm;
        az /= norm;

        roll_acc_deg  = atan2f(ay, az) * RAD2DEG;
        pitch_acc_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;

        a->roll_deg  = CFG_ATT_ALPHA * a->roll_deg
                     + (1.0f - CFG_ATT_ALPHA) * roll_acc_deg;
        a->pitch_deg = CFG_ATT_ALPHA * a->pitch_deg
                     + (1.0f - CFG_ATT_ALPHA) * pitch_acc_deg;
        a->valid = 1;
    }

    if (a->yaw_deg > 180.0f)  a->yaw_deg -= 360.0f;
    if (a->yaw_deg < -180.0f) a->yaw_deg += 360.0f;
}
