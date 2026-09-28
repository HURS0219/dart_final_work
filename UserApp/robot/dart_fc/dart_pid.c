#include "dart_pid.h"

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void pid_init(pid_t *p, float kp, float ki, float kd, float i_limit, float out_limit)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_limit = i_limit;
    p->out_limit = out_limit;
    p->integ = 0.0f;
    p->prev_meas = 0.0f;
    p->started = 0;
}

void pid_reset(pid_t *p)
{
    p->integ = 0.0f;
    p->prev_meas = 0.0f;
    p->started = 0;
}

float pid_update(pid_t *p, float setpoint, float meas, float dt)
{
    float err = setpoint - meas;
    float deriv;
    float out;

    if (dt <= 0.0f) {
        dt = 1e-3f;
    }

    p->integ += p->ki * err * dt;
    p->integ = clampf(p->integ, -p->i_limit, p->i_limit);

    if (!p->started) {
        p->prev_meas = meas;
        p->started = 1;
    }
    deriv = (meas - p->prev_meas) / dt;
    p->prev_meas = meas;

    out = p->kp * err + p->integ - p->kd * deriv;
    return clampf(out, -p->out_limit, p->out_limit);
}
