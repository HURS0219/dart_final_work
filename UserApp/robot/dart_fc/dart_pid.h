/* dart_pid.h — 通用单轴 PID (移植自 dart_fc/App/pid.h) */
#ifndef DART_PID_H
#define DART_PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_limit;
    float out_limit;
    float integ;
    float prev_meas;
    int   started;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd, float i_limit, float out_limit);
void  pid_reset(pid_t *p);
float pid_update(pid_t *p, float setpoint, float meas, float dt);

#ifdef __cplusplus
}
#endif

#endif /* DART_PID_H */
