/* dart_control.h — 三模式控制调度 (移植自 dart_fc/App/control.h) */
#ifndef DART_CONTROL_H
#define DART_CONTROL_H

#include "dart_mode.h"
#include "dart_attitude.h"
#include "dart_guidance.h"
#include "servo_mixer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int             mode;
    int             armed;
    int             link_ok;
    int             vision_ok;
    float           sticks[4];   /* [thr, pitch, yaw, roll] -1..1 */
    dart_attitude_t att;
    guidance_t      guid;
    float           dt;
} control_in_t;

typedef struct {
    float     servo_deg[4];
    mix_cmd_t mix;
    int       failsafe;
} control_out_t;

void control_init(void);
void control_step(const control_in_t *in, control_out_t *out);
float control_stick_shape(int raw, int deadband, float expo);

#ifdef __cplusplus
}
#endif

#endif /* DART_CONTROL_H */
