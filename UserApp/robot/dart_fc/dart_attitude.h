/* dart_attitude.h — 姿态互补滤波 (移植自 dart_fc/App/attitude.h)
 * 类型改名为 dart_attitude_t 以避免与 control-2026 的 attitude_t 冲突。 */
#ifndef DART_ATTITUDE_H
#define DART_ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gx_dps;   /* 滤波后角速率 */
    float gy_dps;
    float gz_dps;
    int   valid;
} dart_attitude_t;

void dart_attitude_init(dart_attitude_t *a);
/* gyro: deg/s; acc: g; dt: s */
void dart_attitude_update(dart_attitude_t *a, const float gyro[3], const float acc[3], float dt);

#ifdef __cplusplus
}
#endif

#endif /* DART_ATTITUDE_H */
