/* dart_guidance.h — 比例导引 (移植自 dart_fc/App/guidance.h) */
#ifndef DART_GUIDANCE_H
#define DART_GUIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float lambda_x;
    float lambda_y;
    float lambda_dot_x;
    float lambda_dot_y;
    float accel_x;       /* PNG 输出横向加速度指令 (m/s^2) */
    float accel_y;
    float prev_lambda_x;
    float prev_lambda_y;
    int   started;
    int   valid;
} guidance_t;

void guidance_init(guidance_t *g);
void guidance_reset(guidance_t *g);
void guidance_update_pixel(guidance_t *g, float px, float py,
                           float cx, float cy, float focal_px,
                           float roll_deg, float sign);
void guidance_derotate(float *x, float *y, float cx, float cy,
                       float roll_deg, float sign);
void guidance_rotate_to_body(float in_x, float in_y, float roll_deg, float sign,
                             float *out_x, float *out_y);
void guidance_update_rate(guidance_t *g, float dt, float alpha);
void guidance_png(guidance_t *g, float N, float Vc, float sign, float max_g);

#ifdef __cplusplus
}
#endif

#endif /* DART_GUIDANCE_H */
