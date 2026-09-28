#include "dart_guidance.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GRAVITY 9.80665f
#define DEG2RAD ((float)M_PI / 180.0f)

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void guidance_derotate(float *x, float *y, float cx, float cy,
                       float roll_deg, float sign)
{
    float dx = *x - cx;
    float dy = *y - cy;
    float a = sign * roll_deg * DEG2RAD;
    float c = cosf(a);
    float s = sinf(a);

    *x = cx + (dx * c - dy * s);
    *y = cy + (dx * s + dy * c);
}

void guidance_rotate_to_body(float in_x, float in_y, float roll_deg, float sign,
                             float *out_x, float *out_y)
{
    float a = -sign * roll_deg * DEG2RAD;
    float c = cosf(a);
    float s = sinf(a);

    *out_x = in_x * c - in_y * s;
    *out_y = in_x * s + in_y * c;
}

void guidance_init(guidance_t *g)
{
    guidance_reset(g);
}

void guidance_reset(guidance_t *g)
{
    g->lambda_x = 0.0f;
    g->lambda_y = 0.0f;
    g->lambda_dot_x = 0.0f;
    g->lambda_dot_y = 0.0f;
    g->accel_x = 0.0f;
    g->accel_y = 0.0f;
    g->prev_lambda_x = 0.0f;
    g->prev_lambda_y = 0.0f;
    g->started = 0;
    g->valid = 0;
}

void guidance_update_pixel(guidance_t *g, float px, float py,
                           float cx, float cy, float focal_px,
                           float roll_deg, float sign)
{
    if (focal_px < 1e-3f) {
        g->valid = 0;
        return;
    }
    guidance_derotate(&px, &py, cx, cy, roll_deg, sign);
    g->lambda_x = (px - cx) / focal_px;
    g->lambda_y = (py - cy) / focal_px;
    g->valid = 1;
}

void guidance_update_rate(guidance_t *g, float dt, float alpha)
{
    float ddx;
    float ddy;

    if (dt <= 0.0f || !g->valid) {
        return;
    }
    if (!g->started) {
        g->prev_lambda_x = g->lambda_x;
        g->prev_lambda_y = g->lambda_y;
        g->started = 1;
        return;
    }

    ddx = (g->lambda_x - g->prev_lambda_x) / dt;
    ddy = (g->lambda_y - g->prev_lambda_y) / dt;
    g->prev_lambda_x = g->lambda_x;
    g->prev_lambda_y = g->lambda_y;

    g->lambda_dot_x += alpha * (ddx - g->lambda_dot_x);
    g->lambda_dot_y += alpha * (ddy - g->lambda_dot_y);
}

void guidance_png(guidance_t *g, float N, float Vc, float sign, float max_g)
{
    float ax = sign * N * Vc * g->lambda_dot_x;
    float ay = sign * N * Vc * g->lambda_dot_y;
    float lim = max_g * GRAVITY;

    g->accel_x = clampf(ax, -lim, lim);
    g->accel_y = clampf(ay, -lim, lim);
}
