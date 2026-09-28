/*
 * dart_app.c — 制导飞镖飞控 (dart_fc) 应用层
 *
 * 分层:
 *   Module 层: IMU(ins_task) / 舵机解耦(servo_mixer)
 *   App 层(本目录):
 *     dart_ptk7350  自研 PTK7350 底层 PWM 驱动 (隔离, 不改 Modules)
 *     dart_cfg      运行时参数 + 掉电保存
 *     dart_surface  4 路 X 型舵面 状态机
 *     dart_link     ESP32 网页调参串口协议
 *     dart_control / dart_guidance / dart_attitude / dart_mode / dart_pid
 *
 * RobotTask 以 ~1kHz 调用 DartAppTask(内部再分频到 100Hz 控制) 与显示任务。
 */

#include "dart_app.h"

#include <math.h>

#include "bsp_dwt.h"
#include "ins_task.h"
#include "main.h"

#include "dart_cfg.h"
#include "dart_config.h"
#include "dart_link.h"
#include "dart_mode.h"
#include "dart_surface.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD2DEG (180.0f / (float)M_PI)

#define DART_CTRL_DIV   5u   /* 调用 500Hz -> 控制 100Hz */

#ifndef CFG_SURFACE_DEFAULT_STATE
#define CFG_SURFACE_DEFAULT_STATE SURFACE_NEUTRAL
#endif

static dart_state_t s_state;
static guidance_t   s_guid;
static uint8_t      s_inited;
static uint32_t     s_last_us;
static uint32_t     s_last_ctrl_us;
static uint32_t     s_ctrl_div;
static float        s_t;

/* 注入视觉 */
static int      s_vis_x, s_vis_y, s_vis_w, s_vis_h;
static uint32_t s_vis_tick;

void DartAppInjectVision(int x, int y, int w, int h){
    s_vis_x = x;
    s_vis_y = y;
    s_vis_w = w;
    s_vis_h = h;
    s_vis_tick = HAL_GetTick();
}

/* ---------- IMU: 优先用板载 INS, 不可用则仿真 ---------- */
static void dart_read_imu(dart_attitude_t *a, float dt)
{
    attitude_t ins;

    if (INS_GetAttitude(&ins)) {
        a->roll_deg  = ins.Roll;
        a->pitch_deg = ins.Pitch;
        a->yaw_deg   = ins.Yaw;
        a->gx_dps = ins.Gyro[1] * RAD2DEG;   /* Roll 轴 */
        a->gy_dps = ins.Gyro[0] * RAD2DEG;   /* Pitch 轴 */
        a->gz_dps = ins.Gyro[2] * RAD2DEG;   /* Yaw 轴 */
        a->valid = 1;

        s_state.accel_g[0] = ins.Accel[0] / 9.80665f;
        s_state.accel_g[1] = ins.Accel[1] / 9.80665f;
        s_state.accel_g[2] = ins.Accel[2] / 9.80665f;
    } else {
        (void)dt;
        a->roll_deg  = 25.0f * sinf(s_t * 0.8f);
        a->pitch_deg = 15.0f * sinf(s_t * 1.1f + 0.6f);
        a->yaw_deg   = 40.0f * sinf(s_t * 0.5f);
        a->gx_dps = 25.0f * 0.8f * cosf(s_t * 0.8f);
        a->gy_dps = 15.0f * 1.1f * cosf(s_t * 1.1f + 0.6f);
        a->gz_dps = 40.0f * 0.5f * cosf(s_t * 0.5f);
        a->valid = 0;
        s_state.accel_g[0] = 0.0f;
        s_state.accel_g[1] = 0.0f;
        s_state.accel_g[2] = 1.0f;
    }
}

/* ---------- 视觉: 真图像流注入优先, 否则演示用移动目标 ---------- */
static void dart_update_vision(guidance_t *g)
{
    float px, py, w;

    if (s_vis_tick != 0 && (HAL_GetTick() - s_vis_tick) < CFG_VISION_TIMEOUT_MS) {
        px = (float)s_vis_x;
        py = (float)s_vis_y;
        w  = (float)s_vis_w;
        s_state.vision_online = 1;
    } else {
        px = CFG_IMAGE_CX + 70.0f * cosf(s_t * 1.3f);
        py = CFG_IMAGE_CY + 50.0f * sinf(s_t * 1.7f);
        w  = 24.0f + 8.0f * sinf(s_t * 2.0f);
        s_state.vision_online = (fmodf(s_t, 6.0f) < 4.5f) ? 1 : 0;
    }

    s_state.target_x = (int)px;
    s_state.target_y = (int)py;
    s_state.target_w = (int)w;
    s_state.target_h = (int)(w * 0.75f);

    if (s_state.vision_online) {
        float roll = CFG_DEROT_ENABLE ? s_state.att.roll_deg : 0.0f;
        guidance_update_pixel(g, px, py, CFG_IMAGE_CX, CFG_IMAGE_CY,
                              CFG_FOCAL_LENGTH_PX, roll, CFG_DEROT_SIGN);
    }
}

/* ---------- 控制 + 舵面输出 ---------- */
static void dart_control_step(float dt)
{
    control_in_t in;
    control_out_t out;
    float raw[4];
    float defl[4];
    int i;

    guidance_update_rate(&s_guid, dt, CFG_LOS_FILTER_ALPHA);

    in.mode      = s_state.mode;
    in.armed     = s_state.armed;
    in.link_ok   = DartLinkIsOk();
    in.vision_ok = s_state.vision_online;
    in.sticks[0] = 0.0f;
    in.sticks[1] = 0.0f;
    in.sticks[2] = 0.0f;
    in.sticks[3] = 0.0f;
    in.att       = s_state.att;
    in.guid      = s_guid;
    in.dt        = dt;

    control_step(&in, &out);

    /* 失效保护: 链路/控制失效时舵面强制回中 */
    DartSurfaceSetFailsafe(out.failsafe);
    DartSurfaceSetMix(out.mix.pitch, out.mix.yaw, out.mix.roll);

    DartSurfaceGetDeflDeg(defl);
    DartSurfaceGetRawDeg(raw);
    for (i = 0; i < 4; i++) {
        s_state.servo_deg[i] = defl[i];
        s_state.servo_raw[i] = raw[i];
    }
    s_state.surface_state = (int)DartSurfaceGetState();
    s_state.surface_failsafe = DartSurfaceIsFailsafe();
    s_state.mix_pitch = out.mix.pitch;
    s_state.mix_yaw   = out.mix.yaw;
    s_state.mix_roll  = out.mix.roll;
    s_state.failsafe  = out.failsafe;

    s_state.guid_valid   = s_guid.valid;
    s_state.guid_started = s_guid.started;
    s_state.lambda_x     = s_guid.lambda_x;
    s_state.lambda_y     = s_guid.lambda_y;
    s_state.lambda_dot_x = s_guid.lambda_dot_x;
    s_state.lambda_dot_y = s_guid.lambda_dot_y;
    s_state.accel_x      = s_guid.accel_x;
    s_state.accel_y      = s_guid.accel_y;
}

void DartAppInit(void)
{
    if (s_inited) return;

    DartCfgInit();
    DartSurfaceInit();
    control_init();
    guidance_init(&s_guid);
    dart_attitude_init(&s_state.att);
    DartLinkInit();

    s_state.mode      = CFG_MODE_DEFAULT;
    s_state.armed     = 1;
    s_state.link_ok   = 1;
    s_state.vision_online = 0;
    s_state.surface_failsafe = 0;
    s_ctrl_div        = 0;
    s_t               = 0.0f;
    s_vis_tick        = 0;

    DartSurfaceSetState(CFG_SURFACE_DEFAULT_STATE);

    s_last_us = (uint32_t)DWT_GetTimeline_us();
    s_last_ctrl_us = s_last_us;
    s_inited = 1;
}

void DartAppTask(void)
{
    uint32_t now_us;
    float dt;

    if (!s_inited) DartAppInit();

    /* 坑 #3: J-Link 断开(qc)会清掉 CoreDebug->DEMCR, 导致 DWT 停走 -> dt=0 任务空转。
     * 每周期兜底重使能。 */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    now_us = (uint32_t)DWT_GetTimeline_us();
    dt = (float)(now_us - s_last_us) * 1e-6f;
    if (dt <= 0.0f) return;
    s_last_us = now_us;
    if (dt > 0.05f) dt = 0.05f;
    s_t += dt;

    dart_read_imu(&s_state.att, dt);
    dart_update_vision(&s_guid);

    if (++s_ctrl_div >= DART_CTRL_DIV) {
        float cdt;
        s_ctrl_div = 0;
        cdt = (float)(now_us - s_last_ctrl_us) * 1e-6f;
        s_last_ctrl_us = now_us;
        if (cdt <= 0.0f) cdt = 0.01f;

        dart_control_step(cdt);
        DartSurfaceTask(cdt);
        DartLinkTask();
        DartCfgTask();
    }
}

const dart_state_t *DartAppGetState(void)
{
    return &s_state;
}
