/*
 * dart_surface.c — 制导飞镖 4 路 X 型舵面 状态机实现
 *
 * PWM1..4 = TIM1 CH1..CH4 = PE9/PE11/PE13/PE14
 */
#include "dart_surface.h"

#include <math.h>

#include "dart_cfg.h"
#include "dart_ptk7350.h"
#include "servo_mixer.h"
#include "tim.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TEST_FREQ_HZ 0.25f

static DartPtk7350_t s_servo[DART_SURFACE_CNT];
static surface_state_t s_state = SURFACE_BOOT;
static int s_failsafe = 0;

static mix_cmd_t s_mix;                              /* ACTIVE 指令 */
static float s_manual[DART_SURFACE_CNT];             /* MANUAL 逻辑偏角 */
static float s_target[DART_SURFACE_CNT];             /* 目标逻辑偏角 */
static float s_applied[DART_SURFACE_CNT];            /* 已输出逻辑偏角 (速率限幅后) */
static uint32_t s_cfg_ver;
static uint8_t s_started;
static float s_t;

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void DartSurfaceSyncCfg(void)
{
    DartCfg_t *c = DartCfg();
    int i;

    if (s_cfg_ver == DartCfgVersion()) return;
    s_cfg_ver = DartCfgVersion();

    for (i = 0; i < DART_SURFACE_CNT; i++) {
        DartPtk7350_Config(&s_servo[i], c->servo_pulse_min_us, c->servo_pulse_max_us,
                           c->servo_range_deg, (c->servo_reverse[i] < 0.0f) ? -1 : 1);
        DartPtk7350_SetNeutral(&s_servo[i], c->servo_neutral_deg[i]);
        DartPtk7350_SetTrim(&s_servo[i], c->servo_trim_deg[i]);
    }
}

void DartSurfaceInit(void)
{
    if (s_started) return;

    DartPtk7350_Init(&s_servo[0], &htim1, TIM_CHANNEL_1);
    DartPtk7350_Init(&s_servo[1], &htim1, TIM_CHANNEL_2);
    DartPtk7350_Init(&s_servo[2], &htim1, TIM_CHANNEL_3);
    DartPtk7350_Init(&s_servo[3], &htim1, TIM_CHANNEL_4);

    s_mix.pitch = 0.0f;
    s_mix.yaw = 0.0f;
    s_mix.roll = 0.0f;
    {
        int i;
        for (i = 0; i < DART_SURFACE_CNT; i++) {
            s_manual[i] = 0.0f;
            s_target[i] = 0.0f;
            s_applied[i] = 0.0f;
        }
    }
    s_failsafe = 0;
    s_state = SURFACE_NEUTRAL;
    s_cfg_ver = 0;
    s_t = 0.0f;
    s_started = 1;

    DartSurfaceSyncCfg();
}

void DartSurfaceSetState(surface_state_t st)
{
    if (st >= SURFACE_STATE_CNT) return;
    s_state = st;
}

surface_state_t DartSurfaceGetState(void)
{
    return s_state;
}

const char *DartSurfaceStateName(surface_state_t st)
{
    switch (st) {
    case SURFACE_BOOT: return "BOOT";
    case SURFACE_NEUTRAL: return "NEUTRAL";
    case SURFACE_ACTIVE: return "ACTIVE";
    case SURFACE_TEST: return "TEST";
    case SURFACE_MANUAL: return "MANUAL";
    case SURFACE_FAULT: return "FAULT";
    default: return "?";
    }
}

void DartSurfaceSetMix(float pitch, float yaw, float roll)
{
    s_mix.pitch = clampf(pitch, -1.0f, 1.0f);
    s_mix.yaw = clampf(yaw, -1.0f, 1.0f);
    s_mix.roll = clampf(roll, -1.0f, 1.0f);
}

void DartSurfaceSetManualDeg(const float deg[DART_SURFACE_CNT])
{
    int i;
    for (i = 0; i < DART_SURFACE_CNT; i++) s_manual[i] = deg[i];
}

void DartSurfaceSetManualOne(int idx, float deg)
{
    if (idx < 0 || idx >= DART_SURFACE_CNT) return;
    s_manual[idx] = deg;
}

void DartSurfaceSetFailsafe(int on)
{
    s_failsafe = on ? 1 : 0;
}

void DartSurfaceZero(int idx)
{
    if (idx < 0 || idx >= DART_SURFACE_CNT) return;
    /* 把当前机械位置记为新中立, 逻辑角归零 */
    DartCfgSetParam(DART_P_SERVO_NEUTRAL + idx, DartPtk7350_GetRawDeg(&s_servo[idx]));
    DartCfgSetParam(DART_P_SERVO_TRIM + idx, 0.0f);
    s_applied[idx] = 0.0f;
    s_target[idx] = 0.0f;
    DartCfgSaveNow();
}

void DartSurfaceGetRawDeg(float out[DART_SURFACE_CNT])
{
    int i;
    for (i = 0; i < DART_SURFACE_CNT; i++) out[i] = DartPtk7350_GetRawDeg(&s_servo[i]);
}

void DartSurfaceGetDeflDeg(float out[DART_SURFACE_CNT])
{
    int i;
    for (i = 0; i < DART_SURFACE_CNT; i++) out[i] = s_applied[i];
}

float DartSurfaceGetTargetDeg(int idx)
{
    if (idx < 0 || idx >= DART_SURFACE_CNT) return 0.0f;
    return s_target[idx];
}

int DartSurfaceIsFailsafe(void)
{
    return s_failsafe;
}

/* 计算当前状态下每路的目标逻辑偏角 */
static void DartSurfaceComputeTarget(void)
{
    DartCfg_t *c = DartCfg();
    int i;

    if (s_failsafe) {
        for (i = 0; i < DART_SURFACE_CNT; i++) s_target[i] = 0.0f;
        return;
    }

    switch (s_state) {
    case SURFACE_ACTIVE: {
        float u[DART_SURFACE_CNT];
        servo_mixer_apply(&s_mix, u);   /* mixer 已配置为 neutral=0, max=1 -> u 为 -1..1 */
        for (i = 0; i < DART_SURFACE_CNT; i++) s_target[i] = u[i] * c->servo_max_deg[i];
        break;
    }
    case SURFACE_TEST:
        for (i = 0; i < DART_SURFACE_CNT; i++)
            s_target[i] = c->servo_max_deg[i] * sinf(2.0f * (float)M_PI * TEST_FREQ_HZ * s_t);
        break;
    case SURFACE_MANUAL:
        for (i = 0; i < DART_SURFACE_CNT; i++)
            s_target[i] = clampf(s_manual[i], -c->servo_max_deg[i], c->servo_max_deg[i]);
        break;
    case SURFACE_BOOT:
    case SURFACE_NEUTRAL:
    case SURFACE_FAULT:
    default:
        for (i = 0; i < DART_SURFACE_CNT; i++) s_target[i] = 0.0f;
        break;
    }
}

void DartSurfaceTask(float dt)
{
    DartCfg_t *c;
    float max_delta;
    int i;

    if (!s_started) DartSurfaceInit();
    DartSurfaceSyncCfg();

    c = DartCfg();
    if (dt <= 0.0f) dt = 1e-3f;
    if (dt > 0.05f) dt = 0.05f;
    s_t += dt;

    DartSurfaceComputeTarget();

    max_delta = c->servo_rate_limit * dt;
    for (i = 0; i < DART_SURFACE_CNT; i++) {
        float t = s_target[i];
        if (t > s_applied[i] + max_delta) t = s_applied[i] + max_delta;
        if (t < s_applied[i] - max_delta) t = s_applied[i] - max_delta;
        s_applied[i] = t;
        DartPtk7350_SetLogicalDeg(&s_servo[i], t);
    }
}
