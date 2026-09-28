/*
 * dart_cfg.c — 制导飞镖运行时参数 + 掉电保存 (双 Bank + FNV-1a 校验)
 */
#include "dart_cfg.h"

#include <string.h>

#include "bsp_flash.h"
#include "dart_config.h"
#include "main.h"
#include "servo_mixer.h"

#define DART_CFG_BANK_A ADDR_FLASH_SECTOR_10  /* 0x080C0000 */
#define DART_CFG_BANK_B ADDR_FLASH_SECTOR_11  /* 0x080E0000 */
#define DART_CFG_MAGIC  0xDA27C0DEu
#define SAVE_DEBOUNCE_MS 300u

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t crc;
    DartCfg_t data;
} DartCfgBank_t;

static DartCfg_t s_cfg;
static volatile uint32_t s_version = 1;
static volatile uint8_t s_dirty = 0;
static volatile uint8_t s_force = 0;
static volatile uint32_t s_dirty_tick = 0;
static volatile uint8_t s_saved_flag = 0;
static uint32_t s_active_addr = 0;
static uint32_t s_seq = 0;
static uint8_t s_inited = 0;

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void DartCfgDefaults(DartCfg_t *c)
{
    static const float kMix[DART_CFG_SERVO_CNT][3] = {
        {CFG_MIX_PITCH_1, CFG_MIX_YAW_1, CFG_MIX_ROLL_1},
        {CFG_MIX_PITCH_2, CFG_MIX_YAW_2, CFG_MIX_ROLL_2},
        {CFG_MIX_PITCH_3, CFG_MIX_YAW_3, CFG_MIX_ROLL_3},
        {CFG_MIX_PITCH_4, CFG_MIX_YAW_4, CFG_MIX_ROLL_4},
    };
    int i;

    for (i = 0; i < DART_CFG_SERVO_CNT; i++) {
        c->servo_neutral_deg[i] = 90.0f + CFG_SERVO_TRIM_DEG;
        c->servo_trim_deg[i] = 0.0f;
        c->servo_max_deg[i] = CFG_SERVO_MAX_ANGLE;
        c->servo_reverse[i] = 1.0f;
    }
    c->servo_pulse_min_us = CFG_SERVO_PULSE_MIN_US;
    c->servo_pulse_max_us = CFG_SERVO_PULSE_MAX_US;
    c->servo_range_deg = 270.0f;
    c->servo_rate_limit = CFG_SERVO_RATE_LIMIT;

    memcpy(c->mix, kMix, sizeof(c->mix));

    c->roll_kp = CFG_ROLL_KP;
    c->roll_ki = CFG_ROLL_KI;
    c->roll_kd = CFG_ROLL_KD;
    c->pitch_kp = CFG_PITCH_KP;
    c->pitch_ki = CFG_PITCH_KI;
    c->pitch_kd = CFG_PITCH_KD;
    c->yaw_kp = CFG_YAW_KP;
    c->yaw_ki = CFG_YAW_KI;
    c->yaw_kd = CFG_YAW_KD;

    c->nav_ratio = CFG_NAV_RATIO_N;
    c->v_close = CFG_V_CLOSE;
    c->png_max_g = CFG_PNG_MAX_G;
    c->failsafe_ms = CFG_FAILSAFE_MS;
}

void DartCfgApply(void)
{
    /* 混控矩阵: 让 mixer 输出 -1..1 的归一化控制量, 各舵面量程/标定在 DartSurface 处理 */
    servo_mixer_init(s_cfg.mix, 0.0f, 1.0f);
    s_version++;
}

DartCfg_t *DartCfg(void)
{
    return &s_cfg;
}

uint32_t DartCfgVersion(void)
{
    return s_version;
}

/* ---------------- 参数表 ---------------- */

int DartCfgSetParam(int id, float value)
{
    DartCfg_t *c = &s_cfg;

    if (id >= DART_P_SERVO_NEUTRAL && id < DART_P_SERVO_NEUTRAL + DART_CFG_SERVO_CNT) {
        c->servo_neutral_deg[id - DART_P_SERVO_NEUTRAL] = clampf(value, 0.0f, c->servo_range_deg);
    } else if (id >= DART_P_SERVO_TRIM && id < DART_P_SERVO_TRIM + DART_CFG_SERVO_CNT) {
        c->servo_trim_deg[id - DART_P_SERVO_TRIM] = clampf(value, -60.0f, 60.0f);
    } else if (id >= DART_P_SERVO_MAX && id < DART_P_SERVO_MAX + DART_CFG_SERVO_CNT) {
        c->servo_max_deg[id - DART_P_SERVO_MAX] = clampf(value, 0.0f, 90.0f);
    } else if (id >= DART_P_SERVO_REVERSE && id < DART_P_SERVO_REVERSE + DART_CFG_SERVO_CNT) {
        c->servo_reverse[id - DART_P_SERVO_REVERSE] = (value < 0.0f) ? -1.0f : 1.0f;
    } else if (id >= DART_P_MIX && id < DART_P_MIX + DART_CFG_SERVO_CNT * 3) {
        int k = id - DART_P_MIX;
        c->mix[k / 3][k % 3] = clampf(value, -2.0f, 2.0f);
    } else {
        switch (id) {
        case DART_P_SERVO_PULSE_MIN: c->servo_pulse_min_us = clampf(value, 200.0f, 2000.0f); break;
        case DART_P_SERVO_PULSE_MAX: c->servo_pulse_max_us = clampf(value, 500.0f, 3000.0f); break;
        case DART_P_SERVO_RANGE: c->servo_range_deg = clampf(value, 90.0f, 360.0f); break;
        case DART_P_SERVO_RATE: c->servo_rate_limit = clampf(value, 10.0f, 2000.0f); break;
        case DART_P_ROLL_KP: c->roll_kp = value; break;
        case DART_P_ROLL_KI: c->roll_ki = value; break;
        case DART_P_ROLL_KD: c->roll_kd = value; break;
        case DART_P_PITCH_KP: c->pitch_kp = value; break;
        case DART_P_PITCH_KI: c->pitch_ki = value; break;
        case DART_P_PITCH_KD: c->pitch_kd = value; break;
        case DART_P_YAW_KP: c->yaw_kp = value; break;
        case DART_P_YAW_KI: c->yaw_ki = value; break;
        case DART_P_YAW_KD: c->yaw_kd = value; break;
        case DART_P_NAV_RATIO: c->nav_ratio = clampf(value, 0.0f, 20.0f); break;
        case DART_P_V_CLOSE: c->v_close = clampf(value, 1.0f, 500.0f); break;
        case DART_P_PNG_MAX_G: c->png_max_g = clampf(value, 0.1f, 20.0f); break;
        case DART_P_FAILSAFE_MS: c->failsafe_ms = (uint32_t)clampf(value, 0.0f, 10000.0f); break;
        default: return 0;
        }
    }

    DartCfgApply();
    DartCfgMarkDirty();
    return 1;
}

float DartCfgGetParam(int id)
{
    const DartCfg_t *c = &s_cfg;

    if (id >= DART_P_SERVO_NEUTRAL && id < DART_P_SERVO_NEUTRAL + DART_CFG_SERVO_CNT)
        return c->servo_neutral_deg[id - DART_P_SERVO_NEUTRAL];
    if (id >= DART_P_SERVO_TRIM && id < DART_P_SERVO_TRIM + DART_CFG_SERVO_CNT)
        return c->servo_trim_deg[id - DART_P_SERVO_TRIM];
    if (id >= DART_P_SERVO_MAX && id < DART_P_SERVO_MAX + DART_CFG_SERVO_CNT)
        return c->servo_max_deg[id - DART_P_SERVO_MAX];
    if (id >= DART_P_SERVO_REVERSE && id < DART_P_SERVO_REVERSE + DART_CFG_SERVO_CNT)
        return c->servo_reverse[id - DART_P_SERVO_REVERSE];
    if (id >= DART_P_MIX && id < DART_P_MIX + DART_CFG_SERVO_CNT * 3) {
        int k = id - DART_P_MIX;
        return c->mix[k / 3][k % 3];
    }

    switch (id) {
    case DART_P_SERVO_PULSE_MIN: return c->servo_pulse_min_us;
    case DART_P_SERVO_PULSE_MAX: return c->servo_pulse_max_us;
    case DART_P_SERVO_RANGE: return c->servo_range_deg;
    case DART_P_SERVO_RATE: return c->servo_rate_limit;
    case DART_P_ROLL_KP: return c->roll_kp;
    case DART_P_ROLL_KI: return c->roll_ki;
    case DART_P_ROLL_KD: return c->roll_kd;
    case DART_P_PITCH_KP: return c->pitch_kp;
    case DART_P_PITCH_KI: return c->pitch_ki;
    case DART_P_PITCH_KD: return c->pitch_kd;
    case DART_P_YAW_KP: return c->yaw_kp;
    case DART_P_YAW_KI: return c->yaw_ki;
    case DART_P_YAW_KD: return c->yaw_kd;
    case DART_P_NAV_RATIO: return c->nav_ratio;
    case DART_P_V_CLOSE: return c->v_close;
    case DART_P_PNG_MAX_G: return c->png_max_g;
    case DART_P_FAILSAFE_MS: return (float)c->failsafe_ms;
    default: return 0.0f;
    }
}

void DartCfgReset(void)
{
    DartCfgDefaults(&s_cfg);
    DartCfgApply();
    DartCfgMarkDirty();
}

/* ---------------- 掉电保存 ---------------- */

void DartCfgMarkDirty(void)
{
    s_dirty = 1;
    s_dirty_tick = HAL_GetTick();
}

void DartCfgSaveNow(void)
{
    s_dirty = 1;
    s_force = 1;
    s_dirty_tick = HAL_GetTick();
}

int DartCfgTakeSaved(void)
{
    int v = s_saved_flag;
    s_saved_flag = 0;
    return v;
}

static uint32_t DartCfgCrc(const DartCfg_t *d)
{
    const uint8_t *p = (const uint8_t *)d;
    uint32_t s = 0x811C9DC5u;
    uint32_t i;
    for (i = 0; i < sizeof(DartCfg_t); i++) {
        s ^= p[i];
        s *= 16777619u;
    }
    return s;
}

static int DartCfgReadBank(uint32_t addr, DartCfgBank_t *b)
{
    flash_read(addr, (uint32_t *)b, sizeof(DartCfgBank_t) / 4);
    if (b->magic != DART_CFG_MAGIC) return 0;
    if (b->crc != DartCfgCrc(&b->data)) return 0;
    return 1;
}

void DartCfgInit(void)
{
    DartCfgBank_t a, b;
    int oka, okb;
    DartCfgBank_t *best = NULL;

    if (s_inited) return;

    DartCfgDefaults(&s_cfg);

    oka = DartCfgReadBank(DART_CFG_BANK_A, &a);
    okb = DartCfgReadBank(DART_CFG_BANK_B, &b);
    if (oka && okb) best = (a.seq >= b.seq) ? &a : &b;
    else if (oka) best = &a;
    else if (okb) best = &b;

    if (best != NULL) {
        s_cfg = best->data;
        s_active_addr = (best == &a) ? DART_CFG_BANK_A : DART_CFG_BANK_B;
        s_seq = best->seq;
    }

    DartCfgApply();
    s_inited = 1;
}

void DartCfgTask(void)
{
    DartCfgBank_t bank;
    uint32_t target;

    if (!s_dirty) return;
    if (!s_force && (HAL_GetTick() - s_dirty_tick) < SAVE_DEBOUNCE_MS) return;

    bank.magic = DART_CFG_MAGIC;
    bank.seq = s_seq + 1;
    bank.data = s_cfg;
    bank.crc = DartCfgCrc(&bank.data);

    target = (s_active_addr == DART_CFG_BANK_A) ? DART_CFG_BANK_B : DART_CFG_BANK_A;

    __disable_irq();
    flash_erase_address(target, 1);
    flash_write_single_address(target, (uint32_t *)&bank, sizeof(bank) / 4);
    __enable_irq();

    s_active_addr = target;
    s_seq = bank.seq;
    s_dirty = 0;
    s_force = 0;
    s_saved_flag = 1;
}
