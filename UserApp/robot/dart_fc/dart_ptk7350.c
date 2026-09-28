/*
 * dart_ptk7350.c — 制导飞镖自研 PTK7350 底层驱动 (TIM1 输出比较 PWM, 50Hz)
 */
#if defined(STM32F407xx)

#include "dart_ptk7350.h"

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 计算定时器 1 个计数对应多少微秒。
 * F4 上 TIM1/8/9/10/11 挂 APB2, 其余挂 APB1; APB 分频 != 1 时定时器时钟 = PCLKx * 2。 */
static float DartPtk_TickUs(TIM_HandleTypeDef *htim)
{
    uintptr_t inst = (uintptr_t)htim->Instance;
    RCC_ClkInitTypeDef clk;
    uint32_t latency = 0;
    uint32_t pclk;
    uint32_t timer_clk;

    HAL_RCC_GetClockConfig(&clk, &latency);

    if (inst == (uintptr_t)TIM1 || inst == (uintptr_t)TIM8 ||
        inst == (uintptr_t)TIM9 || inst == (uintptr_t)TIM10 || inst == (uintptr_t)TIM11) {
        pclk = HAL_RCC_GetPCLK2Freq();
        timer_clk = (clk.APB2CLKDivider == RCC_HCLK_DIV1) ? pclk : (pclk * 2u);
    } else {
        pclk = HAL_RCC_GetPCLK1Freq();
        timer_clk = (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk : (pclk * 2u);
    }

    if (timer_clk == 0u) {
        return 1.0f;
    }
    return (float)(htim->Init.Prescaler + 1u) * 1000000.0f / (float)timer_clk;
}

/* 按 50Hz 设置 ARR */
static void DartPtk_SetPeriod(DartPtk7350_t *s)
{
    uint32_t arr = (uint32_t)((float)DART_PTK_PERIOD_US / s->tick_us + 0.5f);
    if (arr > 0u) arr -= 1u;
    if (arr > 0xFFFFu) arr = 0xFFFFu;
    __HAL_TIM_SET_AUTORELOAD(s->htim, arr);
}

float DartPtk7350_AngleToPulse(const DartPtk7350_t *s, float raw_deg)
{
    float span = s->pulse_max_us - s->pulse_min_us;
    float pulse = s->pulse_min_us + raw_deg / s->range_deg * span;

    if (pulse < s->pulse_min_us) pulse = s->pulse_min_us;
    if (pulse > s->pulse_max_us) pulse = s->pulse_max_us;
    return pulse;
}

void DartPtk7350_Init(DartPtk7350_t *s, TIM_HandleTypeDef *htim, uint32_t channel)
{
    s->htim = htim;
    s->channel = channel;
    s->pulse_min_us = DART_PTK_PULSE_MIN_US;
    s->pulse_max_us = DART_PTK_PULSE_MAX_US;
    s->range_deg = DART_PTK_RANGE_DEG;
    s->tick_us = DartPtk_TickUs(htim);
    s->neutral_deg = 90.0f;
    s->trim_deg = 0.0f;
    s->reverse = 1;
    s->cmd_deg = 0.0f;
    s->raw_deg = s->neutral_deg;
    s->pulse_us = DART_PTK_PULSE_MIN_US;

    DartPtk_SetPeriod(s);
    HAL_TIM_PWM_Start(htim, channel);
    s->started = 1u;

    /* 上电先停在机械中立 */
    DartPtk7350_SetRawDeg(s, s->neutral_deg);
}

void DartPtk7350_Config(DartPtk7350_t *s, float pulse_min_us, float pulse_max_us,
                        float range_deg, int8_t reverse)
{
    s->pulse_min_us = (pulse_min_us > 1.0f) ? pulse_min_us : DART_PTK_PULSE_MIN_US;
    s->pulse_max_us = (pulse_max_us > s->pulse_min_us) ? pulse_max_us : DART_PTK_PULSE_MAX_US;
    s->range_deg = (range_deg > 1.0f) ? range_deg : DART_PTK_RANGE_DEG;
    s->reverse = (reverse < 0) ? -1 : 1;
    /* 标定变化后重新按新脉宽输出一次 */
    DartPtk7350_SetRawDeg(s, s->raw_deg);
}

void DartPtk7350_SetNeutral(DartPtk7350_t *s, float neutral_deg)
{
    s->neutral_deg = clampf(neutral_deg, 0.0f, s->range_deg);
}

void DartPtk7350_SetTrim(DartPtk7350_t *s, float trim_deg)
{
    s->trim_deg = trim_deg;
}

void DartPtk7350_SetRawDeg(DartPtk7350_t *s, float raw_deg)
{
    raw_deg = clampf(raw_deg, 0.0f, s->range_deg);
    s->raw_deg = raw_deg;
    DartPtk7350_SetPulseUs(s, DartPtk7350_AngleToPulse(s, raw_deg));
}

void DartPtk7350_SetLogicalDeg(DartPtk7350_t *s, float deg)
{
    float raw;
    s->cmd_deg = deg;
    raw = s->neutral_deg + s->trim_deg + (float)s->reverse * deg;
    DartPtk7350_SetRawDeg(s, raw);
}

void DartPtk7350_SetPulseUs(DartPtk7350_t *s, float pulse_us)
{
    uint32_t ccr;

    pulse_us = clampf(pulse_us, s->pulse_min_us, s->pulse_max_us);
    s->pulse_us = pulse_us;
    ccr = (uint32_t)(pulse_us / s->tick_us + 0.5f);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, ccr);
}

float DartPtk7350_GetRawDeg(const DartPtk7350_t *s)
{
    return s->raw_deg;
}

float DartPtk7350_GetCmdDeg(const DartPtk7350_t *s)
{
    return s->cmd_deg;
}

void DartPtk7350_Enable(DartPtk7350_t *s)
{
    HAL_TIM_PWM_Start(s->htim, s->channel);
    s->started = 1u;
    DartPtk7350_SetRawDeg(s, s->raw_deg);
}

void DartPtk7350_Disable(DartPtk7350_t *s)
{
    HAL_TIM_PWM_Stop(s->htim, s->channel);
    s->started = 0u;
}

#endif /* STM32F407xx */
