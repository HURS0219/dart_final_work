#if defined(STM32F407xx)

#include "ptk7645.h"

/* 计算定时器 1 个计数对应多少微秒。
 * F4 上 TIM1/8/9/10/11 挂 APB2, 其余挂 APB1;
 * APB 分频 != 1 时定时器时钟 = PCLKx * 2。 */
static float PTK7645_TickUs(TIM_HandleTypeDef *htim)
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

uint16_t PTK7645_AngleToPulse(float angle_deg)
{
    float span = (float)(PTK7645_PULSE_MAX_US - PTK7645_PULSE_MIN_US);
    float pulse = (float)PTK7645_PULSE_MIN_US + angle_deg / PTK7645_ANGLE_MAX_DEG * span;

    if (pulse < (float)PTK7645_PULSE_MIN_US) pulse = (float)PTK7645_PULSE_MIN_US;
    if (pulse > (float)PTK7645_PULSE_MAX_US) pulse = (float)PTK7645_PULSE_MAX_US;
    return (uint16_t)(pulse + 0.5f);
}

float PTK7645_PulseToAngle(uint16_t pulse_us)
{
    float span = (float)(PTK7645_PULSE_MAX_US - PTK7645_PULSE_MIN_US);
    return ((float)pulse_us - (float)PTK7645_PULSE_MIN_US) / span * PTK7645_ANGLE_MAX_DEG;
}

void PTK7645_Init(PTK7645_t *servo, TIM_HandleTypeDef *htim, uint32_t channel)
{
    uint32_t arr;

    servo->htim = htim;
    servo->channel = channel;
    servo->tick_us = PTK7645_TickUs(htim);
    servo->angle_deg = PTK7645_ANGLE_MIN_DEG;
    servo->pulse_us = PTK7645_PULSE_MIN_US;

    /* 设定 50Hz 周期 (ARR 为 16 位, 超出则截断) */
    arr = (uint32_t)((float)PTK7645_PERIOD_US / servo->tick_us + 0.5f);
    if (arr > 0u) arr -= 1u;
    if (arr > 0xFFFFu) arr = 0xFFFFu;
    __HAL_TIM_SET_AUTORELOAD(htim, arr);

    HAL_TIM_PWM_Start(htim, channel);
    servo->started = 1u;

    /* 初始停在 0°, 需要保持其他角度请自行调用 PTK7645_SetAngle */
    PTK7645_SetPulseUs(servo, PTK7645_PULSE_MIN_US);
}

void PTK7645_SetPulseUs(PTK7645_t *servo, uint16_t pulse_us)
{
    uint32_t ccr;

    if (pulse_us < PTK7645_PULSE_MIN_US) pulse_us = PTK7645_PULSE_MIN_US;
    if (pulse_us > PTK7645_PULSE_MAX_US) pulse_us = PTK7645_PULSE_MAX_US;

    servo->pulse_us = pulse_us;
    ccr = (uint32_t)((float)pulse_us / servo->tick_us + 0.5f);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, ccr);
}

void PTK7645_SetAngle(PTK7645_t *servo, float angle_deg)
{
    if (angle_deg < PTK7645_ANGLE_MIN_DEG) angle_deg = PTK7645_ANGLE_MIN_DEG;
    if (angle_deg > PTK7645_ANGLE_MAX_DEG) angle_deg = PTK7645_ANGLE_MAX_DEG;

    servo->angle_deg = angle_deg;
    PTK7645_SetPulseUs(servo, PTK7645_AngleToPulse(angle_deg));
}

void PTK7645_Disable(PTK7645_t *servo)
{
    HAL_TIM_PWM_Stop(servo->htim, servo->channel);
    servo->started = 0u;
}

#endif /* STM32F407xx */
