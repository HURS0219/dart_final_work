#ifndef PTK7645_H
#define PTK7645_H

#if defined(STM32F407xx)

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PTK7645 舵机驱动 (STM32F4 / HAL)
 *
 * 信号: 50Hz PWM, 高电平 500us~2500us 对应 0°~180°
 *
 * RoboMaster C 型开发板接线:
 *   PWM1 = TIM1_CH1 = PE9      PWM2 = TIM1_CH2 = PE11
 *   (PWM3=TIM1_CH3/PE13, PWM4=TIM1_CH4/PE14,
 *    PWM5=TIM8_CH1/PC6,  PWM6=TIM8_CH2/PI6, PWM7=TIM8_CH3/PI7)
 *
 * 定时器要求: 输出比较 PWM 模式, 频率 50Hz。
 *   驱动会按定时器当前 PSC 和时钟自动计算 tick 并设置 ARR,
 *   因此 PSC 按 CubeMX 原样即可 (C 板默认 PSC=167 -> 1us/tick)。
 */

#define PTK7645_PWM_FREQ_HZ    50u      /* 舵机信号频率 */
#define PTK7645_PERIOD_US      20000u   /* 20ms 周期 */
#define PTK7645_PULSE_MIN_US   500u     /* 0°   */
#define PTK7645_PULSE_MAX_US   2500u    /* 180° */
#define PTK7645_ANGLE_MIN_DEG  0.0f
#define PTK7645_ANGLE_MAX_DEG  180.0f

typedef struct {
    TIM_HandleTypeDef *htim;       /* 定时器句柄, 例如 &htim1 */
    uint32_t channel;              /* TIM_CHANNEL_1 ~ 4 */
    float    tick_us;              /* 定时器 1 个计数对应的微秒数 */
    float    angle_deg;            /* 最近设置的角度 */
    uint16_t pulse_us;             /* 最近设置的脉宽 */
    uint8_t  started;              /* PWM 是否已启动 */
} PTK7645_t;

/* 绑定一个已配置为 PWM 的定时器通道并启动输出, 初始角度 0° */
void PTK7645_Init(PTK7645_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);

/* 设置角度 (deg), 自动限幅到 0~180 */
void PTK7645_SetAngle(PTK7645_t *servo, float angle_deg);

/* 直接设置脉宽 (us), 自动限幅到 500~2500 */
void PTK7645_SetPulseUs(PTK7645_t *servo, uint16_t pulse_us);

/* 停止 PWM (舵机失去保持力) */
void PTK7645_Disable(PTK7645_t *servo);

/* 角度 <-> 脉宽 换算 (纯函数, 便于标定) */
uint16_t PTK7645_AngleToPulse(float angle_deg);
float    PTK7645_PulseToAngle(uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* STM32F407xx */

#endif /* PTK7645_H */
