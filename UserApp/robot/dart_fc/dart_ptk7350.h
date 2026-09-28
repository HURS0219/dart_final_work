/*
 * dart_ptk7350.h — 制导飞镖自研 PTK7350 舵机底层驱动 (隔离在 app 内, 不改 Modules)
 *
 * 为什么自研:
 *   Modules/motor/servo_motor 的 servo 抽象不完善 (Bus/PWM 混在一起、角度换算依赖
 *   定时器参数未规范化)。制导飞镖只用 PWM 型 PTK7350, 因此这里写一个专用、可标定、
 *   带反向/中立/微调/量程的干净底层。
 *
 * 硬件 (C 板 STM32F407, TIM1 输出比较 PWM, 50Hz):
 *   PWM1 = TIM1_CH1 = PE9     PWM2 = TIM1_CH2 = PE11
 *   PWM3 = TIM1_CH3 = PE13    PWM4 = TIM1_CH4 = PE14
 *
 * 信号: 周期 20ms, 高电平 pulse_min..pulse_max 线性对应 机械角 0..range_deg。
 * 驱动按定时器当前 PSC 自动换算 tick, 并设置 ARR 为 50Hz, 因此 PSC 用 CubeMX 原样即可。
 */
#ifndef DART_FC_PTK7350_H
#define DART_FC_PTK7350_H

#if defined(STM32F407xx)

#include "main.h"
#include "tim.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 默认信号参数 (可被 DartPtk7350_Config 覆盖) */
#define DART_PTK_PWM_FREQ_HZ   50u
#define DART_PTK_PERIOD_US     20000u
#define DART_PTK_PULSE_MIN_US  500.0f
#define DART_PTK_PULSE_MAX_US  2500.0f
#define DART_PTK_RANGE_DEG     270.0f   /* PTK7350 机械量程 */

typedef struct {
    TIM_HandleTypeDef *htim;      /* 例如 &htim1 */
    uint32_t channel;             /* TIM_CHANNEL_1 ~ 4 */

    /* 信号标定 */
    float pulse_min_us;           /* 机械 0° 对应脉宽 */
    float pulse_max_us;           /* 机械 range_deg 对应脉宽 */
    float range_deg;              /* 机械量程 */
    float tick_us;                /* 1 个计数对应微秒 */

    /* 每路机械标定 (由 DartCfg 下发) */
    float neutral_deg;            /* 逻辑 0 对应的机械角 (舵面中立) */
    float trim_deg;               /* 中立微调 */
    int8_t reverse;               /* +1 / -1 */

    /* 运行时状态 */
    float cmd_deg;                /* 最近一次逻辑指令角 */
    float raw_deg;                /* 最近一次机械输出角 */
    float pulse_us;               /* 最近一次脉宽 */
    uint8_t started;              /* PWM 是否已启动 */
} DartPtk7350_t;

/* 绑定一个已配置为 PWM 的定时器通道并启动输出; 默认中立 90°, 量程 270° */
void DartPtk7350_Init(DartPtk7350_t *s, TIM_HandleTypeDef *htim, uint32_t channel);

/* 修改信号标定 (脉宽范围/机械量程/方向), 会重新计算 tick 与 ARR */
void DartPtk7350_Config(DartPtk7350_t *s, float pulse_min_us, float pulse_max_us,
                        float range_deg, int8_t reverse);

/* 设置机械中立角 / 微调 */
void DartPtk7350_SetNeutral(DartPtk7350_t *s, float neutral_deg);
void DartPtk7350_SetTrim(DartPtk7350_t *s, float trim_deg);

/* 机械角直接输出 (0..range_deg), 不做逻辑映射 */
void DartPtk7350_SetRawDeg(DartPtk7350_t *s, float raw_deg);

/* 逻辑角输出: raw = neutral + trim + reverse*deg, 自动限幅到 [0, range] */
void DartPtk7350_SetLogicalDeg(DartPtk7350_t *s, float deg);

/* 直接给脉宽 (自动限幅) */
void DartPtk7350_SetPulseUs(DartPtk7350_t *s, float pulse_us);

float DartPtk7350_GetRawDeg(const DartPtk7350_t *s);
float DartPtk7350_GetCmdDeg(const DartPtk7350_t *s);

/* 停止 PWM (舵机失去保持力) */
void DartPtk7350_Disable(DartPtk7350_t *s);
void DartPtk7350_Enable(DartPtk7350_t *s);

/* 机械角 -> 脉宽 (纯函数, 便于标定) */
float DartPtk7350_AngleToPulse(const DartPtk7350_t *s, float raw_deg);

#ifdef __cplusplus
}
#endif

#endif /* STM32F407xx */
#endif /* DART_FC_PTK7350_H */
