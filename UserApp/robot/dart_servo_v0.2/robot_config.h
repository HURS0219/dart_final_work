/*
 * robot_config.h — 制导飞镖舵机子系统 v0.2 (完全重写)
 *
 * 设计目标: 逻辑清晰、无隐藏状态。四个 PTK7350 舵机 (X 型舵面)。
 * 硬件 (RoboMaster C 板 / STM32F407):
 *   PWM1 = TIM1_CH1 = PE9   右上
 *   PWM2 = TIM1_CH2 = PE11  左上
 *   PWM3 = TIM1_CH3 = PE13  左下
 *   PWM4 = TIM1_CH4 = PE14  右下
 * 舵机: PTK7350MG-D, 50Hz, 500~2500us, 实测总行程 279° (270° 级)。
 */
#ifndef DART_V2_CONFIG_H
#define DART_V2_CONFIG_H

#include "tim.h"
#include "usart.h"

/* ===================== 定时器 / 通道 / 行程 / 脉宽 ===================== */
#define DART_PWM_TIM             (&htim1)
#define DART_AXIS_N              4

#define DART_TRAVEL_HALF_DEG     139.5f   /* 279° / 2 */
#define DART_TRAVEL_FULL_DEG     279.0f

#define DART_PULSE_CENTER_US     1500.0f
#define DART_PULSE_HALF_US       1000.0f  /* 中位 ± 1000us = ±139.5° */
#define DART_PULSE_MIN_US        500.0f   /* 硬限, 越程会翻转 */
#define DART_PULSE_MAX_US        2500.0f

#define DART_RATE_LIMIT_DPS      600.0f   /* 舵面速率限幅 */
#define DART_DEADBAND_DEG        0.02f

/* ===================== X 型四舵面解耦矩阵 ===================== */
/* 与 dart_fc 保持一致的安装符号约定: 行=舵面, 列=pitch/yaw/roll */
#define DART_MIX_PITCH_0  (+1.0f)
#define DART_MIX_PITCH_1  (-1.0f)
#define DART_MIX_PITCH_2  (-1.0f)
#define DART_MIX_PITCH_3  (+1.0f)
#define DART_MIX_YAW_0    (+1.0f)
#define DART_MIX_YAW_1    (+1.0f)
#define DART_MIX_YAW_2    (-1.0f)
#define DART_MIX_YAW_3    (-1.0f)
#define DART_MIX_ROLL_0   (+1.0f)
#define DART_MIX_ROLL_1   (-1.0f)
#define DART_MIX_ROLL_2   (+1.0f)
#define DART_MIX_ROLL_3   (-1.0f)
#define DART_MIX_MAX_DEG  35.0f           /* 混控满偏对应的舵面角 */

/* ===================== 自检 ===================== */
#define DART_TEST_SWEEP_DEG      20.0f
#define DART_TEST_FREQ_HZ        0.4f

/* ===================== 串口 (ESP32 <-> C板) ===================== */
#define DART_UART_HANDLE         (&huart6)
#define DART_RECV_SIZE           128
#define DART_LINK_TIMEOUT_MS     3000u    /* 超时回 IDLE */
#define DART_TELEM_PERIOD_MS     100u

/* ===================== 掉电保存: 双 Bank 交替 ===================== */
#define DART_CFG_ADDR_A          0x080C0000u  /* sector 10 */
#define DART_CFG_ADDR_B          0x080E0000u  /* sector 11 */
#define DART_CFG_MAGIC           0xD2A70002u
#define DART_CFG_SAVE_DEBOUNCE_MS 800u   /* 标定改动后自动保存的去抖时间 */

#endif /* DART_V2_CONFIG_H */
