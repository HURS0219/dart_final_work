/**
 * @file robot_config.h
 * @author ai
 * @brief servo_test 参数 (单路 PTK7350 PWM 舵机驱动测试)
 * @version 2.0
 * @date 2026-09-28
 *
 * @note  PTK7350: 50Hz(周期 20ms), 500~2500us; 实测 279° 行程
 *        (中位 1500us 对应机械 0°, ±1000us <-> ±139.5°)。
 *        掉电保存由 bsp_flash 的 flash_store_save/load 负责(双 Bank, 地址见 robot.c)。
 */
#pragma once

/* PWM: PWM1 = TIM1_CH1 = PE9, 50Hz(周期 20ms), 初始占空比 1.5ms 中立 */
#define SERVO_TEST_TIM (&htim1)
#define SERVO_TEST_CHANNEL TIM_CHANNEL_1
#define SERVO_TEST_PERIOD_S 0.02f
#define SERVO_TEST_DUTY_INIT 0.075f

/* 标定: 中位 1500us; 中位到行程端 1000us <-> 139.5°; 硬限 500/2500us */
#define SERVO_TEST_CENTER_US 1500.0f
#define SERVO_TEST_HALF_US 1000.0f
#define SERVO_TEST_HALF_DEG 139.5f
#define SERVO_TEST_PULSE_MIN_US 500.0f
#define SERVO_TEST_PULSE_MAX_US 2500.0f
#define SERVO_TEST_SCALE 1.0f      /* 逻辑角 -> 机械角 增益 */
#define SERVO_TEST_TRIM_DEG 0.0f   /* 零点微调 */
#define SERVO_TEST_LIMIT_DEG 90.0f /* 逻辑角对称限位 ±90° */
#define SERVO_TEST_RATE_DPS 0.0f   /* 0 = 不限速(立即到位) */
#define SERVO_TEST_REVERSE 0       /* 0 正常 / 1 反向 */
#define SERVO_TEST_ZERO_ENABLE 1   /* 调零功能开关: 0 禁用 / 1 启用 */
#define SERVO_TEST_INIT_DEG 0.0f   /* 上电逻辑角 */
