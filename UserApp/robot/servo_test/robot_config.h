/**
 * @file robot_config.h
 * @author ai
 * @brief servo_test 参数 (PTK7350 PWM 舵机驱动测试)
 * @version 1.0
 * @date 2026-09-28
 *
 * @note  实测为 270° 型(500~2500us <-> 0~270°); 若有偏差用 N/S 命令在线微调。
 */
#pragma once

/* 舵机编号 */
#define SERVO_TEST_ID 0

/* PWM: PWM1 = TIM1_CH1 = PE9, 50Hz(周期 20ms), 初始占空比 1.5ms 中立 */
#define SERVO_TEST_TIM (&htim1)
#define SERVO_TEST_CHANNEL TIM_CHANNEL_1
#define SERVO_TEST_PERIOD_S 0.02f
#define SERVO_TEST_DUTY_INIT 0.075f

/* 标定: 实测 270° 型 (500~2500us <-> 0~270°) */
#define SERVO_TEST_PULSE_MIN_US 500.0f
#define SERVO_TEST_PULSE_MAX_US 2500.0f
#define SERVO_TEST_RANGE_DEG 270.0f
#define SERVO_TEST_CENTER_DEG 135.0f  /* 逻辑 0° = 机械中位 */
#define SERVO_TEST_TRIM_DEG 0.0f
#define SERVO_TEST_SCALE 1.1f           /* 实测调定值 */
#define SERVO_TEST_REVERSE 1
#define SERVO_TEST_RATE_DPS 0.0f        /* 0 = 不限速(立即到位) */
#define SERVO_TEST_LIMIT_DEG 90.0f      /* 逻辑角对称限位 ±90° */
#define SERVO_TEST_INIT_DEG 90.0f       /* 上电后转到该逻辑角 */
