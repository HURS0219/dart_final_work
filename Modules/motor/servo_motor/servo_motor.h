/**
 * @file servo_motor.h
 * @author ai
 * @brief PWM 舵机(PTK7350 等)底层驱动: 逻辑角 <-> 脉宽 标定 + 速率限幅
 * @version 2.0
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention
 *  1. 50Hz(周期 20ms), 脉宽 pulse_min_us ~ pulse_max_us 线性对应机械角 -half_deg ~ +half_deg,
 *     中位 center_us(通常 1500) 对应机械 0°。
 *     机械偏角 applied = 逻辑角*scale + trim (reverse 时取负), 再换算脉宽:
 *         pulse = center_us + applied * (half_us / half_deg)
 *  2. limit_deg 为逻辑角对称限位(相对逻辑 0°); 调零需 zero_enable 使能。
 *  3. 本模块只做“单路驱动 + 标定 + 速率限幅 + 调零”; 四舵面混控/状态机由 app 或 servo_mixer 负责,
 *     参数掉电保存请使用 bsp_flash 的 flash_store_save/load。
 */
#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "bsp_pwm.h"
#include "stdint.h"

#define SERVO_MOTOR_CNT 4  // 最多注册的舵机实例数

/* 调零安全窗口: 当前机械偏角(相对中位)必须在 ±该值内才允许调零 */
#define SERVO_ZERO_WINDOW_DEG 30.0f

/* 缺省信号参数 (未在 config 中指定时回落) */
#define SERVO_CENTER_US_DEFAULT 1500.0f
#define SERVO_HALF_US_DEFAULT 1000.0f    // 中位到行程端对应脉宽
#define SERVO_HALF_DEG_DEFAULT 139.5f    // 279° 行程的一半
#define SERVO_PULSE_MIN_DEFAULT 500.0f
#define SERVO_PULSE_MAX_DEFAULT 2500.0f

/**
 * @brief 舵机标定/初始化配置
 * @note  脉宽缺省以 1500us 为中位, 500/2500us 为硬限(越程会翻转)。
 */
typedef struct {
  PWM_Init_Config_s pwm;  // PWM 通道配置(周期建议 0.02s)

  float center_us;      // 机械 0° 对应脉宽 (如 1500)
  float half_us;        // 中位到行程端对应脉宽 (如 1000)
  float half_deg;       // 行程一半(机械角, 如 139.5)
  float pulse_min_us;   // 脉宽硬下限 (如 500)
  float pulse_max_us;   // 脉宽硬上限 (如 2500)

  float scale;          // 逻辑角->机械角 增益 (默认 1)
  float trim_deg;       // 零点/中立微调 (机械角)
  float limit_deg;      // 逻辑角对称限位 ±limit_deg; <=0 时默认 half_deg
  float rate_limit_dps; // 速率限幅 deg/s; <=0 表示不限速(立即到位)
  uint8_t reverse;      // 方向: 0 正常, 1 反向
  uint8_t zero_enable;  // 调零功能总开关: 0 禁用(不可调零), 1 启用
} Servo_Init_Config_s;

/* 舵机实例 */
typedef struct {
  PWMInstance *pwm;             // 底层 PWM 实例
  Servo_Init_Config_s cfg;      // 运行时配置(可在线改标定字段)
  float target_deg;             // 目标逻辑角
  float angle_deg;              // 当前逻辑角(速率限幅后)
  float pulse_us;               // 当前脉宽
  float last_time_s;            // 上次 ServoTask 时间戳
} ServoInstance;

/**
 * @brief 注册一路 PWM 舵机
 * @param config 初始化配置(见 Servo_Init_Config_s)
 * @return ServoInstance* 成功返回实例指针, 失败返回 NULL
 */
ServoInstance *ServoInit(Servo_Init_Config_s *config);

/**
 * @brief 设置逻辑角(deg)
 *        未启用限速时立即输出; 启用限速时只设目标, 由 ServoTask() 平滑推进
 */
void ServoSetAngle(ServoInstance *servo, float angle);

/**
 * @brief 直接设置脉宽(us), 自动按 pulse_min/max 限幅, 并反推逻辑角/机械角(标定/测试用)
 */
void ServoSetPulseUs(ServoInstance *servo, float pulse_us);

/**
 * @brief 舵机周期任务: 对启用速率限幅的实例把 angle 平滑逼近 target 并输出
 *        建议放入 app 周期任务(100Hz~1kHz)
 */
void ServoTask(void);

/**
 * @brief 把"当前位置"记为逻辑 0°(物理不动)
 * @note  仅当 zero_enable 启用, 且当前机械偏角落在中位 ±SERVO_ZERO_WINDOW_DEG 内才执行
 * @return 1 成功; 0 未启用/超窗口/无效
 */
uint8_t ServoZero(ServoInstance *servo);

/**
 * @brief 设置逻辑角对称限位 ±limit_deg (<=0 表示恢复默认 half_deg)
 */
void ServoSetLimit(ServoInstance *servo, float limit_deg);

/* 使能/失能: 启动/停止 PWM 输出(失能后舵机失去保持力) */
void ServoEnable(ServoInstance *servo);
void ServoDisable(ServoInstance *servo);

/* 查询 */
float ServoGetAngle(ServoInstance *servo);   // 当前逻辑角
float ServoGetTarget(ServoInstance *servo);  // 目标逻辑角
float ServoGetPulseUs(ServoInstance *servo); // 当前脉宽

#endif  // !SERVO_MOTOR_H
