/**
 * @file servo_motor.h
 * @author ai
 * @brief PWM 舵机(PTK7350 等)底层驱动: 逻辑角 <-> 脉宽 标定 + 速率限幅 + 调零
 * @version 2.0
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention
 *  1. 信号: 50Hz(周期 20ms), 高电平脉宽 pulse_min_us ~ pulse_max_us 线性对应机械偏角
 *     -half_deg ~ +half_deg; 中位 center_us(通常 1500us) 对应机械 0°。
 *     机械偏角 applied = 逻辑角*scale + trim_deg (reverse 时取负), 再换算脉宽:
 *         pulse = center_us + applied * (half_us / half_deg)
 *  2. limit_deg 为"逻辑角"对称限位(相对逻辑 0°, 即调零后的零点);
 *     pulse_min/max_us 为"脉宽"硬限幅(越程可能导致舵机翻转/损坏)。
 *  3. 调零(ServoZero)把当前位置记为逻辑 0°, 需 zero_enable 使能且当前偏角在窗口内。
 *  4. 本模块只做"单路驱动 + 标定 + 速率限幅 + 调零";
 *     四舵面混控/状态机由应用层或 servo_mixer 负责;
 *     参数掉电保存请使用 bsp_flash 的 flash_store_save/load。
 */
#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "bsp_pwm.h"
#include "stdint.h"

#define SERVO_MOTOR_CNT 4  // 最多同时注册的舵机实例数

/* 调零安全窗口: 当前机械偏角(相对中位)必须在 ±该值(deg)内才允许调零 */
#define SERVO_ZERO_WINDOW_DEG 30.0f

/* 缺省信号参数: 未在 config 中显式指定时自动回落 */
#define SERVO_CENTER_US_DEFAULT 1500.0f  // 机械 0° 对应脉宽
#define SERVO_HALF_US_DEFAULT 1000.0f    // 中位到行程端对应脉宽
#define SERVO_HALF_DEG_DEFAULT 139.5f    // 行程一半(279° 总行程)
#define SERVO_PULSE_MIN_DEFAULT 500.0f   // 脉宽硬下限
#define SERVO_PULSE_MAX_DEFAULT 2500.0f  // 脉宽硬上限

/**
 * @brief 舵机标定/初始化配置
 * @note  所有角度单位为 deg, 脉宽单位为 us; 未填写的字段会被 ServoInit() 回落为默认值。
 */
typedef struct {
  PWM_Init_Config_s pwm;  // 底层 PWM 通道配置(htim/channel/period; period 建议 0.02s)

  float center_us;      // 机械 0° 对应脉宽 (如 1500)
  float half_us;        // 中位到行程端对应脉宽 (如 1000)
  float half_deg;       // 行程一半(机械角, 如 139.5)
  float pulse_min_us;   // 脉宽硬下限 (如 500)
  float pulse_max_us;   // 脉宽硬上限 (如 2500)

  float scale;          // 逻辑角 -> 机械角 增益 (默认 1; 用于校准每路的行程比例)
  float trim_deg;       // 零点/中立微调(机械角, 默认 0; 调零会改写它)
  float limit_deg;      // 逻辑角对称限位 ±limit_deg; <=0 时默认 half_deg
  float rate_limit_dps; // 速率限幅 deg/s; <=0 表示不限速(立即到位)
  uint8_t reverse;      // 方向: 0 正常 / 1 反向
  uint8_t zero_enable;  // 调零功能总开关: 0 禁用(不可调零), 1 启用
} Servo_Init_Config_s;

/**
 * @brief 舵机运行时实例
 * @note  cfg 为运行时可改配置(标定字段可在线修改), 其余为状态量。
 */
typedef struct {
  PWMInstance *pwm;         // 底层 PWM 实例(由 PWMRegister 得到)
  Servo_Init_Config_s cfg;  // 运行时配置
  float target_deg;         // 目标逻辑角(限速时逐步逼近)
  float angle_deg;          // 当前逻辑角(限速后)
  float pulse_us;           // 当前脉宽
  float last_time_s;        // 上次 ServoTask 的时间戳(用于算 dt)
} ServoInstance;

/**
 * @brief 注册一路 PWM 舵机
 * @param config 初始化配置(见 Servo_Init_Config_s)
 * @return ServoInstance* 成功返回实例指针, 失败(参数为空/实例已满)返回 NULL
 * @note  注册后立即输出中位(逻辑 0°)
 */
ServoInstance *ServoInit(Servo_Init_Config_s *config);

/**
 * @brief 设置逻辑角(deg)
 * @note  先按 ±limit_deg 限位; 未启用限速时立即输出, 启用后由 ServoTask() 平滑推进
 */
void ServoSetAngle(ServoInstance *servo, float angle);

/**
 * @brief 直接设置脉宽(us), 自动按 pulse_min/max 限幅
 * @note  绕过逻辑角映射, 用于标定/开环测试; 会反推逻辑角保持状态一致
 */
void ServoSetPulseUs(ServoInstance *servo, float pulse_us);

/**
 * @brief 舵机周期任务: 对启用速率限幅的实例把 angle 平滑逼近 target 并输出
 * @note  建议放入 app 周期任务(100Hz~1kHz)
 */
void ServoTask(void);

/**
 * @brief 把"当前位置"记为逻辑 0°(调零; 物理不动)
 * @note  仅当 zero_enable 启用, 且当前机械偏角落在中位 ±SERVO_ZERO_WINDOW_DEG 内才执行
 * @return 1 成功; 0 未启用 / 超窗口 / 无效
 */
uint8_t ServoZero(ServoInstance *servo);

/**
 * @brief 设置逻辑角对称限位 ±limit_deg (<=0 表示恢复默认 half_deg)
 */
void ServoSetLimit(ServoInstance *servo, float limit_deg);

/* 使能/失能: 启动/停止 PWM 输出(失能后舵机失去保持力) */
void ServoEnable(ServoInstance *servo);
void ServoDisable(ServoInstance *servo);

/* ---------------- 查询 ---------------- */
float ServoGetAngle(ServoInstance *servo);   // 当前逻辑角
float ServoGetTarget(ServoInstance *servo);  // 目标逻辑角
float ServoGetPulseUs(ServoInstance *servo); // 当前脉宽

#endif  // !SERVO_MOTOR_H
