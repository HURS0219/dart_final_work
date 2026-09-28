/**
 * @file servo_mix_ai.h
 * @author ai
 * @brief 四舵面混控模块: X 型 4 舵面 —— 混控(MIX) + 四路纯手动(MANUAL), 上电回中
 * @version 1.0
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention
 *  1. 只做两件事: 把 (pitch,yaw,roll) 用解耦矩阵分到 4 路舵面(MIX), 或 4 路直接给逻辑角(MANUAL);
 *     上电默认 MIX + 零指令 => 各舵面停在逻辑 0°(中位), 即“上电回中”。
 *  2. 本模块**直接持有并驱动 4 路 servo_motor 实例**(ServoInit/ServoSetAngle/ServoTask)。
 *     @note 后续若需进一步解耦, 可把“驱动”下沉: 本模块只输出 4 路目标角, 由 app 调 ServoSetAngle。
 *           当前为省事直接驱动, 代码已按“只依赖 servo_motor 公共接口”编写, 便于改造。
 *  3. 纯计算矩阵(不依赖 servo_mixer); 参数集中在 servo_mix_ai_cfg.h(现场调参入口)。
 *  4. 单路标定/限速/脉宽/调零 均由 servo_motor 负责, 本模块不重复实现。
 */
#ifndef SERVO_MIX_AI_H
#define SERVO_MIX_AI_H

#include "servo_mix_ai_cfg.h"  // 取得 SERVO_MIX_N / 默认模式等
#include "servo_motor.h"
#include "stdint.h"

/**
 * @brief 运行模式 (仅两种)
 */
typedef enum {
  SERVO_MIX_MODE_MIX = 0,     // 混控: (pitch,yaw,roll) -> 矩阵 -> 4 路逻辑角
  SERVO_MIX_MODE_MANUAL = 1,  // 手动: 每路单独直接给逻辑角(调试/单路校验)
} ServoMixMode_e;

/**
 * @brief 初始化: 依 servo_mix_ai_cfg.h 注册 4 路舵机并输出中位
 * @note  上电回中即由此保证(默认 MIX + 零指令)。重复调用无副作用。
 */
void ServoMixInit(void);

/**
 * @brief 切换模式(切入 MIX 时沿用当前 cmd; 切入 MANUAL 时沿用当前 manual)
 */
void ServoMixSetMode(ServoMixMode_e mode);

/**
 * @brief 读取当前模式
 */
ServoMixMode_e ServoMixGetMode(void);

/**
 * @brief 设置混控指令(MIX 模式使用), 各分量归一化 -1..1
 * @param pitch 俯仰指令, -1..1
 * @param yaw   偏航指令, -1..1
 * @param roll  滚转指令, -1..1
 * @note  内部会限幅到 ±1; 仅在 MIX 模式生效。
 */
void ServoMixSetCmd(float pitch, float yaw, float roll);

/**
 * @brief 设置某一路的手动逻辑角(MANUAL 模式使用)
 * @param ch  舵面编号 0..SERVO_MIX_N-1
 * @param deg 逻辑角(deg); 会被夹到该路 ±limit_deg(见 cfg)
 */
void ServoMixSetManual(uint8_t ch, float deg);

/**
 * @brief 一次设置 4 路手动逻辑角(MANUAL 模式使用)
 */
void ServoMixSetManualAll(const float deg[SERVO_MIX_N]);

/**
 * @brief 周期任务: 按当前模式计算 4 路目标角 -> ServoSetAngle(), 并推进速率限幅(ServoTask)
 * @note  建议放入 app 周期任务(100Hz~1kHz); app 周期只需调这一个。
 */
void ServoMixTask(void);

/**
 * @brief 对某一路“把当前位置记为逻辑 0°”(转调 servo_motor 的调零, 受窗口/开关限制)
 * @return 1 成功; 0 拒绝(未初始化 / 超窗口 / 该路未启用调零)
 */
uint8_t ServoMixZero(uint8_t ch);

/** @brief 使能/失能全部 4 路 PWM 输出(失能后舵机失去保持力) */
void ServoMixEnable(void);
void ServoMixDisable(void);

/**
 * @brief 读当前 4 路“目标逻辑角”(deg)
 */
void ServoMixGetDeflDeg(float out[SERVO_MIX_N]);

/**
 * @brief 读当前 4 路“实际脉宽”(us)
 */
void ServoMixGetPulseUs(float out[SERVO_MIX_N]);

#endif /* SERVO_MIX_AI_H */
