/*
 * trigger.h — 舵机 app: 扳机/释放舵机(复用 Modules/motor/servo_motor)
 * =============================================================================
 * 【解耦】不直接操作 PWM, 而是复用 servo_motor module(标定/限速/调零/方向)。
 *   订阅 "servo_cmd" (Launcher_ServoCmd_s)
 *   发布 "servo_fb"  (Launcher_ServoFb_s)
 *
 * 【为什么用 module 而不是裸调 bsp_pwm】
 *   servo_motor 已封装"逻辑角 -> 脉宽"标定链(scale/trim/reverse/limit/rate_limit),
 *   以及带安全窗口的调零, 比手写脉宽换算更安全也更省事。
 *
 * 【注意】ServoTask() 是**全局**推进函数(所有舵机实例一起), 必须每周期调用,
 *   否则速率限幅不生效(舵机不会平滑移动)。
 * =============================================================================
 */
#pragma once

#include "robot_def.h"

/** @brief 初始化舵机(注册 PWM + servo_motor 实例), 注册话题 */
void Trigger_Init(void);

/** @brief 周期任务: 取指令 -> 设目标角 -> 推进 ServoTask -> 发布反馈 */
void Trigger_Task(void);

/** @brief 健康状态(Monitor 读取) */
const Launcher_AppStatus_s *Trigger_GetStatus(void);
