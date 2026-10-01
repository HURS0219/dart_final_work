/*
 * motor.h — 电机 app: 统一管理 4 路 CAN 电机(拉簧 A/B + 丝杆 + Yaw)
 * =============================================================================
 * 【解耦说明】
 *   本 app 是**唯一**直接调用 DJImotor 接口的地方(不建 module 层, 直接用 Modules/motor/DJImotor)。
 *   对外只通过话题通信:
 *     订阅 "motor_cmd" (Launcher_MotorCmd_s) —— 逐路模式 + 目标(输出侧 deg / rpm)
 *     发布 "motor_fb"  (Launcher_MotorFb_s)  —— 逐路反馈(在线/角度/到位/保持中等)
 *   app/cmd 与 app/fsm 都不 include 本头文件的业务内容, 只见话题。
 *
 * 【角度制】对外一律"输出侧 deg"; 内部换算到转子侧:
 *     转子目标 = zero + sign * 输出侧角 * ratio
 *     输出侧角 = sign * (total_angle - zero) / ratio
 *
 * 【安全】拉簧 A/B 非自锁: 急停绝不允许直接 DJIMotorStop()(会让弹簧瞬间释放)。
 *   逐路急停策略见 launcher_cfg.h 的 LAUNCH_S?_ESTOP_MODE。
 * =============================================================================
 */
#pragma once

#include "robot_def.h"

/** @brief 初始化 4 路 CAN 电机(按 launcher_cfg.h 的逐路参数), 注册话题 */
void Motor_Init(void);

/** @brief 周期任务: 取指令 -> 逐路控制(含急停策略) -> 发布反馈 */
void Motor_Task(void);

/** @brief 健康状态(Monitor 读取) */
const Launcher_AppStatus_s *Motor_GetStatus(void);

/* ---------------- 只读观测接口(供 robot.c 打 RTT 遥测) ----------------
 * 说明: 仅从 motor_fb 快照取字段, 便于 SWD 调试时在 RTT 看到机构状态
 *       (本工程无网页可用时)。刻意不含任何控制接口, 不构成 app 间控制耦合。 */
float MotorFbAngle(int slot);  /* 输出侧角度(deg) */
float MotorFbTarget(int slot); /* 当前目标(deg) */
float MotorFbRpm(int slot);    /* 输出转速(rpm) */
uint8_t MotorFbOnline(int slot);
uint8_t MotorFbAtTarget(int slot);
uint8_t MotorFbHolding(int slot);
