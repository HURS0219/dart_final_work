/*
 * cmd.h — 大脑 app: 操作员指令 -> 各机构目标
 * =============================================================================
 * 【职责】整个发射架的"大脑"(见 application.md 对 robot_cmd 的定位):
 *   订阅 "launch_cmd"  (Launcher_Cmd_s)     —— link 解析后的操作员指令
 *   订阅 "motor_fb"    (Launcher_MotorFb_s) —— 电机反馈
 *   订阅 "servo_fb"    (Launcher_ServoFb_s) —— 舵机反馈
 *   订阅 "yaw_fb"      (Launcher_YawFb_s)   —— yaw 反馈
 *   订阅 "fsm_fb"      (Launcher_FsmFb_s)   —— 时序反馈
 *   发布 "motor_cmd"   —— 电机目标(角度制, 逐路)
 *   发布 "servo_cmd"   —— 舵机目标位
 *   发布 "yaw_cmd"     —— yaw 模式/目标
 *   发布 "fsm_cmd"     —— 时序触发
 *   发布 "launch_state"—— 整机状态(供 link 组遥测帧)
 *
 * 【本 app 不碰任何硬件】不 include dji_motor / servo_motor, 只见话题。
 *   硬件细节全部由 app/motor 与 app/trigger 承担。
 *
 * 【失联降级】超过 LAUNCH_LINK_TIMEOUT_MS 未收到上位机指令:
 *   停止自动流程 + 拉簧保持(不卸力!) + 舵机回标准位。绝不主动卸掉拉簧。
 * =============================================================================
 */
#pragma once

#include "robot_def.h"

/** @brief 初始化(注册话题) */
void Cmd_Init(void);

/** @brief 周期任务: 翻译指令 -> 发布目标 */
void Cmd_Task(void);

/** @brief 健康状态(Monitor 读取) */
const Launcher_AppStatus_s *Cmd_GetStatus(void);
