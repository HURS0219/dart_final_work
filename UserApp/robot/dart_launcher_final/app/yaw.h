/*
 * yaw.h — 自瞄 yaw 轴 app
 * =============================================================================
 * 【职责】消费视觉误差 -> 应用层 PI 算出目标转速 -> 用速度环驱动 yaw 电机。
 *   订阅 "aim_cmd" (Launcher_Aim_s)     —— 视觉像素误差
 *   订阅 "yaw_cmd" (Launcher_YawCmd_s)  —— 模式/手动目标/上限/急停
 *   发布 "yaw_fb"  (Launcher_YawFb_s)   —— 转速/误差/状态
 *
 * 【分层】为保持"每个机构一个 app"的解耦, yaw 自己调用 DJImotor 接口。
 *   之所以不并入 app/motor: yaw 的控制律(视觉 PI)与拉簧的保持/归零完全不同,
 *   混在一起会让职责不清。
 *
 * 【两层控制律, 勿混淆】
 *   像素误差 --[应用层 PI: LAUNCH_YAW_AIM_KP/KI]--> 目标 rpm
 *            --[电机速度环: LAUNCH_YAW_SPEED_KP/KI]--> 电流
 *  前者是本 app 实现的, 后者由 DJImotor 模块执行。
 * =============================================================================
 */
#pragma once

#include "robot_def.h"

/** @brief 初始化 yaw 电机(按 launcher_cfg.h), 注册话题 */
void Yaw_Init(void);

/** @brief 周期任务: 取视觉+指令 -> 算目标转速 -> 驱动 -> 发布反馈 */
void Yaw_Task(void);

/** @brief 健康状态(Monitor 读取) */
const Launcher_AppStatus_s *Yaw_GetStatus(void);
