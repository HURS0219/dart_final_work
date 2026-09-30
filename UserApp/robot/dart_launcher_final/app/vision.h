/*
 * vision.h — 视觉 app: 绿光中心坐标(PC/Jetson -> STM32)
 * =============================================================================
 * 【现状】坐标由上位机(现阶段是 PC 的 hik_vision.py, 将来是 Jetson)经 ESP32
 *   透传到 STM32, 走 C,x,center 命令注入(见 app/link)。
 *   本 app 负责: 接收注入 -> 算像素误差 -> 发布 "aim_cmd" 给 app/yaw。
 *
 * 【解耦】本 app 不知道 yaw 怎么转, yaw 也不知道坐标从哪来 —— 只见话题。
 *   将来把坐标来源换成 OpenMV/串口直连, 只需改本文件, yaw 一行不动。
 *
 * 【超时】超过 LAUNCH_VIS_TIMEOUT_MS 未收到坐标 -> aim.ok=0(目标丢失),
 *   yaw 侧据此停转(yaw 自己决定降级动作)。
 * =============================================================================
 */
#pragma once

#include "robot_def.h"

/** @brief 初始化(注册话题), 并订阅 link 的注入指令 */
void Vision_Init(void);

/** @brief 周期任务: 处理注入 -> 发布 aim_cmd -> 判超时 */
void Vision_Task(void);

/** @brief 健康状态(Monitor 读取) */
const Launcher_AppStatus_s *Vision_GetStatus(void);
