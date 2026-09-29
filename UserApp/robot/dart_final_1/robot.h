/*
 * robot.h — 制导飞镖整机飞控 (dart_final) 应用层入口
 * =============================================================================
 * 分层(严格解耦, 各 app 平行经 message_center 通信):
 *   Module  : IMU(ins_task) / 制导(png_ai) / 四舵面(servo_mix_ai) / 单路舵机(servo_motor)
 *             / 视觉通信(bsp_usart) / 日志(bsp_log)
 *   App     : imu / vision / guidance / fin (见 app/), 由 robot.c 编排
 *
 * 唯一对外入口(由 UserApp/os_task.c 调用):
 *   RobotInit()  —— 上电初始化(RTOS 起来后调用)
 *   RobotTask()  —— 周期任务, ~1kHz
 * =============================================================================
 */
#pragma once

#ifndef DART_FINAL_ROBOT_H
#define DART_FINAL_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 整机实例(仅保存本 app 级运行状态; 各 app 内部状态各自管理)
 */
typedef struct {
  uint32_t DWT_CNT;  // DWT 计时句柄(用于 DWT_GetDeltaT)
  float dt;          // 本周期时间间隔(秒)
} RobotInstance;

extern RobotInstance *robot;  // 全局唯一实例指针(仅 robot.c/本 app 使用)

/** @brief 上电初始化: 建话题、初始化各 app、注册看门狗 */
void RobotInit(void);

/** @brief 周期任务(~1kHz): 状态机 + 各 app 任务 + Monitor */
void RobotTask(void);

#endif  // DART_FINAL_ROBOT_H
