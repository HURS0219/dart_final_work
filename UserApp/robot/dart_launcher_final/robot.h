/*
 * robot.h — 制导飞镖发射架 整机飞控 (dart_launcher_final) 应用层入口
 * =============================================================================
 * 分层(严格解耦, 各 app 平行经 message_center 通信):
 *   Module  : DJImotor(四路 CAN 电机) / servo_motor(PWM 舵机) / message_center
 *             / bsp_usart(上位机链路) / bsp_log(RTT)
 *   App     : link / cmd / fsm / motor / trigger / vision / yaw (见 app/)
 *   编排    : robot.c 唯一入口, 只做初始化 + 周期调度 + Monitor + 顶层状态机
 *
 * 唯一对外入口(由 UserApp/os_task.c 调用):
 *   RobotInit()  —— 上电初始化(RTOS 起来后调用)
 *   RobotTask()  —— 周期任务, ~1kHz
 * =============================================================================
 */
#pragma once

#ifndef DART_LAUNCHER_FINAL_ROBOT_H
#define DART_LAUNCHER_FINAL_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 整机实例(仅保存本 app 级运行状态; 各 app 内部状态各自管理)
 */
typedef struct {
  uint32_t DWT_CNT; /* DWT 计时句柄(用于 DWT_GetDeltaT) */
  float dt;         /* 本周期时间间隔(秒) */
} RobotInstance;

extern RobotInstance *robot; /* 全局唯一实例指针(仅 robot.c/本 app 使用) */

/** @brief 上电初始化: 建话题、初始化各 app */
void RobotInit(void);

/** @brief 周期任务(~1kHz): 各 app 任务 + Monitor + 顶层状态机 */
void RobotTask(void);

#endif /* DART_LAUNCHER_FINAL_ROBOT_H */
