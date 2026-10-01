/*
 * robot.h — run_motor1 应用层入口
 * =============================================================================
 * os_task.c 会无条件 #include "robot.h" 并调用 RobotInit()/RobotTask():
 *   RobotInit() —— RTOS 起来后调用一次(初始化电机 module、注册话题)
 *   RobotTask() —— 周期任务(~1kHz): 生成角度目标 -> 下发 -> 发布反馈
 *
 * 本 app 只做一件事: 通过 motor module(DJIMotor)让一个 M3508 按角度环连续旋转。
 * =============================================================================
 */
#ifndef RUN_MOTOR1_ROBOT_H
#define RUN_MOTOR1_ROBOT_H

#include <stdint.h>

/* 整机实例(仅保存计时状态; 电机状态在本文件内部管理) */
typedef struct {
  uint32_t DWT_CNT;  /* DWT 计时句柄(用于 DWT_GetDeltaT) */
  float dt;          /* 本周期时间间隔(秒) */
} RobotInstance;

extern RobotInstance *robot;  /* 全局唯一实例指针(仅本 app 使用) */

void RobotInit(void);  /* 上电初始化 */
void RobotTask(void);  /* 周期任务(~1kHz) */

#endif  // RUN_MOTOR1_ROBOT_H
