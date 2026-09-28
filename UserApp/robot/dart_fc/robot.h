/*
 * robot.h — 制导飞镖飞控 (dart_fc) 应用层入口
 *
 * 本兵种即"飞镖本体飞控": IMU 姿态 + 4 舵面, 运行比例导引/PID/模式切换。
 * 硬件相关(IMU/舵机/视觉通信/舵机解耦)都在 Module 层。
 */
#pragma once

#ifndef DART_FC_ROBOT_H
#define DART_FC_ROBOT_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint8_t placeholder;
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif  // DART_FC_ROBOT_H
