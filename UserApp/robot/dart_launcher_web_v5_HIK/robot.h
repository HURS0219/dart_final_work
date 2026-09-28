/*
 * robot.h — 制导飞镖发射架 · 网页控制版 v3 入口
 */
#pragma once

#ifndef DART_LAUNCHER_WEB_V3_ROBOT_H
#define DART_LAUNCHER_WEB_V3_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  void* motors;
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif  // DART_LAUNCHER_WEB_V3_ROBOT_H
