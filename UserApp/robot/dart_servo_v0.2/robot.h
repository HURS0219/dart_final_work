/*
 * robot.h — 制导飞镖舵机子系统 v0.2 应用入口
 */
#ifndef DART_V2_ROBOT_H
#define DART_V2_ROBOT_H

#include <stdint.h>

typedef struct {
  uint8_t placeholder;
} RobotInstance;

extern RobotInstance *robot;

void RobotInit(void);
void RobotTask(void);

#endif /* DART_V2_ROBOT_H */
