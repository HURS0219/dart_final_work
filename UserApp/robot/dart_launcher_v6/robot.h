/*
 * robot.h — dart_launcher_v6 整机抽象(组合根)
 * =============================================================================
 * RobotInstance 持有各组件实例(容器风格, 参考范例); ctrl 通过 robot->
 * 访问组件并填 ctrl_cmd。os_task.c 只调用 RobotInit()/RobotTask()。
 * =============================================================================
 */
#pragma once

#include <stdint.h>

#include "motor.h"
#include "servo.h"
#include "vision.h"

typedef struct {
  MotorInstance* motor;
  ServoInstance* servo;
  VisionInstance* vision;

  uint32_t DWT_CNT;
  float dt;
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);
