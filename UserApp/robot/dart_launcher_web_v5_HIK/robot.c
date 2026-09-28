/*
 * robot.c — 制导飞镖发射架 · 网页控制版 v3 入口 (电机复用 DJImotor 库)
 */
#include "robot.h"

#include "dart_fsm.h"
#include "dart_link.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "dart_store.h"
#include "dart_vision.h"
#include "bsp_dwt.h"
#include "main.h"
#include "user_lib.h"

RobotInstance* robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  DWT_Init(168);  // 确保 DWT 计数 (DJI PID 依赖 DWT dt)
  MotorsInit();   // DJIMotorInit x4
  DartServoInit();
  DartVisionInit();
  DartFsmInit();
  DartStoreInit();
  DartLinkInit();
}

void RobotTask(void) {
  MotorsTask();   // 每周期算 PID 参考 (发送由基类 motor_task 负责)
  DartFsmTask();
  DartVisionTask();
  DartLinkTask();
  DartStoreTask();
}
