/*
 * robot.c — 制导飞镖飞控 (dart_fc) 应用层
 *
 * 分层:
 *   Module 层: IMU(ins_task) / 舵机(servo_motor) / 舵机解耦(servo_mixer) / 视觉通信
 *   App 层(本目录): dart_app(编排) + dart_control/dart_guidance/dart_pid/dart_mode/dart_attitude
 *
 * RobotTask 以 ~1kHz 调用 DartAppTask(内部再分频到 100Hz 控制) 与显示任务。
 */

#include "robot.h"
#include "dart_app.h"
#include "st7735_ui.h"
#include "user_lib.h"

RobotInstance* robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  DartAppInit();
  ST7735_UI_Init();
}

void RobotTask(void) {
  DartAppTask();       /* 姿态 + 制导 + 控制 + 舵面输出 */
  ST7735_UI_Task();    /* ST7735 可视化 (按键切屏) */
}
