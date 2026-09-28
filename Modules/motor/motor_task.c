#include "motor_task.h"

#include "dji_motor.h"

volatile uint32_t g_motor_loops;

void MotorControlTask() {
  g_motor_loops++;

  /* 只负责 DJI 电机控制帧的发送。
   * 飞镖制导(DartAppTask) / 显示(ST7735_UI_Task) 属于应用层,
   * 已移到 UserApp/robot/dart_fc/robot.c 的 RobotTask() 中调用。 */
  DJIMotorTask();
}
