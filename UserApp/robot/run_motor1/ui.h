/*
 * ui.h — run_motor1 的 UI 占位(os_task.c 会无条件 #include "ui.h")
 *
 * 台架测试不需要裁判系统 UI; GIMBAL_BOARD 下 StartUITASK 走空循环, 不引用这两个函数。
 * 这里仅提供占位声明, 使 os_task.c 在 ONE_BOARD/CHASSIS_BOARD 分支下也能编过。
 */
#ifndef RUN_MOTOR1_UI_H
#define RUN_MOTOR1_UI_H

#include "robot.h"

void MyUIInit(RobotInstance *robot);
void UITask(RobotInstance *robot);

#endif  // RUN_MOTOR1_UI_H
