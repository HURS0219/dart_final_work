/*
 * ui.h — dart_final 的 UI 占位
 *
 * os_task.c 会无条件 #include "ui.h"; 飞镖不用裁判系统 UI。这里仅提供占位声明,
 * 使 os_task.c 在 ONE_BOARD/CHASSIS_BOARD 分支下也能编过(当前 GIMBAL_BOARD 下不引用)。
 */
#ifndef DART_FINAL_UI_H
#define DART_FINAL_UI_H

#include "robot.h"

void MyUIInit(RobotInstance *robot);
void UITask(RobotInstance *robot);

#endif  // DART_FINAL_UI_H
