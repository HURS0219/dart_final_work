/*
 * ui.h — v0.2 UI 占位 (GIMBAL_BOARD 下 UI 任务不编译调用)
 */
#ifndef DART_V2_UI_H
#define DART_V2_UI_H

#include "robot.h"

void MyUIInit(RobotInstance *robot);
void UITask(RobotInstance *robot);

#endif /* DART_V2_UI_H */
