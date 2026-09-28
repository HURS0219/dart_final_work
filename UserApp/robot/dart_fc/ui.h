/*
 * ui.h — 制导飞镖飞控 (dart_fc) 的 UI 占位
 *
 * os_task.c 会无条件 #include "ui.h"。飞镖的可视化用 ST7735 (见 st7735_ui),
 * 不用裁判系统 UI, 故这里只提供占位声明。
 */
#ifndef DART_FC_UI_H
#define DART_FC_UI_H

#include "robot.h"

void MyUIInit(RobotInstance* robot);
void UITask(RobotInstance* robot);

#endif  // DART_FC_UI_H
