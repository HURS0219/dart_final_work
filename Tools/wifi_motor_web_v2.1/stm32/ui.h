/*
 * ui.h — WiFi GM6020 应用 UI 占位 (os_task.c 会无条件 #include "ui.h")
 */
#ifndef WIFI_GM6020_UI_H
#define WIFI_GM6020_UI_H

#include "robot.h"

void MyUIInit(RobotInstance* robot);
void UITask(RobotInstance* robot);

#endif  // WIFI_GM6020_UI_H
