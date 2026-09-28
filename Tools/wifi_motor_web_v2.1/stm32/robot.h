/*
 * robot.h — WiFi 手机控制 GM6020 (wifi_gm6020) v0.1 应用层入口
 *
 * 链路: 手机网页 --WiFi--> ESP32 --UART(huart6)--> STM32 --CAN--> GM6020
 * 模式: 1) 速度模式(滑块调转速)  2) 角度模式(滑块转到固定角度)
 */
#pragma once

#ifndef WIFI_GM6020_ROBOT_H
#define WIFI_GM6020_ROBOT_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  void* motor;  // 指向 WifiMotorInstance
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif  // WIFI_GM6020_ROBOT_H
