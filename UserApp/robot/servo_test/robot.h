/**
 * @file robot.h
 * @author ai
 * @brief servo_motor 驱动测试 app (servo_test) 入口
 * @version 1.0
 * @date 2026-09-28
 *
 * @attention 用于在 C 板(GIMBAL_BOARD/STM32F407) 上单测 PTK7350 PWM 舵机驱动。
 *            命令通道: USB-CDC 虚拟串口(插板载 USB 到电脑 -> COM 口, 115200 8N1)。
 */
#pragma once

#include <stdint.h>

typedef struct {
  uint8_t placeholder;
} RobotInstance;

void RobotInit(void);
void RobotTask(void);
