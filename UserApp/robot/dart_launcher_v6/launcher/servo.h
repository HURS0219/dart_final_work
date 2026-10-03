/*
 * launcher/servo.h — 扳机舵机组件 (PWM1 / TIM1_CH1)
 * =============================================================================
 * 只暴露 Init/Task + cmd。ctrl 下发请求, ServoTask 应用并输出。
 * 逻辑角 = 原始角(-零点偏移); 脉宽按 DART_SERVO_* 规格线性映射。
 * =============================================================================
 */
#pragma once

#include <stdint.h>

#include "robot_def.h"

typedef struct {
  uint8_t set_std;  /* 单次: 设标准位 */
  float std_deg;
  uint8_t set_prep; /* 单次: 设预备位 */
  float prep_deg;
  uint8_t go_set; /* 单次: 去某位 */
  uint8_t go;     /* Launcher_ServoPos_e */
  uint8_t set_deg; /* 单次: 直接给逻辑角 */
  float deg;
  uint8_t zero; /* 单次: 当前位置记为 0° */
} Servo_Ctrl_Cmd_s;

typedef struct {
  Servo_Ctrl_Cmd_s cmd;
  float std_deg;    /* 当前标准位设定 */
  float prep_deg;   /* 当前预备位设定 */
  float cur_deg;    /* 逻辑当前角 */
  float offset_deg; /* 零点偏移 */
  uint8_t state;    /* Launcher_ServoPos_e */
} ServoInstance;

ServoInstance* LauncherServoInit(void);
void LauncherServoTask(void);
