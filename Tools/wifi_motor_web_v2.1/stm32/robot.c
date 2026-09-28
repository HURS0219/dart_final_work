/*
 * robot.c — WiFi 手机控制 多电机 (wifi_gm6020) 入口
 *
 * 链路: 手机网页 --WiFi--> ESP32 --UART--> STM32 --CAN--> 多个电机
 * 反馈: F,<mode>,<count>[,<type>,<id>,<rpm>]*count,<tune_type>,<10个参数...>,<limit>
 */
#include "robot.h"

#include <stdio.h>

#include "main.h"
#include "simple_motor.h"
#include "user_lib.h"
#include "wifi_link.h"
#include "wifi_motor.h"
#include "wifi_store.h"

RobotInstance* robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  WifiMotorInit();
  WifiLinkInit();
}

void RobotTask(void) {
  WifiMotorScanTask();
  WifiMotorTask();

  static uint32_t last_fb = 0;
  static char fb_buf[200];
  if (HAL_GetTick() - last_fb >= 100u) {
    last_fb = HAL_GetTick();
    int len = snprintf(fb_buf, sizeof(fb_buf), "F,%d,%d", (int)wifi_motor.mode, (int)g_motor_num);
    for (int i = 0; i < SIMPLE_MOTOR_MAX; i++) {
      if (!g_motors[i].used) continue;
      MotorParamSet_s* p = &g_motor_params[i];
      float ratio = (p->gear_ratio > 0.01f) ? p->gear_ratio : 1.0f;
      int rpm_out = (int)((float)g_motors[i].rpm / ratio);
      len += snprintf(fb_buf + len, sizeof(fb_buf) - len, ",%d,%d,%d,%d",
                      i, (int)g_motors[i].type, (int)g_motors[i].id, rpm_out);
    }
    len += snprintf(fb_buf + len, sizeof(fb_buf) - len,
                    ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    g_sel,
                    (int)(g_speed_ff * 100.0f), (int)(g_speed_kp * 100.0f),
                    (int)(g_speed_ki * 100.0f), (int)(g_speed_i_limit * 100.0f),
                    (int)(g_angle_kp * 100.0f), (int)(g_angle_kd * 100.0f),
                    (int)(g_angle_slew * 100.0f), (int)(g_gear_ratio * 100.0f),
                    (int)(g_angle_deadband * 100.0f), (int)(g_angle_speed_max * 100.0f),
                    g_max_value);
    fb_buf[len++] = '\n';
    fb_buf[len] = 0;
    WifiLinkSend(fb_buf);
  }

  WifiStoreTask();
}
