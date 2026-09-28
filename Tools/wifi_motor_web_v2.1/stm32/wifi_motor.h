/*
 * wifi_motor.h — 多电机同时控制 (基于 simple_motor 多机总线)
 *
 * 总线上可挂多个电机, 同一模式/目标同时驱动全部; 速度/角度以“输出轴”为单位。
 * 每个电机有自己独立的一套参数(FF/Kp/Ki/减速比/死区/限速...)和零点。
 */
#ifndef WIFI_GM6020_MOTOR_H
#define WIFI_GM6020_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "simple_motor.h"

typedef enum {
  WIFI_MODE_STOP = 0,
  WIFI_MODE_SPEED = 1,
  WIFI_MODE_ANGLE = 2
} WifiMotorMode_e;

/* 单个电机的一套参数 */
typedef struct {
  float speed_ff;
  float speed_kp;
  float speed_ki;
  float speed_i_limit;
  float angle_kp;
  float angle_kd;
  float angle_slew;
  float gear_ratio;
  float angle_deadband;
  float angle_speed_max;
  int32_t max_value;
} MotorParamSet_s;

typedef struct {
  WifiMotorMode_e mode;
  float speed_rpm;  // 输出轴 RPM
  float angle_deg;  // 输出轴 度
  uint32_t last_cmd_tick;
} WifiMotorInstance;

extern WifiMotorInstance wifi_motor;

/* 每个电机槽位一套参数 (下标 = g_motors 下标) */
extern MotorParamSet_s g_motor_params[SIMPLE_MOTOR_MAX];
extern int g_sel;  // 当前调参的电机槽位, -1=无

/* 当前活动参数 (g_motor_params[g_sel] 的镜像) */
extern float g_speed_ff;
extern float g_speed_kp;
extern float g_speed_ki;
extern float g_speed_i_limit;
extern float g_angle_kp;
extern float g_angle_kd;
extern float g_angle_slew;
extern float g_gear_ratio;
extern float g_angle_deadband;
extern float g_angle_speed_max;
extern int g_max_value;

void WifiMotorInit(void);
void WifiMotorTask(void);
void WifiMotorSetCommand(int mode, int value);
void WifiMotorSetParam(int id, int value);
void WifiMotorSetSelect(int idx);
void WifiMotorZeroHere(void);
void WifiMotorAddMotor(int type, int id);
void WifiMotorClearMotors(void);

void WifiMotorScanRequest(void);
void WifiMotorScanTask(void);
int WifiMotorIsScanning(void);

void WifiMotorStoreActive(void);
void WifiMotorLoadActive(void);

#endif  // WIFI_GM6020_MOTOR_H
