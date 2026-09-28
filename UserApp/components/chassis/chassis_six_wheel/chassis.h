/**
* @file    chassis.h
* @author  Shiyu Li
* @date    2026/3/28
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   wheel_leg->prostrate
*/
#pragma once

#include <stdint.h>

#include "ins_task.h"
#include "super_cap.h"
#include "dji_motor.h"
#include "dmmotor.h"


// 中科大的功率模型
// ===== M3508轮毂电机 6参数模型系数 =====
#define WHEEL_K0 0.7441993412640775f
#define WHEEL_K1 0.0090164284468539646f
#define WHEEL_K2 0.0001988857226262331f
#define WHEEL_K3 0.024694430204543864f
#define WHEEL_K4 0.20160143850678086f
#define WHEEL_K5 3.715221772539512e-05f
// ===== DJI M3508 =====
#define DJI_CURRENT_SCALE     (16384.0f / 20.0f)

typedef enum {
  CHASSIS_POWER_OFF = 0,
  CHASSIS_PROSTRATE,
} Chassis_Mode_e;

typedef struct {
  Chassis_Mode_e chassis_mode;
  float vx;
  float wz;
  float target_yaw;
  uint16_t max_power;
  int is_rotate;
} Chassis_Ctrl_Cmd_s;

typedef struct {
  float k0;
  float k1;
  float k2;
  float k3;
  float k4;
  float k5;
}Power_Param_3508_s ;

typedef struct {
  float track_width;
  Power_Param_3508_s power_param;       //3508功率模型参数，采用中科大的模型
} Chassis_Param_s;

typedef struct {
  Chassis_Param_s param;
  Motor_Init_Config_s wheel_motor_config[2];
  Motor_Init_Config_s joint_motor_config[4];
  IMU_Init_Config_s imu_init_config;
  SuperCap_Init_Config_s super_cap_config;
  PID_Init_Config_s yaw_prostrate_PID_config;
} Chassis_Init_Config_s;


typedef struct {
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  Chassis_Param_s param;

  INS_t* imu;

  SuperCapInstance* super_cap;

  DJIMotorInstance* wheel_motor[2];
  DMMotorInstance* joint_motor[4];
  PIDInstance yaw_prostrate_PID;
} ChassisInstance;

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config);
void ChassisTask(void);
