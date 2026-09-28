#ifndef CONTROL_2026_GIMBAL_H
#define CONTROL_2026_GIMBAL_H

#pragma once

#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"

typedef enum {
  GIMBAL_POWER_OFF = 0,  // 电流零输入
  GIMBAL_ON,
  GIMBAL_VISION,
} Gimbal_Mode_e;

typedef struct {
  float yaw;
  float pitch;
  float chassis_rotate_wz;
  Gimbal_Mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct {
  Motor_Init_Config_s yaw_motor_config;
  Motor_Init_Config_s pitch_motor_config;
  IMU_Init_Config_s imu_init_config; //用于修正IMU安装误差的参数（新增）
} Gimbal_Init_Config_s;

typedef struct {
  Gimbal_Ctrl_Cmd_s gimbal_ctrl_cmd;
  DJIMotorInstance *yaw_motor;
  DMMotorInstance *pitch_motor;
  INS_t* gimbal_IMU_data;  // 云台IMU数据
} GimbalInstance;

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
GimbalInstance* GimbalInit(Gimbal_Init_Config_s* gimbal_init_config);

/**
 * @brief 云台任务
 *
 */
void GimbalTask();

#endif  // CONTROL_2026_GIMBAL_H
