/**
******************************************************************************
* @file    chassis.h
* @author  NeoZeng
* @author  Annotation and Modification By SRM-Control 2026
* @date    2025/10/10
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   Mecanum Chassis Module
******************************************************************************
* @attention
* Mecanum Chassis Motor Layout:
*
*          motor[0]     motor[1]
*        Left Front   Right Front
*             ╭─────────────╮
*             │      ↑      │
*             │    Front    │
*             ╰─────────────╯
*          motor[2]     motor[3]
*        Left Rear    Right Rear
*
* @note    Motor Index:
*          motor[0] - Left Front Wheel
*          motor[1] - Right Front Wheel
*          motor[2] - Left Rear Wheel
*          motor[3] - Right Rear Wheel
******************************************************************************
*/
#pragma once

#include "dji_motor.h"

typedef enum {
  CHASSIS_POWER_OFF = 0,    // 电流零输入
  CHASSIS_ROTATE,            // 小陀螺模式
  CHASSIS_FOLLOW,            // 跟随模式，底盘叠加角度环控制
} Chassis_Mode_e;

typedef struct {
  // 控制部分
  float vx;            // 前进方向速度
  float vy;            // 横移方向速度
  float wz;            // 旋转速度
  Chassis_Mode_e chassis_mode;
  float offset_angle;  // 底盘和归中位置的夹角
  int chassis_speed_buff;
  uint16_t max_power;  // 最大功率限制
  // UI部分
  //  ...

} Chassis_Ctrl_Cmd_s;

typedef struct {
  float k0;
  float k1;
  float k2;
  float k3;
  float k4;
  float k5;
}Power_Param_3508_s ;

// 机器人底盘修改的参数,单位为mm(毫米)
typedef struct {
  float wheel_base;                     // 纵向轴距(前进后退方向)
  float track_width;                    // 横向轮距(左右平移方向)
  float center_gimbal_offset_x;         // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
  float center_gimbal_offset_y;         // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
  float wheel_radius;                   // 轮子半径
  float wheel_reduction_ratio;          // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
  Power_Param_3508_s power_param;       //3508功率模型参数，采用中科大的模型
} Chassis_Param_s;

typedef struct {
  Chassis_Param_s chassis_param;
  Motor_Init_Config_s wheel_motor_config[4];
  PID_Init_Config_s follow_pid;
} Chassis_Init_Config_s;

typedef struct {
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  DJIMotorInstance* wheel_motor[4];  // left right forward back
} ChassisInstance;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 *
 */
ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config);

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 *
 */
void ChassisTask();