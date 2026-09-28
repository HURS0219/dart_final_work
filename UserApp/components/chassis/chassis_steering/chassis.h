//
// Created by yang6 on 2025/11/17.
//

#ifndef CONTROL_2026_CHASSIS_H
#define CONTROL_2026_CHASSIS_H
 #include "ins_task.h"

#endif  // CONTROL_2026_CHASSIS_H
#pragma once

#include "dji_motor.h"
#define DEG2R(x) ((x)*PI /180.0f)
#define LF 0//数组内表示电机位置
#define LB 1
#define RB 2
#define RF 3
#define MAX_WHEEL_SPEED 50000.0f
//底盘模式
// typedef enum {
//   CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //底盘跟随云台
//   CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //底盘自主
//   CHASSIS_VECTOR_SPIN,                //小陀螺
//   CHASSIS_VECTOR_NO_FOLLOW_YAW,       //底盘不跟随
//   CHASSIS_VECTOR_RAW,				  //底盘原始控制
//   RUDDER_VECTOR_FOLLOW_GIMBAL_YAW,    //舵跟随云台
// //  CHASSIC_TURN,						  //一键掉头
// } Chassis_Mode_e;
typedef enum {
  CHASSIS_POWER_OFF = 0,    // 电流零输入
  CHASSIS_ROTATE,            // 小陀螺模式
  CHASSIS_FOLLOW,            // 跟随模式，底盘叠加角度环控制
//  CHASSIC_TURN,						  //一键掉头
} Chassis_Mode_e;

// 舵轮底盘模式
// typedef enum {
//   CHASSIS_STEERING_POWER_OFF = 0,    // 关闭
//   CHASSIS_STEERING_NORMAL,           // 正常模式
//   CHASSIS_STEERING_FOLLOW_GIMBAL,    // 跟随云台
//   CHASSIS_STEERING_SPIN              // 小陀螺模式
// } Chassis_Steering_Mode_e;

// 舵轮底盘控制命令
typedef struct {
  float vx;                          // x方向速度
  float vy;                          // y方向速度
  float wz;                          // 旋转速度
  Chassis_Mode_e chassis_mode;      // 底盘模式
  float offset_angle;                // 与云台的夹角
  uint16_t max_power;                // 最大功率限制
} Chassis_Ctrl_Cmd_s;

typedef struct {
  float k0;
  float k1;
  float k2;
  float k3;
  float k4;
  float k5;
}Power_Param_3508_s ;

// 舵轮底盘参数，这一坨是从英雄的代码抄过来的，得改，但是逆解算用不上先不管
typedef struct {
  float wheel_base;                     // 纵向轴距(前进后退方向)
  float track_width;                    // 横向轮距(左右平移方向)
  float center_gimbal_offset_x;         // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
  float center_gimbal_offset_y;         // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
  float wheel_radius;                   // 轮子半径
  float wheel_reduction_ratio;          // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
  Power_Param_3508_s power_param;       //3508功率模型参数，采用中科大的模型
  uint16_t rudder_motor_offset[4];      // 6020舵电机零位偏移值，用于校准安装后的零偏
} Chassis_Param_s;

// 舵轮底盘初始化配置
typedef struct {
  Chassis_Param_s chassis_param;
  Motor_Init_Config_s rudder_motor_config[4];
  Motor_Init_Config_s wheel_motor_config[4];
  Motor_Init_Config_s yaw_motor_config;
  PID_Init_Config_s rudder_angle_pid_config;
  PID_Init_Config_s rudder_speed_pid_config;
  PID_Init_Config_s driver_speed_pid_config;
  PID_Init_Config_s follow_pid;
} Chassis_Init_Config_s;

// 舵轮底盘实例



typedef struct {
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  DJIMotorInstance *wheel_motor[4];
  DJIMotorInstance *rudder_motor[4];
  DJIMotorInstance *yaw_motor;
  uint16_t rudder_offset[4];
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