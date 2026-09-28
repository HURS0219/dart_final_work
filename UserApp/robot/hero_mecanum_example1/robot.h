#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "remote_control.h"
#include "rm_referee.h"
#include "super_cap.h"
// todo: add vision_module

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON ,
} Robot_Mode_e;

typedef struct {
  Robot_Mode_e robot_mode;       // 机器人整体工作状态
  referee_info_t* referee_data;   // 裁判系统数据
  RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
  referee_info_t* referee_data;     // 用于获取裁判系统的数据

  SuperCapInstance* super_cap;
  ChassisInstance* chassis;
  GimbalInstance* gimbal;
  ShootInstance* shoot;

} RobotInstance;


/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 *
 */
void RobotInit();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 *
 */
void RobotTask();