#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "remote_control.h"
#include "shoot.h"
#include "referee.h"
#include "can_comm.h"
#include "super_cap.h"
#include "master_process.h"
#include "VT13.h"

#ifndef ONE_BOARD
#pragma pack(1)
typedef struct {
  float Pitch;          // 俯仰角(绕Y轴旋转) 单位: °
  float YawTotalAngle;
  float yaw_speed;
  float bullet_speed;
  uint8_t robot_id;
  int shooter_17mm_barrel_heat;
  int shoot_heat_limit;
  SuperCap_Ctrl_Cmd_e super_cap_ctrl_cmd;
} Chassis_Upload_Data_s;  // means the Chassis board, not the component

typedef struct {
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  SuperCap_Ctrl_Cmd_e super_cap_ctrl_cmd;
  uint8_t force_refresh_ui;
} Chassis_Fetch_Data_s;  // means the Chassis board, not the component
#pragma pack()

#endif

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_CHASSIS_PROSTRATE_FOLLOW,
  ROBOT_CHASSIS_PROSTRATE_ROTATE,
  ROBOT_CHASSIS_PROSTRATE_FREE,
} Robot_Mode_e;

typedef struct {
  Robot_Mode_e robot_mode;  // 机器人整体工作状态
#ifdef USE_RC_CTRL
  RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
#elifdef USE_OCD_CTRL
  VT13_RC_t *rc_data;
#endif
  referee_info_t* referee_data;     // 用于获取裁判系统的数据

  float offset_angle;

  ChassisInstance* chassis;
  GimbalInstance* gimbal;
  ShootInstance* shoot;

  Vision_Receive_s* vision_recv_data;

#ifndef ONE_BOARD
  Chassis_Upload_Data_s* chassis_upload_data;  // 此处chassis定义为chassis_board, 而非chassis模组, 故所有处理在robot中
  Chassis_Fetch_Data_s* chassis_fetch_data;
  CANCommInstance* can_comm;
#endif

  uint32_t DWT_CNT;
  float dt;
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