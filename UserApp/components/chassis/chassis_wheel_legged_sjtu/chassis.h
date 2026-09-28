/**
******************************************************************************
* @file    chassis.h
* @author  Enhao Zhang
* @date    2026/2/25
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   SJTU Full-Chassis Wheel-Legged Module (10-dim state, 4x10 LQR)
******************************************************************************
* @attention
* Wheel-Legged Chassis Layout:
* LEFT Leg[1]       Leg[0] RIGHT
*        ☉----------☉
*        |          |
*        |          |
*        |          |
*        |          |
*        ◉          ◉
*       ---        ---
* State: [x_b, x_b_d, phi, phi_d, theta_l, theta_l_d, theta_r, theta_r_d, theta_b, theta_b_d]
* Control: u = [T_r_to_b, T_l_to_b, T_wr_to_r, T_wl_to_l]
******************************************************************************
*/
#pragma once

#include "ins_task.h"
#include "parallel_leg.h"
#include "super_cap.h"
#include "user_lib.h"
// #include "power_control.h"

typedef enum {
  CHASSIS_POWER_OFF = 0,
  CHASSIS_RECOVERY,
  CHASSIS_ON,
  CHASSIS_JUMP_READY,
  CHASSIS_JUMP_START,
  CHASSIS_PROSTRATE,
  CHASSIS_STAIR,
} Chassis_Mode_e;

typedef enum {
  JUMP_STATE_IDLE,
  JUMP_STATE_COMPRESS,
  JUMP_STATE_EXTEND,
  JUMP_STATE_RETRACT,
} Jump_State_e;

typedef struct {
  float vx;
  // 平衡模式: 目标 yaw 角速度(rad/s), 进入 LQR 的 phi_d 跟踪项。
  // 卧倒模式: 差速转向前馈量, 不是 rad/s, 由 ChassisProstrateMode() 换算为轮速。
  float wz;
  // 目标 yaw 角度(rad)。卧倒小陀螺/自由转向时应对齐当前 yaw, 让 yaw PID 不参与。
  float target_yaw;
  float roll;
  float roll_ff;
  float leg_length;
  float jump_force;
  float theta_ff;
  int chassis_speed_buff;
  uint16_t max_power;
  Chassis_Mode_e chassis_mode;
  uint8_t SuperCapBoost;
  // 小陀螺标志: 1 时 LegController 关闭 f_inertial 前馈, StateVarUpdate 用轮速回授 phi_d/x_b_d 并始终积分 x_b
  uint8_t is_rotate;
} Chassis_Ctrl_Cmd_s;

/* SJTU model: 10-dim state vector */
typedef struct {
  float x_b;     // 机身在水平地面的水平位置（全局系），无传感器，保持为0或积分得到
  float x_b_d;     // 机身在水平地面的水平速度（全局系），底盘前向平均速度
  float phi;       // 机身航向角 yaw（全局系, rad），由IMU YawTotalAngle获得
  float phi_d;      // 机身航向角速度 yaw_rate（rad/s），由IMU陀螺仪Gyro[2]获得
  float theta_l;   // 左腿杆体与Z负方向夹角（rad），由VMC解算
  float theta_l_d;  // 左腿角速度（rad/s）
  float theta_r;   // 右腿杆体与Z负方向夹角（rad），由VMC解算
  float theta_r_d;  // 右腿角速度（rad/s）
  float theta_b;   // 机身俯仰角 pitch（rad），由IMU Pitch获得
  float theta_b_d;  // 机身俯仰角速度 pitch_rate（rad/s），由IMU陀螺仪Gyro[0]获得
} State_Var_t;

typedef struct {
  float track_width;
  float body_mass;
  float initial_leg_length;
  float leg_min_length;
  float leg_max_length;
  float LQR_K_Coefficients[40][6];
  float LQR_K_Stair_Coefficients[40][6];
} Chassis_Param_s;

typedef struct {
  Chassis_Param_s param;
  Leg_Init_Config_s leg_init_config[2];
  PID_Init_Config_s roll_PID_config;
  PID_Init_Config_s yaw_prostrate_PID_config;
  IMU_Init_Config_s imu_init_config;
  SuperCap_Init_Config_s super_cap_config;
  Ramp_Controller_t vx_ramp_config;
  Ramp_Controller_t wz_ramp_config;
} Chassis_Init_Config_s;

// 底盘 planner: 把上层 raw cmd (target_yaw/vx/wz) 在 chassis 时基 (200Hz) 内平滑.
// 迁移自 ctrl 层 PlanYawByAcc + vx_ramp, 避免 50Hz CAN ZOH 把 LQR 参考切成阶梯.
typedef struct {
  Ramp_Controller_t wz_ramp;
  Ramp_Controller_t vx_ramp;
  float yaw_err_filt;  // yaw_err 一阶 LPF 输出
  uint8_t snap_count;  // snap-to-target 滞回计数
  float target_yaw;    // 规划态 yaw (rad), 每帧 += planned_wz * dt
  float vx;            // 平滑 vx, 供 LQR state_err[1] 使用
  float wz;            // 平滑 wz (= wz_ramp.planning_v), 调试用
} ChassisPlanner_t;

typedef struct {
  Jump_State_e jump_state;
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  ChassisPlanner_t planner;
  Chassis_Param_s param;

  INS_t* imu;

  KalmanFilter_t vaEstimateKF;

  State_Var_t state_var;
  State_Var_t last_state_var;

  LegInstance* leg[2];
  PIDInstance roll_PID;
  PIDInstance yaw_prostrate_PID;

  float LQR_K[4][10];  // [4输出][10状态变量]
  float state_err[10]; // 10状态变量误差

  uint32_t DWT_CNT;
  float dt;
  struct {
    uint8_t is_first_update : 1;    // 观测器和状态变量是否完成第一次更新
    uint8_t is_restart : 1;         // 是否需要重启更新（如时间步长过大时）
    uint8_t is_controlled : 1;      // 是否处于受控状态, 1表示受控（如有前进指令时）, 0表示非受控, 用于切换控制策略
    uint8_t gimbal_aligned : 1;     // 云台是否已与底盘正方向对齐（双板时由云台板经 CAN 同步到底盘板）
  } update_flag;

  SuperCapInstance* super_cap;
  struct Power_Ctrl_t* power_ctrl;
} ChassisInstance;

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config);
void ChassisTask(void);
