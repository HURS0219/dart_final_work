/**
 ******************************************************************************
 * @file    parallel_leg.h
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   Parallel-Leg Module
 ******************************************************************************
 * @attention
 *     Tp2         Tp1
 *  joint[1] l5 joint[0]    LEFT       RIGHT
 *    phi4 ☉---☉  phi1       ☉----------☉
 *        /     \             |          |
 *    l4 /       \ l1         |          |
 *      /         \           |          |
 * phi3◉           ◉phi2      ◉          ◉
 *      \         /           |          |
 *    L3 \  ___  / L2         |          |
 *        /     \            ---        ---
 *       |   ◉   |           | |        | |
 *        \ ___ /            ---        ---
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "controller.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"

// 关节电机零点标定模式枚举
typedef enum {
  LEG_PRE_CALI_MODE = 0,  // 预标定模式，上电后不发零点设置报文
  LEG_CALI_MODE,          // 零点标定模式，发送零点设置报文，上电过程中需保持不动，标定完成后需手动切为预标定模式
} Leg_Cali_Mode_e;

// 真实物理模型状态
typedef struct {
  float xb, yb, xb_d, yb_d;
  float xc, yc;
  float xd, yd;
  float phi2, phi3;
  float phi1, phi4;
  float phi1_d, phi4_d;
  float Tp_1, Tp_2;       // 关节最终输出力矩
  float T, T_LQR, T_MPC;  // 轮毂最终输出力矩 (此处T_MPC未使用)
} Real_Model_t;

// 虚拟模型状态 (极坐标/杆长)
typedef struct {
  PIDInstance length_PID;
  PIDInstance length_d_PID;

  float length, length_d, length_dd, last_length_d;
  float phi, phi_d, phi_dd, last_phi_d;
  float alpha, alpha_d;

  float F, FN;               // 虚拟推力, 法向力(用于离地检测)
  float Tp, Tp_LQR, Tp_MPC;  // 虚拟髋关节力矩
} Virtual_Model_t;

// 状态空间变量 (LQR/MPC输入)
typedef struct {
  float x, x_d;
  float theta, theta_d;
  float phi, phi_d;
} State_Var_t;

// 速度观测器变量
typedef struct {
  float w;   // Angular velocity relevant to the earth frame
  float vb;  // Body b frame velocity
} Observer_Var_t;

// 腿部固有参数 & 控制系数
typedef struct {
  float rod_length[5];               // 五连杆长度, 单位是m
  float joint_motor_zero_offset[2];  // 关节电机零点偏移, 单位是rad, 需机械测量，用于编码器读数转换为建模实际角度
  float wheel_radius;                // 轮子半径, 单位是m
  float wheel_reduction_ratio;       // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
  float LQR_K_Coefficient[2][6][4];  // [2输入][6状态变量][4多项式系数]
  float MPC_K_Coefficient[2][6][4];  // [2输入][6状态变量][4指数系数 a,b,c,d]
} Leg_Param_t;

// 腿部控制命令
typedef struct {
  float x_ref, x_d_ref;            // 机体位置与速度目标值
  float length_ref, length_d_ref;  // 腿长与腿长速度目标值
  float theta_ref;
} Leg_Ctrl_Cmd_t;

// 初始化配置结构体
typedef struct {
  Leg_Cali_Mode_e cali_mode;  // 关节电机零点标定模式
  Leg_Param_t param;          // 腿部固有参数

  PID_Init_Config_s length_PID_config;    // 腿长位置环PID参数
  PID_Init_Config_s length_d_PID_config;  // 腿长速度环PID参数 (暂时没启用)
  Motor_Init_Config_s joint_motor_config[2];
  Motor_Init_Config_s wheel_motor_config;
} Leg_Init_Config_s;

// 腿部实例
typedef struct {
  Leg_Ctrl_Cmd_t leg_ctrl_cmd;
  Leg_Param_t param;

  DMMotorInstance* joint_motor[2];
  DJIMotorInstance* wheel_motor;

  Real_Model_t real_model;
  Virtual_Model_t virtual_model;
  State_Var_t state_var;
  State_Var_t last_state_var;
  Observer_Var_t observer_var;
  float J[2][2];
  float LQR_K[2][6];
  float MPC_K[2][6];
  uint32_t DWT_CNT;
  float dt;

  struct {
    uint8_t is_initialized : 1;  // 观测器和状态变量是否完成第一次更新
    uint8_t is_restart : 1;      // 是否需要重启更新（如时间步长过大时）
    uint8_t is_off_ground : 1;   // 离地检测标志, 1表示离地, 0表示接地, 用于切换控制策略
    uint8_t is_controlled : 1;   // 是否处于受控状态, 1表示受控（如有前进指令时）, 0表示非受控, 用于切换控制策略
  } update_flag;
} LegInstance;

LegInstance* LegInit(Leg_Init_Config_s* config);

void LegCtrlUpdate(LegInstance* leg, INS_t* imu);

void JointTorqueUpdate(LegInstance* leg);

void ObserverVarUpdate(LegInstance* leg, INS_t* imu);