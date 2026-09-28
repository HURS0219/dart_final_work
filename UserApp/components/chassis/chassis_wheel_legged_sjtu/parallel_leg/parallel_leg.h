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
  float length, length_d, length_dd, last_length_d;
  float phi, phi_d, phi_dd, last_phi_d;
  float alpha, alpha_d;
  float theta, theta_d;

  float F, FN;  // 虚拟推力, 法向力(用于离地检测)
  float Tp;     // 虚拟髋关节力矩
} Virtual_Model_t;

// 速度观测器变量
typedef struct {
  float w;   // Angular velocity relevant to the earth frame
  float vb;  // Body frame velocity
} Observer_Var_t;

// 关节角度限位配置（虚拟弹性墙）
typedef struct {
  float angle_min;           // 关节角度下限 (rad)，含安全余量
  float angle_max;           // 关节角度上限 (rad)，含安全余量
  float buffer_zone;         // 缓冲区宽度 (rad)，进入此区域开始产生回推力矩
  float kp;                  // 虚拟墙刚度 (Nm/rad^2)，非线性弹簧系数
  float kd;                  // 虚拟墙阻尼 (Nm·s/rad)
  float max_barrier_torque;  // 限位力矩上限 (Nm)，防止数值爆炸
} JointLimit_Config_s;

// 腿部固有参数 & 控制系数
typedef struct {
  float rod_length[5];                 // 五连杆长度, 单位是m
  float joint_motor_zero_offset[2];    // 关节电机零点偏移, 单位是rad, 需机械测量，用于编码器读数转换为建模实际角度
  float wheel_radius;                  // 轮子半径, 单位是m
  float wheel_reduction_ratio;         // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
  JointLimit_Config_s joint_limit[2];  // 两个关节电机的限位配置
} Leg_Param_t;

// 初始化配置结构体
typedef struct {
  Leg_Cali_Mode_e cali_mode;  // 关节电机零点标定模式
  Leg_Param_t param;          // 腿部固有参数
  PID_Init_Config_s length_PID_config;

  Motor_Init_Config_s joint_motor_config[2];
  Motor_Init_Config_s wheel_motor_config;
} Leg_Init_Config_s;

// 腿部实例
typedef struct {
  Leg_Param_t param;

  PIDInstance length_PID;  // 每腿单独腿长环

  DMMotorInstance* joint_motor[2];
  DJIMotorInstance* wheel_motor;

  Real_Model_t real_model;
  Virtual_Model_t virtual_model;

  Observer_Var_t observer_var;

  float J[2][2];
  uint32_t DWT_CNT;
  float dt;

  struct {
    uint8_t is_first_update : 1;  // 观测器和状态变量是否完成第一次更新
    uint8_t is_restart : 1;       // 是否需要重启更新（如时间步长过大时）
    uint8_t is_off_ground : 1;    // 离地检测标志, 1表示离地, 0表示接地, 用于切换控制策略
  } update_flag;
} LegInstance;

LegInstance* LegInit(Leg_Init_Config_s* config);

void LegModelUpdate(LegInstance* leg, INS_t* imu);

void JointTorqueUpdate(LegInstance* leg);

void SpringCompensation(LegInstance* leg);

void JointLimitBarrier(LegInstance* leg);

void ObserverVarUpdate(LegInstance* leg, INS_t* imu);