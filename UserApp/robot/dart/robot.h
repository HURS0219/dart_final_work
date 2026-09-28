//
// Created by PC on 2025/11/18.
//
#pragma once

#ifndef CONTROL_2026_ROBOT_H
#define CONTROL_2026_ROBOT_H

#include "remote_control.h"
#include "dji_motor.h"
#include "controller.h"
#include "general_def.h"
#include <stdint.h>
#include <stdbool.h>

/* 机器人模式定义 */
typedef enum {
  ROBOT_POWER_ON = 0,           // 上电运行
  ROBOT_POWER_OFF,              // 正常关机
  ROBOT_EMERGENCY_STOP          // 紧急停止
} Robot_Mode_e;

/* 飞镖模式定义 */
typedef enum {
  DART_MODE_DEBUG = 0,          // 调试模式 (S1W9先转电机)
  DART_MODE_CALIBRATING,        // 校准模式
  DART_MODE_AUTO_READY,         // 校准完成，等待发射
  DART_MODE_AUTO_FIRE,          // 自动打弹模式 (后续实现)
  DART_MODE_EMERGENCY_STOP      // 急停模式
} DartMode_e;

/* 飞镖自动校准状态枚举 */
typedef enum {
  CALI_STEP_IDLE = 0,
  CALI_STEP_VERT_PUSH,          // 垂直推杆后退
  CALI_STEP_VERT_BACK,          // 垂直推杆回退(消形变)
  CALI_STEP_HORI_PUSH,          // 水平推杆后退
  CALI_STEP_HORI_BACK,          // 水平推杆回退
  CALI_STEP_DONE                // 完成
} DartCaliStep_e;

/* 飞镖自动发射状态枚举 */
typedef enum {
  AUTO_FIRE_INIT = 0,           // 初始化
  PUSH_UP_RIGHT,                // 右上
  PUSH_UP_RIGHT_PAUSE,          // 右上停
  PUSH_DOWN_RIGHT,              // 右下
  PUSH_DOWN_RIGHT_PAUSE,        // 右下停
  PUSH_LEFT,                    // 左移
  PUSH_LEFT_PAUSE,              // 左停
  PUSH_UP_LEFT,                 // 左上
  PUSH_UP_LEFT_PAUSE,           // 左上停
  PUSH_DOWN_LEFT,               // 左下
  PUSH_DOWN_LEFT_PAUSE,         // 左下停
  PUSH_RIGHT,                   // 右移
  PUSH_RIGHT_PAUSE,             // 右移停
} AutoFireStep_e;

/* 飞镖控制命令结构 */
typedef struct {
  float friction_speed;         // 摩擦轮速度
  float vertical_speed;         // 上下推杆速度
  float horizontal_speed;       // 左右推杆速度
  bool fire_command;            // 发射命令
} Dart_Ctrl_Cmd_s;

/* 飞镖实例结构体定义 */
typedef struct {
  // 电机实例
  DJIMotorInstance* friction_motor[4];  // 3508 x4 - 摩擦轮
  DJIMotorInstance* yaw_motor;          // 6020 x1 - 发射架旋转
  DJIMotorInstance* vertical_pushrod;   // 2006 x1 - 上下推杆
  DJIMotorInstance* horizontal_pushrod; // 2006 x1 - 左右推杆

  // 状态管理
  DartMode_e current_mode;
  Dart_Ctrl_Cmd_s dart_ctrl_cmd;        // 飞镖控制命令
  bool is_fire_active;                  // 检测自动发射是否正在运行

  // 控制参数
  float friction_speed_target;          // 摩擦轮目标速度
  float vertical_speed_target;          // 上下推杆目标速度
  float horizontal_speed_target;        // 左右推杆目标速度

  // 系统参数
  float fric_base_speed;                // 摩擦轮基础速度（用于自动发射模式摇杆微调）

  // 校准与限位专用变量
  bool is_calibrated;                   // 是否已完成校准
  DartCaliStep_e calibration_step;
  AutoFireStep_e current_step;
  bool is_hori_blocked;                 // 水平推杆是否已堵转
  float hori_block_position;            // 水平推杆堵转时的位置
  bool hori_direction;

  float vert_zero_ecd;                  // 垂直推杆的零点编码器值
  float hori_zero_ecd;                  // 水平推杆的零点编码器值

} DartInstance;

/* 机器人主实例结构体 */
typedef struct {
  DartInstance* dart;                   // 飞镖组件
  RC_ctrl_t* rc_data;                   // 输入设备
  Robot_Mode_e robot_mode;              // 机器人状态
} RobotInstance;

extern RobotInstance* robot;            // 全局变量声明

/*=======核心机器人函数=======*/
/**
 * @brief 机器人系统初始化
 */
void RobotInit(void);

/**
 * @brief 机器人主任务循环
 */
void RobotTask(void);

/**
 * @brief 机器人命令解析任务
 */
//void RobotCMDTask(void);

/*=======飞镖函数=======*/
/**
 * @brief 飞镖系统初始化
 */
void DartInit(void);

/**
 * @brief 飞镖主任务
 */
//void DartTask(void);

/**
 * @brief 飞镖调试模式处理
 */
void DartDebugModeHandler(void);

/**
 * @brief 飞镖自动发射处理
 */
void DartAutoFireHandler(void);

/**
 * @brief 飞镖急停处理
 */
// DartEmergencyHandler(void);

/**
 * @brief 飞镖状态机更新
 */
void DartStateMachineUpdate(void);

/*=======工具函数=======*/
bool IsInDeadzone(int16_t value);
float MapStickToSpeed(int16_t stick_value, float max_speed);

#endif  // CONTROL_2026_ROBOT_H
