#pragma once

#include "dji_motor.h"

#define SHOOT_CNT_MAX 1

// 发射模式设置
typedef enum {
  SHOOT_OFF = 0,
  SHOOT_ON,
} Shoot_Mode_e;

typedef enum {
  FRICTION_OFF = 0,  // 摩擦轮关闭
  FRICTION_ON,       // 摩擦轮开启
} Friction_Mode_e;

typedef enum {
  NO_CONTROL = 0,       // 不启用热量控制
  REFEREE_CONTROL,      // 裁判系统数据控制
  SIMULLATE_CONTROL,    // 模拟控制，不采用裁判系统数据，程序模拟，独立计算
} HEAT_Mode_e;

typedef enum {
  DISABLE_BULLET_SPEED= 0,          // 不启用弹速控制
  ENABLE_BULLET_SPEED,              // 裁判系统数据控制
  MANUAL_BULLET_SPEED,              // 操作手自己加减
} BULLET_Speed_Mode_e;

typedef enum {
  LOAD_STOP = 0,   // 停止发射
  LOAD_REVERSE,    // 反转
  LOAD_1_BULLET,   // 单发
  LOAD_3_BULLET,   // 三发
  LOAD_BURSTFIRE,  // 连发
} Loader_Mode_e;

typedef enum {
  BULLET_SPEED_NONE = 0,
  BIG_AMU_10 = 10,
  SMALL_AMU_15 = 15,
  BIG_AMU_16 = 16,
  SMALL_AMU_18 = 18,
  SMALL_AMU_30 = 30,
} Bullet_Speed_e;

// 机器人底盘修改的参数,单位为mm(毫米)
typedef struct {
  float one_bullet_delta_angle;  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
  float reduction_ratio_loader;  // 2006拨盘电机的减速比,英雄拨弹盘的3508减速比为100
  float num_per_circle;          // 拨盘一圈的装  载量
  int loader_direction;          // 拨弹盘方向
  int friction_num;              // 摩擦轮数量
  float friction_speed;          // 摩擦轮速度
  float friction_speed_min;  // 摩擦轮转速绝对值下限
  float friction_speed_max;  // 摩擦轮转速绝对值上限
  float target_speed;
  float friction_coefficients[FRICTION_NUM];  // 摩擦轮的系数
  float deadtime_onebullet;                   // 单发死时间
  float deadtime_burstfire;                   // 连发死时间
  float bullet_speed_adjustment;
  float feedforward;
  float bullet_speed_deadband;                //速度死区
  uint16_t shooter_barrel_cooling_value;      // 机器人射击热量每秒冷却值
  uint16_t shooter_barrel_heat_limit;         // 机器人射击热量上限
  uint16_t one_barrel_heat_value;             // 一个弹丸的热量
} Shoot_Param_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct {
  Shoot_Mode_e shoot_mode;
  Loader_Mode_e load_mode;
  Friction_Mode_e friction_mode;
  BULLET_Speed_Mode_e bullet_speed_mode;
  HEAT_Mode_e heat_mode;
  uint16_t shooter_barrel_heat;// 机器人当前射击热量,从裁判系统获取
  float initial_speed;  // 当前弹速
  float friction_speed; // 摩擦轮转速
  uint16_t shooter_barrel_heat_limit;  // 机器人射击热量上限
} Shoot_Ctrl_Cmd_s;

typedef struct {
  Motor_Init_Config_s loader_motor_config;
  Shoot_Param_s shoot_param;
  Motor_Init_Config_s friction_motor_config[FRICTION_NUM];
} Shoot_Init_Config_s;

typedef struct {
  Shoot_Ctrl_Cmd_s shoot_ctrl_cmd;
  DJIMotorInstance* loader_motor;  // 拨盘电机
  DJIMotorInstance* friction_motor[FRICTION_NUM];
} ShootInstance;

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
ShootInstance* ShootInit(Shoot_Init_Config_s* shoot_init_config);
/**
 * @brief 云台任务
 *
 */
void ShootTask();