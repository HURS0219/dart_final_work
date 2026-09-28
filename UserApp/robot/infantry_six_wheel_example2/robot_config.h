/**
******************************************************************************
* @file    robot_config.h
* @author  Enhao Zhang
* @date    2025/8/8
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief Infantry wheeled-legged robot control module
******************************************************************************
* @attention
* None
*
******************************************************************************
*/
#pragma once

#include "general_def.h"
#include "robot.h"

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,浮点数需要以.0或f结尾,无符号以u结尾 */

// 机器人底盘修改的参数,单位为m(米)
#define CENTER_GIMBAL_OFFSET_X 0                // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0                // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define WHEEL_RADIUS 0.077f                     // 轮子半径
#define WHEEL_REDUCTION_RATIO (268.0f / 17.0f)  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define TRACK_WIDTH 0.495f

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5070  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define PITCH_HORIZON_ECD 4215      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 20.0f       // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -30.0f      // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)  // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36.0f  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 90.0f  // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10             // 拨盘一圈的装载量

// 不用Macro会传不进去
#define JOINT_MOTOR_CONFIG(motor_reverse, feedback_reverse, can, tx, rx)                                       \
  {                                                                                                            \
      .controller_param_init_config =                                                                          \
          {                                                                                                    \
              .angle_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 20.0f,                                                                             \
                      .Ki = 0.0f,                                                                              \
                      .Kd = 0.0f,                                                                              \
                      .MaxOut = 10.0f,                                                                         \
                      .DeadBand = 0.01f,                                                                       \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .IntegralLimit = 0.0f,                                                                   \
                  },                                                                                           \
              .speed_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 5.0f,                                                                              \
                      .Ki = 0.0f,                                                                              \
                      .Kd = 0.05f,                                                                             \
                      .MaxOut = 15.0f,                                                                         \
                      .DeadBand = 0.01f,                                                                       \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .IntegralLimit = 0.0f,                                                                   \
                  },                                                                                           \
          },                                                                                                   \
      .controller_setting_init_config =                                                                        \
          {                                                                                                    \
              .outer_loop_type = ANGLE_LOOP,                                                                   \
              .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                                      \
              .angle_feedback_source = MOTOR_FEED,                                                             \
              .speed_feedback_source = MOTOR_FEED,                                                             \
              .motor_reverse_flag = motor_reverse,                                                             \
              .feedback_reverse_flag = feedback_reverse,                                                       \
          },                                                                                                   \
      .motor_type = J8009P,                                                                                    \
      .can_init_config =                                                                                       \
          {                                                                                                    \
              .can_handle = can,                                                                               \
              .tx_id = tx,                                                                                     \
              .rx_id = rx,                                                                                     \
          },                                                                                                   \
  }

#define WHEEL_MOTOR_CONFIG(handle, id)                                                                         \
  ((Motor_Init_Config_s){                                                                                      \
      .can_init_config =                                                                                       \
          {                                                                                                    \
              .can_handle = handle,                                                                            \
              .tx_id = id,                                                                                     \
          },                                                                                                   \
      .controller_param_init_config =                                                                          \
          {                                                                                                    \
              .speed_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 1.0,                                                                               \
                      .Ki = 0.5,                                                                               \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 6000,                                                                   \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 15000,                                                                         \
                  },                                                                                           \
              .current_PID =                                                                                   \
                  {                                                                                            \
                      .Kp = 0,                                                                                 \
                      .Ki = 0,                                                                                 \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 3000,                                                                   \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 15000,                                                                         \
                  },                                                                                           \
          },                                                                                                   \
      .controller_setting_init_config =                                                                        \
          {                                                                                                    \
              .angle_feedback_source = MOTOR_FEED,                                                             \
              .speed_feedback_source = MOTOR_FEED,                                                             \
              .outer_loop_type = SPEED_LOOP,                                                                   \
              .close_loop_type = SPEED_LOOP,                                                                   \
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,                                                   \
              .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,                                             \
          },                                                                                                   \
      .motor_type = M3508,                                                                                     \
  })

static Chassis_Init_Config_s chassis_init_config = {
    .param =
        {
            .track_width = TRACK_WIDTH,
        },
    .yaw_prostrate_PID_config =
        {
            .Kp = 3.0f,
            .Ki = 0.0f,
            .Kd = 0.02f,
            .MaxOut = 6.0f,
            .DeadBand = 0.01f,
            .IntegralLimit = 3.0f,
        },
    .wheel_motor_config[0] = WHEEL_MOTOR_CONFIG(&hcan1, 1),
    .wheel_motor_config[1] = WHEEL_MOTOR_CONFIG(&hcan1, 2),
    .joint_motor_config[0] = JOINT_MOTOR_CONFIG(MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL, &hcan2, 0x02, 0x01),
    .joint_motor_config[1] = JOINT_MOTOR_CONFIG(MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL, &hcan2, 0x06, 0x03),
    .joint_motor_config[2] = JOINT_MOTOR_CONFIG(MOTOR_DIRECTION_REVERSE, MOTOR_DIRECTION_REVERSE, &hcan2, 0x08, 0x04),
    .joint_motor_config[3] = JOINT_MOTOR_CONFIG(MOTOR_DIRECTION_REVERSE, MOTOR_DIRECTION_REVERSE, &hcan2, 0x0A, 0x05),
    .imu_init_config =
        {
            .flag = 0,
            .scale = {1.0f, 1.0f, 1.0f},
            .Yaw = 0.0f,
            .Pitch = 180.0f,
            .Roll = 0.0f,
            .CenterOffset[0] = 0.15413f,
            .CenterOffset[1] = 0.04612f,
            .CenterOffset[2] = 0.09348f,
            .GyroOffset[0] = 0.00708952406,
            .GyroOffset[1] = 0.00323308632,
            .GyroOffset[2] = 0.00078589347,
            .offset_flag = 1,
        },
    .super_cap_config = {.can_config = {
                             .can_handle = &hcan1,
                             .tx_id = 0x210,  // 超级电容默认接收id
                             .rx_id = 0x211,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
                         }}};

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {.controller_param_init_config =
             {
                 // .angle_PID =
                 //     {
                 //         .Kp = 2.0f,
                 //         .Ki = 0.0f,
                 //         .Kd = 0.03f,
                 //         .DeadBand = 0.01f,
                 //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                 //         .IntegralLimit = 5.0f,
                 //         .MaxOut = 22.0f,
                 //     },
                 .angle_PID =
                     {
                         .Kp = 2.5f,
                         .Ki = 0.0f,
                         .Kd = 0.05f,
                         .DeadBand = 0.01f,
                         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                         .IntegralLimit = 5.0f,
                         .MaxOut = 28.0f,
                     },
                 .speed_PID =
                     {
                         .Kp = -6000.0f,
                         .Ki = -100.0f,
                         .Kd = 0.0f,
                         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                         .IntegralLimit = 12000.0f,
                         .MaxOut = 25000.0f,
                     },

             },
         .motor_type = GM6020,
         .can_init_config =
             {
                 .can_handle = &hcan2,
                 .tx_id = 6,
             },
         .controller_setting_init_config =
             {
                 .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                 .feedforward_flag = SPEED_FEEDFORWARD,
             }},
    .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 1.8f,
                            .Ki = 0.0f,
                            .Kd = 0.03f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -5000.0f,
                            .Ki = -200.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 28000.0f,
                        },
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 2,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    .imu_init_config =
        {
            .flag = 1,
            .scale = {1.0f, 1.0f, 1.0f},
            .Yaw = -90.0f,
            .Pitch = 0.0f,
            .Roll = 0.0f,
            .GyroOffset[0] = -0.0014910656f,
            .GyroOffset[1] = -0.00283604953f,
            .GyroOffset[2] = 0.00104337547f,
            .offset_flag = 1,
        },
    .pitch_feedforward_scale = 7000.0f};

#define FRICTION_MOTOR_CONFIG(handle, id, motor_direction, feedback_direction) \
  ((Motor_Init_Config_s){                                                      \
      .controller_param_init_config =                                          \
          {                                                                    \
              .speed_PID =                                                     \
                  {                                                            \
                      .Kp = 1.5f,                                              \
                      .Ki = 0.2f,                                              \
                      .Kd = 0.0f,                                              \
                      .Improve = PID_Integral_Limit,                           \
                      .IntegralLimit = 10000.0f,                               \
                      .MaxOut = 15000.0f,                                      \
                  },                                                           \
          },                                                                   \
      .controller_setting_init_config =                                        \
          {                                                                    \
              .angle_feedback_source = MOTOR_FEED,                             \
              .speed_feedback_source = MOTOR_FEED,                             \
              .outer_loop_type = SPEED_LOOP,                                   \
              .close_loop_type = SPEED_LOOP,                                   \
              .motor_reverse_flag = motor_direction,                           \
              .feedback_reverse_flag = feedback_direction,                     \
          },                                                                   \
      .motor_type = M3508,                                                     \
      .can_init_config =                                                       \
          {                                                                    \
              .can_handle = handle,                                            \
              .tx_id = id,                                                     \
          },                                                                   \
  })

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = ONE_BULLET_DELTA_ANGLE,  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
            .reduction_ratio_loader = REDUCTION_RATIO_LOADER,  // M2006拨盘电机的减速比
            .num_per_circle = NUM_PER_CIRCLE,                  // 拨盘一圈的装载量
            .loader_direction = 1,                             // 拨盘旋转方向,1为正向，-1为反向
            .friction_num = 2,                                 // 摩擦轮数量
            .friction_speed = 37500.0f,                        // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},            // 摩擦轮速度比例系数
            .deadtime_burstfire = 55,
            .deadtime_onebullet = 350,
            .target_speed = 22.0f,
            .bullet_speed_adjustment = 50.0f,
            .feedforward = 200.0f,
            .one_barrel_heat_value = 10,         // 一发弹丸所需热量
            .shooter_barrel_cooling_value = 40,  // 每秒冷却回复
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 4, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 5, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL),

    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 60.0f,
                            .Ki = 0.0f,
                            .Kd = 1.0f,
                            .MaxOut = 45000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1.5f,
                            .Ki = 0.4f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 5000.0f,
                            .MaxOut = 8000.0f,
                        },
                },
            .motor_type = M2006,  // 拨盘电机为M2006
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 3,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
            .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
};

#if defined(GIMBAL_BOARD)
static CANComm_Init_Config_s gimbal_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
    .recv_data_len = sizeof(Chassis_Upload_Data_s),
    .send_data_len = sizeof(Chassis_Fetch_Data_s),  // chassis_ctrl_cmd
};

#endif
#if defined(CHASSIS_BOARD)
static CANComm_Init_Config_s chassis_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan3,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
    .recv_data_len = sizeof(Chassis_Fetch_Data_s),  // chassis_ctrl_cmd
    .send_data_len = sizeof(Chassis_Upload_Data_s),
};
#endif