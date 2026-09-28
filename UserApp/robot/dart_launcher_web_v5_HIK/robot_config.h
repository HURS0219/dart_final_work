/*
 * robot_config.h — 制导飞镖发射架 · 网页控制版 v3 (完全复用 DJImotor 库)
 *
 * 电机 SDK 全部走 Modules/motor/DJImotor (dji_motor.c) + controller,
 * 本工程只做: 角色映射 / 模式切换 / 零点 / 网页调参与保存 / 状态机。
 *
 * 四电机 (CAN1, 1Mbps):
 *   拉簧A  M3508 ID2 ; 拉簧B M3508 ID3 ; 扳机 M3508 ID4 ; Yaw M2006 ID1
 */
#ifndef DART_LAUNCHER_WEB_V3_ROBOT_CONFIG_H
#define DART_LAUNCHER_WEB_V3_ROBOT_CONFIG_H

#define DART_V5_VERSION "5.0"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "dji_motor.h"
#include "motor_def.h"
#include "controller.h"

/* ================= CAN / 电机 ID ================= */
#define DART_CAN_HANDLE (&hcan1)

#define DART_SPRING_A_ID 2
#define DART_SPRING_B_ID 3
#define DART_TRIGGER_ID  4
#define DART_YAW_ID      1

#define DART_M3508_GEAR_RATIO 19.2032f
#define DART_M2006_GEAR_RATIO 36.0f

/* 方向 (MOTOR_DIRECTION_NORMAL/REVERSE); 网页可运行时切换并保存 */
#define DART_SPRING_A_REVERSE MOTOR_DIRECTION_NORMAL
#define DART_SPRING_B_REVERSE MOTOR_DIRECTION_NORMAL
#define DART_TRIGGER_REVERSE  MOTOR_DIRECTION_NORMAL
#define DART_YAW_REVERSE      MOTOR_DIRECTION_NORMAL

/* ================= 串口 ================= */
#define DART_UART_HANDLE (&huart6)
#define DART_RECV_SIZE   128
#define DART_CMD_TIMEOUT_MS 2000u
#define DART_FB_PERIOD_MS   150u

/* ================= 拉簧/圈数 ================= */
#define DART_SPRING_PREP_TURNS 5.0f
#define DART_SPRING_MAX_TURNS  30.0f
#define DART_POS_TOL_TURNS     0.03f
#define DART_TASK_STEP_TIMEOUT_MS 8000u

/* ================= 扳机舵机 (PWM) ================= */
#define DART_SERVO_TIM     (&htim1)
#define DART_SERVO_CHANNEL TIM_CHANNEL_1
#define DART_SERVO_PERIOD_S 0.02f
#define DART_SERVO_MIN_US  500.0f
#define DART_SERVO_MAX_US  2500.0f
#define DART_SERVO_DEG_RANGE 270.0f
#define DART_SERVO_STD_DEG  0.0f
#define DART_SERVO_PREP_DEG 90.0f

/* ================= Yaw / 视觉 ================= */
#define DART_VIS_CENTER      720      // 1440x1080 画面中心
#define DART_YAW_AIM_KP      3.00f    // 自瞄 P: 每 1 像素误差输出的转速(rpm)
#define DART_YAW_AIM_KI      5.00f    // 自瞄 I: 积分消除静差, 加速收敛
#define DART_YAW_AIM_I_LIMIT 200.0f   // 积分限幅(px*s)
#define DART_YAW_AIM_DEADBAND 2       // 自瞄死区(像素)
#define DART_YAW_AIM_RPM     200.0f   // 自瞄最大转速(输出rpm), 网页可设
#define DART_YAW_AIM_MIN_RPM 30.0f    // 自瞄最小转速: 克服静摩擦, 防末端蠕动
#define DART_YAW_GUIDE_DEG   0.0f
#define DART_VIS_TIMEOUT_MS  500u

/* ================= DJI 电机初始化配置宏 ================= */
/* M3508: 速度+角度双环, 速度环带堵转检测 */
#define DART_M3508_CONFIG(_id, _rev)                                                              \
  {                                                                                               \
    .motor_type = M3508,                                                                          \
    .can_init_config = {.can_handle = DART_CAN_HANDLE, .tx_id = _id},                             \
    .controller_setting_init_config =                                                             \
        {                                                                                         \
            .angle_feedback_source = MOTOR_FEED,                                                  \
            .speed_feedback_source = MOTOR_FEED,                                                  \
            .outer_loop_type = SPEED_LOOP,                                                        \
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                           \
            .motor_reverse_flag = _rev,                                                           \
            .feedback_reverse_flag = _rev,                                                        \
        },                                                                                        \
    .controller_param_init_config =                                                               \
        {                                                                                         \
            .speed_PID =                                                                          \
                {.Kp = 2.5f, .Ki = 0.1f, .Kd = 0.0f, .MaxOut = 16384.0f, .DeadBand = 10.0f,       \
                 .IntegralLimit = 800.0f,                                                         \
                 .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle},      \
            .angle_PID =                                                                          \
                {.Kp = 5.0f, .Ki = 0.0f, .Kd = 0.3f, .MaxOut = 13826.0f, .DeadBand = 8.0f,        \
                 .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement},                  \
        },                                                                                        \
  }

/* M2006: 速度+角度双环 */
#define DART_M2006_CONFIG(_id, _rev)                                                              \
  {                                                                                               \
    .motor_type = M2006,                                                                          \
    .can_init_config = {.can_handle = DART_CAN_HANDLE, .tx_id = _id},                             \
    .controller_setting_init_config =                                                             \
        {                                                                                         \
            .angle_feedback_source = MOTOR_FEED,                                                  \
            .speed_feedback_source = MOTOR_FEED,                                                  \
            .outer_loop_type = SPEED_LOOP,                                                        \
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                           \
            .motor_reverse_flag = _rev,                                                           \
            .feedback_reverse_flag = _rev,                                                        \
        },                                                                                        \
    .controller_param_init_config =                                                               \
        {                                                                                         \
            .speed_PID =                                                                          \
                {.Kp = 3.0f, .Ki = 0.3f, .Kd = 0.0f, .MaxOut = 10000.0f, .DeadBand = 3.0f,        \
                 .IntegralLimit = 1000.0f,                                                        \
                 .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle},      \
            .angle_PID =                                                                          \
                {.Kp = 5.0f, .Ki = 0.0f, .Kd = 0.3f, .MaxOut = 25920.0f, .DeadBand = 2.0f,        \
                 .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement},                  \
        },                                                                                        \
  }

#endif  // DART_LAUNCHER_WEB_V3_ROBOT_CONFIG_H
