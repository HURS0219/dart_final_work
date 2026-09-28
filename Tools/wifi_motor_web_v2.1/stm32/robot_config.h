/*
 * robot_config.h — WiFi 手机控制 GM6020 (wifi_gm6020) 参数
 *
 * 电机用最简直接 CAN 接口 (simple_motor), 不走 dji_motor/电机任务。
 *
 * 通信协议 (ESP32 -> STM32, ASCII, '\n' 结尾):  C,<mode>,<value>
 *   mode 0 停止 / 1 速度(RPM) / 2 角度(度 x10)
 */
#ifndef WIFI_GM6020_ROBOT_CONFIG_H
#define WIFI_GM6020_ROBOT_CONFIG_H

#define WIFI_GM6020_VERSION "1.0"

#include "main.h"
#include "usart.h"
#include "simple_motor.h"

/* ================= 电机 ================= */
#define SIMPLE_MOTOR_TYPE SIMPLE_MOTOR_GM6020  // 电机类型: SIMPLE_MOTOR_GM6020 / _M3508 / _M2006
#define SIMPLE_MOTOR_ID   2                    // 电调拨码 ID
#define GM6020_MAX_RPM    120.0f   // 速度模式限幅 (RPM)
#define GM6020_MAX_VALUE  10000    // 输出电压硬上限 (任何模式都不超过)
#define GM6020_ANGLE_MIN  (-180.0f)
#define GM6020_ANGLE_MAX  (180.0f)
#define WIFI_ANGLE_SCALE  10.0f    // 协议角度值 = 角度 * 10
/* 角度模式固定绝对零点(GM6020 绝对编码器): 0 对应编码器 ecd=该值的方向。
 * 用页面“复位(设零)”可把当前物理方向设为 0; 想永久生效就把测到的 ecd 填这里。 */
#define ANGLE_ZERO_ECD    0.0f

/* 闭环速度: PI 控制 (用反馈 RPM), 输出仍是电压值 (-30000~30000) */
#define SPEED_FF        90.0f     // 前馈: 每 1 RPM 需要的电压值 (实测标定)
#define SPEED_KP        30.0f     // 每 1 RPM 误差输出的电压值
#define SPEED_KI        10.0f     // 积分系数
#define SPEED_I_LIMIT   5000.0f   // 积分限幅 (抗饱和, 防过冲)
#define SPEED_CTRL_DT   0.001f    // 控制周期 (RobotTask ~1kHz)

/* 角度模式: PD + 输出斜率限制 (基于反馈编码器) */
#define ECD_PER_DEG       22.7556f  // 8192 / 360
#define ANGLE_KP          60.0f     // 每 1 度误差输出的电压值 (调小更柔和)
#define ANGLE_KD          15.0f     // 阻尼: 每 1 RPM 反馈速度的抑制量 (调大更稳)
#define ANGLE_SLEW        40.0f     // 输出斜率限制: 每 1ms 输出最大变化量 (调小更柔和)

/* ================= ESP32 串口 ================= */
#define WIFI_UART_HANDLE  (&huart6)
#define WIFI_RECV_SIZE    32
#define WIFI_CMD_TIMEOUT_MS 1000u   // 超时无指令自动停车

#endif  // WIFI_GM6020_ROBOT_CONFIG_H
