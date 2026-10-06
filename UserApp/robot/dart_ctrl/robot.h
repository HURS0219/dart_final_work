/*
 * robot.h — dart_ctrl · 制导飞镖整机飞控 (组合根直连, 无 PubSub)
 * =============================================================================
 * 由 dart_final 改写而来: 去掉 message_center 发布订阅, 各子功能(imu/vision/
 * guidance/fin)不再拆成独立 app, 而是收进 robot.c, 经本 RobotInstance 直连数据。
 *
 * 对外唯二入口(由 UserApp/os_task.c 调用):
 *   RobotInit() —— 上电初始化
 *   RobotTask() —— 周期任务
 * =============================================================================
 */
#pragma once

#ifndef DART_CTRL_ROBOT_H
#define DART_CTRL_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

/* ================================ 健康/错误位 ================================= */
#define DART_ERR_NONE 0x0000u
#define DART_ERR_IMU_OFF 0x0001u    /* 姿态无效/IMU 异常 */
#define DART_ERR_VISION_OFF 0x0002u /* 视觉超时(未收到目标帧) —— 非致命 */
#define DART_ERR_GUID_OFF 0x0004u   /* 制导异常(预留) */
#define DART_ERR_FIN_OFF 0x0008u    /* 舵面异常(预留) */
#define DART_ERR_SERVO 0x0010u      /* 舵机注册失败 */

/* ============================== 共享数据结构 =============================== */
#pragma pack(1)

/** @brief 姿态(IMU 产出) */
typedef struct {
  float roll_deg;   /* 横滚角(deg) */
  float pitch_deg;  /* 俯仰角(deg) */
  float yaw_deg;    /* 偏航角(deg) */
  float gx_dps;     /* 机体系 x 角速率(deg/s), 对应 roll */
  float gy_dps;     /* 机体系 y 角速率(deg/s), 对应 pitch */
  float gz_dps;     /* 机体系 z 角速率(deg/s), 对应 yaw */
  uint32_t tick;    /* 更新时刻(ms) */
  uint8_t valid;    /* 1=本周期姿态有效 */
} Dart_Attitude_s;

/** @brief 视觉目标(vision 产出) —— OpenMV 7 字节帧解码结果 */
typedef struct {
  int16_t x;      /* 目标中心像素 x */
  int16_t y;      /* 目标中心像素 y */
  uint8_t found;  /* 1=本帧识别到绿光 */
  uint32_t tick;  /* 收到时刻(ms) */
} Dart_Target_s;

/** @brief 舵面混控指令(guidance 产出) —— 归一化 -1..1 */
typedef struct {
  float pitch;      /* 俯仰指令(本机恒 0) */
  float yaw;        /* 偏航指令(制导) */
  float roll;       /* 滚转指令(姿态稳定) */
  uint8_t failsafe; /* 1=失效, 舵面回中 */
  uint32_t tick;    /* 发布时刻(ms) */
} Dart_Mix_s;

/** @brief 舵面反馈(fin 产出) */
typedef struct {
  float defl_deg[4]; /* 4 路目标逻辑角(deg) */
  float pulse_us[4]; /* 4 路当前脉宽(us) */
  uint32_t tick;     /* 更新时刻(ms) */
} Dart_ServoFb_s;

#pragma pack()

/* ============================== 飞行状态机 ============================== */
typedef enum {
  DART_IDLE = 0, /* 未使能: 舵面回中 */
  DART_ARMED,    /* 已使能, 待制导触发: 舵面回中 */
  DART_GUIDING,  /* 制导中 */
  DART_FAULT,    /* 失效: 舵面回中 */
} DartState_e;

/** @brief 整机实例(组合根: 集中各子功能产出, 供直连读取) */
typedef struct {
  uint32_t DWT_CNT;  /* DWT 计时句柄 */
  float dt;          /* 本周期时间(s) */

  Dart_Attitude_s attitude; /* 来自 IMU 步骤 */
  Dart_Target_s target;     /* 来自视觉步骤 */
  Dart_Mix_s mix;           /* 来自制导步骤 */
  Dart_ServoFb_s servo_fb;  /* 来自舵面步骤 */
} RobotInstance;

extern RobotInstance* robot;

/** @brief 上电初始化 */
void RobotInit(void);

/** @brief 周期任务: IMU → 视觉 → 监控/状态机 → 制导 → 舵面 → 遥测 */
void RobotTask(void);

#endif  // DART_CTRL_ROBOT_H
