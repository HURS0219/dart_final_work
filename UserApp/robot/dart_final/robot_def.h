/*
 * robot_def.h — dart_final 共享定义: 话题名 / 健康位 / 跨 app 数据结构
 * =============================================================================
 * 只放“应用之间共享”的东西; 不放任何实现, 不放全局变量。
 * 通信结构体用 #pragma pack(1) 包裹, 取消字节对齐(遵循框架约定), 便于 memcpy 传输。
 * =============================================================================
 */
#pragma once

#ifndef DART_FINAL_ROBOT_DEF_H
#define DART_FINAL_ROBOT_DEF_H

#include <stdbool.h>
#include <stdint.h>

/* ============================ message_center 话题名 ============================ */
#define TOPIC_ATTITUDE "attitude"  /* imu   -> 姿态        (Dart_Attitude_s) */
#define TOPIC_TARGET "target"      /* vision-> 视觉目标     (Dart_Target_s)   */
#define TOPIC_MIX "mix"            /* guidance -> 舵面指令  (Dart_Mix_s)      */
#define TOPIC_SERVO_FB "servo_fb"  /* fin   -> 舵面反馈     (Dart_ServoFb_s)  */

/* ================================ 健康/错误位 ================================= */
/* Monitor 汇总各 app 的 err; 每位一个故障, 便于“一眼定位是哪个 app 异常” */
#define DART_ERR_NONE 0x0000u
#define DART_ERR_IMU_OFF 0x0001u   /* IMU/姿态 app 卡死或姿态无效 */
#define DART_ERR_VISION_OFF 0x0002u /* 视觉超时(未收到目标帧) */
#define DART_ERR_GUID_OFF 0x0004u  /* 制导 app 卡死 */
#define DART_ERR_FIN_OFF 0x0008u   /* 舵面 app 卡死 */
#define DART_ERR_SERVO 0x0010u     /* 舵机注册失败(servo_mix_ai) */

/* ============================== 跨 app 数据结构 =============================== */
/* 约定: 所有“发布/订阅”的主题负载都以此定义; 每个结构末尾带 tick(发布时刻 ms)用于判活。*/

#pragma pack(1)

/** @brief 姿态(imu 发布) */
typedef struct {
  float roll_deg;   /* 横滚角(deg) */
  float pitch_deg;  /* 俯仰角(deg) */
  float yaw_deg;    /* 偏航角(deg) */
  float gx_dps;     /* 机体系 x 角速率(deg/s), 对应 roll */
  float gy_dps;     /* 机体系 y 角速率(deg/s), 对应 pitch */
  float gz_dps;     /* 机体系 z 角速率(deg/s), 对应 yaw */
  uint32_t tick;    /* 发布时刻(ms) */
  uint8_t valid;    /* 1=本周期姿态有效 */
} Dart_Attitude_s;

/** @brief 视觉目标(vision 发布) —— 对应 OpenMV 9 字节帧解码结果 */
typedef struct {
  int16_t x;        /* 目标中心像素 x */
  int16_t y;        /* 目标中心像素 y */
  uint8_t w;        /* 目标框宽(像素) */
  uint8_t h;        /* 目标框高(像素) */
  uint8_t found;    /* 1=本帧识别到绿光 */
  uint32_t tick;    /* 收到时刻(ms) */
} Dart_Target_s;

/** @brief 舵面混控指令(guidance 发布) —— 归一化 -1..1 */
typedef struct {
  float pitch;       /* 俯仰指令(本机恒 0) */
  float yaw;         /* 偏航指令(制导) */
  float roll;        /* 滚转指令(姿态稳定) */
  uint8_t failsafe;  /* 1=失效, 舵面回中 */
  uint32_t tick;     /* 发布时刻(ms) */
} Dart_Mix_s;

/** @brief 舵面反馈(fin 发布) */
typedef struct {
  float defl_deg[4];  /* 4 路目标逻辑角(deg), 下标 0..3 = 舵面编号 */
  float pulse_us[4];  /* 4 路当前脉宽(us) */
  uint32_t tick;      /* 发布时刻(ms) */
} Dart_ServoFb_s;

#pragma pack()

/** @brief 每个 app 的健康状态(Monitor 通过 GetStatus() 读取, 不走话题) */
typedef struct {
  uint32_t hb;     /* 心跳: app 每执行一次 +1; Monitor 比对是否更新 */
  uint16_t err;    /* 错误位(DART_ERR_* 或运算) */
  float dt_us;     /* 上次任务耗时(us), 用于排查卡顿 */
  float key;       /* 关键量(调试观察用, 含义见各 app 注释) */
} Dart_AppStatus_s;

#endif  // DART_FINAL_ROBOT_DEF_H
