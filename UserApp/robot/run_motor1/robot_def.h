/*
 * robot_def.h — run_motor1 跨 app 共享定义(话题名 + 通信结构体)
 * =============================================================================
 * 遵循 application.md: app 之间只用 message_center 发布-订阅通信, 不相互包含。
 *   run_motor_fb  : 本 app 发布(电机反馈) —— 供上位机/其它 app 订阅
 *   run_motor_cmd : 本 app 订阅(运行指令) —— 供未来的 cmd app 下发(可选)
 *
 * 通信结构体用 #pragma pack(1) 包裹, 取消字节对齐(框架约定)。
 * =============================================================================
 */
#ifndef RUN_MOTOR1_ROBOT_DEF_H
#define RUN_MOTOR1_ROBOT_DEF_H

#include <stdint.h>

/* ============================ message_center 话题名 ============================ */
#define TOPIC_RUN_MOTOR_CMD "run_motor_cmd"  /* cmd -> 本 app (RunMotor_Cmd_s) */
#define TOPIC_RUN_MOTOR_FB "run_motor_fb"    /* 本 app -> 订阅者 (RunMotor_Fb_s) */

#pragma pack(1)

/* @brief 运行指令(订阅; 没有 cmd app 时本 app 用 cfg 默认值自动旋转) */
typedef struct {
  uint8_t enable;      /* 1=运行, 0=停止(关闭输出) */
  float target_dps;    /* 输出轴目标转速(deg/s): >0 正转, <0 反转 */
  uint32_t tick;       /* 发布时刻(ms) */
} RunMotor_Cmd_s;

/* @brief 电机反馈(发布) —— 全部为输出侧量, 已含方向符号 */
typedef struct {
  float angle_deg;     /* 输出轴角度(deg, 相对上电零点) */
  float speed_dps;     /* 输出轴转速(deg/s) */
  float target_deg;    /* 当前角度环目标(deg, 输出侧) */
  int16_t current;     /* 实际电流(原始值) */
  uint8_t online;      /* 1=CAN 有回传 */
  uint8_t enabled;     /* 1=已使能(响应目标) */
  uint32_t tick;       /* 发布时刻(ms) */
} RunMotor_Fb_s;

#pragma pack()

#endif  // RUN_MOTOR1_ROBOT_DEF_H
