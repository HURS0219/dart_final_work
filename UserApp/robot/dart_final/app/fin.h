/*
 * fin.h — 舵面 app: 订阅 "mix", 用 servo_mix_ai 驱动 4 路舵机, 发布 "servo_fb"
 */
#pragma once

#include "robot_def.h"

/* 初始化: servo_mix_ai 注册 4 路舵机(上电回中) + 注册话题订阅/发布 */
void Fin_Init(void);

/* 周期任务(app 主循环调用): 取指令 -> 驱动 4 舵机 -> 发布反馈 */
void Fin_Task(void);

/* 健康状态(Monitor 读取) */
const Dart_AppStatus_s *Fin_GetStatus(void);

/* ---------------- 测试接口(供 dart_final_test_app 使用; 生产 app 不用) ---------------- */
void Fin_SetMode(uint8_t manual);           /* 0=MIX 混控, 1=MANUAL 逐路手动 */
void Fin_SetManual(uint8_t ch, float deg);  /* MANUAL: 给第 ch 路逻辑角(deg) */
uint8_t Fin_Zero(uint8_t ch);               /* 对第 ch 路调零(当前位置记为 0°) */
