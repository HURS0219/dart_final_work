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
