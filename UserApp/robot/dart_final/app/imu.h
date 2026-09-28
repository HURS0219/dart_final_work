/*
 * imu.h — 姿态 app: 封装 Modules/imu(ins_task), 发布 "attitude" 话题
 */
#pragma once

#include "robot_def.h"

/* 初始化: INS_Init(内部会自建 1kHz INS 任务并初始化 BMI088); 注册话题 */
void Imu_Init(void);

/* 周期任务(app 主循环调用, ~1kHz): 取姿态 -> 发布 "attitude" */
void Imu_Task(void);

/* 健康状态(Monitor 读取) */
const Dart_AppStatus_s *Imu_GetStatus(void);

/* 读取最近一次姿态快照(供状态机等使用) */
void Imu_GetAttitude(Dart_Attitude_s *out);
