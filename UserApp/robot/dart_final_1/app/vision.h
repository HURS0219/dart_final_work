/*
 * vision.h — 视觉 app: 解析 OpenMV 经串口发来的 9 字节帧, 发布 "target" 话题
 */
#pragma once

#include "robot_def.h"

/* 初始化: 注册串口(回调在中断里解析帧) + 注册话题 */
void Vision_Init(void);

/* 周期任务(app 主循环调用): 把中断里解好的帧发布出去; 超时则置 found=0 */
void Vision_Task(void);

/* 健康状态(Monitor 读取) */
const Dart_AppStatus_s *Vision_GetStatus(void);

/* 读取最近一次目标快照(供状态机等使用) */
void Vision_GetTarget(Dart_Target_s *out);

/* 收帧统计: ok=校验通过, bad=收到但校验失败(用于判断链路是否通) */
void Vision_GetStats(uint32_t *ok, uint32_t *bad);
