/*
 * guidance.h — 制导+控制 app: 订阅 "attitude"+"target", 解算 -> 发布 "mix"
 */
#pragma once

#include "robot_def.h"

/* 初始化: 初始化 png_ai 与 roll PID; 注册订阅/发布 */
void Guidance_Init(void);

/**
 * @brief 周期任务
 * @param dt           本周期时间(s)
 * @param guide_enable 1=状态机处于 GUIDING(允许制导); 0=失效/未制导(输出回中指令)
 */
void Guidance_Task(float dt, uint8_t guide_enable);

/* 健康状态(Monitor 读取) */
const Dart_AppStatus_s *Guidance_GetStatus(void);
