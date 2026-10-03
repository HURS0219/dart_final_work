/*
 * ctrl/ctrl.h — 大脑 (robot_cmd): 解析上位机指令 -> 填各组件 ctrl_cmd
 */
#pragma once

#include <stdint.h>

#include "robot_def.h"

void CtrlInit(void);
void CtrlTask(void);

/* 供 store 读写的可持久化设定 */
float CtrlGetSpringTurns(void);
void CtrlSetSpringTurns(float t);
uint8_t CtrlGetYawMode(void);
void CtrlSetYawMode(uint8_t m);
float CtrlGetAimRpm(void);
void CtrlSetAimRpm(float r);
uint8_t CtrlGetTaskStep(void);
uint8_t CtrlGetEstop(void);
