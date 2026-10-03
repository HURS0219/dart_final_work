/*
 * launcher/link.h — 上位机通信组件 (USART6 + J-Link RTT)
 * =============================================================================
 * 职责: 把收到的 ASCII 行解析成 Launcher_Cmd_s(不做机构语义); 按周期发遥测。
 * ctrl 通过 LinkGetCmd() 取走指令; robot 通过 LinkUpdateTelemetry() 提供快照。
 * 无 ESP 时, 走 J-Link RTT 专用通道 1(与串口协议一致)。
 * =============================================================================
 */
#pragma once

#include <stdint.h>

#include "robot_def.h"

void LinkInit(void);
void LinkTask(void);

/* 取走本批解析出的指令; 返回 1 表示有有效指令(取走后清空) */
uint8_t LinkGetCmd(Launcher_Cmd_s* out);

/* 链路活性: 从未收到数据 或 最近在超时内收到过 -> 1 */
uint8_t LinkIsAlive(void);

/* robot 每周期更新遥测快照(link 按 DART_FB_PERIOD_MS 发送; 也用于 S 扫描回包) */
void LinkUpdateTelemetry(const Launcher_Telemetry_s* t);

void LinkSend(const char* s);
