/*
 * link.h — 通信 app: 上位机(ESP32 网页 / PC 直连)串口协议
 * =============================================================================
 * 【职责】本 app 是全工程**唯一**感知"上位机/网页存在"的地方:
 *   下行: 串口收 ASCII 行 -> 解析成 Launcher_Cmd_s -> 发 "launch_cmd" 话题
 *         特殊: PING 直接回 PONG; H 心跳仅刷新活性; S 扫描直接回结果
 *   上行: 订阅各 *_fb + "launch_state" -> 拼成一帧 "F,..." -> 串口发出
 *
 * 【与其他 app 的关系】link **不做任何机构语义**, 只做"文本 <-> 结构体"。
 *   怎么动由 app/cmd 与 app/fsm 决定。因此换掉上位机(ESP32/Jetson/直连)
 *   只需改本文件, 其余 6 个 app 一行不动。
 *
 * 【协议契约】改动需同步 esp32/ 与 pc_vision/:
 *   下行: PING | H | S | Z[,slot] | M,slot,mode,value | P,slot,id,value
 *         | R,slot | D,slot | N,slot,deg | Y,mode | A,rpm100
 *         | C,x,center | V,a[,b] | G,cmd
 *   上行: F,<n>,<每电机12项>*n,<舵机4项>,<任务5项>,<视觉4项>,<每电机11参数>*n
 *   注意: 旧的 W,turns100(圈) 已废弃(改角度制), 收到 W 回 "ERR" 明确拒绝,
 *         避免旧上位机静默发出错误的角度值。
 * =============================================================================
 */
#pragma once

#include "robot_def.h"

/** @brief 初始化串口 + 注册话题 */
void Link_Init(void);

/** @brief 周期任务: 处理收帧(回调里) + 周期发遥测 + 判链路活性 */
void Link_Task(void);

/** @brief 健康状态(Monitor 读取) */
const Launcher_AppStatus_s *Link_GetStatus(void);
