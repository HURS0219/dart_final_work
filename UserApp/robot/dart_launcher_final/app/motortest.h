/*
 * motortest.h — 最小可用电机测试 app(只用 DJIMotor module 接口)
 * =============================================================================
 * 【目的】把"让一个 M3508 转到指定角度"这件事做到最简单、可验证。
 *         不参与状态机、不做急停策略、不做多路抽象, 只保留:
 *           取零 -> 发目标角度 -> 用角度环闭环 -> 反馈当前角度
 *
 * 【为什么重写】原 app/motor.c 引入过多层次(保持型目标/急停策略/速度环外挂),
 *   在方向符号与闭环组合上反复出错(正反馈冲飞、NaN 不动)。这里回到最小集合。
 *
 * 【依赖】Modules/motor/DJImotor 的公开接口:
 *   DJIMotorInit / DJIMotorEnable / DJIMotorStop / DJIMotorOuterLoop
 *   DJIMotorSetPIDRef / DJIMotorSetRef
 *   DJIMotorTask() 由 os_task.c 的 MotorControlTask() 调用, app 不要自己调。
 * =============================================================================
 */
#pragma once

#include <stdint.h>

#include "robot_def.h" /* Launcher_AppStatus_s */

/** @brief 初始化电机(CAN1, 按 launcher_cfg.h 的 ID/方向) */
void Motortest_Init(void);

/** @brief 周期任务: 刷新在线状态/取零, 按目标下发角度环 */
void Motortest_Task(void);

/* ---------------- 供调试/联调的最小控制接口 ---------------- */

/** @brief 设定第 slot 路的目标输出角度(deg, 相对零点); 内部自动切到角度环 */
void Motortest_SetAngle(int slot, float deg);

/** @brief 停止第 slot 路(卸力, DJIMotorStop) */
void Motortest_Stop(int slot);

/** @brief 把第 slot 路当前位置记为 0° */
void Motortest_Zero(int slot);

/** @brief 读第 slot 路当前输出角度(deg, 相对零点) */
float Motortest_GetAngle(int slot);

/** @brief 读第 slot 路输出转速(rpm) */
float Motortest_GetRpm(int slot);

/** @brief 1=该路在线(CAN 有回传) */
uint8_t Motortest_IsOnline(int slot);

/** @brief 1=该路已到位(在容差内) */
uint8_t Motortest_AtTarget(int slot);

/** @brief 读该路最终下发的电流值(用于调试判断出力) */
float Motortest_GetOutput(int slot);

/** @brief 健康状态(Monitor 读取, 与其它 app 同构) */
const Launcher_AppStatus_s *Motortest_GetStatus(void);
