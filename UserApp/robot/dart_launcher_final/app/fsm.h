/*
 * fsm.h — 时序 app: 自动发射流程状态机(独立 app)
 * =============================================================================
 * 【职责】只负责"自动流程的步骤推进", 不管单步怎么执行:
 *   订阅 "fsm_cmd"  (Launcher_FsmCmd_s)  —— 启动/停止/急停/单步(来自 cmd)
 *   订阅 "motor_fb" (Launcher_MotorFb_s)—— 靠"到位"反馈推进步骤
 *   订阅 "servo_fb" (Launcher_ServoFb_s)—— 舵机无位置反馈, 靠固定延时
 *   发布 "motor_cmd" (Launcher_MotorCmd_s) —— 拉簧目标(逐路独立角度)
 *   发布 "servo_cmd" (Launcher_ServoCmd_s) —— 舵机目标位
 *   发布 "fsm_fb"   (Launcher_FsmFb_s)   —— 当前步/完成/超时(供 cmd 汇总)
 *
 * 【为什么独立成 app】时序逻辑(步骤/超时/到位判定)本身够复杂, 且与"指令解析"
 *   是两件事; 拆开后 cmd 只管"把用户指令翻译成目标", fsm 只管"按剧本推进"。
 *
 * 【丝杆为何不参与】丝杆是自锁的, 转到设定角度后由 app/motor 自行卸力保持,
 *   不需要时序协调, 故本 app 不碰丝杆。
 *
 * 【角度来源】每一步各机构该到的角度全部取自 launcher_cfg.h 的
 *   LAUNCH_STEP_* 宏 —— 现场调参只改 cfg, 不动本文件。
 * =============================================================================
 */
#pragma once

#include "robot_def.h"

/** @brief 初始化(注册话题) */
void Fsm_Init(void);

/** @brief 周期任务: 推进状态机 */
void Fsm_Task(void);

/** @brief 健康状态(Monitor 读取) */
const Launcher_AppStatus_s *Fsm_GetStatus(void);
