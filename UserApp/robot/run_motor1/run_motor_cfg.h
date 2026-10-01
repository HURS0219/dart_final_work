/*
 * run_motor_cfg.h — run_motor1 全部可调参数(台架"转一个电机"入口)
 * =============================================================================
 * 目标: 用最小 app 调通 motor module(DJIMotor), 让一个 M3508 按角度环连续旋转。
 *
 * 【单位约定(与本工程一致: 角度制 / 输出侧)】
 *   角度 = 输出轴 deg(相对上电零点); 转速 = 输出轴 deg/s。
 *   减速比 ratio = 转子:输出(M3508=19.2032, M2006=36.0, GM6020=1.0)。
 *   库的 measure.total_angle / speed_aps 都是**转子侧**, 换算成输出侧必除 ratio。
 *
 * 【怎么调参(只改本文件 + 烧录)】
 *   电机不转/乱转       -> RUN_MOTOR_TYPE / RUN_MOTOR_TX_ID / RUN_MOTOR_CAN 与实物对不上
 *   转向反了            -> RUN_MOTOR_REVERSE 取另一个值(改后重新上电取零)
 *   转太快/太慢         -> RUN_MOTOR_TARGET_DPS(输出轴 deg/s)
 *   起步发烫/电流大     -> RUN_MOTOR_SPEED_MAXOUT(电流原始值上限)
 *   跟踪差/抖动/追不上  -> RUN_MOTOR_ANGLE_MAXOUT 或下面两个环的 PID
 *
 * 【安全】默认 RUN_MOTOR_SPEED_MAXOUT 只给约 1/4 满量程。确认方向正确后再逐步加大;
 *        绝不要一上来给满电流去试方向。
 * =============================================================================
 */
#ifndef RUN_MOTOR1_CFG_H
#define RUN_MOTOR1_CFG_H

#include "bsp_can.h"    /* hcan1 等 CAN 句柄 */
#include "motor_def.h"  /* M3508 / MOTOR_DIRECTION_* / Motor_Reverse_Flag_e */

/* -------------------- 0. 型号 / 接线(必须与实物一致, 否则不转) -------------------- */
#define RUN_MOTOR_TYPE M3508                          /* M3508 / M2006 / GM6020 */
#define RUN_MOTOR_TX_ID 2u                            /* 电调拨码 ID(1..8); 反馈为 0x200+ID */
#define RUN_MOTOR_CAN hcan1                           /* GIMBAL_BOARD(F407 C板) 的 CAN1 */
#define RUN_MOTOR_RATIO 19.2032f                      /* 减速比 转子:输出 */
#define RUN_MOTOR_REVERSE MOTOR_DIRECTION_NORMAL      /* 转向反了改成 MOTOR_DIRECTION_REVERSE */

/* -------------------- 1. 目标 -------------------- */
/* 运行模式: 0 = 连续匀速旋转(用 RUN_MOTOR_TARGET_DPS)
 *           1 = 转到 RUN_MOTOR_TARGET_DEG 并保持(给"给定角度") */
#define RUN_MOTOR_MODE 0
/* 模式 0: 输出轴目标转速(deg/s): >0 正转, <0 反转, 0 = 原地锁位。 */
#define RUN_MOTOR_TARGET_DPS 60.0f  /* 60 deg/s = 输出轴每 6 秒转一圈 */
/* 模式 1: 目标角度(输出轴 deg, 相对上电零点)。1000 = 输出轴转 1000°(约 2.78 圈)。 */
#define RUN_MOTOR_TARGET_DEG 1000.0f
/* 圈数上限(输出轴圈, 仅模式 0): 到达后停在限位(角度环锁住); 0 = 无限转。 */
#define RUN_MOTOR_MAX_ROUNDS 0.0f

/* -------------------- 2. 限幅(安全) -------------------- */
/* 角度环输出限幅 = 转子侧转速上限(deg/s)。
 * 必须 >= |TARGET_DPS|*RATIO 才跟得上; 太小会"一直掉队/追不上"。 */
#define RUN_MOTOR_ANGLE_MAXOUT 2500.0f
/* 速度环输出限幅 = 电流原始值上限(±16384 满量程)。保守 ~1/4, 确认后再加。 */
#define RUN_MOTOR_SPEED_MAXOUT 4000.0f

/* -------------------- 3. 角度环 PID(外环: 角度 -> 速度设定) -------------------- */
/* DeadBand 必须为 0: 连续旋转时若进死区会清零输出, 表现为走走停停。 */
#define RUN_MOTOR_ANGLE_KP 5.0f
#define RUN_MOTOR_ANGLE_KI 0.0f
#define RUN_MOTOR_ANGLE_KD 0.3f

/* -------------------- 4. 速度环 PID(内环: 速度 -> 电流) —— 必须配置 -------------------- */
/* 库在 ANGLE|SPEED 串级下, 最终下发的是**速度环输出**; 速度环若漏配(Kp=Ki=0)
 * 会输出恒 0 -> 现象"目标角度对、电机一动不动"(踩过这个坑)。 */
#define RUN_MOTOR_SPEED_KP 2.5f
#define RUN_MOTOR_SPEED_KI 0.10f
#define RUN_MOTOR_SPEED_KD 0.0f
#define RUN_MOTOR_SPEED_DEADBAND 10.0f
#define RUN_MOTOR_SPEED_ILIMIT 800.0f

#endif  // RUN_MOTOR1_CFG_H
