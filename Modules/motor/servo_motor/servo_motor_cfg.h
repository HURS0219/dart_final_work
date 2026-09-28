/**
 * @file servo_motor_cfg.h
 * @author ai
 * @brief PWM 舵机(PTK7350)驱动模块的“可调默认参数” (集中调参入口)
 * @version 1.0
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention
 *  本文件只放“可调参数的默认值”, 目的是把散落的魔法数字集中到一处, 方便现场直接改源码 + 烧录
 *  (本阶段不做掉电保存)。分两类:
 *
 *    [信号层] center_us / half_us / half_deg / pulse_min_us / pulse_max_us
 *             = 舵机自身的“脉宽 <-> 机械角”规格, 同一型号 4 路相同, 通常不碰, 换型号才改。
 *
 *    [逻辑层] scale / trim_deg / reverse / limit_deg / rate_limit_dps / zero_enable
 *             = 装机后按实际安装逐个标定的量(每个舵机不同), 现场主要调这些。
 *             注: 本文件给的是“单舵机默认值/兜底值”; 本飞镖 4 路的具体标定在
 *                 Modules/algorithm/servo_mix_ai/servo_mix_ai_cfg.h 里逐路设置。
 *
 *  标定模型(见 servo_motor.md《调参指南》):
 *      applied = 逻辑角 * scale + trim_deg        (机械偏角, deg; reverse 时取负)
 *      pulse   = center_us + applied * (half_us / half_deg)
 *  - 比例/行程不对 -> 改 scale;  零点/中位偏 -> 改 trim;  方向反 -> 改 reverse。
 */
#ifndef SERVO_MOTOR_CFG_H
#define SERVO_MOTOR_CFG_H

/* ============================================================================
 *                              信号层参数 (型号规格)
 * ==========================================================================*/

/*
 * 机械 0°(中位) 对应的 PWM 高电平脉宽, 单位: us。
 * 来源: PTK7350MG-D 实测, 1500us = 机械中位。一般 4 路相同。
 * 改动影响: 所有舵机中位整体平移; 除非换到“中位脉宽不同”的舵机, 否则不要动。
 * 注意: 调“零点”请不要改这里, 应该改 trim_deg(逻辑层)。
 */
#define SERVO_CFG_CENTER_US (1500.0f)

/*
 * 从“中位”到“行程一端”对应的脉宽跨度, 单位: us。即 pulse_min = center-half, pulse_max = center+half。
 * 来源: PTK7350MG-D 500~2500us, 故 half = (2500-500)/2 = 1000us。
 * 作用: 与 HALF_DEG 一起给出“每度多少微秒”: half_us / half_deg ≈ 7.17 us/°。
 * 改动影响: 脉宽-角度比例(整条刻度); 换到脉宽范围不同的舵机时才改。
 */
#define SERVO_CFG_HALF_US (1000.0f)

/*
 * 从“中位”到“行程一端”对应的机械角跨度(半行程), 单位: deg。
 * 来源: PTK7350MG-D 总行程 279°, 故 half = 139.5°。
 * 作用: 与 HALF_US 一起定义脉宽-角度比例; 换到 180° 型舵机时改为 90°。
 * 改动影响: 整条角度刻度; 一般不动。
 */
#define SERVO_CFG_HALF_DEG (139.5f)

/*
 * 脉宽硬下限, 单位: us。输出脉宽会被夹到此值以上(防越程/舵机翻转)。
 *  正常应等于 center - half (1500-1000=500)。
 */
#define SERVO_CFG_PULSE_MIN_US (500.0f)

/*
 * 脉宽硬上限, 单位: us。输出脉宽会被夹到此值以下(防越程/舵机翻转)。
 *  正常应等于 center + half (1500+1000=2500)。
 */
#define SERVO_CFG_PULSE_MAX_US (2500.0f)

/* ============================================================================
 *                        逻辑层默认值 (装机标定; 逐路以 servo_mix_ai_cfg.h 为准)
 * ==========================================================================*/

/*
 * 逻辑角 -> 机械角 增益(无量纲), 必须 > 0。1.0 表示 1:1。
 * 用途: 当“指令角度 : 实际偏角”比例不对时改它(斜率)。
 *       例: 指令 30° 实际偏了 33°, 说明比例偏大, 把 scale 按 30/33≈0.91 缩小。
 * 注意: 改 scale 不影响零点(angle=0 时 applied=trim, 与 scale 无关)。默认 1.0。
 */
#define SERVO_CFG_SCALE (1.0f)

/*
 * 零点(中立)机械偏移, 单位: deg。logical=0 时的机械偏角 = trim。
 * 用途: 当“指令 0° 时舵面停的位置偏了”时改它(平移)。也可以调用 ServoZero() 自动对零。
 * 注意: 改 trim 不影响比例; reverse 在 trim 之后取负, 故翻转方向后需重新对零。默认 0。
 */
#define SERVO_CFG_TRIM_DEG (0.0f)

/*
 * 方向: 0 = 正常, 1 = 反向(整体取负)。
 * 用途: 指令“正”而舵面“反”时置 1。
 * 注意: 反向等效于把当前偏角与零点都取负, 改动后要重新确认零点。默认 0。
 */
#define SERVO_CFG_REVERSE (0)

/*
 * 逻辑角对称限位, 单位: deg。逻辑角会被夹到 ±limit_deg。
 * 用途: 保护舵面/机构不越程, 同时限制混控满偏范围。
 * 注意: <= 0 时视为“不限位”, 自动采用 half_deg(满行程)。默认 90。
 */
#define SERVO_CFG_LIMIT_DEG (90.0f)

/*
 * 速率限幅, 单位: deg/s。> 0 时 ServoTask() 会把逻辑角以该速率平滑逼近目标; <= 0 表示不限速(立即到位)。
 * 用途: 减小舵机冲击/电流, 让动作平滑。默认 0(不限速)。
 */
#define SERVO_CFG_RATE_DPS (0.0f)

/*
 * 调零功能总开关: 1 = 允许 ServoZero() 调零, 0 = 禁用(返回 0, 不做任何改动)。
 * 用途: 某些安装不需要/不允许调零时可关闭。默认 1(启用)。
 */
#define SERVO_CFG_ZERO_ENABLE (1)

/*
 * 调零安全窗口, 单位: deg。仅当“当前机械偏角绝对值 ≤ 本值”时才允许调零。
 * 用途: 防止在极端/饱和位置误触发调零导致零点被写坏。默认 30(即中位 ±30° 内)。
 */
#define SERVO_CFG_ZERO_WINDOW_DEG (30.0f)

#endif /* SERVO_MOTOR_CFG_H */
