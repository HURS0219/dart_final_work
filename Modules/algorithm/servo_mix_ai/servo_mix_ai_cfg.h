/**
 * @file servo_mix_ai_cfg.h
 * @author ai
 * @date 2026-09-28
 * @brief 四舵面混控模块 servo_mix_ai 的“本机配置 + 可调参数”
 * @version 1.0
 *
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention
 *  本文件是**现场调参主入口**(改这里 -> 直接烧录, 本阶段不做掉电保存)。包含三部分:
 *    1) 4 路舵机的硬件通道(TIM/CH);
 *    2) 4 路舵机各自的“装机标定”(scale / trim / reverse / limit / rate / zero_enable);
 *    3) 混控本身(满偏角度 MAX_DEG / 解耦矩阵 / 默认模式)。
 *
 *  [信号层] center_us / half_us / half_deg / pulse_min / pulse_max 属“型号规格”,
 *           统一放在 Modules/motor/servo_motor/servo_motor_cfg.h, 本文件直接复用, 不重复定义。
 *
 *  调参速查(详见 servo_motor.md《调参指南》):
 *    比例/行程不对 -> 改 SERVO_MIX_SCALE[ch]
 *    零点/中位偏   -> 改 SERVO_MIX_TRIM_DEG[ch]
 *    方向反        -> 改 SERVO_MIX_REVERSE[ch]
 *  其中 ch = 0..3 对应 4 路舵机, 顺序见下面的通道宏。
 */
#ifndef SERVO_MIX_AI_CFG_H
#define SERVO_MIX_AI_CFG_H

#include "servo_motor_cfg.h"  // 复用信号层参数 SERVO_CFG_*
#include "tim.h"              // &htim1 / TIM_CHANNEL_x

/* ============================================================================
 *                          1) 舵机数量 / 硬件通道
 * ==========================================================================*/

/*
 * 舵面(舵机)数量。制导飞镖为 X 型 4 舵面, 固定为 4。
 * 改动影响: 数组长度/循环边界; 本机型不要改。
 */
#define SERVO_MIX_N 4

/*
 * 4 路舵机共用的定时器句柄(C 板为 TIM1)。
 * 依据 CubeMX: PWM1~4 = TIM1_CH1~CH4 = PE9 / PE11 / PE13 / PE14, 50Hz。
 */
#define SERVO_MIX_TIM (&htim1)

/*
 * 4 路舵机各自的定时器通道, 顺序即“舵面编号 0/1/2/3”。
 * 与硬件对应关系(装机时确认): 0=CH1(右上) 1=CH2(左上) 2=CH3(左下) 3=CH4(右下)。
 * 改动影响: 哪一路对应哪个物理舵面; 装错会导致混控方向整体错乱。
 */
#define SERVO_MIX_CH {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}

/* ============================================================================
 *                     2) 每路装机标定 (数组下标 = 舵面编号 0..3)
 * ==========================================================================*/

/*
 * [比例/行程] 逻辑角 -> 机械角 增益, 逐路。必须 > 0; 1.0 = 1:1。
 * 现象“指令角度:实际偏角”比例不对时改这一路(斜率)。改它不影响零点。
 * 默认 4 路均为 1.0, 现场按实测逐个标。
 */
#define SERVO_MIX_SCALE {1.0f, 1.0f, 1.0f, 1.0f}

/*
 * [零点/中位] 逐路机械零点偏移(deg): logical=0 时的机械偏角 = trim。
 * 现象“指令 0° 时舵面停偏”时改这一路(平移)。也可用 ServoMixZero(ch) 自动对零。
 * 默认 4 路均为 0.0。
 */
#define SERVO_MIX_TRIM_DEG {0.0f, 0.0f, 0.0f, 0.0f}

/*
 * [方向] 逐路: 0 = 正常, 1 = 反向(整体取负)。
 * 指令“正”而该路舵面“反”时置 1。反向会连零点一起取负, 改完需重新对零。
 * 默认 4 路均为 0。
 */
#define SERVO_MIX_REVERSE {0, 0, 0, 0}

/*
 * [限位] 逐路逻辑角对称限位(deg): 逻辑角夹到 ±limit。
 * 保护舵面不越程, 同时限制该路参与混控的幅度。<= 0 表示用满行程(不推荐)。
 * 默认 4 路 35°(与 SERVO_MIX_MAX_DEG 一致, 留一点余量可自行调)。
 */
#define SERVO_MIX_LIMIT_DEG {35.0f, 35.0f, 35.0f, 35.0f}

/*
 * [限速] 逐路速率限幅(deg/s): >0 平滑逼近, <=0 立即到位。
 * 影响舵面动作的快慢/冲击。默认 4 路 600。
 */
#define SERVO_MIX_RATE_DPS {600.0f, 600.0f, 600.0f, 600.0f}

/*
 * [调零开关] 逐路: 1 = 允许 ServoMixZero(ch) 调零, 0 = 禁用。
 * 默认 4 路均为 1。
 */
#define SERVO_MIX_ZERO_ENABLE {1, 1, 1, 1}

/* ============================================================================
 *                              3) 混控参数
 * ==========================================================================*/

/*
 * 混控满偏对应的舵面逻辑角(deg)。
 * 含义: 当 (pitch/yaw/roll) 指令为 1.0(满)时, 每路舵面的偏角幅值 = 该值。
 *       out[ch] = clamp( Σ matrix[ch][j]*cmd[j] * MAX_DEG, ±MAX_DEG )。
 * 取值范围建议 20~40; 越大动作越猛(可能失速/超行程), 越小越缓。
 * 默认 35。
 */
#define SERVO_MIX_MAX_DEG 35.0f

/*
 * 解耦矩阵: 行 = 舵面编号(0..3), 列 = (0=pitch, 1=yaw, 2=roll)。
 * 每项为 +1 / -1 (可给小数微调), 描述该舵面受哪个控制量、正向还是反向驱动。
 * X 型 4 舵面的经典符号约定(与 dart_fc/dart_servo_v0.2 一致):
 *        pitch yaw roll
 *   0:   +1    +1   +1
 *   1:   -1    +1   -1
 *   2:   -1    -1   +1
 *   3:   +1    -1   -1
 * 装机方向不一致时, 优先改对应的 SERVO_MIX_REVERSE[ch], 再考虑改本矩阵符号。
 */
#define SERVO_MIX_MATRIX \
  { {+1.0f, +1.0f, +1.0f}, {-1.0f, +1.0f, -1.0f}, {-1.0f, -1.0f, +1.0f}, {+1.0f, -1.0f, -1.0f} }

/*
 * 上电默认模式: 0 = MIX(混控), 1 = MANUAL(四路纯手动)。
 * 上电后默认为 MIX 且指令为 0, 因此各舵面停在逻辑 0°(= 中位), 即“上电回中”。
 * 需要上电即进入手动调试可改为 1。
 */
#define SERVO_MIX_DEFAULT_MODE 0

#endif /* SERVO_MIX_AI_CFG_H */
