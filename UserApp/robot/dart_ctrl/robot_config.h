/*
 * robot_config.h — dart_ctrl 全部可调参数 (现场调参入口, 改源码 + 烧录)
 * =============================================================================
 * 由 dart_final 的 dart_final_cfg.h + png_cfg.h + servo_motor_cfg.h +
 * servo_mix_ai_cfg.h "汇总而来"(原来分散在各 Module, 现统一到 app)。
 *
 * ⚠ 板子暂留空(以后上 F405): 端口/定时器宏先按 C 板(F407)占位, 换板只改本文件。
 *
 * 调参速查:
 *   整机过偏/欠偏    -> DART_MIX_MAX_DEG
 *   某一路偏多/偏少  -> DART_SERVO_SCALE[ch]
 *   零点/中位偏      -> DART_SERVO_TRIM[ch]
 *   方向反           -> DART_SERVO_REVERSE[ch] (改后重新对零)
 *   制导震荡/收敛慢  -> PNG_N 或 GUID_GAIN_K
 *   滚转压不住       -> ROLL_KP / ROLL_KD
 * =============================================================================
 */
#pragma once

#ifndef DART_CTRL_CONFIG_H
#define DART_CTRL_CONFIG_H

#include "main.h"
#include "tim.h"    /* &htim1 / TIM_CHANNEL_x */
#include "usart.h"  /* &huart3 ... */

/* ===================== 板级开关 / IMU ===================== */
/* 实板(F405)以后开; H743 板若上再按板调整 */
#define DART_IMU_ENABLE 1

/* ===================== 端口 (板子留空, 先占位) =====================
 *   C 板(F407): OpenMV=huart3, ESP=huart6(预留), VOFA=huart1(预留)
 *   F405 实板 : OpenMV=huart2 + SPI1(从); ESP=huart1 (见 HARDWARE.md, 以后再改)
 */
#define DART_USART_OPENMV (&huart3)
#define DART_USART_ESP32 (&huart6)
#define DART_USART_VOFA (&huart1)

/* ===================== 视觉协议 (OpenMV 7 字节帧) =====================
 *   [0]0xAA [1]0x55 [2]X_hi [3]X_lo [4]Y_hi [5]Y_lo [6]CRC8
 *   CRC8: SHT75(poly=0x31, init=0), 对 [0..5] 校验; 丢目标时 X=Y=0
 */
#define OPENMV_HEAD1 0xAA
#define OPENMV_HEAD2 0x55
#define OPENMV_RECV_SIZE 7

/* ===================== 舵机 (PWM, 4 路) ===================== */
#define DART_SERVO_N 4
#define DART_SERVO_TIM (&htim1)     /* C 板: TIM1 CH1-4 = PE9/PE11/PE13/PE14 */
#define DART_SERVO_PERIOD_S 0.02f   /* 50Hz */

/* [信号层] 型号规格(PTK7350 类), 换型号才改 */
#define SERVO_CENTER_US 1500.0f     /* 机械中位脉宽 */
#define SERVO_HALF_US 1000.0f       /* 中位到行程一端的脉宽跨度 */
#define SERVO_HALF_DEG 139.5f       /* 中位到行程一端的机械角跨度 */
#define SERVO_PULSE_MIN_US 500.0f   /* 脉宽硬下限 */
#define SERVO_PULSE_MAX_US 2500.0f  /* 脉宽硬上限 */

/* [逻辑层] 逐路装机标定 (下标 = 舵面编号 0..3) */
#define DART_SERVO_SCALE {1.0f, 1.0f, 1.0f, 1.0f}   /* 逻辑角->机械角 比例 */
#define DART_SERVO_TRIM {0.0f, 0.0f, 0.0f, 0.0f}    /* 零点机械偏移(deg) */
#define DART_SERVO_REVERSE {0, 0, 0, 0}             /* 逐路方向: 1=反向 */
#define DART_SERVO_RATE {600.0f, 600.0f, 600.0f, 600.0f} /* 速率限幅(deg/s), <=0 立即到位 */

/* ===================== 四舵面混控 ===================== */
#define DART_MIX_MAX_DEG 35.0f  /* 满偏对应的舵面逻辑角(deg), 兼作每路 ±限位 */
/* 解耦矩阵: 行=舵面 0..3, 列=(pitch, yaw, roll), 每项 ±1 */
#define DART_MIX_MATRIX \
  { {+1.0f, +1.0f, +1.0f}, {-1.0f, +1.0f, -1.0f}, {-1.0f, -1.0f, +1.0f}, {+1.0f, -1.0f, -1.0f} }

/* ===================== 制导 (PNG 比例导引) ===================== */
#define GUID_GAIN_K (1.0f / 5.0f)     /* mix.yaw = k * a_cmd (物理值 1/MaxOut) */
#define GUID_DLC_V_C 12.0f            /* 接近速度 v_c (m/s) */
#define GUID_LOS_FILTER_ALPHA 0.30f   /* 视线角速率一阶低通 α */
#define GUID_FOCAL_PX 120.0f          /* 相机焦距(px), 现场标定 */
#define GUID_IMAGE_CX 160.0f          /* 画面中心 x */
#define GUID_IMAGE_CY 120.0f          /* 画面中心 y */
#define PNG_MODE_APN 0                /* 0=PPN(纯比例), 1=APN */
#define PNG_N 4.0f                    /* 导航常数 N */
#define PNG_MAX_OUT 5.0f              /* 横向加速度输出限幅(m/s^2) */
#define PNG_DEADBAND 0.02f            /* 输出死区(m/s^2) */
#define PNG_AO_GAIN 0.5f              /* APN 目标加速度项系数 */
#define PNG_AO 0.0f                   /* APN 目标加速度(预留, 暂 0) */

/* ===================== 滚转稳定 (roll PID) ===================== */
#define ROLL_KP 1.5f
#define ROLL_KI 0.0f
#define ROLL_KD 0.15f
#define ROLL_CMD_LIMIT 1.0f /* 输出(mix.roll)限幅 ± */

/* ===================== 状态机 / 失效判定 ===================== */
#define DIVE_PITCH_DEG 0.0f        /* 俯冲判定阈值(deg): Pitch < 该值视为已俯冲 */
#define ATTITUDE_TIMEOUT_MS 100u   /* 姿态超时 */
#define VISION_TIMEOUT_MS 200u     /* 视觉超时 */
#define DART_ENABLE_ON_BOOT 1      /* 1=上电即使能(ARMED) */

#endif /* DART_CTRL_CONFIG_H */
