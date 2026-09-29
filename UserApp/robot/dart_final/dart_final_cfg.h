/*
 * dart_final_cfg.h — dart_final 应用层可调参数 (现场调参入口, 改源码 + 烧录)
 * =============================================================================
 * 只放“本 app 的调参量”; 舵机/混控/制导算法自身的默认值分别在:
 *   servo_motor_cfg.h / servo_mix_ai_cfg.h / png_cfg.h (由 dart_all_cfg.h 汇总查看)
 *
 * ⚠ 重要调参原则(详见 README《调参对照表》):
 *   过偏/欠偏(整机)  -> SERVO_MIX_MAX_DEG (不是这里的 k)
 *   某一路偏多/偏少  -> servo_mix_ai_cfg.h 的 SERVO_MIX_SCALE[ch]
 *   制导震荡/收敛慢  -> 这里的 GUID_GAIN_K 或 png_cfg.h 的 N
 *   k 建议固定为物理值 1/PNG_MAX_OUT, 不要用它去补“过偏”。
 * =============================================================================
 */
#pragma once

#ifndef DART_FINAL_CFG_H
#define DART_FINAL_CFG_H

/* ===================== 制导 (guidance app) ===================== */

/*
 * 控制模式: 0 = 线性增益 mix = k*a_cmd (默认, 飞镖 <20m/s 足够);
 *           1 = 速率环(预留, 大概率用不到, 仅留接口)。
 * 改动影响: 换控制律; 默认勿动。
 */
#define GUID_CTRL_MODE 0

/*
 * 制导线性增益 k (无量纲, 单位 1/(m/s^2)):  mix.yaw = k * a_cmd。
 * 建议固定为 1/PNG_MAX_OUT(=1/png_cfg 的 PNG_DEFAULT_MAX_OUT)的物理值, 即“指令上限 -> 满舵”。
 * 只在需要改“制导律强度”时动它; “过偏”请改 SERVO_MIX_MAX_DEG。
 */
#define GUID_GAIN_K (1.0f / 5.0f) /* 对应 PNG_DEFAULT_MAX_OUT = 5.0 */

/*
 * 接近速度 v_c (m/s): png_ai 比例项使用(vel_src=VC)。飞镖实测/估算填入, 20m/s 以下。
 * 改动影响: 制导指令整体幅度(与 k 相乘的效果), 影响收敛速度。
 */
#define GUID_DLC_V_C (12.0f)

/*
 * 视线角速率一阶低通 α ∈ (0,1]: dλ = α*dλ_new + (1-α)*dλ_old。
 * 越小越平滑(滞后大), 越大越灵敏(噪声大)。默认 0.3。
 */
#define GUID_LOS_FILTER_ALPHA (0.30f)

/*
 * 相机焦距(像素)与画面中心(像素): λ_yaw = (x - CX)/focal。
 * QVGA 320x240 默认 CX=160, CY=120; focal 现场标定(视野越窄 focal 越大)。
 */
#define GUID_FOCAL_PX (120.0f)
#define GUID_IMAGE_CX (160.0f)
#define GUID_IMAGE_CY (120.0f)

/*
 * 图像解旋(用 roll 角把视线角旋回机体, 消除滚转耦合): 0 关 / 1 开。
 * 我们已用 PID 稳 roll, 基本可关; 若 roll 残差较大再开。sign 为方向。
 */
#define GUID_DEROT_ENABLE 0
#define GUID_DEROT_SIGN (-1.0f)

/* ===================== 滚转稳定 (roll PID) =====================
 * 用现成 Modules/algorithm/controller 的 PID: 输入 roll 角, 目标 0, 输出 mix.roll。
 * 微分项作用在误差(= -roll)上, 等效提供角速率阻尼。
 * 现象“滚转压不住/来回摆” -> 加大 ROLL_KP/ROLL_KD。
 */
#define ROLL_KP (1.5f)
#define ROLL_KI (0.0f)
#define ROLL_KD (0.15f)
#define ROLL_CMD_LIMIT (1.0f) /* 输出(mix.roll)限幅 ± */

/*
 * 滚转控制器选择: 0 = PID(Modules/algorithm/controller), 1 = SMC(smc_ai)。
 * 本分支(smc)默认 1。
 * ⚠ 该开关是为“后期三实验 + main 合并”预留; 启用多控制器会明显增加复杂度与出错面,
 *   前期请固定一种。SMC 参数见 Modules/algorithm/smc_ai/smc_cfg.h。
 */
#define ROLL_CTRL_MODE 1


/* ===================== 状态机 / 失效判定 ===================== */

/*
 * 制导触发: 姿态俯冲判定阈值(deg)。Pitch < 该值 视为“已进入俯冲”, 允许制导。
 * 默认 0(即俯角)。以后可改为其它判据(见 robot.c 的 Dart_ShouldGuide)。
 */
#define DIVE_PITCH_DEG (0.0f)

/*
 * 失效超时(ms): 超过该时间未收到姿态/视觉 -> 对应 app 置错 / 整机 FAULT(回中)。
 */
#define ATTITUDE_TIMEOUT_MS (100u)
#define VISION_TIMEOUT_MS (200u)

/* 上电默认是否使能: 1=上电即 ARMED(当前); 0=需外部使能(赛事规则定后改)。 */
#define DART_ENABLE_ON_BOOT 1

/* ===================== 通信端口(引脚待定, 先占位) =====================
 * 填对应 CubeMX 句柄即可; 确定为哪个 USART/SPI 后替换。宏是“惰性”的, 不用到不受影响。
 *   ESP32 : 沿用 dart_servo_v0.2 的 USART6
 *   OpenMV: 另一路 USART + 并行 SPI(防丢帧)
 *   VOFA  : 预留(本次只用 RTT)
 */
#define DART_USART_ESP32 (&huart6)
#define DART_USART_OPENMV (&huart3)
#define DART_SPI_OPENMV (&hspi2)  /* BMI088 占 hspi1, 故 OpenMV 用 hspi2 */
#define DART_USART_VOFA (&huart1) /* 预留 */

/* OpenMV 7 字节帧参数 (AA 55 X_hi X_lo Y_hi Y_lo CRC8) */
#define OPENMV_RECV_SIZE 7

#endif /* DART_FINAL_CFG_H */
