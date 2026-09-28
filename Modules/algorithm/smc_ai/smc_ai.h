/**
 * @file smc_ai.h
 * @author ai (smc_ai)
 * @version V1.0.0
 * @date 2026/09/28
 * @brief 滑模控制(Sliding Mode Control) 纯算法模块
 *
 * @attention
 *  - 本模块只实现“一阶滑模面 + 边界层趋近律”, 不依赖 HAL/BSP, 可主机端 gcc 自测;
 *  - 通用接口: 输入 (误差 e, 误差导数 e_dot, dt), 输出控制量 u;
 *    **符号约定: e = 被测 - 期望 (output - desired)**, 配合 u = -k·sat(s) (教科书形式)。
 *    roll 轴用法: e = roll_deg - 0, e_dot = gx_dps - 0, u 作为归一化 roll 指令。
 *  - 用边界层(饱和函数)代替 sign 以抑制抖振(舵机友好)。
 *  - dt 当前不参与基础控制律, 作为**预留接口**(将来做等效控制/DWT 取时用)。
 *  - 详见 smc_ai.md。
 */
#ifndef _SMC_AI_H
#define _SMC_AI_H

#include "smc_cfg.h"
#include "stdint.h"

/* 初始化配置 (仅“必须为正”的可调项; 未填/非法 -> 回落 smc_cfg.h 默认) */
typedef struct {
  float c;        // 滑模面斜率 (s = c*e + e_dot), 单位 1/s
  float k;        // 切换增益
  float phi;      // 边界层厚度 (与 s 同量纲)
  float max_out;  // 输出限幅 (>0)
} SMC_Init_Config_s;

/* SMC 实例: 前段=配置块, 后段=运行时状态(便于调试器观察) */
typedef struct {
  /* ---- config ---- */
  float c;
  float k;
  float phi;
  float max_out;
  /* ---- runtime ---- */
  float e;      // 本轮误差
  float e_dot;  // 本轮误差导数
  float s;      // 滑模面
  float u;      // 控制输出
} SMCInstance;

/**
 * @brief 初始化 SMC 实例
 * @param smc    SMC 实例指针
 * @param config 配置(为 NULL 时全部采用 smc_cfg.h 默认值)
 */
void SMCInit(SMCInstance *smc, const SMC_Init_Config_s *config);

/**
 * @brief 清运行时状态(保留配置)
 */
void SMCClear(SMCInstance *smc);

/**
 * @brief 执行一次 SMC 解算
 * @param smc     SMC 实例指针
 * @param err     误差 e
 * @param err_dot 误差导数 e_dot
 * @param dt      控制周期(秒) —— 预留, 当前基础律不使用
 * @return float  控制量 u (已限幅 ±max_out)
 */
float SMCCalculate(SMCInstance *smc, float err, float err_dot, float dt);

#endif /* _SMC_AI_H */
