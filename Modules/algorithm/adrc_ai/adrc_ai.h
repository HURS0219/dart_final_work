/**
 * @file adrc_ai.h
 * @author ai (adrc_ai)
 * @version V1.0.0
 * @date 2026/09/28
 * @brief 自抗扰控制(ADRC) 纯算法模块 —— 线性二阶 ADRC (LADRC: ESO + PD)
 *
 * @attention
 *  - 本模块只实现“线性扩张状态观测器(ESO) + PD 控制律”的二阶 ADRC, 不依赖 HAL/BSP,
 *    可主机端 gcc 自测。
 *  - 对象模型: y'' = f + b0·u (f=总扰动, 由 ESO 估计并补偿; b0=控制增益)。
 *  - 通用接口: (measure, ref, dt) -> u;  roll 轴用法: measure=roll_deg, ref=0, u 作归一化 roll 指令。
 *  - 优点: 不依赖精确模型, 把耦合/未建模动态当“扰动”估计补偿, 鲁棒性强。
 *  - 详见 adrc_ai.md。
 */
#ifndef _ADRC_AI_H
#define _ADRC_AI_H

#include "adrc_cfg.h"
#include "stdint.h"

/* 初始化配置 (未填/非法 -> 回落 adrc_cfg.h 默认) */
typedef struct {
  float wo;       // 观测器带宽 (rad/s)
  float wc;       // 控制器带宽 (rad/s)
  float b0;       // 控制增益 (对象 y''=f+b0·u)
  float max_out;  // 输出限幅 (>0)
  float zeta;     // 阻尼比 (默认 1.0)
} ADRC_Init_Config_s;

/* ADRC 实例: 前段=配置块, 后段=运行时状态(便于调试器观察) */
typedef struct {
  /* ---- config ---- */
  float wo;
  float wc;
  float b0;
  float max_out;
  float zeta;
  /* ---- runtime (ESO 状态) ---- */
  float z1;  // 估计的 y
  float z2;  // 估计的 y'
  float z3;  // 估计的总扰动 f
  float u;   // 上次控制量(ESO 需要)
  float err; // 观测误差(调试)
} ADRCInstance;

/**
 * @brief 初始化 ADRC 实例
 * @param adrc   ADRC 实例指针
 * @param config 配置(为 NULL 时全部采用 adrc_cfg.h 默认值)
 */
void ADRCInit(ADRCInstance *adrc, const ADRC_Init_Config_s *config);

/**
 * @brief 清运行时状态(保留配置)
 */
void ADRCClear(ADRCInstance *adrc);

/**
 * @brief 执行一次 ADRC 解算
 * @param adrc    ADRC 实例指针
 * @param measure 被控量测量值(本项目 = roll 角)
 * @param ref     期望值(本项目 = 0)
 * @param dt      控制周期(秒)
 * @return float  控制量 u (已限幅 ±max_out)
 */
float ADRCCalculate(ADRCInstance *adrc, float measure, float ref, float dt);

#endif /* _ADRC_AI_H */
