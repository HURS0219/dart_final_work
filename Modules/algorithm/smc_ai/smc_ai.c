/**
 * @file smc_ai.c
 * @author ai (smc_ai)
 * @version V1.0.0
 * @date 2026/09/28
 * @brief 滑模控制(SMC)实现: 一阶滑模面 + 边界层趋近律
 *
 * 仿照 Modules/algorithm/png_ai 的组织方式: 纯算法、逐段中文注释、可主机自测。
 *
 * 控制律 (符号约定: e = 被测 - 期望 = output - desired):
 *   s   = c*e + e_dot                 (一阶滑模面)
 *   u   = clamp( -k * sat(s/phi), ±max_out )   (边界层趋近律, sat 抑抖)
 *   sat(x) = x (|x|<1) / sign(x) (否则)
 */
#include "smc_ai.h"

#include <math.h>
#include "string.h" /* memset */

/* ============================================================================
 *                          内部工具函数
 * =========================================================================== */

/** @brief 饱和函数: |x|<1 线性, 否则 ±1 (用于边界层抑抖) */
static float Sat(float x) {
  if (x > 1.0f) return 1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
}

/** @brief 数值限幅 */
static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/** @brief 参数兜底: <=0 视为未填写, 回落默认值 */
static float ValidOrDef(float v, float d) { return (v > 0.0f) ? v : d; }

/* ============================================================================
 *                          外部算法接口
 * =========================================================================== */

void SMCInit(SMCInstance *smc, const SMC_Init_Config_s *config) {
  if (smc == NULL) return;

  /* 1) 整体清零, 保证运行时状态干净 */
  memset(smc, 0, sizeof(SMCInstance));

  /* 2) 拷贝配置(若有) */
  if (config != NULL) {
    smc->c = config->c;
    smc->k = config->k;
    smc->phi = config->phi;
    smc->max_out = config->max_out;
  }

  /* 3) 关键参数兜底: 未填/非法 -> smc_cfg.h 默认(避免除零、恒零、符号异常) */
  smc->c = ValidOrDef(smc->c, SMC_DEFAULT_C);
  smc->k = ValidOrDef(smc->k, SMC_DEFAULT_K);
  smc->phi = ValidOrDef(smc->phi, SMC_DEFAULT_PHI);
  smc->max_out = ValidOrDef(smc->max_out, SMC_DEFAULT_MAXOUT);

  /* 4) 清运行时状态 */
  SMCClear(smc);
}

void SMCClear(SMCInstance *smc) {
  if (smc == NULL) return;
  smc->e = 0.0f;
  smc->e_dot = 0.0f;
  smc->s = 0.0f;
  smc->u = 0.0f;
}

float SMCCalculate(SMCInstance *smc, float err, float err_dot, float dt) {
  float u;

  if (smc == NULL) return 0.0f;

  (void)dt; /* 预留: 将来做“等效控制”或从 DWT 取 dt 时使用; 基础律不需要 */

  /* 记录输入 */
  smc->e = err;
  smc->e_dot = err_dot;

  /* 步骤 1: 滑模面 s = c*e + e_dot */
  smc->s = smc->c * err + err_dot;

  /* 步骤 2: 边界层趋近律 u = -k*sat(s/phi) (sat 抑制抖振, 舵机友好) */
  u = -smc->k * Sat(smc->s / smc->phi);

  /* 步骤 3: 输出限幅 */
  smc->u = Clamp(u, -smc->max_out, smc->max_out);
  return smc->u;
}
