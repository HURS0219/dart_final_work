/**
 * @file adrc_ai.c
 * @author ai (adrc_ai)
 * @version V1.0.0
 * @date 2026/09/28
 * @brief 线性二阶 ADRC(LADRC)实现: ESO + PD
 *
 * 仿照 Modules/algorithm/png_ai 的组织方式: 纯算法、逐段中文注释、可主机自测。
 *
 * 对象:  y'' = f + b0·u
 *
 * ESO(扩张状态观测器, 带宽 wo):
 *   e  = z1 - y
 *   z1 += dt*( z2 - 3·wo·e )
 *   z2 += dt*( z3 - 3·wo²·e + b0·u )
 *   z3 += dt*( -wo³·e )
 *
 * 控制律(带宽 wc, 阻尼 ζ):
 *   u0 = wc²·(ref - z1) - 2·ζ·wc·z2
 *   u  = clamp( (u0 - z3)/b0 , ±max_out )
 */
#include "adrc_ai.h"

#include <math.h>
#include "string.h" /* memset */

/* ============================================================================
 *                          内部工具函数
 * =========================================================================== */

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

void ADRCInit(ADRCInstance *adrc, const ADRC_Init_Config_s *config) {
  if (adrc == NULL) return;

  /* 1) 整体清零 */
  memset(adrc, 0, sizeof(ADRCInstance));

  /* 2) 拷贝配置(若有) */
  if (config != NULL) {
    adrc->wo = config->wo;
    adrc->wc = config->wc;
    adrc->b0 = config->b0;
    adrc->max_out = config->max_out;
    adrc->zeta = config->zeta;
  }

  /* 3) 兜底: wo/wc/max_out 必须 >0; zeta 缺省 1.0; b0 需非零(大小写由现场定) */
  adrc->wo = ValidOrDef(adrc->wo, ADRC_DEFAULT_WO);
  adrc->wc = ValidOrDef(adrc->wc, ADRC_DEFAULT_WC);
  adrc->max_out = ValidOrDef(adrc->max_out, ADRC_DEFAULT_MAXOUT);
  adrc->zeta = ValidOrDef(adrc->zeta, ADRC_DEFAULT_ZETA);
  if (adrc->b0 == 0.0f) adrc->b0 = ADRC_DEFAULT_B0; /* 防止除零; 符号由现场定 */

  /* 4) 清运行时状态 */
  ADRCClear(adrc);
}

void ADRCClear(ADRCInstance *adrc) {
  if (adrc == NULL) return;
  adrc->z1 = 0.0f;
  adrc->z2 = 0.0f;
  adrc->z3 = 0.0f;
  adrc->u = 0.0f;
  adrc->err = 0.0f;
}

float ADRCCalculate(ADRCInstance *adrc, float measure, float ref, float dt) {
  float e, b1, b2, b3, kp, kd, u0, un;

  if (adrc == NULL) return 0.0f;
  if (dt <= 0.0f) dt = 0.001f; /* dt 兜底 */

  /* ---- 步骤 1: ESO 更新(扩张状态观测器) ---- */
  e = adrc->z1 - measure;              /* 观测误差 */
  b1 = 3.0f * adrc->wo;                /* 观测器增益(带宽整定) */
  b2 = 3.0f * adrc->wo * adrc->wo;
  b3 = adrc->wo * adrc->wo * adrc->wo;

  adrc->z1 += dt * (adrc->z2 - b1 * e);
  adrc->z2 += dt * (adrc->z3 - b2 * e + adrc->b0 * adrc->u); /* 用上次 u */
  adrc->z3 += dt * (-b3 * e);

  /* ---- 步骤 2: PD 控制 + 扰动前馈补偿 ---- */
  kp = adrc->wc * adrc->wc;
  kd = 2.0f * adrc->zeta * adrc->wc;
  u0 = kp * (ref - adrc->z1) - kd * adrc->z2; /* 用估计值, 非实测, 抗噪 */
  un = (u0 - adrc->z3) / adrc->b0;            /* 除以 b0 并抵消总扰动 z3 */

  /* ---- 步骤 3: 限幅, 记录供下周期 ESO 使用 ---- */
  adrc->err = e;
  adrc->u = Clamp(un, -adrc->max_out, adrc->max_out);
  return adrc->u;
}
