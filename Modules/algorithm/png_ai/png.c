/**
 * @file png.c
 * @author ai (png_ai)
 * @brief  视线比例引导(PNG)算法实现
 * @version V1.0.0
 * @date 2026/09/27
 *
 * 仿照 Modules/algorithm/controller/controller.c 的组织方式:
 *   - 文件开头是算法主体实现;
 *   - 之后是“外部算法接口”(Init / Clear / Calculate / Setter);
 *   - 每一段都配中文注释, 便于人工审阅。
 *
 * 控制律:
 *   PPN: a_cmd = N * v * d_lambda
 *   APN: a_cmd = N * v * d_lambda + a_o_gain * N * a_o
 *   其中 v 由 vel_src 选择 v_c 或 v_m。
 */
#include "png.h"

#include "string.h" /* memset / memcpy */

/* ============================================================================
 *                          内部工具函数
 * =========================================================================== */

/**
 * @brief 取值辅助: 若配置值无效(<=0)则回退到默认值
 * @param val     用户填写的值
 * @param fallback 默认值
 * @return float  有效值
 * @note 仅用于“必须为正”的参数(如 N、MaxOut、a_o_gain)。
 */
static float png_valid_or_default(float val, float fallback) {
  /* 参数为 0 或负数视为“未填写/非法”, 用默认值兜底, 防止指令恒为 0 或符号反转 */
  if (val > 0.0f)
    return val;
  return fallback;
}

/* ============================================================================
 *                          外部算法接口
 * =========================================================================== */

/**
 * @brief 初始化 PNG 实例
 * @param png    PNG 实例指针
 * @param config 初始化配置 (为 NULL 时全部采用默认值)
 */
void PNGInit(PNGInstance *png, PNG_Init_Config_s *config) {
  /* 1) 仿照 PIDInit: 先整体清零, 保证运行时状态干净 */
  memset(png, 0, sizeof(PNGInstance));

  /* 2) 拷贝配置块。PNGInstance 头部与 PNG_Init_Config_s 布局一致, 可直接 memcpy;
   *    未提供配置时跳过, 后续全部走默认值。 */
  if (config != NULL) {
    memcpy(png, config, sizeof(PNG_Init_Config_s));
  }

  /* 3) 关键参数兜底: 未填写/非法的项回退到 png_cfg.h 默认值 */
  png->N = png_valid_or_default(png->N, PNG_DEFAULT_N);                 // 导航常数
  png->MaxOut = png_valid_or_default(png->MaxOut, PNG_DEFAULT_MAX_OUT);  // 输出限幅
  png->a_o_gain = png_valid_or_default(png->a_o_gain, PNG_DEFAULT_AO_GAIN);

  /* 4) 死区允许为 0(表示关闭死区), 但负数非法, 置 0 */
  if (png->DeadBand < 0.0f) {
    png->DeadBand = 0.0f;
  }

  /* 5) 引导模式/速度来源: 仅在用户确实给了合法枚举时才覆盖默认值。
   *    由于 memcpy 后无法区分“未填写(0)”与“PPN(0)”, 此处采用默认值优先策略:
   *    若用户需要非默认模式, 请在配置里显式传入并配合 PNGSetMode 使用。 */
  if (config == NULL) {
    png->mode = PNG_DEFAULT_MODE;
    png->vel_src = PNG_DEFAULT_VEL_SRC;
  }

  /* 6) 清一次运行时状态 */
  PNGClear(png);
}

/**
 * @brief 清除运行时状态(保留配置)
 * @param png PNG 实例指针
 */
void PNGClear(PNGInstance *png) {
  if (png == NULL) return;

  /* 逐项清零本周期输入/中间量/输出, 配置块保持不变 */
  png->d_lambda = 0.0f;  // 视线角速率输入
  png->v_eff = 0.0f;     // 实际参与计算的速度
  png->a_prop = 0.0f;    // 比例项
  png->a_aug = 0.0f;     // 增广项
  png->a_cmd_raw = 0.0f; // 限幅前指令
  png->a_cmd = 0.0f;     // 最终指令
}

/**
 * @brief 执行一次制导解算
 * @param png PNG 实例指针
 * @param in  本周期输入
 * @return float 横向加速度指令 a_cmd [m/s^2]
 */
float PNGCalculate(PNGInstance *png, const PNG_Input_s *in) {
  /* 输入合法性保护: 空指针直接返回上一次指令 */
  if (png == NULL || in == NULL) {
    return (png != NULL) ? png->a_cmd : 0.0f;
  }

  /* ---- 步骤 1: 记录输入, 并选择参与比例项的速度 v ----
   *   vel_src = VC -> v = v_c (纯比例导引经典形式)
   *   vel_src = VM -> v = v_m */
  png->d_lambda = in->d_lambda;
  png->v_eff = (png->vel_src == PNG_VEL_USE_VM) ? in->v_m : in->v_c;

  /* ---- 步骤 2: 比例项 a_prop = N * v * d_lambda ----
   *   这是需求给出的核心控制律, 视线转得越快, 需要的横向过载越大 */
  png->a_prop = png->N * png->v_eff * png->d_lambda;

  /* ---- 步骤 3: 增广项(仅 APN) a_aug = a_o_gain * N * a_o ----
   *   PPN 模式不引入目标加速度前馈; APN 用目标加速度做前置补偿,
   *   a_o 可由卡尔曼滤波推算, 也可人工输入固定值 */
  if (png->mode == PNG_MODE_APN) {
    png->a_aug = png->a_o_gain * png->N * in->a_o;
  } else {
    png->a_aug = 0.0f;
  }

  /* ---- 步骤 4: 合成原始指令 a_cmd_raw = a_prop + a_aug ---- */
  png->a_cmd_raw = png->a_prop + png->a_aug;

  /* ---- 步骤 5: 死区 + 输出限幅(过载保护) ---- */
  if (fabsf(png->a_cmd_raw) < png->DeadBand) {
    /* 5.1 落在死区内: 清零, 抑制零附近抖动 */
    png->a_cmd = 0.0f;
  } else if (png->a_cmd_raw > png->MaxOut) {
    /* 5.2 正向超限: 截断到 +MaxOut */
    png->a_cmd = png->MaxOut;
  } else if (png->a_cmd_raw < -png->MaxOut) {
    /* 5.3 负向超限: 截断到 -MaxOut */
    png->a_cmd = -png->MaxOut;
  } else {
    /* 5.4 正常范围: 直接输出 */
    png->a_cmd = png->a_cmd_raw;
  }

  /* ---- 步骤 6: 返回最终指令, 并保留在实例中供观测 ---- */
  return png->a_cmd;
}

/**
 * @brief 在线修改导航常数
 * @param png PNG 实例指针
 * @param N   新的导航常数
 */
void PNGSetNavConstant(PNGInstance *png, float N) {
  if (png == NULL) return;
  /* 传入非法值(<=0)时保持原值不变, 避免误操作使指令恒为 0 */
  if (N > 0.0f) {
    png->N = N;
  }
}

/**
 * @brief 在线切换引导模式(PPN/APN)
 * @param png  PNG 实例指针
 * @param mode 目标模式
 */
void PNGSetMode(PNGInstance *png, PNG_Mode_e mode) {
  if (png == NULL) return;
  png->mode = mode;
}

/**
 * @brief 读取最近一次输出的指令
 * @param png PNG 实例指针
 * @return float 最近一次的 a_cmd [m/s^2]
 */
float PNGGetCommand(PNGInstance *png) {
  if (png == NULL) return 0.0f;
  return png->a_cmd;
}
