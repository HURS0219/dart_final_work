#pragma once
#include "chassis.h"

// 中科大的功率模型
// ===== M3508轮毂电机 6参数模型系数 =====
// #define WHEEL_K0 0.7441993412640775f
// #define WHEEL_K1 0.0090164284468539646f
// #define WHEEL_K2 0.0001988857226262331f
// #define WHEEL_K3 0.024694430204543864f
// #define WHEEL_K4 0.20160143850678086f
// #define WHEEL_K5 3.715221772539512e-05f

#define WHEEL_K0 0.6641993412640775f 
#define WHEEL_K1 0.006444284468539646f
#define WHEEL_K2 0.0001423857226262331f 
#define WHEEL_K3 0.017644430204543864f
#define WHEEL_K4 0.1650143850678086f 
#define WHEEL_K5 3.096721772539512e-05f 

// #define M_3508_K0 0.65213f
// #define M_3508_K1 (-0.15659f)
// #define M_3508_K2 0.00041660f
// #define M_3508_K3 0.00235415f
// #define M_3508_K4 0.20022f
// #define M_3508_K5 1.08e-7f

// ===== DJI M3508 =====
#define DJI_CURRENT_SCALE (20.0f / 16384.0)

// 3508改268/17减速比 扭矩-电流转换
#define t2i(torque) ((torque) * (3591.0f / 187.0f) / (268.0f / 17.0f) / 0.3f)
#define i2t(current) ((current) * 0.3f * (268.0f / 17.0f) / (3591.0f / 187.0f))

/**
 * @brief 功率控制结构体（无超级电容版本）
 *
 * 控制逻辑：
 *   1. 用6参数模型实时估计总功率 P_est（低通滤波后得 P_filtered）
 *   2. 开环基准：vel_max_base = Kp_vel * sqrt(P_limit)
 *   3. PI闭环：根据 (P_limit - P_filtered) 修正 vel_max
 *   4. vel_max 非对称低通：超功率时快速降速，功率富余时缓慢恢复
 *   5. 对目标速度做幅值限制 + 斜率限制，输出 limited_vx
 */
typedef struct Power_Ctrl_t {
  float k[6];  // 功率控制拟合参数
  float w[2];  // 轮电机角速度

  float T_motion[2];   // 运动分量扭矩（LQR motion 部分）
  float T_balance[2];  // 平衡分量扭矩（LQR balance 部分）

  float I[2];     // 单电机期望电流（由 final_output 换算）
  float P[2];     // 单电机期望功率（用 I_total 估算）
  float P_total;  // 整车期望总功率

  float P_ref[2];     // 单电机许用功率
  float P_total_ref;  // 整车许用总功率

  float I_ref[2];         // 由 P_ref 反解的单电机电流
  float T_ref[2];         // 由 I_ref 反解的允许扭矩
  float T_motion_ref[2];  // T_ref - T_balance 后的 motion 目标扭矩

  float scale_motion[2];  // 每电机 motion 缩放系数 T_motion_ref / T_motion
  float scale_combined;   // 用于回写 chassis_ctrl_cmd 的合成缩放

  float P_peak_threshold;  // 峰值功率阈值，超过后开始削平衡分量
  float scale_balance;     // 平衡分量缩放系数 (下限 0.5，防止完全丧失平衡)

  float v_max_dynamic;  // 根据功率预算动态计算的最大速度
} Power_Ctrl_t;

void PowerControl_Prostrate(ChassisInstance* chassis);
void PowerControl(ChassisInstance* chassis);
void PowerControlInit(ChassisInstance* chassis);
void PowerControlRuntimeReset(ChassisInstance* chassis);
