#include "power_control.h"

#include <math.h>

#include "chassis.h"
#include "general_def.h"
#include "user_lib.h"

static Power_Ctrl_t* power_ctrl;
/**
 * @brief  单电机 6 参数功率估算（中科大模型，正运算）
 *         P = k0 + k1·I + k2·ω + k3·I·ω + k4·I² + k5·ω²
 *
 * @param[in]  k  模型系数数组 [k0 … k5]
 * @param[in]  I  电调电流 (A)，由 final_output / (16384/20) 换算得到
 * @param[in]  w  转子角速度 (rad/s)，由 speed_aps × DEGREE_2_RAD 换算得到
 *                注意：speed_aps 为转子侧 deg/s，不除减速比
 * @return     估算电功率 (W)，钳位到 [0, +∞)
 */
static float power_ctrl_k_coff = 1.0f;
static float MotorEstimatePower(float k[6], float I, float w) {
  for (int i = 0; i < 6; i++) k[i] *= power_ctrl_k_coff;
  float P = k[0] + k[1] * I + k[2] * w + k[3] * I * w + k[4] * I * I + k[5] * w * w;
  return fmaxf(P, 0.0f);
}

void PowerControlRuntimeReset(ChassisInstance* chassis) {
  if (chassis == NULL || chassis->power_ctrl == NULL) return;

  Power_Ctrl_t* pc = chassis->power_ctrl;
  for (int i = 0; i < 2; i++) {
    pc->w[i] = 0.0f;
    pc->T_motion[i] = 0.0f;
    pc->T_balance[i] = 0.0f;
    pc->I[i] = 0.0f;
    pc->P[i] = 0.0f;
    pc->P_ref[i] = 0.0f;
    pc->I_ref[i] = 0.0f;
    pc->T_ref[i] = 0.0f;
    pc->T_motion_ref[i] = 0.0f;
    pc->scale_motion[i] = 1.0f;
  }

  pc->P_total = 0.0f;
  pc->P_total_ref = 0.0f;
  pc->scale_combined = 1.0f;
  pc->scale_balance = 1.0f;
}

/**
 * @brief  根据目标功率反解允许的最大电流（中科大模型，逆运算）
 *
 *         在功率模型中令 x = |I| ≥ 0，得到一元二次方程
 *             k4·x² + (k1 + k3·|ω|)·x + (k0 + k2·|ω| + k5·ω² − P) = 0
 *         物理约束下 A = k4 > 0，B = k1 + k3·|ω| ≥ 0，
 *         较小根恒 ≤ 0，故只需考虑较大根。
 *
 * @param[in]  k          模型系数数组 [k0 … k5]，与 MotorEstimatePower 共用
 * @param[in]  P_target   目标电功率 (W)，应 ≥ 0
 * @param[in]  w          转子角速度 (rad/s)
 * @param[in]  I_current  当前电流 (A)，仅用于确定返回值的符号方向
 * @return     满足功率约束的最大电流 (A)，符号与 I_current 一致；
 *             无有效解时返回 0
 *
 * @note   要求 k4 > 0，k1 ≥ 0，k3 ≥ 0（物理上通常成立）
 */
static float MotorEstimateCurrent(const float k[6], float P_target, float w, float I_current) {
  float A = k[4];
  float B = k[1] + k[3] * w;
  float C = k[0] + k[2] * w + k[5] * w * w - fabsf(P_target);

  float discriminant = B * B - 4.0f * A * C;

  if (discriminant < 0.0f) {
    /* 目标功率过低，无实根；取顶点作为近似解 */
    return -B / (2.0f * A);
  }

  float sqrt_disc = sqrtf(discriminant);

  /* 唯一需要考虑的根 */
  float temp1 = (-B + sqrt_disc) / (2.0f * A);
  float temp2 = (-B - sqrt_disc) / (2.0f * A);

  if (I_current > 0.0f) {
    return (fabsf(temp1 - I_current) < fabsf(temp2 - I_current) ? fminf(20.f, temp1) : fminf(20.f, temp2));

  } else {
    return (fabsf(temp1 - I_current) < fabsf(temp2 - I_current) ? fmaxf(-20.f, temp1) : fmaxf(-20.f, temp2));
  }
}

/**
 * @brief 功率控制器（按比例分配 + 状态参考回写）
 *
 * 控制律: u = -K(x - x_ref); 仅对 motion 状态 (x_b, x_b_d, phi, phi_d) 调整 x_ref,
 * balance 状态 (theta_l/_d, theta_r/_d, theta_b/_d) 的 ref 恒为 0。
 *
 * 流程:
 *   1. 由实际电机电流 I 估算每电机功率 P_i, 求 P_chassis = P_0 + P_1
 *   2. 若 P_chassis > P_ref:
 *        P_ref_i = P_i / P_chassis * P_ref            (按当前用量比例分配)
 *        I_ref_i = MotorEstimateCurrent(P_ref_i, ω)   (功率反解)
 *        I_motion_ref_i = I_ref_i - I_balance_i       (扣除平衡分量)
 *        T_motion_ref_i = i2t(I_motion_ref_i)
 *   3. 对每个 motion 状态 j ∈ {0,1,2,3}, 每电机 i 的力矩贡献为
 *        T_motion_xj_i = -K[i][j] * state_err[j]
 *      按比例缩放:
 *        T_ref_motion_xj_i = T_motion_xj_i / T_motion_i * T_motion_ref_i
 *      由 x_ref_j = x_j + T_ref_motion_xj_i / K[i][j] 反推, 化简为
 *        state_err_new[j]_i = state_err[j] * scale_i,  scale_i = T_motion_ref_i / T_motion_i
 *   4. 两电机给出两个 scale, 取较小值后缩放本帧 LQR motion state_err。
 *      raw chassis_ctrl_cmd 仍保持上层命令，只回写 chassis planner 参考，避免和 ramp 抢写。
 */
void PowerControl(ChassisInstance* chassis) {
  Power_Ctrl_t* pc = power_ctrl;
  // 1.获取力矩
  for (int i = 0; i < 2; i++) {
    pc->T_motion[i] = 0.0f;
    pc->T_balance[i] = 0.0f;
    pc->scale_motion[i] = 1.0f;
  }
  pc->scale_combined = 1.0f;
  for (int j = 0; j < 4; j++) {
    pc->T_motion[0] -= chassis->LQR_K[2][j] * chassis->state_err[j];
    pc->T_motion[1] -= chassis->LQR_K[3][j] * chassis->state_err[j];
  }
  for (int j = 4; j < 10; j++) {
    pc->T_balance[0] -= chassis->LQR_K[2][j] * chassis->state_err[j];
    pc->T_balance[1] -= chassis->LQR_K[3][j] * chassis->state_err[j];
  }

  // 2.估算功率
  for (int i = 0; i < 2; i++) {
    // 实际电机电流 / 力矩 (来自上一拍 final_output)
    // float current_I = (float)chassis->leg[i]->wheel_motor->motor_controller.final_output * DJI_CURRENT_SCALE;
    // 实际电机电流 / 力矩 (来自当前拍 T)
    float current_I = t2i(pc->T_motion[i] + pc->T_balance[i]);
    pc->I[i] = current_I;

    // 对 w 应用一阶低通滤波 (当前不加滤波)
    float current_w = chassis->leg[i]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;
    pc->w[i] = current_w;

    // speed_aps 与底盘前进方向相反; P_motion_mech < 0 表示 motion 净刹车/回收, 不参与限功率缩放。

    // 总功率用实际电机电流估算，并应用一阶低通滤波 (当前不加滤波)
    float current_P = MotorEstimatePower(pc->k, pc->I[i], pc->w[i]);  // 使用原始 current_w 计算瞬态功率
    pc->P[i] = current_P;
  }

  pc->P_total = pc->P[0] + pc->P[1];
  // pc->P_total_ref = chassis->chassis_ctrl_cmd.max_power;
  pc->P_total_ref = 250.f;
  pc->P_peak_threshold = 300.f;

  // 3.功率控制逻辑
  if (pc->P_total > pc->P_total_ref) {
    for (int i = 0; i < 2; i++) {
      // 1) 按当前总功率占比分配每电机的许用总功率
      pc->P_ref[i] = pc->P[i] / pc->P_total * pc->P_total_ref;

      // 2) 由 P_ref 反解允许的总电流 (符号跟随 I_total)
      // pc->I_ref[i] = MotorEstimateCurrent(pc->k, pc->P_ref[i], pc->w[i], pc->I[i]);
      // 计算当前拍的预期输出总电流，用于决定功率逆解求出的允许电流的符号
      // float I_cmd = t2i(pc->T_motion[i] + pc->T_balance[i]);
      // 2) 由 P_ref 反解允许的总电流 (符号跟随 I_cmd)
      pc->I_ref[i] = MotorEstimateCurrent(pc->k, pc->P_ref[i], pc->w[i], pc->I[i]);

      pc->T_ref[i] = i2t(pc->I_ref[i]);

      // 3) 扣除 balance 分量, 得到 motion 部分的扭矩
      pc->T_motion_ref[i] = pc->T_ref[i] - pc->T_balance[i];

      // 4) motion 缩放系数
      if (fabsf(pc->T_motion[i]) > 1e-6f) {
        float s = pc->T_motion_ref[i] / pc->T_motion[i];
        VAL_LIMIT(s, 0.0f, 1.0f);  // 不放大, 也不允许符号反转(反解奇异时)
        pc->scale_motion[i] = s;
      }
    }

    // 两电机给出两个 scale, state_err 在两电机间共享, 取算术平均回写
    pc->scale_combined = (pc->scale_motion[0] + pc->scale_motion[1]) * 0.5f;  // todo: 取最小值还是平均？

    chassis->state_err[1] *= pc->scale_combined;
    chassis->state_err[2] *= pc->scale_combined;
    chassis->state_err[3] *= pc->scale_combined;

    // x_ref_j = x_j - state_err[j] * scale_combined
    // x_b 的 ref 恒为 0, 不回写; raw chassis_ctrl_cmd 保持上层命令, 只同步 planner
    // chassis->chassis_ctrl_cmd.vx = sv->x_b_d - chassis->state_err[1];
    // chassis->chassis_ctrl_cmd.target_yaw = sv->phi - chassis->state_err[2];
    // chassis->chassis_ctrl_cmd.wz = sv->phi_d - chassis->state_err[3];
    // chassis->planner.vx = sv->x_b_d - chassis->state_err[1];
    // chassis->planner.target_yaw = sv->phi - chassis->state_err[2];
    // chassis->planner.wz = sv->phi_d - chassis->state_err[3];
    // chassis->planner.vx_ramp.planning_v = chassis->planner.vx;
    // chassis->planner.wz_ramp.planning_v = chassis->planner.wz;
    // VAL_LIMIT(chassis->planner.vx_ramp.planning_v, -chassis->planner.vx_ramp.max_v, chassis->planner.vx_ramp.max_v);
    // VAL_LIMIT(chassis->planner.wz_ramp.planning_v, -chassis->planner.wz_ramp.max_v, chassis->planner.wz_ramp.max_v);
  } else {
    // 默认无缩放
    pc->scale_motion[0] = 1.0f;
    pc->scale_motion[1] = 1.0f;
    pc->scale_combined = 1.0f;
    for (int i = 0; i < 2; i++) {
      pc->P_ref[i] = pc->P[i];
      pc->I_ref[i] = pc->I[i];
      pc->T_motion_ref[i] = pc->T_motion[i];
    }
  }

  // 4. 峰值保护：motion 缩放后重新估算总功率，若仍超过峰值阈值则削平衡分量
  pc->scale_balance = 1.0f;
  if (pc->P_total > pc->P_peak_threshold) {
    float P_after = 0.0f;
    for (int i = 0; i < 2; i++) {
      float T_scaled = pc->T_motion[i] * pc->scale_combined + pc->T_balance[i];
      float I_scaled = t2i(T_scaled);
      P_after += MotorEstimatePower(pc->k, I_scaled, pc->w[i]);
    }
    if (P_after > pc->P_peak_threshold) {
      float s = pc->P_peak_threshold / P_after;
      VAL_LIMIT(s, 0.5f, 1.0f);
      pc->scale_balance = s;
      for (int j = 4; j < 10; j++) {
        chassis->state_err[j] *= pc->scale_balance;
      }
    }
  }

  // 5. 动态速度上限：根据功率预算限制 planner 最大速度，防止到达刹不住的速度
  //    P_brake = m * a_decel * v → v_max = P_motion_available / (m * a_decel)
  float P_balance_est = 0.0f;
  for (int i = 0; i < 2; i++) {
    float I_bal = t2i(pc->T_balance[i]);
    P_balance_est += pc->k[4] * I_bal * I_bal;
  }
  float P_motion_budget = pc->P_total_ref - P_balance_est;
  if (P_motion_budget < 20.0f) P_motion_budget = 20.0f;

  float min_decel = chassis->planner.vx_ramp.max_decel;
  float v_max = P_motion_budget / (chassis->param.body_mass * min_decel);
  VAL_LIMIT(v_max, 0.5f, 2.97f);
  chassis->planner.vx_ramp.max_v = v_max;
  pc->v_max_dynamic = v_max;
}

/**
 * @brief prostrate模式功率控制
 *
 */
void PowerControl_Prostrate(ChassisInstance* chassis) {
  Power_Ctrl_t* pc = power_ctrl;
  // 计算每个电机的功率贡献

  for (int i = 0; i < 2; i++) {
    float current_I = chassis->leg[i]->wheel_motor->motor_controller.final_output * DJI_CURRENT_SCALE;
    pc->I[i] = current_I;

    // 对 w 应用一阶低通滤波 (当前不加滤波)
    float current_w = chassis->leg[i]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;
    pc->w[i] = current_w;

    // 总功率用实际电机电流估算，并应用一阶低通滤波 (当前不加滤波)
    float current_P = MotorEstimatePower(pc->k, pc->I[i],
                                         pc->w[i]);  // 使用原始 current_w 计算瞬态功率
    pc->P[i] = current_P;
  }

  pc->P_total = pc->P[0] + pc->P[1];
  pc->P_total_ref = chassis->chassis_ctrl_cmd.max_power;
  // pc->P_total_ref = 180.f;
  // pc->P_total_ref = 50.f;

  // 功率超限时进行动态调整
  if (pc->P_total > pc->P_total_ref) {
    // 重新计算每个电机的电流参考值
    for (int i = 0; i < 2; i++) {
      pc->P_ref[i] = pc->P[i] / pc->P_total * pc->P_total_ref;
      pc->I_ref[i] = MotorEstimateCurrent(pc->k, pc->P_ref[i], pc->w[i], pc->I[i]);
    }
  } else {
    // 不超限时透传实际电流, 写回 = no-op, 避免 I_ref 残值反推到 final_output
    for (int i = 0; i < 2; i++) {
      pc->P_ref[i] = pc->P[i];
      pc->I_ref[i] = pc->I[i];
    }
  }
  for (int i = 0; i < 2; i++) {
    chassis->leg[i]->wheel_motor->motor_controller.final_output = (int16_t)(pc->I_ref[i] / DJI_CURRENT_SCALE);
  }
}

void PowerControlInit(ChassisInstance* chassis) {
  power_ctrl = (Power_Ctrl_t*)zmalloc(sizeof(Power_Ctrl_t));
  chassis->power_ctrl = power_ctrl;
  Power_Ctrl_t* pc = power_ctrl;
  // 6参数模型系数
  pc->k[0] = WHEEL_K0;
  pc->k[1] = WHEEL_K1;
  pc->k[2] = WHEEL_K2;
  pc->k[3] = WHEEL_K3;
  pc->k[4] = WHEEL_K4;
  pc->k[5] = WHEEL_K5;
}
