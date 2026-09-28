/**
 ******************************************************************************
 * @file    parallel_leg.c
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   Parallel-Leg Module
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#include "parallel_leg.h"

#include <math.h>
#include <string.h>  // for memcpy

#include "bsp_dwt.h"
#include "general_def.h"
#include "user_lib.h"

/**
 * @brief   更新腿部机构真实模型参数
 */
static void RealModelUpdate(LegInstance* leg) {
  // 空间换时间，结构体内的参数直接访问，避免频繁传递参数，同时简化书写
  Real_Model_t* rm = &leg->real_model;
  Leg_Param_t* p = &leg->param;

  // 1. 获取电机角度
  rm->phi1 = p->joint_motor_zero_offset[0] + leg->joint_motor[0]->measure.position;
  rm->phi4 = p->joint_motor_zero_offset[1] + leg->joint_motor[1]->measure.position;
  rm->phi1_d = leg->joint_motor[0]->measure.velocity;
  rm->phi4_d = leg->joint_motor[1]->measure.velocity;

  // 2. 计算 B、D 点坐标
  rm->xb = p->rod_length[0] * mcos(rm->phi1);
  rm->yb = p->rod_length[0] * msin(rm->phi1);
  rm->xd = p->rod_length[4] + p->rod_length[3] * mcos(rm->phi4);
  rm->yd = p->rod_length[3] * msin(rm->phi4);

  // 3. 计算中间变量 (局部变量，用完即焚)
  float A0 = 2.0f * p->rod_length[1] * (rm->xd - rm->xb);
  float B0 = 2.0f * p->rod_length[1] * (rm->yd - rm->yb);
  float C0 = p->rod_length[1] * p->rod_length[1] + (rm->xb - rm->xd) * (rm->xb - rm->xd) +
             (rm->yb - rm->yd) * (rm->yb - rm->yd) - p->rod_length[2] * p->rod_length[2];

  // 4. 计算 phi2, phi3, C点
  rm->phi2 = 2 * atan2f((B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
  rm->phi3 =
      atan2f(rm->yb - rm->yd + p->rod_length[1] * msin(rm->phi2), rm->xb - rm->xd + p->rod_length[1] * mcos(rm->phi2));

  rm->xc = rm->xb + p->rod_length[1] * mcos(rm->phi2);
  rm->yc = rm->yb + p->rod_length[1] * msin(rm->phi2);
}

/**
 * @brief   VMC虚拟模型更新
 */
static void VirtualModelUpdate(LegInstance* leg) {
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;
  Leg_Param_t* p = &leg->param;

  // 1. 腿长与角度
  float term_x = rm->xc - p->rod_length[4] / 2.0f;
  vm->length = sqrtf(term_x * term_x + rm->yc * rm->yc);
  vm->phi = atan2f(rm->yc, term_x);
  vm->alpha = PI / 2.0f - vm->phi;
  vm->alpha_d = -vm->phi_d;

  // 2. 雅可比相关中间量 A1
  float A1 = (p->rod_length[0] * rm->phi1_d * msin(rm->phi1 - rm->phi3) +
              p->rod_length[3] * rm->phi4_d * msin(rm->phi3 - rm->phi4)) /
             msin(rm->phi3 - rm->phi2);

  rm->xb_d = -p->rod_length[0] * rm->phi1_d * msin(rm->phi1);
  rm->yb_d = p->rod_length[0] * rm->phi1_d * mcos(rm->phi1);

  // 3. 导数计算
  vm->length_d = (rm->yc * (rm->yb_d + A1 * mcos(rm->phi2)) + term_x * (rm->xb_d - A1 * msin(rm->phi2))) / vm->length;

  vm->phi_d = (term_x * (rm->yb_d + A1 * mcos(rm->phi2)) - rm->yc * (rm->xb_d - A1 * msin(rm->phi2))) /
              (vm->length * vm->length);

  // 4. 初始化处理
  if (!leg->update_flag.is_initialized) {
    vm->last_length_d = vm->length_d;
    vm->last_phi_d = vm->phi_d;
    leg->update_flag.is_initialized = 1;
  }

  // 5. 二阶导数 (加速度)
  vm->phi_dd = (vm->phi_d - vm->last_phi_d) / leg->dt;
  vm->length_dd = (vm->length_d - vm->last_length_d) / leg->dt;

  vm->last_length_d = vm->length_d;
  vm->last_phi_d = vm->phi_d;
}

/**
 * @brief   状态变量更新，其中x_d在速度观测器中更新
 */
static void StateVarUpdate(LegInstance* leg, INS_t* imu) {
  Virtual_Model_t* vm = &leg->virtual_model;
  State_Var_t* sv = &leg->state_var;

  // 位移积分
  if (leg->update_flag.is_controlled || leg->update_flag.is_off_ground) {
    sv->x = 0;
  } else {
    sv->x += ((sv->x_d + leg->last_state_var.x_d) / 2.0f) * leg->dt;
  }

  sv->phi = DEGREE_2_RAD * imu->Pitch;
  sv->phi_d = imu->Gyro[0];
  sv->theta = PI / 2.0f - vm->phi - sv->phi;
  sv->theta_d = -vm->phi_d - sv->phi_d;
}

/**
 * @brief   观测器更新
 */
void ObserverVarUpdate(LegInstance* leg, INS_t* imu) {
  Observer_Var_t* ov = &leg->observer_var;
  Virtual_Model_t* vm = &leg->virtual_model;
  State_Var_t* sv = &leg->state_var;
  Leg_Param_t* p = &leg->param;

  ov->w = -leg->wheel_motor->measure.speed_aps / p->wheel_reduction_ratio * DEGREE_2_RAD + vm->alpha_d - imu->Gyro[0];

  ov->vb = ov->w * p->wheel_radius + vm->length * sv->theta_d * mcos(sv->theta) + vm->length_d * msin(sv->theta);
}

// 计算LQR_K矩阵
static float LQR_K_Calc(const float* coe, float len) {
  return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}

// 计算MPC_K矩阵
static float MPC_K_Calc(const float* coe, float len) {
  return coe[0] * expf(coe[1] * len) + coe[2] * expf(coe[3] * len);
}

/**
 * @brief   离地检测
 */
static void OffGroundDetection(LegInstance* leg) {
  Virtual_Model_t* vm = &leg->virtual_model;
  // 简单近似计算支持力
  float current_FN =
      vm->F * arm_cos_f32(leg->state_var.theta) + vm->Tp * arm_sin_f32(leg->state_var.theta) / vm->length + 6.0f;

// 低通滤波
#define FN_FILTER_COEF 0.2f
  vm->FN = vm->FN * (1.0f - FN_FILTER_COEF) + current_FN * FN_FILTER_COEF;

  if (vm->FN < 30.0f) {
    leg->update_flag.is_off_ground = 1;
  } else {
    leg->update_flag.is_off_ground = 0;
  }
}

/**
 * @brief   计算关节力矩 (基于雅可比矩阵)
 */
void JointTorqueUpdate(LegInstance* leg) {
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;
  Leg_Param_t* p = &leg->param;

  // 计算分母
  float denominator = msin(rm->phi3 - rm->phi2);
  // 保护除零 (极端情况)
  if (fabsf(denominator) < 1e-4f) denominator = 1e-4f;

  // 更新雅可比矩阵 J (存储在实例中)
  leg->J[0][0] = (p->rod_length[0] * msin(vm->phi - rm->phi3) * msin(rm->phi1 - rm->phi2)) / denominator;
  leg->J[0][1] = (p->rod_length[0] * mcos(vm->phi - rm->phi3) * msin(rm->phi1 - rm->phi2)) / (vm->length * denominator);
  leg->J[1][0] = (p->rod_length[3] * msin(vm->phi - rm->phi2) * msin(rm->phi3 - rm->phi4)) / denominator;
  leg->J[1][1] = (p->rod_length[3] * mcos(vm->phi - rm->phi2) * msin(rm->phi3 - rm->phi4)) / (vm->length * denominator);

  // 虚拟力 -> 实际关节力矩
  rm->Tp_1 = leg->J[0][0] * vm->F + leg->J[0][1] * vm->Tp;
  rm->Tp_2 = leg->J[1][0] * vm->F + leg->J[1][1] * vm->Tp;
}

/**
 * @brief 并联腿初始化
 */
LegInstance* LegInit(Leg_Init_Config_s* config) {
  LegInstance* leg_instance = (LegInstance*)zmalloc(sizeof(LegInstance));

  // 1. 复制参数配置到实例内部
  leg_instance->param = config->param;

  // 2. 初始化 PID
  PIDInit(&leg_instance->virtual_model.length_PID, &config->length_PID_config);
  PIDInit(&leg_instance->virtual_model.length_d_PID, &config->length_d_PID_config);

  // 3. 初始化电机
  leg_instance->joint_motor[0] = DMMotorInit(&config->joint_motor_config[0]);
  leg_instance->joint_motor[1] = DMMotorInit(&config->joint_motor_config[1]);
  leg_instance->wheel_motor = DJIMotorInit(&config->wheel_motor_config);

  // 4. 其他初始化
  leg_instance->virtual_model.last_phi_d = 0.0f;
  leg_instance->virtual_model.last_length_d = 0.0f;
  leg_instance->update_flag.is_initialized = 0;
  leg_instance->update_flag.is_restart = 1;
  leg_instance->update_flag.is_off_ground = 0;
  leg_instance->update_flag.is_controlled = 0;

  DWT_GetDeltaT(&leg_instance->DWT_CNT);

  if (config->cali_mode == LEG_CALI_MODE) {
    DMMotorCaliEncoder(leg_instance->joint_motor[0]);
    DMMotorCaliEncoder(leg_instance->joint_motor[1]);
  }

  return leg_instance;
}

/**
 * @brief   并联腿主控制循环
 */
void LegCtrlUpdate(LegInstance* leg, INS_t* imu) {
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;
  Leg_Param_t* p = &leg->param;
  // ==================== 1. 时间步长处理 ====================
  float dt_raw = DWT_GetDeltaT(&leg->DWT_CNT);

  if (dt_raw > 0.05f) {
    leg->dt = 0.001f;
    leg->update_flag.is_restart = 1;
  } else {
    leg->dt = dt_raw;
    leg->update_flag.is_restart = 0;
  }

  // ==================== 2. 模型更新 ====================
  RealModelUpdate(leg);
  VirtualModelUpdate(leg);

  if (leg->update_flag.is_restart) {
    vm->last_phi_d = vm->phi_d;
    vm->last_length_d = vm->length_d;
    vm->phi_dd = 0;
    vm->length_dd = 0;
  }

  StateVarUpdate(leg, imu);

  if (leg->update_flag.is_restart) {
    leg->last_state_var = leg->state_var;
    leg->state_var.x = 0.0f;
    leg->last_state_var.x = 0.0f;
  }

  OffGroundDetection(leg);

  // ==================== 3. 增益计算 & 误差更新 ====================
  // 计算 K 矩阵
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 6; j++) {
      leg->LQR_K[i][j] = LQR_K_Calc(&p->LQR_K_Coefficient[i][j][0], leg->virtual_model.length);
      leg->MPC_K[i][j] = MPC_K_Calc(&p->MPC_K_Coefficient[i][j][0], leg->virtual_model.length);
    }
  }
  float state_err[6];
  float state_delta[6];

  leg->leg_ctrl_cmd.theta_ref = 0;
  leg->leg_ctrl_cmd.x_ref = 0;

  // 计算 State Error
  state_err[0] = leg->state_var.theta - leg->leg_ctrl_cmd.theta_ref;
  state_err[1] = leg->state_var.theta_d - 0.0f;
  state_err[2] = leg->state_var.x - leg->leg_ctrl_cmd.x_ref;
  state_err[3] = leg->state_var.x_d - leg->leg_ctrl_cmd.x_d_ref;
  state_err[4] = leg->state_var.phi - 0.0f;
  state_err[5] = leg->state_var.phi_d - 0.0f;

  // 计算 State Delta
  state_delta[0] = leg->state_var.theta - leg->last_state_var.theta;
  state_delta[1] = leg->state_var.theta_d - leg->last_state_var.theta_d;
  state_delta[2] = leg->state_var.x - leg->last_state_var.x;
  state_delta[3] = leg->state_var.x_d - leg->last_state_var.x_d;
  state_delta[4] = leg->state_var.phi - leg->last_state_var.phi;
  state_delta[5] = leg->state_var.phi_d - leg->last_state_var.phi_d;

  // 特殊情况处理, x项不参与控制
  if (leg->update_flag.is_controlled || leg->update_flag.is_off_ground) {
    state_err[2] = 0;
    state_delta[2] = 0;
  }

  // ==================== 4. 力矩计算 ====================
  rm->T_LQR = 0.0f;
  vm->Tp_LQR = 0.0f;
  vm->Tp_MPC = 0.0f;

  // --- 轮子力矩 (T) ---
  if (!leg->update_flag.is_off_ground) {
    for (int k = 0; k < 6; k++) {
      rm->T_LQR += leg->LQR_K[0][k] * state_err[k];
    }
    leg->real_model.T = rm->T_LQR;
  } else {
    leg->real_model.T = 0;
  }

  // --- 关节力矩 (Tp) ---
  for (int k = 0; k < ((!leg->update_flag.is_off_ground) ? 6 : 2); k++) {
    vm->Tp_LQR += leg->LQR_K[1][k] * state_err[k];
    vm->Tp_MPC += leg->MPC_K[1][k] * state_delta[k];
  }

  // 合成虚拟关节力矩
  leg->virtual_model.Tp = vm->Tp_LQR;
  // leg->virtual_model.Tp = vm->Tp_LQR - vm->Tp_MPC;

  // 腿长 PID
  leg->virtual_model.F =
      PIDCalculate(&leg->virtual_model.length_PID, leg->virtual_model.length, leg->leg_ctrl_cmd.length_ref);

  // ==================== 5. 历史状态更新 ====================
  leg->last_state_var = leg->state_var;
}
