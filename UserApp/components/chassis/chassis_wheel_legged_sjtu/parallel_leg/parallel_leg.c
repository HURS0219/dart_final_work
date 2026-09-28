/**
 ******************************************************************************
 * @file    parallel_leg.h
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   Parallel-Leg Module
 ******************************************************************************
 * @attention
 *     Tp2         Tp1
 *  joint[1] l5 joint[0]    LEFT       RIGHT
 *    phi4 ☉---☉  phi1       ☉----------☉
 *        /     \             |          |
 *    l4 /       \ l1         |          |
 *      /         \           |          |
 * phi3◉           ◉phi2      ◉          ◉
 *      \         /           |          |
 *    L3 \  ___  / L2         |          |
 *        /     \            ---        ---
 *       |   ◉   |           | |        | |
 *        \ ___ /            ---        ---
 ******************************************************************************
 */
#include "parallel_leg.h"

#include <math.h>

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
static void VirtualModelUpdate(LegInstance* leg, INS_t* imu) {
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;
  Leg_Param_t* p = &leg->param;

  // 1. 腿长与角度
  float term_x = rm->xc - p->rod_length[4] / 2.0f;
  vm->length = sqrtf(term_x * term_x + rm->yc * rm->yc);
  vm->phi = atan2f(rm->yc, term_x);
  vm->alpha = PI / 2.0f - vm->phi;
  vm->alpha_d = -vm->phi_d;

  // 2. 计算中间量 A1
  float A1 = (p->rod_length[0] * rm->phi1_d * msin(rm->phi1 - rm->phi3) +
              p->rod_length[3] * rm->phi4_d * msin(rm->phi3 - rm->phi4)) /
             msin(rm->phi3 - rm->phi2);

  rm->xb_d = -p->rod_length[0] * rm->phi1_d * msin(rm->phi1);
  rm->yb_d = p->rod_length[0] * rm->phi1_d * mcos(rm->phi1);

  // 3. 导数计算
  vm->length_d = (rm->yc * (rm->yb_d + A1 * mcos(rm->phi2)) + term_x * (rm->xb_d - A1 * msin(rm->phi2))) / vm->length;

  vm->phi_d = (term_x * (rm->yb_d + A1 * mcos(rm->phi2)) - rm->yc * (rm->xb_d - A1 * msin(rm->phi2))) /
              (vm->length * vm->length);

  vm->theta = PI / 2.0f - vm->phi - DEGREE_2_RAD * imu->Pitch;
  vm->theta_d = -vm->phi_d - imu->Gyro[0];

  // 4. 重载处理
  if (leg->update_flag.is_restart || leg->update_flag.is_first_update) {
    vm->last_phi_d = vm->phi_d;
    vm->last_length_d = vm->length_d;
    leg->update_flag.is_first_update = 0;
  }
  // 5. 二阶导数 (加速度)
  vm->phi_dd = (vm->phi_d - vm->last_phi_d) / leg->dt;
  vm->length_dd = (vm->length_d - vm->last_length_d) / leg->dt;

  vm->last_length_d = vm->length_d;
  vm->last_phi_d = vm->phi_d;
}

/**
 * @brief   观测器更新
 */
void ObserverVarUpdate(LegInstance* leg, INS_t* imu) {
  Observer_Var_t* ov = &leg->observer_var;
  Virtual_Model_t* vm = &leg->virtual_model;
  Leg_Param_t* p = &leg->param;

  ov->w = -leg->wheel_motor->measure.speed_aps / p->wheel_reduction_ratio * DEGREE_2_RAD + vm->alpha_d - imu->Gyro[0];
  ov->vb = ov->w * p->wheel_radius + vm->length * vm->theta_d * mcos(vm->theta) + vm->length_d * msin(vm->theta);
}

/**
 * @brief   离地检测
 */
static void OffGroundDetection(LegInstance* leg) {
  Virtual_Model_t* vm = &leg->virtual_model;

  // 简单近似计算支持力
  float current_FN = vm->F * arm_cos_f32(leg->virtual_model.theta) +
                     vm->Tp * arm_sin_f32(leg->virtual_model.theta) / vm->length + 6.0f;

// 低通滤波
#define FN_FILTER_COEF 0.2f
  vm->FN = vm->FN * (1.0f - FN_FILTER_COEF) + current_FN * FN_FILTER_COEF;

  if (vm->FN < 50.0f) {
    leg->update_flag.is_off_ground = 1;
  } else {
    leg->update_flag.is_off_ground = 0;
  }
}

/**
 * @brief  计算并叠加两个关节的非线性限位力矩到 Tp_1 / Tp_2
 *
 * 当关节角度进入缓冲区时，产生二次方非线性回推力矩 + 单向阻尼力矩，
 * 防止电机到达机械限位而堵转。正常工作区间内力矩严格为零。
 */
void JointLimitBarrier(LegInstance* leg) {
  float* tp[2] = {&leg->real_model.Tp_1, &leg->real_model.Tp_2};

  for (int i = 0; i < 2; i++) {
    const JointLimit_Config_s* cfg = &leg->param.joint_limit[i];
    float angle = leg->joint_motor[i]->measure.position;
    float velocity = leg->joint_motor[i]->measure.velocity;
    float tau = 0.0f;

    // 上限缓冲区
    float upper_threshold = cfg->angle_max - cfg->buffer_zone;
    if (angle > upper_threshold) {
      float d = angle - upper_threshold;
      tau += -cfg->kp * d * d;
      if (velocity > 0.0f) {
        tau += -cfg->kd * velocity;
      }
    }

    // 下限缓冲区
    float lower_threshold = cfg->angle_min + cfg->buffer_zone;
    if (angle < lower_threshold) {
      float d = lower_threshold - angle;
      tau += cfg->kp * d * d;
      if (velocity < 0.0f) {
        tau += -cfg->kd * velocity;
      }
    }

    // 硬限位穿越保护：线性项叠加，确保能拉回
    if (angle > cfg->angle_max) {
      float overshoot = angle - cfg->angle_max;
      tau -= cfg->kp * cfg->buffer_zone * overshoot * 10.0f;
    }
    if (angle < cfg->angle_min) {
      float overshoot = cfg->angle_min - angle;
      tau += cfg->kp * cfg->buffer_zone * overshoot * 10.0f;
    }

    if (tau > cfg->max_barrier_torque) {
      tau = cfg->max_barrier_torque;
    } else if (tau < -cfg->max_barrier_torque) {
      tau = -cfg->max_barrier_torque;
    }

    *tp[i] += tau;
  }
}

void SpringCompensation(LegInstance* leg) {
  float k = 6000.0f;
  float j2f = 0.06403f;  // 关节2到固定点的距离
  float j2k = 0.12135f;  // 关节2到膝盖点的距离
  float j2f_a = 38.66f;
  float j2k_a = 9.01f;
  float spring_init_length = 0.100f;
  float alpha_1 = leg->real_model.phi1 - j2f_a * DEGREE_2_RAD + j2k_a * DEGREE_2_RAD;
  float alpha_2 = PI - (leg->real_model.phi4 - j2k_a * DEGREE_2_RAD) - j2f_a * DEGREE_2_RAD;
  float spring_length_1 = sqrtf(j2f * j2f + j2k * j2k - 2.0f * j2f * j2k * mcos(alpha_1));
  float spring_length_2 = sqrtf(j2f * j2f + j2k * j2k - 2.0f * j2f * j2k * mcos(alpha_2));

  float spring_F_1 = (spring_length_1 - spring_init_length) * k;
  float spring_F_2 = (spring_length_2 - spring_init_length) * k;
  float T1 = (j2f * msin(alpha_1) / spring_length_1) * spring_F_1 * j2k;
  float T2 = (j2f * msin(alpha_2) / spring_length_2) * spring_F_2 * j2k;

  leg->real_model.Tp_1 += T1;
  leg->real_model.Tp_2 -= T2;
}

/**
 * @brief   根据关节力矩反解虚拟模型的 F 和 Tp（雅可比逆映射）
 *
 * 已知两个关节电机的实际力矩 Tp_1、Tp_2，通过雅可比矩阵的逆
 * 反算出虚拟模型空间中的腿部推力 F 和髋关节力矩 Tp。
 *
 * 映射关系：
 *   正向（JointTorqueUpdate）:  [Tp_1, Tp_2]^T = J * [F, Tp]^T
 *   逆向（本函数）:             [F, Tp]^T = J^{-1} * [Tp_1, Tp_2]^T
 *
 * @param[in,out] leg    腿部实例
 * @param[in]     Tp_1   关节0（phi1侧）的实际力矩 (Nm)
 * @param[in]     Tp_2   关节1（phi4侧）的实际力矩 (Nm)
 *
 * @note 调用前需确保 RealModelUpdate() 和 VirtualModelUpdate() 已执行，
 *       以保证 phi1~phi4、phi、length 等几何量为最新值。
 *       函数会同步更新 leg->J[][] 雅可比矩阵。
 *       解算结果写入 vm->F 和 vm->Tp。
 */
void VirtualForceFromJointTorque(LegInstance* leg, float Tp_1, float Tp_2) {
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;
  Leg_Param_t* p = &leg->param;
  /* ── Step 1: 计算雅可比矩阵（与 JointTorqueUpdate 完全一致）── */
  float denominator = msin(rm->phi3 - rm->phi2);
  if (fabsf(denominator) < 1e-4f)
    denominator = (denominator >= 0.0f) ? 1e-4f : -1e-4f;
  leg->J[0][0] = (p->rod_length[0] * msin(vm->phi - rm->phi3) *
                   msin(rm->phi1 - rm->phi2)) /
                  denominator;
  leg->J[0][1] = (p->rod_length[0] * mcos(vm->phi - rm->phi3) *
                   msin(rm->phi1 - rm->phi2)) /
                  (vm->length * denominator);
  leg->J[1][0] = (p->rod_length[3] * msin(vm->phi - rm->phi2) *
                   msin(rm->phi3 - rm->phi4)) /
                  denominator;
  leg->J[1][1] = (p->rod_length[3] * mcos(vm->phi - rm->phi2) *
                   msin(rm->phi3 - rm->phi4)) /
                  (vm->length * denominator);
  /* ── Step 2: 计算行列式并保护除零 ── */
  float det = leg->J[0][0] * leg->J[1][1] - leg->J[0][1] * leg->J[1][0];
  if (fabsf(det) < 1e-6f)
    det = (det >= 0.0f) ? 1e-6f : -1e-6f;
  /* ── Step 3: 逆矩阵求解 ──
   *   [F ]     1    [ J11  -J01 ] [Tp_1]
   *   [Tp] = ----- * [-J10   J00 ] [Tp_2]
   *           det
   */
  float inv_det = 1.0f / det;
  vm->F  = inv_det * ( leg->J[1][1] * Tp_1 - leg->J[0][1] * Tp_2);
  vm->Tp = inv_det * (-leg->J[1][0] * Tp_1 + leg->J[0][0] * Tp_2);
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

  leg_instance->param = config->param;

  leg_instance->joint_motor[0] = DMMotorInit(&config->joint_motor_config[0]);
  leg_instance->joint_motor[1] = DMMotorInit(&config->joint_motor_config[1]);
  leg_instance->wheel_motor = DJIMotorInit(&config->wheel_motor_config);

  PIDInit(&leg_instance->length_PID, &config->length_PID_config);
  leg_instance->update_flag.is_first_update = 1;
  leg_instance->update_flag.is_restart = 1;
  leg_instance->update_flag.is_off_ground = 0;

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
void LegModelUpdate(LegInstance* leg, INS_t* imu) {
  float dt_raw = DWT_GetDeltaT(&leg->DWT_CNT);

  if (dt_raw > 0.05f) {
    leg->dt = 0.001f;
    leg->update_flag.is_restart = 1;
  } else {
    leg->dt = dt_raw;
    leg->update_flag.is_restart = 0;
  }

  RealModelUpdate(leg);

  VirtualModelUpdate(leg, imu);

  OffGroundDetection(leg);
}