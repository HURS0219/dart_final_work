/**
 * @file    chassis.c
 * @author  Enhao Zhang
 * @date    2026/2
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   SJTU Full-Chassis Wheel-Legged Module
 * @todo    unify motor control and value limit to fully decouple this module (also probably for imu)
 */
#include "chassis.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "general_def.h"
#include "parallel_leg.h"
#include "power_control.h"
#include "referee.h"
#include "speed_observer.h"
#include "super_cap.h"
#include "user_lib.h"

static ChassisInstance* chassis;
static LegInstance* leg[2];
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;
static referee_info_t* referee_data;
static float wheel_speed_ref[2];
static float yaw_prostrate_pid_output_lpf = 0.0f;
// 平衡 → 卧倒 平滑过渡标志：1 表示正在用 LQR 把腿降到最低，再切真正的卧倒控制
static uint8_t descending_to_prostrate = 0;

static float FirstOrderLowPass(float input, float last_output, float dt, float rc) {
  if (rc <= 0.0f) {
    return input;
  }
  if (dt <= 0.0f) {
    return last_output;
  }

  float alpha = dt / (rc + dt);
  return last_output + alpha * (input - last_output);
}

static void ResetLegMotorRuntime(LegInstance* leg_instance) {
  if (leg_instance == NULL) return;

  PIDClear(&leg_instance->length_PID);
  PIDClear(&leg_instance->wheel_motor->motor_controller.speed_PID);
  PIDClear(&leg_instance->wheel_motor->motor_controller.angle_PID);
  leg_instance->wheel_motor->motor_controller.final_output = 0.0f;

  for (int i = 0; i < 2; i++) {
    PIDClear(&leg_instance->joint_motor[i]->motor_controller.angle_PID);
    PIDClear(&leg_instance->joint_motor[i]->motor_controller.speed_PID);
    leg_instance->joint_motor[i]->motor_controller.final_output = 0.0f;
  }
}

static void ResetChassisBalanceMemory(void) {
  memset(&chassis->state_var, 0, sizeof(chassis->state_var));
  memset(&chassis->last_state_var, 0, sizeof(chassis->last_state_var));
  memset(chassis->state_err, 0, sizeof(chassis->state_err));
  Kalman_Filter_Clear(&chassis->vaEstimateKF);
  PIDClear(&chassis->roll_PID);
  PowerControlRuntimeReset(chassis);
  chassis->update_flag.is_restart = 1;

  // planner: 规划态对齐当前 yaw, 速度/滤波/计数清零 (ramp 配置参数保持)
  chassis->planner.target_yaw = chassis->imu->YawTotalAngle * DEGREE_2_RAD;
  chassis->planner.vx = 0.0f;
  chassis->planner.wz = 0.0f;
  chassis->planner.vx_ramp.planning_v = 0.0f;
  chassis->planner.vx_ramp.expected_a = 0.0f;
  chassis->planner.wz_ramp.planning_v = 0.0f;
  chassis->planner.wz_ramp.expected_a = 0.0f;
  chassis->planner.yaw_err_filt = 0.0f;
  chassis->planner.snap_count = 0;

  for (int i = 0; i < 2; i++) {
    leg[i]->observer_var.w = 0.0f;
    leg[i]->observer_var.vb = 0.0f;
    leg[i]->real_model.T = 0.0f;
    leg[i]->real_model.Tp_1 = 0.0f;
    leg[i]->real_model.Tp_2 = 0.0f;
    leg[i]->virtual_model.Tp = 0.0f;
    ResetLegMotorRuntime(leg[i]);
  }
}

static void ResetProstrateMemory(void) {
  PIDClear(&chassis->yaw_prostrate_PID);
  PowerControlRuntimeReset(chassis);
  wheel_speed_ref[0] = 0.0f;
  wheel_speed_ref[1] = 0.0f;
  yaw_prostrate_pid_output_lpf = 0.0f;
  for (int i = 0; i < 2; i++) {
    ResetLegMotorRuntime(leg[i]);
  }
}
/**
 * @brief  计算LQR增益矩阵K
 *
 * 根据左腿长度l_l和右腿长度l_r，通过二维二次多项式插值系数coef，计算每个K[i][j]的值。
 * K_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
 *
 * @param[out] K      计算得到的LQR增益矩阵，大小为4x10
 * @param[in]  coef   二次多项式系数数组，每个元素为6个系数，长度共40（4 * 10）
 * @param[in]  l_l    左腿长度
 * @param[in]  l_r    右腿长度
 */
static void LQR_K_Calc(float K[4][10], const float coef[40][6], float l_l, float l_r) {
  for (int n = 0; n < 40; n++) {
    int row = n / 10;
    int col = n % 10;
    K[row][col] = coef[n][0] + coef[n][1] * l_l + coef[n][2] * l_r + coef[n][3] * l_l * l_l + coef[n][4] * l_l * l_r +
                  coef[n][5] * l_r * l_r;
  }
}

static void SpeedEstimate(void) {
  float chassis_aver_v;
  ObserverVarUpdate(leg[0], chassis->imu);
  ObserverVarUpdate(leg[1], chassis->imu);
  chassis_aver_v = (leg[0]->observer_var.vb + leg[1]->observer_var.vb) * 0.5f;
  xvEstimateKF_Update(&chassis->vaEstimateKF, chassis->imu->MotionAccel_n[1], chassis_aver_v);
}

/**
 * SJTU 状态与 VMC/IMU 映射（须与 MATLAB 线性化/推导文档约定一致）:
 *   phi, phi_d     ← chassis_IMU->YawTotalAngle (rad), IMU Gyro[2] (yaw, rad/s)
 *   theta_b      ← IMU Pitch (rad)，俯仰角
 *   theta_b_d      ← IMU Gyro[0] (pitch 角速度, rad/s)
 *   theta_l       ← leg[1]->state_var.theta (左腿与 Z 负方向夹角，VMC 解算)
 *   theta_r       ← leg[0]->state_var.theta (右腿)
 * 若 IMU 或 VMC 的符号/零点与推导文档不一致，LQR 反馈可能反号，需在文档或测试中核对。
 */
static void StateVarUpdate(void) {
  State_Var_t* sv = &chassis->state_var;
  INS_t* imu = chassis->imu;
  if (chassis->update_flag.is_restart) {
    chassis->state_var.x_b = 0.0f;
    chassis->last_state_var = chassis->state_var;
  }
  /* x_b: no position sensor, keep 0 or integrate in observer path */
  SpeedEstimate();

  const uint8_t is_rotate = chassis_ctrl_cmd->is_rotate;
  const uint8_t off_ground = leg[0]->update_flag.is_off_ground || leg[1]->update_flag.is_off_ground;

  // 小陀螺时 phi_d 来自轮速差, 低速段 SNR 差, 用一阶 LPF 抑制抖动. 进入 rotate 或冷启时对齐到 raw.
  static float phi_d_filt = 0.0f;
  static uint8_t last_is_rotate = 0;

  if (is_rotate) {
    // 小陀螺: 绕过 IMU yaw 陀螺/KF, 直接用左右轮体系速度算 x_b_d 与 phi_d.
    // phi_d 符号约定: phi_d>0 为左转 → 右轮(leg[0])线速度大于左轮(leg[1]).
    float vb_r = leg[0]->observer_var.vb;
    float vb_l = leg[1]->observer_var.vb;
    sv->x_b_d = sv->x_b_d * 0.8f + (0.2f * (vb_r + vb_l) * 0.5f);
    sv->phi_d = sv->phi_d * 0.8f + (0.2f * (vb_r - vb_l) / chassis->param.track_width);
    // sv->x_b_d = chassis->vaEstimateKF.FilteredValue[0];
    // sv->phi_d = imu->Gyro[2];
  } else {
    sv->x_b_d = chassis->vaEstimateKF.FilteredValue[0];
    sv->phi_d = imu->Gyro[2];
  }
  last_is_rotate = is_rotate;

  // 位移积分
  if (off_ground) {
    sv->x_b = 0.0f;
  } else if (is_rotate) {
    // 小陀螺: 始终积分 x_b, 不因 is_controlled / x_b_d 阈值清零
    sv->x_b += ((sv->x_b_d + chassis->last_state_var.x_b_d) / 2.0f) * chassis->dt;
  } else if (chassis->update_flag.is_controlled || sv->x_b_d > 0.15f) {
    sv->x_b = 0.0f;
  } else {
    sv->x_b += ((sv->x_b_d + chassis->last_state_var.x_b_d) / 2.0f) * chassis->dt;
  }

  sv->phi = imu->YawTotalAngle * DEGREE_2_RAD;
  /* Left leg = leg[1], Right leg = leg[0] */
  sv->theta_r = leg[0]->virtual_model.theta;
  sv->theta_r_d = leg[0]->virtual_model.theta_d;
  sv->theta_l = leg[1]->virtual_model.theta;
  sv->theta_l_d = leg[1]->virtual_model.theta_d;
  sv->theta_b = -imu->Pitch * DEGREE_2_RAD;
  sv->theta_b_d = -imu->Gyro[0];

  chassis->last_state_var = *sv;
}

// 底盘 planner: 把上层 raw cmd 在 chassis 时基 (200Hz) 平滑.
// 输入: chassis_ctrl_cmd.{target_yaw, vx, wz, is_rotate} (raw, 50Hz CAN ZOH 到达)
// 输出: chassis->planner.{target_yaw, vx, wz} (200Hz 连续, LQR 实际跟踪对象)
// is_rotate 时强制 yaw_err=0, 仅靠 cmd.wz 作 wz 期望 FF 推进 target_yaw, 避免 raw=imu 时残留 lag 项反向拉.
static void ChassisPlannerUpdate(float dt) {
  ChassisPlanner_t* p = &chassis->planner;

  // 1. yaw 误差: ROTATE 模式置零, FOLLOW/FREE 用 raw - 规划态
  float yaw_err_raw = chassis_ctrl_cmd->is_rotate ? 0.0f : rad_format(chassis_ctrl_cmd->target_yaw - p->target_yaw);
  const float kLpfTau = 0.008f;
  float alpha = dt / (kLpfTau + dt);
  p->yaw_err_filt += alpha * (yaw_err_raw - p->yaw_err_filt);
  float yaw_err = p->yaw_err_filt;

  // 2. 期望角速度: |err|<=eps 线性, >eps sqrt 制动律. 连接点 K = sqrt(2a/eps), C0 连续.
  const float kEpsLin = 0.05f;
  float abs_err = fabsf(yaw_err);
  float desired_wz;
  if (abs_err <= kEpsLin) {
    float k_lin = sqrtf(2.0f * p->wz_ramp.max_accel / kEpsLin);
    desired_wz = k_lin * yaw_err;
  } else {
    desired_wz = sqrtf(2.0f * p->wz_ramp.max_accel * abs_err);
    if (yaw_err < 0.0f) desired_wz = -desired_wz;
  }
  VAL_LIMIT(desired_wz, -p->wz_ramp.max_v, p->wz_ramp.max_v);
  desired_wz += chassis_ctrl_cmd->wz;  // wz_demand 作 FF (ROTATE: rotate_scale; FOLLOW/FREE: 0)

  p->wz = ramp_controller_update(&p->wz_ramp, desired_wz, 0.0f, dt);
  p->target_yaw += p->wz * dt;

  // 3. snap-to-target 滞回, 仅 FOLLOW/FREE 启用 (ROTATE 期望持续移动)
  if (!chassis_ctrl_cmd->is_rotate) {
    float yaw_err_final = rad_format(chassis_ctrl_cmd->target_yaw - p->target_yaw);
    if (fabsf(yaw_err_final) < 0.003f && fabsf(p->wz_ramp.planning_v) < 0.03f) {
      if (p->snap_count < 5) {
        p->snap_count++;
      } else {
        p->target_yaw = chassis_ctrl_cmd->target_yaw;
        p->wz_ramp.planning_v = 0.0f;
        p->wz_ramp.expected_a = 0.0f;
        p->wz = 0.0f;
      }
    } else {
      p->snap_count = 0;
    }
    if (chassis_ctrl_cmd->is_rotate == 0) p->wz = 0.0f;
  } else {
    p->snap_count = 0;
  }

  // 4. vx ramp (使用本地 x_b_d 实测, 优于 gimbal 板的 stale 值)
  p->vx = ramp_controller_update(&p->vx_ramp, chassis_ctrl_cmd->vx, chassis->state_var.x_b_d, dt);
}

static void StateErrCalc(void) {
  State_Var_t* sv = &chassis->state_var;

  chassis->state_err[0] = sv->x_b - 0.0f;
  chassis->state_err[1] = sv->x_b_d - chassis->planner.vx;
  VAL_LIMIT(chassis->state_err[1], -3.2f, 3.2f);
  chassis->state_err[2] = sv->phi - chassis->planner.target_yaw;
  // VAL_LIMIT(chassis->state_err[2], -0.5f, 0.5f);  // ±25°
  // phi_d 参考 = planner.wz: ROTATE 下避免位置项推进/速度项阻尼相互打架形成极限环;
  // FOLLOW/FREE 稳态 planner.wz≈0, 等价旧行为.
  chassis->state_err[3] = sv->phi_d - chassis->planner.wz;
  VAL_LIMIT(chassis->state_err[3], -3.14f, 3.14f);  // ±30°
  chassis->state_err[4] = sv->theta_l - chassis_ctrl_cmd->theta_ff;
  chassis->state_err[5] = sv->theta_l_d;
  chassis->state_err[6] = sv->theta_r - chassis_ctrl_cmd->theta_ff;
  chassis->state_err[7] = sv->theta_r_d;
  chassis->state_err[8] = sv->theta_b;
  chassis->state_err[9] = sv->theta_b_d;
}

static void LocomotionController(void) {
  chassis->update_flag.is_controlled = chassis_ctrl_cmd->vx != 0;

  /* u[0] = T_{r→b} (hip torque on body, same convention as virtual_model.Tp)
   * u[1] = T_{l→b}
   * u[2] = T_{wr→r} (reaction on leg; motor drive torque = -T_{wr→r})
   * u[3] = T_{wl→l} */
  float u[4];
  for (int i = 0; i < 4; i++) {
    u[i] = 0.0f;
    for (int j = 0; j < 10; j++) {
      u[i] -= chassis->LQR_K[i][j] * chassis->state_err[j];
    }
  }

  /* 离地检测逻辑：T置零，Tp只保留theta的影响（不包含theta_b） */
  /* 右腿 leg[0] */
  if (leg[0]->update_flag.is_off_ground) {
    u[2] = 0.0f;
    // theta_r (idx 6) 和 theta_r_d (idx 7)
    u[0] = -chassis->LQR_K[0][6] * chassis->state_err[6] - chassis->LQR_K[0][7] * chassis->state_err[7];
  }
  /* 左腿 leg[1] */
  if (leg[1]->update_flag.is_off_ground) {
    u[3] = 0.0f;
    // theta_l (idx 4) 和 theta_l_d (idx 5)
    u[1] = -chassis->LQR_K[1][4] * chassis->state_err[4] - chassis->LQR_K[1][5] * chassis->state_err[5];
  }

  leg[0]->virtual_model.Tp = u[0];
  leg[1]->virtual_model.Tp = u[1];
  leg[0]->real_model.T = u[2];
  leg[1]->real_model.T = u[3];
}

/*
 * Force distribution (plan formulas 3.6–3.8):
 *   F_bl,l = +F_psi + F_l + F_gravity - F_inertial   (LEFT = leg[1])
 *   F_bl,r = -F_psi + F_l + F_gravity + F_inertial   (RIGHT = leg[0])
 * F_psi = roll_comp: positive when body rolls left → boost left support.
 * 离心力补偿符号：phi_d>0 为左转，机身向外（右）倾，右腿(leg[0])需更大支撑 → leg[0] +f_inertial, leg[1]
 * -f_inertial，当前实现正确。
 */
static void LegController(void) {
  float roll_err = chassis_ctrl_cmd->roll - chassis->imu->Roll * DEGREE_2_RAD;
  float f_psi = PIDCalculate(&chassis->roll_PID, chassis->imu->Roll * DEGREE_2_RAD, chassis_ctrl_cmd->roll) +
                chassis_ctrl_cmd->roll_ff;
  float l_avg = (leg[0]->virtual_model.length + leg[1]->virtual_model.length) * 0.5f;
  float leg_length_r = chassis->chassis_ctrl_cmd.leg_length - chassis->param.track_width * roll_err;
  float leg_length_l = chassis->chassis_ctrl_cmd.leg_length + chassis->param.track_width * roll_err;
  float f_l_r = PIDCalculate(&chassis->leg[0]->length_PID, leg[0]->virtual_model.length, leg_length_r);
  float f_l_l = PIDCalculate(&chassis->leg[1]->length_PID, leg[1]->virtual_model.length, leg_length_l);
  float f_gravity = 0.5f * chassis->param.body_mass * 9.81f;
  float f_inertial = 0.5f * chassis->param.body_mass * (l_avg / chassis->param.track_width) * chassis->state_var.phi_d *
                     chassis->state_var.x_b_d;
  // float f_inertial = 0.0f;
  // 小陀螺时 phi_d/x_b_d 来自轮速差/和, f_inertial 会随转速近似线性扩张并把支撑力推飞, 直接关闭前馈
  // 退出小陀螺后可能仍有较高转速，加入转速阈值衰减，避免离心力补偿突变导致 roll 飞掉
  float phi_d_abs = fabsf(chassis->state_var.phi_d);
  float inertial_scale = 1.0f;
  if (phi_d_abs > 3.0f) {
    inertial_scale = 1.0f - (phi_d_abs - 3.0f) / 2.0f;  // >5.0rad/s时为0, 3.0~5.0rad/s之间线性过渡
    if (inertial_scale < 0.0f) inertial_scale = 0.0f;
  }

  if (chassis_ctrl_cmd->is_rotate) {
    f_inertial = 0.0f;
  } else {
    f_inertial *= inertial_scale;
  }

  leg[0]->virtual_model.F = -f_psi + f_l_r + f_gravity + f_inertial * 5.0f;
  if (leg[0]->update_flag.is_off_ground) {
    leg[0]->virtual_model.F = -f_psi * 0.3f + f_l_r + f_gravity;
  }
  leg[1]->virtual_model.F = f_psi + f_l_l + f_gravity - f_inertial * 5.0f;
  if (leg[1]->update_flag.is_off_ground) {
    leg[1]->virtual_model.F = f_psi * 0.3f + f_l_l + f_gravity;
  }
}

static void ChassisCtrlUpdate(void) {
  float dt_raw = DWT_GetDeltaT(&chassis->DWT_CNT);

  if (dt_raw > 0.05f) {
    chassis->dt = 0.001f;
    chassis->update_flag.is_restart = 1;
  } else {
    chassis->dt = dt_raw;
    chassis->update_flag.is_restart = 0;
  }

  LegModelUpdate(leg[0], chassis->imu);
  LegModelUpdate(leg[1], chassis->imu);

  StateVarUpdate();

  // 由于 LQR_K_Calc 依赖 leg length，故需要先更新 K，再算 state_err
  float l_l = leg[1]->virtual_model.length;
  float l_r = leg[0]->virtual_model.length;
  LQR_K_Calc(chassis->LQR_K, chassis->param.LQR_K_Coefficients, l_l, l_r);

  // planner 平滑上层 raw cmd, StateErrCalc 实际跟踪 planner 输出
  // ChassisCtrlUpdate 始终是 LQR 平衡输出路径；真实卧倒输出在 LimitChassisOutput() 中限功率。
  ChassisPlannerUpdate(chassis->dt);
  StateErrCalc();
  // PowerControl(chassis);

  LocomotionController();
  LegController();

  // 状态机形式处理不同跳跃状态下的腿部力分配
  switch (chassis->jump_state) {
    case JUMP_STATE_EXTEND: {
      // 起跳时给两条腿注入跳跃力
      float jump_f = chassis_ctrl_cmd->jump_force;
      leg[0]->virtual_model.F = jump_f;
      leg[1]->virtual_model.F = jump_f;
      break;
    }
    case JUMP_STATE_RETRACT: {
      // 收腿时加入前馈力以更快速收腿
      const float retract_feedforward = -250.0f;  // 可调整收腿前馈力大小
      leg[0]->virtual_model.F += retract_feedforward;
      leg[1]->virtual_model.F += retract_feedforward;
      break;
    }
    default: {
      break;
    }
  }
  VAL_LIMIT(leg[0]->virtual_model.F, -3500.0f, 3500.0f);
  VAL_LIMIT(leg[1]->virtual_model.F, -3500.0f, 3500.0f);

  for (int i = 0; i < 2; i++) {
    JointTorqueUpdate(leg[i]);
    // SpringCompensation(leg[i]);
    JointLimitBarrier(leg[i]);
  }
}

static void ChassisRecovery(void) {
  // 将target_yaw对齐到当前底盘航向，避免LQR启动时产生大yaw误差
  chassis_ctrl_cmd->target_yaw = chassis->imu->YawTotalAngle * DEGREE_2_RAD;
  // planner 规划态同步, 避免恢复期间 planner.target_yaw 漂移导致放行时 LQR 起步剧动
  chassis->planner.target_yaw = chassis_ctrl_cmd->target_yaw;
  chassis->planner.wz_ramp.planning_v = 0.0f;
  chassis->planner.vx_ramp.planning_v = 0.0f;
  chassis->planner.yaw_err_filt = 0.0f;
  chassis->planner.snap_count = 0;

  // 1. 清除状态并设置外环为角度环，参考值为 -0.1/0.1，准备复位
  chassis->update_flag.is_controlled = 0;
  for (int i = 0; i < 2; i++) {
    leg[i]->update_flag.is_off_ground = 0;

    DMMotorOuterLoop(leg[i]->joint_motor[0], ANGLE_LOOP);
    DMMotorOuterLoop(leg[i]->joint_motor[1], ANGLE_LOOP);

    DMMotorSetPIDRef(leg[i]->joint_motor[0], -0.1f);
    DMMotorSetPIDRef(leg[i]->joint_motor[1], 0.1f);

    leg[i]->real_model.Tp_1 = leg[i]->joint_motor[0]->motor_controller.final_output + 8.0f;
    leg[i]->real_model.Tp_2 = leg[i]->joint_motor[1]->motor_controller.final_output - 8.0f;
  }

  // 刷新腿模型和状态变量，否则 theta_l/theta_r 冻结在倒地时的旧值
  LegModelUpdate(leg[0], chassis->imu);
  LegModelUpdate(leg[1], chassis->imu);
  StateVarUpdate();

  // 2. 判断关节是否均到达目标位，且腿角 theta 是否已回到可平衡范围
  //    两者都满足才允许进入 ChassisCtrlUpdate（即 LQR 平衡控制），
  //    否则保持挂零轮力，避免在大角度倾倒时进 LQR 导致输出震荡。
  uint8_t all_in_position = 1;
  for (int i = 0; i < 2; i++) {
    if (fabsf(leg[i]->joint_motor[0]->measure.position - (-0.1f)) > 0.5f ||
        fabsf(leg[i]->joint_motor[1]->measure.position - (0.1f)) > 0.5f) {
      all_in_position = 0;
      break;
    }
  }

  const float kRecoveryLqrThresh = 30.0f * DEGREE_2_RAD;
  uint8_t theta_safe = fabsf(chassis->state_var.theta_l) < kRecoveryLqrThresh &&
                        fabsf(chassis->state_var.theta_r) < kRecoveryLqrThresh;

  if (all_in_position && theta_safe) {
    ChassisCtrlUpdate();
  } else {
    for (int i = 0; i < 2; i++) {
      leg[i]->real_model.T = 0.0f;
    }
  }
}

static void ChassisJump(void) {
  switch (chassis->jump_state) {
    case JUMP_STATE_COMPRESS:
      chassis->chassis_ctrl_cmd.leg_length = chassis->param.leg_min_length;
      ChassisCtrlUpdate();
      break;
    case JUMP_STATE_EXTEND:
      chassis->chassis_ctrl_cmd.leg_length = chassis->param.leg_max_length;
      ChassisCtrlUpdate();
      if (fabsf(leg[0]->virtual_model.length - chassis->param.leg_max_length) <= 0.05f &&
          fabsf(leg[1]->virtual_model.length - chassis->param.leg_max_length) <= 0.05f) {
        osDelay(25);
        chassis->jump_state = JUMP_STATE_RETRACT;
      }
      break;
    case JUMP_STATE_RETRACT:
      chassis->chassis_ctrl_cmd.leg_length = chassis->param.leg_min_length;
      ChassisCtrlUpdate();
      if (fabsf(leg[0]->virtual_model.length - chassis->param.leg_min_length) <= 0.05f &&
          fabsf(leg[1]->virtual_model.length - chassis->param.leg_min_length) <= 0.05f) {
        osDelay(275);
        chassis->jump_state = JUMP_STATE_IDLE;
      }
      break;
    case JUMP_STATE_IDLE:
    default:
      ChassisCtrlUpdate();
      break;
  }
}

/**
 * @brief 卧倒模式
 *
 * 差速：右轮 = vx + wz，左轮 = vx - wz
 *
 * 入参单位已与 stand 统一: chassis_ctrl_cmd->vx 为 m/s, chassis_ctrl_cmd->wz 为 rad/s;
 * 此处做趴下特有的 m/s / rad/s → 电机 rpm 换算, 避开 LQR 路径.
 * target_yaw 负责 yaw 闭环保持. 小陀螺/自由转向若不希望 yaw PID 参与, 上层需把 target_yaw 对齐当前 yaw.
 */
void ChassisProstrate(void) {
  // 与原始摇杆 ±660 满量等效峰值:
  //   vx: 660 × (30000/660) = 30000 rpm  ↔ 站立 FOLLOW ±2.5 m/s → 12000
  //   wz: 660 × (28000/660) = 28000 rpm-diff ↔ 站立 ROTATE ±15 rad/s → 28000/15

#define VX_MPS_TO_MOTOR 14000.0f
#define WZ_PID_TO_MOTOR 20000.0f
#define WZ_RADPS_TO_MOTOR (54000.0f / 15.0f)
  float vx_motor = 0.0f;

  // 设置速度环
  for (int i = 0; i < 2; i++) {
    leg[i]->wheel_motor->motor_settings.close_loop_type = SPEED_LOOP;
    leg[i]->wheel_motor->motor_settings.outer_loop_type = SPEED_LOOP;
  }

  float wz_pid = 0.0f;
  if (fabsf(chassis_ctrl_cmd->wz) > 0.01f) {
    PIDClear(&chassis->yaw_prostrate_PID);
    yaw_prostrate_pid_output_lpf = 0.0f;
  } else {
    float yaw_pid_output = PIDCalculate(&chassis->yaw_prostrate_PID, chassis->imu->YawTotalAngle * DEGREE_2_RAD,
                                        chassis_ctrl_cmd->target_yaw);
    yaw_prostrate_pid_output_lpf =
        FirstOrderLowPass(yaw_pid_output, yaw_prostrate_pid_output_lpf, chassis->yaw_prostrate_PID.dt, 0.02f);
    chassis->yaw_prostrate_PID.Output = yaw_prostrate_pid_output_lpf;
    wz_pid = -yaw_prostrate_pid_output_lpf;
  }

  vx_motor = chassis_ctrl_cmd->vx * VX_MPS_TO_MOTOR;
  float wz_motor = wz_pid * WZ_PID_TO_MOTOR + chassis_ctrl_cmd->wz * WZ_RADPS_TO_MOTOR;

  // 差速分配
  wheel_speed_ref[0] = -1.0f * (vx_motor - wz_motor);  // 右轮 leg[0]
  wheel_speed_ref[1] = -1.0f * (vx_motor + wz_motor);  // 左轮 leg[1]
  chassis->yaw_prostrate_PID.Last_Output = yaw_prostrate_pid_output_lpf;
}
/**
 * @brief 卧倒模式缩关节
 */
static void EnableJointMotor() {
  for (int i = 0; i < 2; i++) {
    leg[i]->update_flag.is_off_ground = 0;

    DMMotorOuterLoop(leg[i]->joint_motor[0], ANGLE_LOOP);
    DMMotorOuterLoop(leg[i]->joint_motor[1], ANGLE_LOOP);

    DMMotorSetPIDRef(leg[i]->joint_motor[0], -0.15f);
    DMMotorSetPIDRef(leg[i]->joint_motor[1], 0.15f);
  }
}

static void LimitChassisOutput(void) {
  // 平衡 → 卧倒 过渡期间仍走 LQR 力矩输出路径，不能切到卧倒的速度环 + 关节角度环
  if (chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_PROSTRATE && !descending_to_prostrate) {
    for (int i = 0; i < 2; i++) {
      VAL_LIMIT(wheel_speed_ref[i], -53000.0f, 53000.0f);
      DJIMotorSetPIDRef(leg[i]->wheel_motor, wheel_speed_ref[i]);
    }
    // 速度 PID 已生成 final_output, 此处再限流才能真正影响本帧 CAN 输出。
    PowerControl_Prostrate(chassis);
    EnableJointMotor();
  } else {
    for (int i = 0; i < 2; i++) {
      VAL_LIMIT(leg[i]->real_model.Tp_1, -28.0f, 28.0f);
      VAL_LIMIT(leg[i]->real_model.Tp_2, -28.0f, 28.0f);
      // VAL_LIMIT(leg[i]->real_model.T, -3.65f, 3.65f);// 限制额定扭矩
      VAL_LIMIT(leg[i]->real_model.T, -4.92f, 4.92f);  // 限制峰值扭矩
      // VAL_LIMIT(leg[i]->real_model.T, -4.2f, 4.2f);  // 限制峰值扭矩
      DMMotorSetRef(leg[i]->joint_motor[0], leg[i]->real_model.Tp_1);
      DMMotorSetRef(leg[i]->joint_motor[1], leg[i]->real_model.Tp_2);

      if (leg[i]->update_flag.is_off_ground) {
        DJIMotorSetRef(leg[i]->wheel_motor, 0);
      } else {
        DJIMotorSetRef(leg[i]->wheel_motor, leg[i]->real_model.T * (3591.0f / 187.0f) /
                                                chassis->leg[i]->param.wheel_reduction_ratio / 0.3f *
                                                (16384.0f / 20.0f));
      }
      // DJIMotorSetRef(leg[i]->wheel_motor, 0);
    }
  }
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  referee_data = GetReferee();

  chassis_instance->leg[1] = LegInit(&chassis_init_config->leg_init_config[1]);
  chassis_instance->leg[0] = LegInit(&chassis_init_config->leg_init_config[0]);

  // PIDInit(&chassis_instance->delta_theta_PID, &chassis_init_config->delta_theta_PID_config);
  PIDInit(&chassis_instance->roll_PID, &chassis_init_config->roll_PID_config);
  PIDInit(&chassis_instance->yaw_prostrate_PID, &chassis_init_config->yaw_prostrate_PID_config);

  chassis_instance->imu = INS_Init(&chassis_init_config->imu_init_config);

  xvEstimateKF_Init(&chassis_instance->vaEstimateKF);

  PowerControlInit(chassis_instance);

  chassis_instance->param = chassis_init_config->param;
  chassis_instance->chassis_ctrl_cmd.leg_length = chassis_init_config->param.initial_leg_length;

  chassis_instance->planner.vx_ramp = chassis_init_config->vx_ramp_config;
  chassis_instance->planner.wz_ramp = chassis_init_config->wz_ramp_config;

  chassis_instance->jump_state = JUMP_STATE_IDLE;

  chassis_instance->update_flag.is_first_update = 1;
  chassis_instance->update_flag.is_restart = 1;
  chassis_instance->update_flag.is_controlled = 0;
  // 双板时由云台板经 CAN 同步；单板没有独立对齐步骤，默认视为已对齐
#if defined(ONE_BOARD)
  chassis_instance->update_flag.gimbal_aligned = 1;
#else
  chassis_instance->update_flag.gimbal_aligned = 0;
#endif
  chassis = chassis_instance;

  leg[0] = chassis->leg[0];
  leg[1] = chassis->leg[1];

  chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;

  chassis->super_cap = SuperCapInit(&chassis_init_config->super_cap_config);

  DWT_GetDeltaT(&chassis->DWT_CNT);

  return chassis_instance;
}

static uint8_t AllMotorsOnline(void) {
  for (int i = 0; i < 2; i++) {
    if (!DaemonIsOnline(leg[i]->joint_motor[0]->daemon)) return 0;
    if (!DaemonIsOnline(leg[i]->joint_motor[1]->daemon)) return 0;
    if (!DaemonIsOnline(leg[i]->wheel_motor->daemon)) return 0;
  }
  return 1;
}

void ChassisTask(void) {
  if (!AllMotorsOnline()) {
    chassis->chassis_ctrl_cmd.chassis_mode = CHASSIS_POWER_OFF;
  }

  if (chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_POWER_OFF) {
    for (int i = 0; i < 2; i++) {
      DMMotorStop(chassis->leg[i]->joint_motor[0]);
      DMMotorStop(chassis->leg[i]->joint_motor[1]);
      DJIMotorStop(chassis->leg[i]->wheel_motor);
    }
    chassis->jump_state = JUMP_STATE_IDLE;
  } else {
    for (int i = 0; i < 2; i++) {
      DMMotorEnable(chassis->leg[i]->joint_motor[0]);
      DMMotorEnable(chassis->leg[i]->joint_motor[1]);
      DJIMotorEnable(chassis->leg[i]->wheel_motor);
    }
  }

  // 平衡工作态 → 卧倒 平滑过渡：若直接切到 ChassisProstrate 会因关节角度环目标突变
  // (-0.15/0.15) 而导致机身砸下；切入 PROSTRATE 时若腿仍然较长，则先用 LQR 把腿降到
  // leg_min_length，再放行进入真正的卧倒控制。基于实测腿长触发，避免依赖 CAN 时序观察
  // 到 CHASSIS_ON 中间态。POWER_OFF/RECOVERY 直接进 PROSTRATE，不借一次平衡态。

  static Chassis_Mode_e last_chassis_mode = CHASSIS_POWER_OFF;
  Chassis_Mode_e cur_mode = chassis->chassis_ctrl_cmd.chassis_mode;
  if (cur_mode != last_chassis_mode) {
    if (cur_mode == CHASSIS_PROSTRATE) {
      ResetProstrateMemory();
    } else if (last_chassis_mode == CHASSIS_PROSTRATE) {
      ResetChassisBalanceMemory();
      chassis_ctrl_cmd->vx = 0.0f;
      chassis_ctrl_cmd->wz = 0.0f;
      chassis_ctrl_cmd->theta_ff = 0.0f;
      chassis_ctrl_cmd->target_yaw = chassis->imu->YawTotalAngle * DEGREE_2_RAD;
    } else if (last_chassis_mode == CHASSIS_POWER_OFF && cur_mode != CHASSIS_POWER_OFF) {
      // POWER_OFF → balance: 必须把 planner.target_yaw 对齐当前 imu yaw,
      // 否则 LQR state_err[2] = phi - 0 (zmalloc 初值) 暴增 → 输出饱和, 表现为底盘失控.
      ResetChassisBalanceMemory();
    }

    if (cur_mode == CHASSIS_POWER_OFF) {
      ResetChassisBalanceMemory();
      ResetProstrateMemory();
      chassis->jump_state = JUMP_STATE_IDLE;
      descending_to_prostrate = 0;
    }
  }

  uint8_t should_descend_to_prostrate = last_chassis_mode == CHASSIS_ON || last_chassis_mode == CHASSIS_STAIR ||
                                        last_chassis_mode == CHASSIS_JUMP_READY ||
                                        last_chassis_mode == CHASSIS_JUMP_START;
  if (should_descend_to_prostrate && cur_mode == CHASSIS_PROSTRATE) {
    float l_avg = (leg[0]->virtual_model.length + leg[1]->virtual_model.length) * 0.5f;
    if (l_avg > chassis->param.leg_min_length + 0.05f) {
      descending_to_prostrate = 1;
    }
  }
  if (cur_mode != CHASSIS_PROSTRATE) {
    descending_to_prostrate = 0;
  }
  last_chassis_mode = cur_mode;

  if (descending_to_prostrate) {
    // 强制最低腿长目标，沿用 LQR 平衡，机身平稳下沉
    chassis_ctrl_cmd->leg_length = chassis->param.leg_min_length;
    ChassisCtrlUpdate();
    chassis->jump_state = JUMP_STATE_IDLE;
    if (fabsf(leg[0]->virtual_model.length - chassis->param.leg_min_length) < 0.05f &&
        fabsf(leg[1]->virtual_model.length - chassis->param.leg_min_length) < 0.05f) {
      descending_to_prostrate = 0;
    }
  } else {
    switch (cur_mode) {
      case CHASSIS_RECOVERY:
        ChassisRecovery();
        chassis->jump_state = JUMP_STATE_IDLE;
        break;
      case CHASSIS_ON:
        ChassisCtrlUpdate();
        chassis->jump_state = JUMP_STATE_IDLE;
        break;
      case CHASSIS_STAIR:
        ChassisCtrlUpdate();
        chassis->jump_state = JUMP_STATE_IDLE;
        break;
      case CHASSIS_JUMP_READY:
        if (chassis->jump_state == JUMP_STATE_IDLE || chassis->jump_state == JUMP_STATE_COMPRESS) {
          chassis->jump_state = JUMP_STATE_COMPRESS;
        }
        ChassisJump();
        break;
      case CHASSIS_JUMP_START:
        if (chassis->jump_state == JUMP_STATE_COMPRESS) {
          if (fabsf(leg[0]->virtual_model.length - chassis->param.leg_min_length) <= 0.05f &&
              fabsf(leg[1]->virtual_model.length - chassis->param.leg_min_length) <= 0.05f) {
            chassis->jump_state = JUMP_STATE_EXTEND;
          }
        }
        ChassisJump();
        break;
      case CHASSIS_PROSTRATE:
        ChassisProstrate();
        break;
      default:
        break;
    }
  }

  chassis_ctrl_cmd->max_power =
      SuperCapModeControl(chassis->super_cap, referee_data->GameRobotState.chassis_power_limit);

  static float last_super_cap_send_time = 0.0f;
  float now_ms = DWT_GetTimeline_ms();
  if (now_ms - last_super_cap_send_time >= 10.0f) {
    last_super_cap_send_time = now_ms;
    SuperCapSendMessage(chassis->super_cap, (int16_t)(referee_data->GameRobotState.chassis_power_limit),
                        referee_data->PowerHeatData.buffer_energy,
                        referee_data->GameRobotState.power_management_chassis_output);
  }

  LimitChassisOutput();
}
