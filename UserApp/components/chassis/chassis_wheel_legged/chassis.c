/**
 * @file    chassis.h
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   Parallel Wheel-Legged Chassis Module
 */
#include "chassis.h"

#include <math.h>

#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "robot_config.h"
#include "speed_observer.h"
#include "user_lib.h"

static ChassisInstance* chassis;
static LegInstance* leg[2];
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;  // 声明但不初始化

// 中间变量
static float chassis_aver_v;
static float q2i_coeff;

// robot param
static float robot_mass;
static float track_width;
static float leg_force_ff;
static float leg_force_ff_gain;
static float wheel_radius;
static float wheel_reduction_ratio;

// 中科大的功率模型
static float k0, k1, k2, k3, k4, k5;

static void ChassisCtrlUpdate() {
  chassis->roll_comp =
      PIDCalculate(&chassis->roll_PID, chassis->chassis_IMU->Roll * DEGREE_2_RAD, chassis->chassis_ctrl_cmd.roll);

  for (int i = 0; i < 2; i++) {
    leg[i]->update_flag.is_controlled = chassis->chassis_ctrl_cmd.vx != 0;
    leg[i]->leg_ctrl_cmd.x_d_ref = chassis->chassis_ctrl_cmd.vx;
    leg[i]->leg_ctrl_cmd.theta_ref = chassis->chassis_ctrl_cmd.theta_ff;
    if (chassis->jump_state == JUMP_STATE_IDLE) {
      leg[i]->leg_ctrl_cmd.length_ref =
          chassis->chassis_ctrl_cmd.leg_length -
          (float)(1 - 2 * i) * track_width *
              (chassis->chassis_ctrl_cmd.roll - chassis->chassis_IMU->Roll * DEGREE_2_RAD);
    } else {
      leg[i]->leg_ctrl_cmd.length_ref =
          leg[i]->leg_ctrl_cmd.length_ref -
          (float)(1 - 2 * i) * track_width *
              (chassis->chassis_ctrl_cmd.roll - chassis->chassis_IMU->Roll * DEGREE_2_RAD);
    }
    LegCtrlUpdate(leg[i], chassis->chassis_IMU);
    if (chassis->jump_state == JUMP_STATE_EXTEND) {
      leg_force_ff = chassis_ctrl_cmd->jump_force;
    } else {
      leg_force_ff = leg_force_ff_gain * 9.8f * robot_mass / 2.0f /
                     mcos(leg[i]->state_var.theta);  // 不超过半边重力的一半(看机器）
    }
    leg[i]->virtual_model.F += leg_force_ff - (float)(1 - 2 * i) * chassis->roll_comp;
    VAL_LIMIT(leg[i]->virtual_model.F, -1500.0f, 1500.0f);
    leg[i]->real_model.T -= (float)(1 - 2 * i) * chassis->chassis_ctrl_cmd.wz;
  }

  chassis->delta_theta_comp =
      PIDCalculate(&chassis->delta_theta_PID, leg[0]->state_var.theta - leg[1]->state_var.theta, 0);
  for (int i = 0; i < 2; i++) {
    leg[i]->virtual_model.Tp -= (float)(1 - 2 * i) * chassis->delta_theta_comp;
    JointTorqueUpdate(leg[i]);
  }
}

/**
 * @brief 倒地自启
 * @todo 没测过
 */
static void ChassisRecovery() {
  for (int i = 0; i < 2; i++) {
    // 关节收腿
    DMMotorOuterLoop(leg[i]->joint_motor[0], ANGLE_LOOP);
    DMMotorOuterLoop(leg[i]->joint_motor[1], ANGLE_LOOP);
    DMMotorSetPIDRef(leg[i]->joint_motor[0], -0.1);
    DMMotorSetPIDRef(leg[i]->joint_motor[1], 0.1);
    leg[i]->real_model.Tp_1 = leg[i]->joint_motor[0]->motor_controller.final_output;
    leg[i]->real_model.Tp_2 = leg[i]->joint_motor[1]->motor_controller.final_output;
    leg[i]->update_flag.is_controlled = 0;  // todo: 可以看看哪个好
    // 没收完两条腿不动轮毂, 防止位移项错误累加
    if (abs((leg[0]->joint_motor[0]->measure.position - (-0.1f))) <= 0.5f &&
        abs(leg[0]->joint_motor[1]->measure.position - (0.1f)) <= 0.5f &&
        abs((leg[1]->joint_motor[0]->measure.position - (-0.1f))) <= 0.5f &&
        abs(leg[1]->joint_motor[1]->measure.position - (0.1f)) <= 0.5f) {
      leg[i]->leg_ctrl_cmd.x_d_ref = chassis->chassis_ctrl_cmd.vx;
      LegCtrlUpdate(leg[i], chassis->chassis_IMU);
      leg[i]->real_model.T -= (float)(1 - 2 * i) * chassis->chassis_ctrl_cmd.wz;
    } else {
      leg[i]->real_model.T = 0;
    }
    // 位置环下离地检测有误，需手动将标志位置零
    leg[i]->update_flag.is_off_ground = 0;
  }
}

static void ChassisJump() {
  switch (chassis->jump_state) {
    case JUMP_STATE_COMPRESS:
      for (int i = 0; i < 2; i++) {
        leg[i]->leg_ctrl_cmd.length_ref = LEG_MIN_LENGTH;
      }
      ChassisCtrlUpdate();
      break;
    case JUMP_STATE_EXTEND:
      for (int i = 0; i < 2; i++) {
        leg[i]->leg_ctrl_cmd.length_ref = LEG_MAX_LENGTH;
      }
      ChassisCtrlUpdate();
      if (fabs(leg[0]->virtual_model.length - LEG_MAX_LENGTH) <= 0.05 &&
          fabs(leg[1]->virtual_model.length - LEG_MAX_LENGTH) <= 0.05) {
        chassis->jump_state = JUMP_STATE_RETRACT;
      }
      break;
    case JUMP_STATE_RETRACT:
      for (int i = 0; i < 2; i++) {
        leg[i]->leg_ctrl_cmd.length_ref = LEG_MIN_LENGTH;
      }
      ChassisCtrlUpdate();
      if (fabs(leg[0]->virtual_model.length - LEG_MIN_LENGTH) <= 0.05 &&
          fabs(leg[1]->virtual_model.length - LEG_MIN_LENGTH) <= 0.05) {
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
 * @brief 功率控制
 * @todo 有待模块化,djimotor也得改改
 */
static void PowerControl() {
  // 获取电机角速度
  float wheel_motor_speed_fdb[2];
  for (int i = 0; i < 2; i++) {
    wheel_motor_speed_fdb[i] = leg[i]->wheel_motor->measure.speed_aps;
    wheel_motor_speed_fdb[i] *= DEGREE_2_RAD;
  }

  // 获取电机电流
  float wheel_motor_current_fdb[2];
  for (int i = 0; i < 2; i++) {
    wheel_motor_current_fdb[i] = leg[i]->wheel_motor->motor_controller.final_output;
  }

  // 获取电机各力矩,之后转换为各分量实际电流值
  float M[2];        // 轮毂力矩的平衡分量,之后转换为实际电流值
  float T_speed[2];  // 轮毂力矩的速度分量,之后转换为实际电流值
  float T_yaw[2];    // 轮毂力矩的yaw分量,之后转换为实际电流值
  for (int i = 0; i < 2; i++) {
    M[i] = leg[i]->LQR_K[0][0] * (leg[i]->state_var.theta - 0.0f) +
           leg[i]->LQR_K[0][1] * (leg[i]->state_var.theta_d - 0.0f) +
           leg[i]->LQR_K[0][4] * (leg[i]->state_var.phi - 0.0f) +
           leg[i]->LQR_K[0][5] * (leg[i]->state_var.phi_d - 0.0f);
    T_speed[i] =
        !leg[i]->update_flag.is_controlled * leg[i]->LQR_K[0][2] * (leg[i]->state_var.x - leg[i]->leg_ctrl_cmd.x_ref) +
        leg[i]->LQR_K[0][3] * (leg[i]->state_var.x_d - leg[i]->leg_ctrl_cmd.x_d_ref);
    T_yaw[i] = (float)(2 * i - 1) * chassis->chassis_ctrl_cmd.wz;

    // 将力矩转化为实际电流(A)
    M[i] *= q2i_coeff;
    T_speed[i] *= q2i_coeff;
    T_yaw[i] *= q2i_coeff;
  }

  float initial_give_power[2] = {0.0f};  // 每个电机的初始估计功率
  float initial_total_power = 0.0f;      // 估计初始总功率

  // 计算电机当前功率
  for (int i = 0; i < 2; i++) {
    initial_give_power[i] =
        k0 + k1 * wheel_motor_current_fdb[i] / (16384.0f / 20.0f) + k2 * wheel_motor_speed_fdb[i] +
        k3 * wheel_motor_current_fdb[i] / (16384.0f / 20.0f) * wheel_motor_speed_fdb[i] +
        k4 * wheel_motor_current_fdb[i] / (16384.0f / 20.0f) * wheel_motor_current_fdb[i] / (16384.0f / 20.0f) +
        k5 * wheel_motor_speed_fdb[i] * wheel_motor_speed_fdb[i];
    // 只累加正向功率
    if (initial_give_power[i] > 0) {
      initial_total_power += initial_give_power[i];
    }
  }

  float k[2] = {0.0f};        // 约束条件：T_speed = k * T_yaw 和 T_yaw_target = k * T_speed_target
  float k_speed[2] = {1.0f};  // speed分量的衰减系数
  float k_yaw[2] = {1.0f};    // yaw分量的衰减系数
  float T_yaw_target[2] = {0.0f};

  // 功率超限时进行动态调整
  if (initial_total_power > (float)chassis_ctrl_cmd->max_power) {
    for (int i = 0; i < 2; i++) {
      k[i] = T_speed[i] / T_yaw[i];

      // 以T_yaw_target为x，解方程，x单位是A
      float a = k4 * (k[i] + 1) * (k[i] + 1);
      float b = (k1 + k3 * M[i] + 2 * k4 * M[i]) * (k[i] + 1);
      float c = k0 + k1 * M[i] + k2 * wheel_motor_speed_fdb[i] + k3 * M[i] * wheel_motor_speed_fdb[i] +
                k4 * M[i] * M[i] + k5 * wheel_motor_speed_fdb[i] * wheel_motor_speed_fdb[i] -
                (float)chassis_ctrl_cmd->max_power;
      float discriminant = b * b - 4 * a * c;  // 判别式
      if (discriminant > 0) {
        float sqrt_disc = sqrtf(discriminant);
        float x1 = (-b + sqrt_disc) / (2 * a);
        float x2 = (-b - sqrt_disc) / (2 * a);
        if (T_yaw_target[i] > 0) {
          T_yaw_target[i] =
              (fabsf(x1 - T_yaw_target[i]) < fabsf(x2 - T_yaw_target[i])) ? fminf(10.0f, x1) : fminf(10.0f, x2);
        } else {
          T_yaw_target[i] =
              (fabsf(x1 - T_yaw_target[i]) < fabsf(x2 - T_yaw_target[i])) ? fmaxf(-10.0f, x1) : fmaxf(-10.0f, x2);
        }

        k_yaw[i] = T_yaw_target[i] / T_yaw[i];
        k_speed[i] = k_yaw[i];
      } else if (discriminant == 0) {
        T_yaw_target[i] = (-b) / (2 * a);

        k_yaw[i] = T_yaw_target[i] / T_yaw[i];
        k_speed[i] = k_yaw[i];
      } else {
        k_speed[i] = 0.0f;
        k_yaw[i] = 0.0f;
      }

      // 限制衰减系数在0~1内
      VAL_LIMIT(k_speed[i], 0.0f, 1.0f);
      VAL_LIMIT(k_yaw[i], 0.0f, 1.0f);

      leg[i]->real_model.T = M[i] + k_speed[i] * T_speed[i] + k_yaw[i] * T_yaw[i];
    }
  }
}

/**
 * @brief 预测电机功率并进行限制
 *
 */
static void LimitChassisOutput() {
  // PowerControl();
  for (int i = 0; i < 2; i++) {
    VAL_LIMIT(leg[i]->real_model.Tp_1, -20.0f, 20.0f);
    VAL_LIMIT(leg[i]->real_model.Tp_2, -20.0f, 20.0f);
    // VAL_LIMIT(leg[i]->real_model.Tp_1, -3.0f, 3.0f);
    // VAL_LIMIT(leg[i]->real_model.Tp_2, -3.0f, 3.0f);
    VAL_LIMIT(leg[i]->real_model.T, -2.45f, 2.45f);
    DMMotorSetRef(leg[i]->joint_motor[0], leg[i]->real_model.Tp_1);
    DMMotorSetRef(leg[i]->joint_motor[1], leg[i]->real_model.Tp_2);
    // DMMotorSetRef(leg[i]->joint_motor[1], 0);
    // DMMotorSetRef(leg[i]->joint_motor[0], 0);
    // DMMotorSetRef(leg[1]->joint_motor[0], leg[1]->real_model.Tp_1);
    // DMMotorSetRef(leg[1]->joint_motor[1], leg[1]->real_model.Tp_2);
    if (leg[i]->update_flag.is_off_ground) {
      DJIMotorSetRef(leg[i]->wheel_motor, 0);
    } else {
      DJIMotorSetRef(leg[i]->wheel_motor, leg[i]->real_model.T * q2i_coeff * (16384.0f / 20.0f));
      // DJIMotorSetRef(leg[i]->wheel_motor, 0);
    }
    // DJIMotorSetRef(leg[0]->wheel_motor, 0);
    // DJIMotorSetRef(leg[i]->wheel_motor, ref);
  }
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed() {
#if 1
  ObserverVarUpdate(leg[0], chassis->chassis_IMU);
  ObserverVarUpdate(leg[1], chassis->chassis_IMU);
  chassis_aver_v = (leg[0]->observer_var.vb + leg[1]->observer_var.vb) / 2.0f;
  xvEstimateKF_Update(&chassis->vaEstimateKF, chassis->chassis_IMU->MotionAccel_n[1], chassis_aver_v);
  for (int i = 0; i < 2; i++) {
    leg[i]->state_var.x_d = chassis->vaEstimateKF.FilteredValue[0];
  }
#else
#define X_D_FILTER_ALPHA 0.1f
  for (int i = 0; i < 2; i++) {
    // 计算当前速度值
    float current_x_d = -1 * (leg[0]->wheel_motor->measure.speed_aps + leg[1]->wheel_motor->measure.speed_aps) / 2 /
                        wheel_reduction_ratio * DEGREE_2_RAD * wheel_radius;
    // 应用一阶低通滤波
    leg[i]->state_var.x_d = leg[i]->state_var.x_d * (1.0f - X_D_FILTER_ALPHA) + current_x_d * X_D_FILTER_ALPHA;
  }
#endif
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  chassis_instance->leg[0] = LegInit(&chassis_init_config->leg_init_config[0]);
  chassis_instance->leg[1] = LegInit(&chassis_init_config->leg_init_config[1]);

  robot_mass = chassis_init_config->chassis_param.robot_mass;
  track_width = chassis_init_config->chassis_param.track_width;
  leg_force_ff_gain = chassis_init_config->chassis_param.leg_force_ff_gain;
  wheel_radius = chassis_init_config->leg_init_config[0].param.wheel_radius;
  wheel_reduction_ratio = chassis_init_config->leg_init_config[0].param.wheel_reduction_ratio;
  q2i_coeff = (3591.0f / 187.0f) / wheel_reduction_ratio / 0.3f;

  k0 = chassis_init_config->chassis_param.power_param_3508.k0;
  k1 = chassis_init_config->chassis_param.power_param_3508.k1;
  k2 = chassis_init_config->chassis_param.power_param_3508.k2;
  k3 = chassis_init_config->chassis_param.power_param_3508.k3;
  k4 = chassis_init_config->chassis_param.power_param_3508.k4;
  k5 = chassis_init_config->chassis_param.power_param_3508.k5;

  PIDInit(&chassis_instance->delta_theta_PID, &chassis_init_config->delta_theta_PID_config);
  PIDInit(&chassis_instance->roll_PID, &chassis_init_config->roll_PID_config);

  chassis_instance->chassis_IMU = INS_Init(&chassis_init_config->imu_init_config);

  xvEstimateKF_Init(&chassis_instance->vaEstimateKF);

  chassis_instance->jump_state = JUMP_STATE_IDLE;

  chassis = chassis_instance;
  leg[0] = chassis->leg[0];
  leg[1] = chassis->leg[1];
  chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;  // 在运行时初始化指针
  return chassis_instance;
}

/* 机器人底盘控制核心任务 */
void ChassisTask() {
  if (chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_POWER_OFF) {
    // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    for (int i = 0; i < 2; i++) {
      DMMotorStop(chassis->leg[i]->joint_motor[0]);
      DMMotorStop(chassis->leg[i]->joint_motor[1]);
      DJIMotorStop(chassis->leg[i]->wheel_motor);
    }
    chassis->jump_state = JUMP_STATE_IDLE;
  } else {
    // 正常工作
    for (int i = 0; i < 2; i++) {
      DMMotorEnable(chassis->leg[i]->joint_motor[0]);
      DMMotorEnable(chassis->leg[i]->joint_motor[1]);
      DJIMotorEnable(chassis->leg[i]->wheel_motor);
    }
  }

  // 根据电机的反馈速度和IMU(如果有)计算真实速度
  EstimateSpeed();

  switch (chassis->chassis_ctrl_cmd.chassis_mode) {
    case CHASSIS_RECOVERY:
      ChassisRecovery();
      chassis->jump_state = JUMP_STATE_IDLE;
      break;
    case CHASSIS_ON:
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
        if (fabs(leg[0]->virtual_model.length - LEG_MIN_LENGTH) <= 0.02 &&
            fabs(leg[1]->virtual_model.length - LEG_MIN_LENGTH) <= 0.02) {
          chassis->jump_state = JUMP_STATE_EXTEND;
        }
      }
      ChassisJump();
      break;
    default:
      break;
  }

  // 功率控制与输出限幅
  LimitChassisOutput();
}