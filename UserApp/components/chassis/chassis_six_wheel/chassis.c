/**
 * @file    chassis.h
 * @author  Shiyu Li
 * @date    2026/3/28
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   wheel_leg->prostrate
 * @note    wheel_motor[0]————>右轮
 *          wheel_motor[1]————>左轮
 */
#include "chassis.h"

#include <math.h>
#include <string.h>

#include "general_def.h"
#include "referee.h"
#include "super_cap.h"
#include "user_lib.h"

static ChassisInstance* chassis;
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;
static referee_info_t* referee_data;

static float wheel_speed_ref[2];
static float power_control_k_coff = 1.0f;
static float k0 = 0.7441993412640775f, k1 = 0.0090164284468539646f, k2 = 0.0001988857226262331f,
             k3 = 0.024694430204543864f, k4 = 0.20160143850678086f, k5 = 3.715221772539512e-05f;  // 中科大的功率模型

/**
 * @brief 功率模型
 * @todo 有待模块化,djimotor也得改改
 */
static void PowerControl() {
  // 获取电机速度反馈,化成单位rad/s
  k0 *= power_control_k_coff;
  k1 *= power_control_k_coff;
  k2 *= power_control_k_coff;
  k3 *= power_control_k_coff;
  k4 *= power_control_k_coff;
  k5 *= power_control_k_coff;
  float motor_speed_fdb[2];
  for (int i = 0; i < 2; i++) {
    motor_speed_fdb[i] = (float)chassis->wheel_motor[i]->measure.speed_aps / 6.f;
  }

  // 获取当前电机参考电流，统一位单位为A
  float motor_current_list[2];
  for (int i = 0; i < 2; i++) {
    motor_current_list[i] = (float)chassis->wheel_motor[i]->motor_controller.final_output;
  }

  float initial_give_power[2] = {0.0f};  // 每个电机的初始估计功率
  float initial_total_power = 0.0f;      // 估计初始总功率

  // 计算每个电机的功率贡献
  for (int i = 0; i < 2; i++) {
    initial_give_power[i] =
        k0 + k1 * motor_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
        k3 * motor_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
        k4 * motor_current_list[i] / (16384.0f / 20.0f) * motor_current_list[i] / (16384.0f / 20.0f) +
        k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);

    // 只累加正向功率
    if (initial_give_power[i] > 0) {
      initial_total_power += initial_give_power[i];
    }
  }
  // 功率超限时进行动态调整
  if (initial_total_power > (float)chassis_ctrl_cmd->max_power) {
    float power_scale = (float)chassis_ctrl_cmd->max_power / initial_total_power;  // 削减功率比例
    float scaled_give_power[2];
    // 计算缩放后的功率目标
    for (int i = 0; i < 2; i++) {
      scaled_give_power[i] = initial_give_power[i] * power_scale;
    }

    // 重新计算每个电机的电流参考值
    for (int i = 0; i < 2; i++) {
      // 二次方程系数计算，参数
      float a = k4 / (16384.0f / 20.0f) / (16384.0f / 20.0f);
      float b = k1 / (16384.0f / 20.0f) + k3 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) / (16384.0f / 20.0f);
      float c = k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
                k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) -
                scaled_give_power[i] + k0;
      float discriminant = b * b - 4 * a * c;  // 判别式
      if (discriminant >= 0) {
        float sqrt_disc = sqrtf(discriminant);
        float temp1 = (-b + sqrt_disc) / (2 * a);
        float temp2 = (-b - sqrt_disc) / (2 * a);

        // 选择最接近当前电流的解
        if (motor_current_list[i] > 0) {
          motor_current_list[i] = (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                                      ? fminf(16000.f, temp1)
                                      : fminf(16000.f, temp2);
        } else {
          motor_current_list[i] = (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                                      ? fmaxf(-16000.f, temp1)
                                      : fmaxf(-16000.f, temp2);
        }
      } else {
        // 无解时归零
        motor_current_list[i] = 0.0f;
      }
    }
  }
  for (int i = 0; i < 2; i++) {
    chassis->wheel_motor[i]->motor_controller.final_output = (int16_t)(motor_current_list[i]);
  }
}

/**
 * @brief 卧倒模式
 *
 * 差速：右轮 = vx + wz，左轮 = vx - wz
 */
void ChassisProstrateMode(void) {
#define VX_TO_MOTOR (30000.0f / 660.0f)
#define WZ_PID_TO_MOTOR 10000.0f
#define WZ_FF_TO_MOTOR (28000.0f / 660.0f)  // wz 前馈(摇杆量级) → 电机量
  float vx_motor = 0.0f;
  float wz_motor = 0.0f;
  if (chassis_ctrl_cmd->is_rotate == 0) {
    float wz_pid = -PIDCalculate(&chassis->yaw_prostrate_PID, chassis->imu->YawTotalAngle * DEGREE_2_RAD,
                                 chassis_ctrl_cmd->target_yaw);
    vx_motor = chassis_ctrl_cmd->vx * VX_TO_MOTOR;
    wz_motor = wz_pid * WZ_PID_TO_MOTOR + chassis_ctrl_cmd->wz * WZ_FF_TO_MOTOR;
  } else if (chassis_ctrl_cmd->is_rotate == 1) {
    vx_motor = chassis_ctrl_cmd->vx * VX_TO_MOTOR;
    wz_motor = chassis_ctrl_cmd->wz * WZ_FF_TO_MOTOR;
  }
  // 差速分配
  wheel_speed_ref[0] = -1.0f * (vx_motor - wz_motor);  // 右轮 leg[0]
  wheel_speed_ref[1] = vx_motor + wz_motor;            // 左轮 leg[1]
}

static void EnableJointMotor() {
  for (int i = 0; i < 4; i++) DMMotorOuterLoop(chassis->joint_motor[i], ANGLE_LOOP);

  DMMotorSetPIDRef(chassis->joint_motor[0], -0.125f);
  DMMotorSetPIDRef(chassis->joint_motor[1], 0.125f);
  DMMotorSetPIDRef(chassis->joint_motor[2], -0.125f);
  DMMotorSetPIDRef(chassis->joint_motor[3], 0.125f);
}

/**
 * @brief  最终输出
 * @note   leg[0]->joint_motor[0] ————> joint_motor[0]
 *         leg[0]->joint_motor[1] ————> joint_motor[1]
 *         leg[1]->joint_motor[0] ————> joint_motor[2]
 *         leg[1]->joint_motor[1] ————> joint_motor[3]
 *
 */
static void LimitChassisOutput(void) {
  for (int i = 0; i < 2; i++) {
    VAL_LIMIT(wheel_speed_ref[i], -53000.0f, 53000.0f);
    DJIMotorSetPIDRef(chassis->wheel_motor[i], wheel_speed_ref[i]);
  }
  EnableJointMotor();
  PowerControl();
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  referee_data = GetReferee();

  for (int i = 0; i < 4; i++)
    chassis_instance->joint_motor[i] = DMMotorInit(&chassis_init_config->joint_motor_config[i]);
  for (int i = 0; i < 2; i++)
    chassis_instance->wheel_motor[i] = DJIMotorInit(&chassis_init_config->wheel_motor_config[i]);
  PIDInit(&chassis_instance->yaw_prostrate_PID, &chassis_init_config->yaw_prostrate_PID_config);

  chassis_instance->imu = INS_Init(&chassis_init_config->imu_init_config);

  chassis_instance->param = chassis_init_config->param;
  chassis = chassis_instance;
  chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;

  chassis->super_cap = SuperCapInit(&chassis_init_config->super_cap_config);

  return chassis_instance;
}

void ChassisTask(void) {
  if (chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_POWER_OFF) {
    for (int i = 0; i < 4; i++) DMMotorStop(chassis->joint_motor[i]);
    for (int i = 0; i < 2; i++) DJIMotorStop(chassis->wheel_motor[i]);
  } else {
    for (int i = 0; i < 4; i++) DMMotorEnable(chassis->joint_motor[i]);
    for (int i = 0; i < 2; i++) DJIMotorEnable(chassis->wheel_motor[i]);
  }
  switch (chassis->chassis_ctrl_cmd.chassis_mode) {
    case CHASSIS_PROSTRATE:
      ChassisProstrateMode();
      break;
    default:
      break;
  }

  chassis_ctrl_cmd->max_power =
      SuperCapModeControl(chassis->super_cap, referee_data->GameRobotState.chassis_power_limit);
  SuperCapSendMessage(chassis->super_cap, (int16_t)referee_data->GameRobotState.chassis_power_limit * (13.0f/14.0f),
                      referee_data->PowerHeatData.buffer_energy,
                      referee_data->GameRobotState.power_management_chassis_output);
  LimitChassisOutput();
}