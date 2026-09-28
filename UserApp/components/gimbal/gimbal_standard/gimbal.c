/**
******************************************************************************
* @file    gimbal.h
* @author  NeoZeng
* @author  Annotation and Modification By Enhao Zhang
* @date    2025/10/10
* @copyright Copyright (c) SHU SRM 2025 all rights reserved
* @brief   Standard Gimbal Module
******************************************************************************
* @attention
******************************************************************************
*/

#include "gimbal.h"

#include "general_def.h"
#include "ins_task.h"
#include "user_lib.h"

static GimbalInstance* gimbal;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;  // 声明但不初始化
static float pitch_feedforward_scale;
static float last_yaw_cmd=0.0f;

/**
 * @brief pitch重力补偿前馈
 *        F = k * cos(pitch)
 *        pitch=0°水平时补偿最大，pitch=±90°时补偿为0
 */
static float GetPitchGravityFeedforward(void) {
  float pitch_rad = gimbal->gimbal_IMU_data->Pitch * DEGREE_2_RAD;
  return pitch_feedforward_scale * arm_cos_f32(pitch_rad);
}

// static BMI088Instance *bmi088; // 云台IMU
GimbalInstance* GimbalInit(Gimbal_Init_Config_s* gimbal_init_config) {
  GimbalInstance* gimbal_instance = (GimbalInstance*)zmalloc(sizeof(GimbalInstance));
  gimbal_instance->gimbal_IMU_data =
      INS_Init(&gimbal_init_config->imu_init_config);  // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

  // pitch重力补偿前馈
  pitch_feedforward_scale = gimbal_init_config->pitch_feedforward_scale;
  // YAW控制器参数配置
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->YawTotalAngle;
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[2];

  // YAW控制器设置配置
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  // PITCH控制器参数配置
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Pitch;
  // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[0];

  // PITCH控制器设置配置
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  gimbal = gimbal_instance;
  gimbal_ctrl_cmd = &gimbal->gimbal_ctrl_cmd;  // 在运行时初始化指针
  gimbal_init_config->yaw_motor_config.controller_param_init_config.speed_feedforward_ptr=&gimbal_ctrl_cmd->chassis_rotate_wz;
  gimbal_instance->yaw_motor = DJIMotorInit(&gimbal_init_config->yaw_motor_config);
  gimbal_instance->pitch_motor = DJIMotorInit(&gimbal_init_config->pitch_motor_config);
  return gimbal_instance;
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask() {
  // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_POWER_OFF) {
    // 停止
    DJIMotorStop(gimbal->yaw_motor);
    DJIMotorStop(gimbal->pitch_motor);
    gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
    last_yaw_cmd = gimbal_ctrl_cmd->yaw;
  } else {
    DJIMotorEnable(gimbal->yaw_motor);
    DJIMotorEnable(gimbal->pitch_motor);
    //pid调参测试用
    //gimbal_ctrl_cmd->yaw=40*sin(DWT_GetTimeline_s()*2.5f);
    gimbal_ctrl_cmd->yaw = last_yaw_cmd + wrap180(gimbal_ctrl_cmd->yaw - last_yaw_cmd);
    DJIMotorSetPIDRef(gimbal->yaw_motor, gimbal_ctrl_cmd->yaw);  // yaw和pitch会在robot_cmd中处理好多圈和单圈
    DJIMotorSetPIDRef(gimbal->pitch_motor, gimbal_ctrl_cmd->pitch);
    // gimbal->pitch_motor->motor_controller.final_output += GetPitchGravityFeedforward();  // pitch重力补偿前馈
    last_yaw_cmd = gimbal_ctrl_cmd->yaw;
  }
}
