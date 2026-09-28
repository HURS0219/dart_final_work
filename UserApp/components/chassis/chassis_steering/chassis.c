//
// Created by yang6 on 2025/11/17.
//
/**
 * @file    chassis.h
 * @author  NeoZeng
 * @author  Annotation and Modification By SRM-Control 2026
 * @date    2025/10/10
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   Mecanum Chassis Module
 */
#include "chassis.h"
#include "robot_config.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "user_lib.h"
#define MIN_WITH_WHEEL_MAX_SLEW_RATE(var) ((var) < (WHEEL_MAX_SLEW_RATE) ? (var) : (WHEEL_MAX_SLEW_RATE))


static ChassisInstance* chassis;
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;  // 声明但不初始化
static Chassis_Param_s chassis_param;          // 声明为静态局部变量
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;  // 底盘速度解算后的临时输出,待进行限幅
static float vt[4]={0};//限幅后的数据
static float AS_VT[4]={0};
static float st_lf,st_rf,st_lb,st_rb;
// 添加变量来存储上一次的角度值
//static float last_st_lf = 0.0f, last_st_rf = 0.0f, last_st_lb = 0.0f, last_st_rb = 0.0f;
static float lf_radius;
static float rf_radius;
static float lb_radius;
static float rb_radius;
static PIDInstance follow_pid;
static float k0,k1,k2,k3,k4,k5;       //中科大的功率模型

/**
 * @brief 角度标准化
 */




// 函数用于将角度转换为最优角度，带方向控制（必要时反转）和转劣弧
static float AngleToOptimalAngle(float target_angle, float current_angle, int8_t* direction) {
    // 计算最短角度差
    float diff = target_angle - current_angle;
    
    // 将差值标准化到 [-180, 180]
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    
    // 最优角：必要时反转而不是打舵
    // 当误差大于90度或小于-90度时
    if (diff > 90.0f) {
        // 如果误差大于90度，则反转方向更有效
        *direction = -1;//这里没有用motor库里的反转标志
        // 通过180度调整目标（反向）
        return current_angle +(diff - 180.0f);
    } else if (diff < -90.0f) {
        // 如果误差小于-90度，则反转方向更有效
        *direction = -1;
        // 通过180度调整目标（反向）
        return current_angle + (diff + 180.0f);
    } else {
        // 正常情况，直接前往目标（最短路径）
        *direction = 1;
        return current_angle + diff;
    }
}
//6020安装角度补偿
static void RudderOffset() {
  st_lf-=(float)chassis->rudder_offset[LF]*ECD_ANGLE_COEF_DJI;
  st_rf-=(float)chassis->rudder_offset[RF]*ECD_ANGLE_COEF_DJI;
  st_lb-=(float)chassis->rudder_offset[LB]*ECD_ANGLE_COEF_DJI;
  st_rb-=(float)chassis->rudder_offset[RB]*ECD_ANGLE_COEF_DJI;
}
//轮限幅
static void WheelLimit() {
  uint8_t i =0;
  float temp, max_vector= 0;

  for (i = 0; i < 4; i++)
  {
    temp = fabsf(vt[i]);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }
  if (max_vector > MAX_WHEEL_SPEED)
  {
    float vector_rate = 0;
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      vt[i] *= vector_rate;
    }
  }
}
void AntiSpin() {
  float temp;
  for (int i = 0; i < 4; i++) {
    if (AS_VT[i] < vt[i] ) {

    }
  }
}

// 添加静态变量用于存储前一个角度以实现连续跟踪
static float last_st_lf = 0.0f, last_st_rf = 0.0f, last_st_lb = 0.0f, last_st_rb = 0.0f;
// 添加变量用于存储每个电机的方向标志
static int8_t dir_lf = 1, dir_rf = 1, dir_lb = 1, dir_rb = 1;

static void SteeringCalculate() {
  vt_lf = sqrtf(powf(chassis_vy - chassis_ctrl_cmd->wz * arm_cos_f32(DEG2R(45)), 2) + powf(chassis_vx + chassis_ctrl_cmd->wz * arm_sin_f32(DEG2R(45)), 2));//lf
  vt_lb = sqrtf(powf(chassis_vy + chassis_ctrl_cmd->wz * arm_cos_f32(DEG2R(45)), 2) + powf(chassis_vx + chassis_ctrl_cmd->wz * arm_sin_f32(DEG2R(45)), 2));// lb
  vt_rb = sqrtf(powf(chassis_vy + chassis_ctrl_cmd->wz * arm_cos_f32(DEG2R(45)), 2) + powf(chassis_vx - chassis_ctrl_cmd->wz * arm_sin_f32(DEG2R(45)), 2));// rb
  vt_rf = sqrtf(powf(chassis_vy - chassis_ctrl_cmd->wz * arm_cos_f32(DEG2R(45)), 2) + powf(chassis_vx - chassis_ctrl_cmd->wz * arm_sin_f32(DEG2R(45)), 2));// rf
  
  // 修改此处的角度计算，确保正确的方向
  st_lf=RAD_2_DEGREE * atan2f(chassis_vy - chassis_ctrl_cmd->wz * arm_cos_f32(DEG2R(45)), chassis_vx + chassis_ctrl_cmd->wz * arm_sin_f32(DEG2R(45)));
  st_lb=RAD_2_DEGREE * atan2f(chassis_vy + chassis_ctrl_cmd->wz * arm_cos_f32(DEG2R(45)), chassis_vx + chassis_ctrl_cmd->wz * arm_sin_f32(DEG2R(45)));
  st_rb=RAD_2_DEGREE * atan2f(chassis_vy + chassis_ctrl_cmd->wz * arm_cos_f32(DEG2R(45)), chassis_vx - chassis_ctrl_cmd->wz * arm_sin_f32(DEG2R(45)));
  st_rf=RAD_2_DEGREE * atan2f(chassis_vy - chassis_ctrl_cmd->wz * arm_cos_f32(DEG2R(45)), chassis_vx - chassis_ctrl_cmd->wz * arm_sin_f32(DEG2R(45)));
  
  RudderOffset();//补偿6020的偏置

  // 转换为带方向控制的最优角度
  st_lf = AngleToOptimalAngle(st_lf, last_st_lf, &dir_lf);
  st_rf = AngleToOptimalAngle(st_rf, last_st_rf, &dir_rf);
  st_lb = AngleToOptimalAngle(st_lb, last_st_lb, &dir_lb);
  st_rb = AngleToOptimalAngle(st_rb, last_st_rb, &dir_rb);
  
  // 将方向应用于速度命令
  vt[LF]=vt_lf*dir_lf;
  vt[RF]=vt_rf*dir_rf;
  vt[LB]=vt_lb*dir_lb;
  vt[RB]=vt_rb*dir_rb;
   
  WheelLimit();
  //AntiSpin();
  // 更新前一个角度
  last_st_lf = st_lf;
  last_st_rf = st_rf;
  last_st_lb = st_lb;
  last_st_rb = st_rb;
}
/**
 * @brief 功率模型
 * @todo 有待模块化,djimotor也得改改
 */
//  static void PowerControl() {
//    // 获取电机速度反馈,化成单位rad/s
//    float motor_speed_fdb[4];
//    for (int i = 0; i < 4; i++) {
//      motor_speed_fdb[i] = (float)chassis->wheel_motor[i]->measure.speed_aps / 6.f;
//    }
//
//   // 获取当前电机参考电流，统一位单位为A
//   float motor_current_list[4];
//   for (int i = 0; i < 4; i++) {
//     motor_current_list[i] = (float)chassis->wheel_motor[i]->motor_controller.final_output;
//   }
//
//   float initial_give_power[4] = {0.0f};  // 每个电机的初始估计功率
//   float initial_total_power = 0.0f;      // 估计初始总功率
//
//   // 计算每个电机的功率贡献
//   for (int i = 0; i < 4; i++) {
//     initial_give_power[i] =
//         k0 + k1 * motor_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
//         k3 * motor_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
//         k4 * motor_current_list[i] / (16384.0f / 20.0f) * motor_current_list[i] / (16384.0f / 20.0f) +
//         k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);
//
//     // 只累加正向功率
//     if (initial_give_power[i] > 0) {
//       initial_total_power += initial_give_power[i];
//     }
//   }
//   // 功率超限时进行动态调整
//   if (initial_total_power > (float)chassis_ctrl_cmd->max_power) {
//     float power_scale = (float)chassis_ctrl_cmd->max_power / initial_total_power;  // 削减功率比例
//     float scaled_give_power[4];
//     // 计算缩放后的功率目标
//     for (int i = 0; i < 4; i++) {
//       scaled_give_power[i] = initial_give_power[i] * power_scale;
//     }
//
//     // 重新计算每个电机的电流参考值
//     for (int i = 0; i < 4; i++) {
//       // 二次方程系数计算，参数
//       float a = k4 / (16384.0f / 20.0f) / (16384.0f / 20.0f);
//       float b = k1 / (16384.0f / 20.0f) + k3 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) / (16384.0f / 20.0f);
//       float c = k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
//                 k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) -
//                 scaled_give_power[i] + k0;
//       float discriminant = b * b - 4 * a * c;  // 判别式
//       if (discriminant >= 0) {
//         float sqrt_disc = sqrtf(discriminant);
//         float temp1 = (-b + sqrt_disc) / (2 * a);
//         float temp2 = (-b - sqrt_disc) / (2 * a);
//
//         // 选择最接近当前电流的解
//         if (motor_current_list[i] > 0) {
//           motor_current_list[i] = (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
//                                       ? fminf(16000.f, temp1)
//                                       : fminf(16000.f, temp2);
//         } else {
//           motor_current_list[i] = (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
//                                       ? fmaxf(-16000.f, temp1)
//                                       : fmaxf(-16000.f, temp2);
//         }
//       } else {
//         // 无解时归零
//         motor_current_list[i] = 0.0f;
//       }
//     }
//   }
//   for (int i = 0; i < 4; i++) {
//     chassis->wheel_motor[i]->motor_controller.final_output = (int16_t)(motor_current_list[i]);
//   }
// }

/**
 * @brief 预测电机功率并进行限制
 *
 */
static void LimitChassisOutput() {
  DJIMotorSetPIDRef(chassis->wheel_motor[LF], vt[LF]);
  DJIMotorSetPIDRef(chassis->wheel_motor[RF], vt[RF]);
  DJIMotorSetPIDRef(chassis->wheel_motor[LB], vt[LB]);
  DJIMotorSetPIDRef(chassis->wheel_motor[RB], vt[RB]);
  DJIMotorSetPIDRef(chassis->rudder_motor[LF], st_lf);
  DJIMotorSetPIDRef(chassis->rudder_motor[LB], st_lb);
  DJIMotorSetPIDRef(chassis->rudder_motor[RF], st_rf);
  DJIMotorSetPIDRef(chassis->rudder_motor[RB], st_rb);
  //PowerControl();
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed() {
  // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
  // chassis_feedback_data.vx vy wz =
  // DJIMotor得改otherfeed
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));


  chassis_param = chassis_init_config->chassis_param;  // 在运行时赋值


  float half_wheel_base = chassis_param.wheel_base / 2.0f;
  float half_track_width = chassis_param.track_width / 2.0f;
  float center_gimbal_offset_x = chassis_param.center_gimbal_offset_x;
  float center_gimbal_offset_y = chassis_param.center_gimbal_offset_y;
  k0 = chassis_param.power_param.k0;
  k1 = chassis_param.power_param.k1;
  k2 = chassis_param.power_param.k2;
  k3 = chassis_param.power_param.k3;
  k4 = chassis_param.power_param.k4;
  k5 = chassis_param.power_param.k5;

  lf_radius = sqrtf((half_track_width + center_gimbal_offset_x) * (half_track_width + center_gimbal_offset_x) +
                    (half_wheel_base - center_gimbal_offset_y) * (half_wheel_base - center_gimbal_offset_y)) *
              DEGREE_2_RAD;

  rf_radius = sqrtf((half_track_width - center_gimbal_offset_x) * (half_track_width - center_gimbal_offset_x) +
                    (half_wheel_base - center_gimbal_offset_y) * (half_wheel_base - center_gimbal_offset_y)) *
              DEGREE_2_RAD;

  lb_radius = sqrtf((half_track_width + center_gimbal_offset_x) * (half_track_width + center_gimbal_offset_x) +
                    (half_wheel_base + center_gimbal_offset_y) * (half_wheel_base + center_gimbal_offset_y)) *
              DEGREE_2_RAD;

  rb_radius = sqrtf((half_track_width - center_gimbal_offset_x) * (half_track_width - center_gimbal_offset_x) +
                    (half_wheel_base + center_gimbal_offset_y) * (half_wheel_base + center_gimbal_offset_y)) *
              DEGREE_2_RAD;
  PIDInit(&follow_pid,&chassis_init_config->follow_pid);
  for (int i = 0; i < 4; i++) {
    chassis_init_config->wheel_motor_config[i].controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    chassis_init_config->wheel_motor_config[i].controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    chassis_init_config->wheel_motor_config[i].controller_setting_init_config.outer_loop_type = SPEED_LOOP;
    chassis_init_config->wheel_motor_config[i].controller_setting_init_config.close_loop_type = SPEED_LOOP;
    chassis_instance->wheel_motor[i] = DJIMotorInit(&chassis_init_config->wheel_motor_config[i]);
  }
  for (int i = 0; i < 4; i++) {
    chassis_init_config->rudder_motor_config[i].controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    chassis_init_config->rudder_motor_config[i].controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    chassis_init_config->rudder_motor_config[i].controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    chassis_init_config->rudder_motor_config[i].controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
    chassis_instance->rudder_motor[i] = DJIMotorInit(&chassis_init_config->rudder_motor_config[i]);
    
    // // 如果是GM6020电机，则设置零位偏移,改这个没用
    // if (chassis_init_config->rudder_motor_config[i].motor_type == GM6020) {
    //   // 设置电机零位偏移值，用于校准安装后的零偏
    //   chassis_instance->rudder_offset[i] = chassis_param.rudder_motor_offset[i];
    // }
  }
  //chassis_instance->yaw_motor=DJIMotorInit(&chassis_init_config->yaw_motor_config);
  for (int i = 0; i < 4; i++) {
    chassis_instance->rudder_offset[i]=chassis_init_config->chassis_param.rudder_motor_offset[i];
  }
  chassis = chassis_instance;
  chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;  // 在运行时初始化指针
  return chassis_instance;
}

/* 机器人底盘控制核心任务 */
void ChassisTask() {
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF) {
    // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    for (int i = 0; i < 4; i++) DJIMotorStop(chassis->wheel_motor[i]);
    for (int i = 0; i < 4; i++) DJIMotorStop(chassis->rudder_motor[i]);
  } else {
    // 正常工作
    for (int i = 0; i < 4; i++) DJIMotorEnable(chassis->wheel_motor[i]);
    for (int i = 0; i < 4; i++) DJIMotorEnable(chassis->rudder_motor[i]);
  }
  // 根据控制模式设定旋转速度
  switch (chassis_ctrl_cmd->chassis_mode)
  {
    case CHASSIS_FOLLOW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
      chassis_ctrl_cmd->wz+=PIDCalculate(&follow_pid,chassis_ctrl_cmd->offset_angle,0);
      break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
       chassis_ctrl_cmd->wz = 30000;
      chassis_ctrl_cmd->offset_angle+=0.001f*chassis_ctrl_cmd->wz;
      break;
    default:
      break;
  }
  // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
  // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
  static float sin_theta, cos_theta;
  cos_theta = arm_cos_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
  sin_theta = arm_sin_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
  chassis_vx = chassis_ctrl_cmd->vx * cos_theta -chassis_ctrl_cmd->vy * sin_theta;
  chassis_vy = chassis_ctrl_cmd->vx * sin_theta + chassis_ctrl_cmd->vy * cos_theta;
  // 根据电机的反馈速度和IMU(如果有)计算真实速度
  EstimateSpeed();

  // 根据控制模式进行正运动学解算,计算底盘输出
  SteeringCalculate();

  // 功率控制与输出限幅
  LimitChassisOutput();
}