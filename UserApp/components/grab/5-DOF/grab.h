#pragma once
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"

typedef enum
{
    GRAB_POWER_OFF = 0, // 电流零输入
    GRAB_POWER_ON
} Grab_Mode_e;

typedef enum
{
    GRAB_PRE_CALI_MODE = 0,
    GRAB_CALI_MODE,
} Grab_Cali_Mode_e;

typedef struct {
    float wrist_roll_MAX;// 腕部关节旋转角度
    float wrist_roll_MIN;
    float wrist_pitch_MAX;   // 腕部关节俯仰角度
    float wrist_pitch_MIN;
    float base_joint_MAX;    // 基座旋转关节角度
    float base_joint_MIN;
    float elbow_roll_MAX;   // 肘部关节旋转角度
    float elbow_roll_MIN;
    float elbow_pitch_MAX;   // 肘部关节俯仰角度
    float elbow_pitch_MIN;
    float vedio_forward_MAX;// 图传的前后移动距离
    float vedio_forward_MIN;
    float vedio_pitch_MAX;   // 图传的pitch旋转角度
    float vedio_pitch_MIN;

    float base_joint_sens_keyboard;   // 基座旋转关节灵敏度(键鼠)
    float elbow_roll_sens_keyboard;   // 肘部旋转关节灵敏度(键鼠)
    float elbow_pitch_sens_keyboard;  // 肘部俯仰关节灵敏度(键鼠)
    float wrist_roll_sens_keyboard;   // 腕部旋转关节灵敏度(键鼠)
    float wrist_pitch_sens_keyboard;  // 腕部俯仰关节灵敏度(键鼠)
    float vedio_forward_sens_keyboard; // 图传前后移动灵敏度(键鼠)
    float vedio_pitch_sens_keyboard;   // 图传pitch旋转灵敏度(键鼠)


} Garb_Param_s;

typedef struct
{
    Motor_Init_Config_s Grab_motor_config[8]; // 修改为数组以支持多个电机
    Grab_Cali_Mode_e Grab_cali_mode;
    Garb_Param_s  Grab_param;
} Grab_Init_Config_s;

typedef struct
{
    float wrist_roll;    // 腕部关节旋转角度
    float wrist_pitch;   // 腕部关节俯仰角度
    float base_joint;    // 基座旋转关节角度
    float elbow_roll;    // 肘部关节旋转角度
    float elbow_pitch;   // 肘部关节俯仰角度
    float vedio_forward; // 图传的前后移动距离
    float vedio_pitch;   // 图传的pitch旋转角度
    float torque;        // 夹爪电机目标扭矩
    Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;


typedef struct
{
    DJIMotorInstance *grab_djimotor[2];
    DMMotorInstance *grab_dmmotor[1];
    float wrist_roll;    // 腕部关节旋转角度
    float wrist_pitch;   // 腕部关节俯仰角度
    float gripper_joint; // 末端夹爪关节角度
    float torque;        // 夹爪电机目标扭矩
    float L_target;      // 左侧电机旋转角度
    float R_target;      // 右侧电机旋转角度
    float T_target;      // 夹爪电机目标扭矩
} ActuatorInstance;

typedef struct
{
    DMMotorInstance *grab_dmmotor[3];
    float base_joint;  // 基座旋转关节角度
    float elbow_roll;  // 肘部关节旋转角度
    float elbow_pitch; // 肘部关节俯仰角度
} ArmInstance;

typedef struct
{
    DJIMotorInstance *grab_djimotor[2];
    float vedio_forward; // 图传的前后移动距离
    float vedio_pitch;   // 图传的pitch旋转角度
    float F_target;      // 前后移动电机目标角度
    float P_target;      // pitch轴电机目标角度
} VedioInstance;

typedef struct
{
    Grab_Ctrl_Cmd_s grab_ctrl_cmd;
    ArmInstance *arm;
    ActuatorInstance *actuator;
    VedioInstance *vedio;
} GrabInstance;

GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config);

void GrabTask();
