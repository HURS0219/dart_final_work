#ifndef GANTRY_H
#define GANTRY_H

#include "dji_motor.h"
#include "remote_control.h"

/// @brief 龙门架运行模式
typedef enum
{
    GANTRY_MODE_POWER_OFF,       // 机械臂断电
    GANTRY_MODE_LOCK,            // 机械臂锁死
    GANTRY_MODE_CONTROL_REMOTE,  // 遥控器控制
    GANTRY_MODE_CONTROL_PC,      // 客户端键鼠控制
} Gantry_Mode_e;

/// @brief 龙门架电机控制单元
typedef struct {
    DJIMotorInstance* motor;     // 电机实例
} GantryMotorUnit_t;

/// @brief 龙门架控制命令结构体
typedef struct {
    Gantry_Mode_e Gantry_mode;   // 运行模式
    float x, y, z;               // 龙门架位置矢量(x:横移  y:前伸  z:高度)
    uint8_t controller_st;       // 自定义控制器开关
} Gantry_Ctrl_Cmd_s;

/// @brief 龙门架系统参数
typedef struct {
    float GANTRY_MAX_Y;   // 前伸最前位置
    float GANTRY_MAX_Z;   // 抬升最高位置
    float GANTRY_MAX_X;   // 横移最右位置

    float lift_sens_remote;       // 抬升电机灵敏度(遥控器)
    float stretch_sens_remote;    // 前伸电机灵敏度(遥控器)
    float sidesway_sens_remote;   // 横移电机灵敏度(遥控器)

    float lift_sens_keyboard;     // 抬升电机灵敏度(键鼠)
    float stretch_sens_keyboard;  // 前伸电机灵敏度(键鼠)
    float sidesway_sens_keyboard; // 横移电机灵敏度(键鼠)

    float position_ecd_ratio;     // 位置矢量与电机转动角度的比例
} Gantry_Param_s;

/// @brief 龙门架电机初始化
typedef struct {
    Gantry_Param_s Gantry_param;
    Motor_Init_Config_s lift_motor_config[2];    // 抬升电机配置
    Motor_Init_Config_s stretch_motor_config[2]; // 前伸电机配置
    Motor_Init_Config_s sidesway_motor_config;// 横移电机配置
} Gantry_Init_Config_s;

/// @brief 龙门架电机实例
typedef struct {
    Gantry_Param_s Gantry_param;          // 参数
    Gantry_Ctrl_Cmd_s Gantry_ctrl_cmd;    // 控制命令
    GantryMotorUnit_t lift_motor[2];         // 抬升电机实例
    GantryMotorUnit_t stretch_motor[2];      // 前伸电机实例
    GantryMotorUnit_t sidesway_motor;     // 横移电机实例
} GantryInstance;

GantryInstance* GantryInit(Gantry_Init_Config_s* init_config);
void StartGantryTask(void const *argument);
void GantryTask(void);

#endif //GANTRY_H
