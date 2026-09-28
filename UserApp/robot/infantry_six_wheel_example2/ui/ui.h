#pragma once
#include "robot.h"

// 车辆示宽线
#define WIDTHLINE_UP 330    // 上方间距
#define WIDTHLINE_DOWN 610  // 下方间距

// 摩擦轮转速范围
#define FRIC_LOWER 30000
#define FRIC_UPPER 50000

// 电容电压范围
#define CAP_VOL_LOWER 16.0f
#define CAP_VOL_UPPER 23.0f

// 弹量上限
#define AMMO_UPPER 500

// 圆心
#define CENTER_X 960
#define CENTER_Y 540

// 辅助瞄准线相对于准心的偏移量
#define Aim_Line_1 -30
#define Aim_Line_2 -60
#define Aim_Line_3 -90
#define Aim_Line_4 -120

typedef struct {
  float chassis_power_mx;
  float chassis_power;
} Chassis_Power_Data_s;

#pragma pack(1)
typedef struct {
  uint32_t chassis_flag : 1;
  uint32_t gimbal_flag : 1;
  uint32_t shoot_flag : 1;
  uint32_t fric_flag : 1;
  uint32_t power_flag : 1;
  uint32_t ammo_flag : 1;
  uint32_t pitch_flag : 1;
  uint32_t autoaim_flag : 1;
  uint32_t cap_flag : 1;
  uint32_t yaw_flag : 1;
} UI_Interactive_Flag_t;
#pragma pack()

typedef struct {
  UI_Interactive_Flag_t UI_Interactive_Flag;
  // 为UI绘制以及交互数据所用
  Robot_Mode_e robot_mode;
  Chassis_Mode_e chassis_mode;    // 底盘模式
  Gimbal_Mode_e gimbal_mode;      // 云台模式
  Shoot_Mode_e shoot_mode;        // 发射模式设置
  Friction_Mode_e friction_mode;  // 摩擦轮关闭
  SuperCap_Measure_s cap_msg;     // 超级电容信息

  Chassis_Power_Data_s Chassis_Power_Data;  // 功率控制
  float pitch_angle;                        // 云台俯仰角
  uint8_t Shoot_heat;
  float Shoot_rate;
  uint8_t autoaim_mode;       // 当前自瞄模式 (0/1/2)
  float cap_voltage;          // 电容电压 (V)
  uint16_t bullet_left_real;  // 实体弹丸剩余量
  uint16_t fric_speed_left;   // 左摩擦轮转速
  uint16_t fric_speed_right;  // 右摩擦轮转速
  float chassis_relative_angle;  // 底盘相对角度 (弧度)
  BULLET_Speed_Mode_e bullet_speed_mode_e;
  HEAT_Mode_e heat_mode_e;
  // 上一次的模式，用于flag判断
  Chassis_Mode_e chassis_last_mode;
  Gimbal_Mode_e gimbal_last_mode;
  Shoot_Mode_e shoot_last_mode;
  Friction_Mode_e friction_last_mode;
  uint8_t force_refresh_ui;
  Chassis_Power_Data_s Chassis_last_Power_Data;
  float last_pitch_angle;  // 上一次的俯仰角
  uint8_t last_Shoot_heat;
  float last_Shoot_rate;
  uint8_t last_autoaim_mode;
  float last_cap_voltage;
  uint8_t last_cap_mode;
  uint16_t last_bullet_left_real;
  uint16_t last_fric_speed_left;
  uint16_t last_fric_speed_mid;
  uint16_t last_fric_speed_right;
  float last_chassis_relative_angle;
} Referee_Interactive_info_t;

Referee_Interactive_info_t *getUI();

/**
 * @brief UI初始化
 */
void MyUIInit(RobotInstance* robot);

/**
 * @brief UI任务
 */
void UITask(RobotInstance *robot);
