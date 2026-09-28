#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>

#include "bsp_can.h"
#include "controller.h"
#include "daemon.h"
#include "motor_def.h"

#define DM_MOTOR_CNT 6

#define DM_P_MIN_J4310 (-12.5f)
#define DM_P_MAX_J4310 12.5f
#define DM_V_MIN_J4310 (-30.0f)  // J4310 30.0f J8009P 45.0f
#define DM_V_MAX_J4310 30.0f
#define DM_T_MIN_J4310 (-10.0f)  // J4310 10.0f J8009P 54.0f
#define DM_T_MAX_J4310 10.0f

#define DM_P_MIN_H6215 (-12.0f)
#define DM_P_MAX_H6215 12.0f
#define DM_V_MIN_H6215 (-45.0f)  // J4310 30.0f J8009P 45.0f
#define DM_V_MAX_H6215 45.0f
#define DM_T_MIN_H6215 (-18.0f)  // J4310 10.0f J8009P 54.0f
#define DM_T_MAX_H6215 18.0f

#define DM_P_MIN_J8009P (-12.5f)
#define DM_P_MAX_J8009P 12.5f
#define DM_V_MIN_J8009P (-45.0f)  // J4310 30.0f J8009P 45.0f
#define DM_V_MAX_J8009P 45.0f
#define DM_T_MIN_J8009P (-54.0f)  // J4310 10.0f J8009P 54.0f
#define DM_T_MAX_J8009P 54.0f

#define DM_P_MIN_J4340 (-12.5f)
#define DM_P_MAX_J4340 12.5f
#define DM_V_MIN_J4340 (-10.0f)
#define DM_V_MAX_J4340 10.0f
#define DM_T_MIN_J4340 (-28.0f)
#define DM_T_MAX_J4340 28.0f

typedef struct {
  uint8_t id;
  uint8_t state;
  float velocity;
  float last_position;
  float position;
  float torque;
  float T_Mos;
  float T_Rotor;
  float total_angle;    // 总角度,注意方向
  int32_t total_round;  // 总跳变次数,注意方向
} DM_Motor_Measure_s;

typedef struct {
  uint16_t position_des;
  uint16_t velocity_des;
  uint16_t torque_des;
  uint16_t Kp;
  uint16_t Kd;
} DM_Motor_Send_s;

typedef struct {
  DM_Motor_Measure_s measure;
  Motor_Control_Setting_s motor_settings;
  Motor_Controller_s motor_controller;

  CANInstance* motor_can_instance;
  Motor_Type_e motor_type;
  Motor_Working_Type_e stop_flag;

  DaemonInstance* daemon;
  uint32_t feed_cnt;
  float dt;
} DMMotorInstance;

typedef enum {
  DM_CMD_MOTOR_MODE = 0xfc,     // 使能,会响应指令
  DM_CMD_RESET_MODE = 0xfd,     // 停止
  DM_CMD_ZERO_POSITION = 0xfe,  // 将当前的位置设置为编码器零位
  DM_CMD_CLEAR_ERROR = 0xfb     // 清除电机过热错误
} DMMotor_Mode_e;

DMMotorInstance* DMMotorInit(Motor_Init_Config_s* config);

void DMMotorSetRef(DMMotorInstance* motor, float ref);

void DMMotorOuterLoop(DMMotorInstance* motor, Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance* motor);

void DMMotorStop(DMMotorInstance* motor);

void DMMotorCaliEncoder(DMMotorInstance* motor);

void DMMotorSetPIDRef(DMMotorInstance* motor, float pid_ref);

void DMMotorTask(void const* argument);

void DMMotorTaskInit();
#endif  // !DMMOTOR