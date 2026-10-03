/*
 * launcher/motor.h — 4 路电机组件 (拉簧A/B + 丝杆/扳机电机 + yaw)
 * =============================================================================
 * 只暴露 Init/Task + ctrl_cmd + 反馈接口; 电机实例指针在实例结构里可被
 * robot/ctrl 读取(容器风格, 同范例 )。底层用 Modules/motor/DJImotor。
 * CAN 发送由 Modules/motor/motor_task.c 的 DJIMotorTask() 在独立任务负责。
 * =============================================================================
 */
#pragma once

#include <stdint.h>

#include "dji_motor.h"
#include "robot_def.h"

typedef enum {
  LMODE_STOP = 0,
  LMODE_SPEED = 1, /* 输出 rpm */
  LMODE_ANGLE = 2, /* 输出 deg */
  LMODE_TURNS = 3, /* 输出 圈 */
} Launcher_MotorMode_e;

typedef struct {
  DJIMotorInstance* inst;
  float ratio; /* 转子:输出 */
  float zero;  /* 零点(转子 total_angle) */
  uint8_t zero_valid;
  uint8_t online;
  uint32_t last_feed;
  Launcher_MotorMode_e mode; /* 当前已应用 */
  float target;              /* 输出单位 */
} MotorAxis;

/* ctrl -> motor 的指令 */
typedef struct {
  uint8_t mode[LM_COUNT];
  float target[LM_COUNT];
  uint8_t estop;
  uint8_t zero_req[LM_COUNT];  /* 单次: 取零 */
  uint8_t reset_req[LM_COUNT]; /* 单次: PID 复位到默认 */
  uint8_t dir_req[LM_COUNT];   /* 单次: 方向翻转 */
  uint8_t set_param;           /* 单次: 参数设定 */
  int8_t param_slot;
  uint8_t param_id;
  float param_value;
} Motor_Ctrl_Cmd_s;

typedef struct {
  Motor_Ctrl_Cmd_s ctrl_cmd;
  MotorAxis axis[LM_COUNT];
} MotorInstance;

MotorInstance* MotorsInit(void);
void MotorsTask(void);

/* 只读反馈 */
float MotorOutAngle(int idx); /* 输出 deg(相对零点, 含方向) */
float MotorOutTurns(int idx); /* 输出 圈 */

/* 读取该路 11 项 PID/比值参数(顺序见 motor.c) */
void MotorReadParams(int idx, float* out11);

/* 写入单路配置(供 store 恢复) */
void MotorLoadParamRaw(int idx, int id, float value); /* id 见 MotorReadParams 顺序 */
void MotorLoadZero(int idx, float zero, uint8_t valid);
void MotorLoadDir(int idx, uint8_t reverse);
void MotorResetParams(int idx);
