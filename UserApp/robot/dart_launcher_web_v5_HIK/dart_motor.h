/*
 * dart_motor.h — 四个电机 = 原始 DJIMotorInstance (不再自定义电机类)
 *
 * 直接复用 Modules/motor/DJImotor:
 *   初始化  DJIMotorInit()
 *   控制    DJIMotorOuterLoop() + DJIMotorSetPIDRef() + DJIMotorStop()/Enable()
 *   反馈    motor->measure.total_angle / speed_aps / real_current / temperature
 *   PID     motor->motor_controller.angle_PID / speed_PID
 *   CAN 发送由基类 motor_task.c 的 DJIMotorTask() 统一负责
 *
 * 队友调试时可直接观察下面四个实例指针。
 */
#ifndef DART_LAUNCHER_WEB_V4_DART_MOTOR_H
#define DART_LAUNCHER_WEB_V4_DART_MOTOR_H

#include <stdint.h>

#include "dji_motor.h"

/* ===== 四个电机实例 (原始 DJIMotorInstance) ===== */
extern DJIMotorInstance* MotorSpringA;
extern DJIMotorInstance* MotorSpringB;
extern DJIMotorInstance* MotorTrigger;
extern DJIMotorInstance* MotorYaw;

#define MOTOR_COUNT 4
enum { M_SPRING_A = 0, M_SPRING_B, M_TRIGGER, M_YAW };

typedef enum {
  MODE_STOP = 0,
  MODE_SPEED = 1,  // 目标: 输出 rpm
  MODE_ANGLE = 2,  // 目标: 输出 deg
  MODE_TURNS = 3,  // 目标: 输出 圈
} MotorMode_e;

/* 应用层少量状态; 电机本身仍是上面的 DJIMotorInstance */
typedef struct {
  DJIMotorInstance* inst;
  float ratio;  // 转子:输出
  float zero;   // 零点 (转子 total_angle)
  MotorMode_e mode;
  float target;  // 输出单位
  uint8_t zero_valid;
  uint8_t online;
  uint32_t last_feed;
} MotorAxis;

extern MotorAxis Axis[MOTOR_COUNT];

void MotorsInit(void);
void MotorsTask(void);

void MotorSet(int idx, int mode, float target);
void MotorsStop(void);
void MotorZero(int idx);  // -1 = 全部
void MotorDirToggle(int idx);

/* 参数 id: 1速度Kp 2速度Ki 3速度Kd 4速度积分限幅 5速度输出限幅
 *          6角度Kp 7角度Ki 8角度Kd 9角度死区 10角度输出限幅 11减速比 (浮点*100) */
void MotorSetParam(int idx, int id, int value);
void MotorLoadParamRaw(int idx, int id, float value);
void MotorResetParams(int idx);
void MotorReadParams(int idx, float* out);  // 11 项
void MotorLoadZero(int idx, float zero, uint8_t valid);
void MotorLoadDir(int idx, uint8_t reverse);

float MotorOutAngle(int idx);  // 输出 deg (相对零点, 已含方向)
float MotorOutTurns(int idx);  // 输出 圈
int MotorAtTurns(int idx, float turns, float tol);

#endif  // DART_LAUNCHER_WEB_V4_DART_MOTOR_H
