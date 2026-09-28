/*
 * dart_servo.h — 扳机舵机 (PWM1 / TIM1_CH1), 角度制 (逻辑角 = 原始角 - 零点偏移)
 */
#ifndef DART_LAUNCHER_WEB_V2_DART_SERVO_H
#define DART_LAUNCHER_WEB_V2_DART_SERVO_H

#include <stdint.h>

#define DART_SERVO_STD  0
#define DART_SERVO_PREP 1

extern float g_servo_std_deg;
extern float g_servo_prep_deg;
extern float g_servo_cur_deg;     // 逻辑当前角
extern float g_servo_offset_deg;  // 零点偏移
extern int g_servo_state;

void DartServoInit(void);
void DartServoSetPos(int which, float deg);  // 设标准/预备位
void DartServoGo(int which);                 // 去标准/预备位
void DartServoSetDeg(float deg);             // 直接给逻辑角
void DartServoZero(void);                    // 当前角设为 0
void DartServoSetOffset(float off);

#endif  // DART_LAUNCHER_WEB_V2_DART_SERVO_H
