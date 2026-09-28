/*
 * dart_pwm.h — 4 路 50Hz 舵机 PWM 输出 (TIM1 CH1~4)
 * 只负责"把脉宽写进比较寄存器", 不做任何角度/标定逻辑。
 */
#ifndef DART_V2_PWM_H
#define DART_V2_PWM_H

#include <stdint.h>

void DartPwmInit(void);
void DartPwmSetPulseUs(uint8_t ch, float us);
float DartPwmGetPulseUs(uint8_t ch);

#endif /* DART_V2_PWM_H */
