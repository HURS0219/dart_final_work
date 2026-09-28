/*
 * dart_servo.c — 扳机舵机 PWM 输出
 */
#include "dart_servo.h"

#include "bsp_pwm.h"
#include "robot_config.h"
#include "user_lib.h"

float g_servo_std_deg = DART_SERVO_STD_DEG;
float g_servo_prep_deg = DART_SERVO_PREP_DEG;
float g_servo_cur_deg = DART_SERVO_STD_DEG;
float g_servo_offset_deg = 0.0f;
int g_servo_state = DART_SERVO_STD;

static PWMInstance* s_pwm = NULL;

static void Output(float raw_deg) {
  if (s_pwm == NULL) return;
  float us = DART_SERVO_MIN_US + raw_deg / DART_SERVO_DEG_RANGE * (DART_SERVO_MAX_US - DART_SERVO_MIN_US);
  VAL_LIMIT(us, DART_SERVO_MIN_US, DART_SERVO_MAX_US);
  PWMSetDutyRatio(s_pwm, us / (DART_SERVO_PERIOD_S * 1000000.0f));
}

void DartServoInit(void) {
  PWM_Init_Config_s cfg = {
      .htim = DART_SERVO_TIM,
      .channel = DART_SERVO_CHANNEL,
      .period = DART_SERVO_PERIOD_S,
      .dutyratio = 0.0f,
      .callback = NULL,
      .id = NULL,
  };
  s_pwm = PWMRegister(&cfg);
  DartServoGo(DART_SERVO_STD);
}

void DartServoSetDeg(float deg) {
  g_servo_cur_deg = deg;
  Output(deg + g_servo_offset_deg);
}

void DartServoSetPos(int which, float deg) {
  if (which == DART_SERVO_PREP) {
    g_servo_prep_deg = deg;
    if (g_servo_state == DART_SERVO_PREP) DartServoSetDeg(deg);
  } else {
    g_servo_std_deg = deg;
    if (g_servo_state == DART_SERVO_STD) DartServoSetDeg(deg);
  }
}

void DartServoGo(int which) {
  g_servo_state = (which == DART_SERVO_PREP) ? DART_SERVO_PREP : DART_SERVO_STD;
  DartServoSetDeg(g_servo_state == DART_SERVO_PREP ? g_servo_prep_deg : g_servo_std_deg);
}

void DartServoSetOffset(float off) { g_servo_offset_deg = off; }

void DartServoZero(void) {
  /* 当前逻辑角变 0: 偏移 = 当前原始角 */
  g_servo_offset_deg += g_servo_cur_deg;
  g_servo_cur_deg = 0.0f;
  Output(g_servo_offset_deg);
}
