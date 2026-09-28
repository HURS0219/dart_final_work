/**
 * @file servo_motor.c
 * @author ai
 * @brief PWM 舵机(PTK7350 等)底层驱动实现
 * @version 2.0
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention 详细说明见 servo_motor.md
 */
#include "servo_motor.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"

static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];  // 所有实例
static uint8_t servo_idx = 0;                                 // 已注册数量

/* ------------------------------ 内部工具 ------------------------------ */

static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* 逻辑角对称上限(未设置时默认 half_deg) */
static float LimitDeg(const ServoInstance *servo) {
  return (servo->cfg.limit_deg > 0.0f) ? servo->cfg.limit_deg : servo->cfg.half_deg;
}

/* PWM 周期(us), 异常时回落 20000us(50Hz) */
static float PeriodUs(const ServoInstance *servo) {
  float period_s = (servo->pwm != NULL) ? servo->pwm->period : 0.0f;
  if (period_s <= 0.0f) period_s = 0.02f;
  return period_s * 1000000.0f;
}

/* 当前机械偏角(相对中位, 已含 scale/trim/reverse), 用于调零判断 */
static float AppliedOf(const ServoInstance *servo) {
  float applied = servo->angle_deg * servo->cfg.scale + servo->cfg.trim_deg;
  if (servo->cfg.reverse) applied = -applied;
  return Clamp(applied, -servo->cfg.half_deg, servo->cfg.half_deg);
}

/* 逻辑角 -> 输出脉宽(含限位/标定/方向) */
static void ApplyLogical(ServoInstance *servo, float logical_deg) {
  float lim = LimitDeg(servo);
  float applied, pulse;

  logical_deg = Clamp(logical_deg, -lim, lim);
  applied = logical_deg * servo->cfg.scale + servo->cfg.trim_deg;
  if (servo->cfg.reverse) applied = -applied;
  applied = Clamp(applied, -servo->cfg.half_deg, servo->cfg.half_deg);

  pulse = servo->cfg.center_us + applied * (servo->cfg.half_us / servo->cfg.half_deg);
  pulse = Clamp(pulse, servo->cfg.pulse_min_us, servo->cfg.pulse_max_us);

  servo->angle_deg = logical_deg;
  servo->pulse_us = pulse;
  if (servo->pwm != NULL) PWMSetDutyRatio(servo->pwm, pulse / PeriodUs(servo));
}

/* ------------------------------ 对外接口 ------------------------------ */

ServoInstance *ServoInit(Servo_Init_Config_s *config) {
  ServoInstance *servo;

  if (config == NULL) return NULL;
  if (servo_idx >= SERVO_MOTOR_CNT) {
    LOGERROR("[servo] exceed max instance count %d", SERVO_MOTOR_CNT);
    return NULL;
  }

  servo = (ServoInstance *)malloc(sizeof(ServoInstance));
  memset(servo, 0, sizeof(ServoInstance));
  servo->cfg = *config;

  /* 缺省/非法参数回落, 避免除零或方向异常 */
  if (servo->cfg.center_us <= 0.0f) servo->cfg.center_us = SERVO_CENTER_US_DEFAULT;
  if (servo->cfg.half_us <= 0.0f) servo->cfg.half_us = SERVO_HALF_US_DEFAULT;
  if (servo->cfg.half_deg <= 0.0f) servo->cfg.half_deg = SERVO_HALF_DEG_DEFAULT;
  if (servo->cfg.pulse_max_us <= servo->cfg.pulse_min_us) {
    servo->cfg.pulse_min_us = SERVO_PULSE_MIN_DEFAULT;
    servo->cfg.pulse_max_us = SERVO_PULSE_MAX_DEFAULT;
  }
  if (servo->cfg.scale <= 0.0f) servo->cfg.scale = 1.0f;

  servo->pwm = PWMRegister(&servo->cfg.pwm);
  servo->last_time_s = DWT_GetTimeline_s();
  ApplyLogical(servo, 0.0f);  // 上电即输出中位

  servo_motor_instance[servo_idx++] = servo;
  return servo;
}

void ServoSetAngle(ServoInstance *servo, float angle) {
  float lim;
  if (servo == NULL) return;

  lim = LimitDeg(servo);
  angle = Clamp(angle, -lim, lim);
  servo->target_deg = angle;
  if (servo->cfg.rate_limit_dps <= 0.0f) ApplyLogical(servo, angle);  // 不限速: 立即到位
}

void ServoSetPulseUs(ServoInstance *servo, float pulse_us) {
  float applied, logical;
  if (servo == NULL) return;

  pulse_us = Clamp(pulse_us, servo->cfg.pulse_min_us, servo->cfg.pulse_max_us);
  servo->pulse_us = pulse_us;
  if (servo->pwm != NULL) PWMSetDutyRatio(servo->pwm, pulse_us / PeriodUs(servo));

  /* 反推逻辑角/机械角, 保持状态一致 */
  applied = (pulse_us - servo->cfg.center_us) * (servo->cfg.half_deg / servo->cfg.half_us);
  if (servo->cfg.reverse) applied = -applied;
  logical = Clamp((applied - servo->cfg.trim_deg) / servo->cfg.scale, -LimitDeg(servo), LimitDeg(servo));
  servo->angle_deg = logical;
  servo->target_deg = logical;
}

void ServoTask(void) {
  float now = DWT_GetTimeline_s();
  uint8_t i;

  for (i = 0; i < servo_idx; i++) {
    ServoInstance *servo = servo_motor_instance[i];
    float dt, max_delta, err;

    if (servo == NULL) continue;

    dt = now - servo->last_time_s;
    servo->last_time_s = now;
    if (servo->cfg.rate_limit_dps <= 0.0f) continue;  // 未启用限速

    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;
    max_delta = servo->cfg.rate_limit_dps * dt;

    err = servo->target_deg - servo->angle_deg;
    if (err > max_delta) err = max_delta;
    if (err < -max_delta) err = -max_delta;
    ApplyLogical(servo, servo->angle_deg + err);
  }
}

uint8_t ServoZero(ServoInstance *servo) {
  float applied;

  if (servo == NULL) return 0;
  if (!servo->cfg.zero_enable) return 0;  // 调零功能未启用

  applied = AppliedOf(servo);
  if (applied < -SERVO_ZERO_WINDOW_DEG || applied > SERVO_ZERO_WINDOW_DEG) return 0;  // 超出调零窗口

  /* 让逻辑 0° 对应当前位置: out=0 时 applied 不变 -> trim = ±applied (reverse 在 trim 之后) */
  servo->cfg.trim_deg = servo->cfg.reverse ? -applied : applied;
  servo->target_deg = 0.0f;
  servo->angle_deg = 0.0f;
  ApplyLogical(servo, 0.0f);
  return 1;
}

void ServoSetLimit(ServoInstance *servo, float limit_deg) {
  if (servo == NULL) return;
  servo->cfg.limit_deg = (limit_deg > 0.0f) ? limit_deg : 0.0f;
  ApplyLogical(servo, servo->angle_deg);
}

void ServoEnable(ServoInstance *servo) {
  if (servo == NULL || servo->pwm == NULL) return;
  PWMStart(servo->pwm);
  ApplyLogical(servo, servo->angle_deg);
}

void ServoDisable(ServoInstance *servo) {
  if (servo == NULL || servo->pwm == NULL) return;
  PWMStop(servo->pwm);
}

float ServoGetAngle(ServoInstance *servo) { return (servo != NULL) ? servo->angle_deg : 0.0f; }
float ServoGetTarget(ServoInstance *servo) { return (servo != NULL) ? servo->target_deg : 0.0f; }
float ServoGetPulseUs(ServoInstance *servo) { return (servo != NULL) ? servo->pulse_us : 0.0f; }
