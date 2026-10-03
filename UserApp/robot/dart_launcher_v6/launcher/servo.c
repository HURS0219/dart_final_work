/*
 * launcher/servo.c — 扳机舵机组件实现 (移植 dart_launcher_web_v5_HIK/dart_servo.c)
 */
#include "servo.h"

#include "bsp_pwm.h"
#include "robot_config.h"
#include "user_lib.h"

static ServoInstance* s_sv = NULL;
static PWMInstance* s_pwm = NULL;

static void OutputRaw(float raw_deg) {
  if (s_pwm == NULL) return;
  float us = DART_SERVO_MIN_US +
             raw_deg / DART_SERVO_DEG_RANGE * (DART_SERVO_MAX_US - DART_SERVO_MIN_US);
  VAL_LIMIT(us, DART_SERVO_MIN_US, DART_SERVO_MAX_US);
  PWMSetDutyRatio(s_pwm, us / (DART_SERVO_PERIOD_S * 1000000.0f));
}

static void ApplyDeg(float logic_deg) {
  s_sv->cur_deg = logic_deg;
  OutputRaw(logic_deg + s_sv->offset_deg);
}

ServoInstance* LauncherServoInit(void) {
  s_sv = (ServoInstance*)zmalloc(sizeof(ServoInstance));
  if (s_sv == NULL) return NULL;
  s_sv->std_deg = DART_SERVO_STD_DEG;
  s_sv->prep_deg = DART_SERVO_PREP_DEG;
  s_sv->offset_deg = 0.0f;
  s_sv->cur_deg = DART_SERVO_STD_DEG;
  s_sv->state = LAUNCH_SERVO_STD;

  PWM_Init_Config_s cfg = {
      .htim = DART_SERVO_TIM,
      .channel = DART_SERVO_CHANNEL,
      .period = DART_SERVO_PERIOD_S,
      .dutyratio = 0.0f,
      .callback = NULL,
      .id = NULL,
  };
  s_pwm = PWMRegister(&cfg);
  ApplyDeg(s_sv->std_deg);
  return s_sv;
}

void LauncherServoTask(void) {
  if (s_sv == NULL) return;
  Servo_Ctrl_Cmd_s* c = &s_sv->cmd;

  if (c->set_std) {
    c->set_std = 0;
    s_sv->std_deg = c->std_deg;
    if (s_sv->state == LAUNCH_SERVO_STD) ApplyDeg(s_sv->std_deg);
  }
  if (c->set_prep) {
    c->set_prep = 0;
    s_sv->prep_deg = c->prep_deg;
    if (s_sv->state == LAUNCH_SERVO_PREP) ApplyDeg(s_sv->prep_deg);
  }
  if (c->go_set) {
    c->go_set = 0;
    s_sv->state = (c->go == LAUNCH_SERVO_PREP) ? LAUNCH_SERVO_PREP : LAUNCH_SERVO_STD;
    ApplyDeg(s_sv->state == LAUNCH_SERVO_PREP ? s_sv->prep_deg : s_sv->std_deg);
  }
  if (c->set_deg) {
    c->set_deg = 0;
    ApplyDeg(c->deg);
  }
  if (c->zero) {
    c->zero = 0;
    s_sv->offset_deg += s_sv->cur_deg; /* 当前逻辑角变 0 */
    s_sv->cur_deg = 0.0f;
    OutputRaw(s_sv->offset_deg);
  }
}
