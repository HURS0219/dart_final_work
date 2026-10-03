/*
 * launcher/motor.c — 4 路电机组件实现
 * =============================================================================
 * ★核心沿用 dart_launcher_web_v5_HIK/dart_motor.c 的可用逻辑:
 *   - MotorsTask() 里的 DWT 兜底(TRCENA/CYCCNT): 调试器连接/断开会清 TRCENA,
 *     导致 DJI PID 的 dt=0 -> 电机乱转/不转。这是实机可用的关键。
 *   - 在线判定 + 首次反馈自动取零; 拉簧 A/B 零点对齐;
 *   - 每周期 Enable + OuterLoop + SetPIDRef(SPEED: sign*out*rpm*ratio*6;
 *     ANGLE: zero + sign*out*ratio; TURNS 先 *360)。
 * =============================================================================
 */
#include "motor.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

static MotorInstance* s_motor = NULL;

static const float kRatio[LM_COUNT] = {DART_M3508_GEAR_RATIO, DART_M3508_GEAR_RATIO,
                                       DART_M3508_GEAR_RATIO, DART_M2006_GEAR_RATIO};

typedef struct {
  float skp, ski, skd, silim, smax;
  float akp, aki, akd, adead, amax;
} PidDef_s;

static const PidDef_s kDefM3508 = {2.5f, 0.1f, 0.0f, 800.0f, 16384.0f,
                                   5.0f, 0.0f, 0.3f, 8.0f, 13826.0f};
static const PidDef_s kDefM2006 = {3.0f, 0.3f, 0.0f, 1000.0f, 10000.0f,
                                   5.0f, 0.0f, 0.3f, 2.0f, 25920.0f};

/* 反向电机的 total_angle 已被库取反, 换算机械方向需乘此符号 */
static float SignOf(DJIMotorInstance* m) {
  return (m->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
}

static PidDef_s DefOf(int idx) { return (idx == LM_YAW) ? kDefM2006 : kDefM3508; }

void MotorLoadParamRaw(int idx, int id, float v) {  PIDInstance* sp = &s_motor->axis[idx].inst->motor_controller.speed_PID;
  PIDInstance* ap = &s_motor->axis[idx].inst->motor_controller.angle_PID;
  switch (id) {
    case 1: sp->Kp = v; break;
    case 2: sp->Ki = v; break;
    case 3: sp->Kd = v; break;
    case 4: sp->IntegralLimit = v; break;
    case 5: sp->MaxOut = v; break;
    case 6: ap->Kp = v; break;
    case 7: ap->Ki = v; break;
    case 8: ap->Kd = v; break;
    case 9: ap->DeadBand = v; break;
    case 10: ap->MaxOut = v; break;
    case 11: if (v > 0.01f) s_motor->axis[idx].ratio = v; break;
    default: break;
  }
}

void MotorResetParams(int idx) {
  PidDef_s d = DefOf(idx);
  MotorLoadParamRaw(idx, 1, d.skp);
  MotorLoadParamRaw(idx, 2, d.ski);
  MotorLoadParamRaw(idx, 3, d.skd);
  MotorLoadParamRaw(idx, 4, d.silim);
  MotorLoadParamRaw(idx, 5, d.smax);
  MotorLoadParamRaw(idx, 6, d.akp);
  MotorLoadParamRaw(idx, 7, d.aki);
  MotorLoadParamRaw(idx, 8, d.akd);
  MotorLoadParamRaw(idx, 9, d.adead);
  MotorLoadParamRaw(idx, 10, d.amax);
  MotorLoadParamRaw(idx, 11, kRatio[idx]);
}

float MotorOutAngle(int idx) {
  if (idx < 0 || idx >= LM_COUNT) return 0.0f;
  MotorAxis* a = &s_motor->axis[idx];
  return SignOf(a->inst) * (a->inst->measure.total_angle - a->zero) / a->ratio;
}
float MotorOutTurns(int idx) { return MotorOutAngle(idx) / 360.0f; }

void MotorReadParams(int idx, float* out11) {
  if (out11 == NULL || idx < 0 || idx >= LM_COUNT) return;
  PIDInstance* sp = &s_motor->axis[idx].inst->motor_controller.speed_PID;
  PIDInstance* ap = &s_motor->axis[idx].inst->motor_controller.angle_PID;
  out11[0] = sp->Kp;
  out11[1] = sp->Ki;
  out11[2] = sp->Kd;
  out11[3] = sp->IntegralLimit;
  out11[4] = sp->MaxOut;
  out11[5] = ap->Kp;
  out11[6] = ap->Ki;
  out11[7] = ap->Kd;
  out11[8] = ap->DeadBand;
  out11[9] = ap->MaxOut;
  out11[10] = s_motor->axis[idx].ratio;
}

void MotorLoadZero(int idx, float zero, uint8_t valid) {
  if (idx < 0 || idx >= LM_COUNT) return;
  s_motor->axis[idx].zero = zero;
  s_motor->axis[idx].zero_valid = valid;
}

void MotorLoadDir(int idx, uint8_t reverse) {
  if (idx < 0 || idx >= LM_COUNT) return;
  Motor_Reverse_Flag_e r = reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
  s_motor->axis[idx].inst->motor_settings.motor_reverse_flag = r;
  s_motor->axis[idx].inst->motor_settings.feedback_reverse_flag =
      reverse ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;
}

MotorInstance* MotorsInit(void) {
  s_motor = (MotorInstance*)zmalloc(sizeof(MotorInstance));
  memset(s_motor, 0, sizeof(MotorInstance));

  Motor_Init_Config_s cfg_a = DART_M3508_CONFIG(DART_SPRING_A_ID, DART_SPRING_A_REVERSE);
  Motor_Init_Config_s cfg_b = DART_M3508_CONFIG(DART_SPRING_B_ID, DART_SPRING_B_REVERSE);
  Motor_Init_Config_s cfg_t = DART_M3508_CONFIG(DART_TRIGGER_ID, DART_TRIGGER_REVERSE);
  Motor_Init_Config_s cfg_y = DART_M2006_CONFIG(DART_YAW_ID, DART_YAW_REVERSE);

  s_motor->axis[LM_SPRING_A].inst = DJIMotorInit(&cfg_a);
  s_motor->axis[LM_SPRING_B].inst = DJIMotorInit(&cfg_b);
  s_motor->axis[LM_TRIGGER].inst = DJIMotorInit(&cfg_t);
  s_motor->axis[LM_YAW].inst = DJIMotorInit(&cfg_y);

  for (int i = 0; i < LM_COUNT; i++) {
    s_motor->axis[i].ratio = kRatio[i];
    s_motor->axis[i].mode = LMODE_STOP;
    DJIMotorStop(s_motor->axis[i].inst);
  }
  return s_motor;
}

/* 处理 ctrl 下发的单次请求(取零/复位/换向/参数) */
static void HandleRequests(void) {
  Motor_Ctrl_Cmd_s* c = &s_motor->ctrl_cmd;

  for (int i = 0; i < LM_COUNT; i++) {
    MotorAxis* a = &s_motor->axis[i];

    if (c->zero_req[i]) {
      c->zero_req[i] = 0;
      a->zero = a->inst->measure.total_angle;
      a->zero_valid = 1;
      if (a->mode != LMODE_STOP) a->target = 0.0f;
      /* 拉簧 A/B 零点对齐 */
      if (i == LM_SPRING_A || i == LM_SPRING_B) {
        int o = (i == LM_SPRING_A) ? LM_SPRING_B : LM_SPRING_A;
        s_motor->axis[o].zero = s_motor->axis[o].inst->measure.total_angle;
        s_motor->axis[o].zero_valid = 1;
        if (s_motor->axis[o].mode != LMODE_STOP) s_motor->axis[o].target = 0.0f;
        c->zero_req[o] = 0;
      }
    }
    if (c->reset_req[i]) {
      c->reset_req[i] = 0;
      MotorResetParams(i);
    }
    if (c->dir_req[i]) {
      c->dir_req[i] = 0;
      DJIMotorInstance* m = a->inst;
      Motor_Reverse_Flag_e r = (m->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_NORMAL)
                                   ? MOTOR_DIRECTION_REVERSE
                                   : MOTOR_DIRECTION_NORMAL;
      m->motor_settings.motor_reverse_flag = r;
      m->motor_settings.feedback_reverse_flag =
          (r == MOTOR_DIRECTION_REVERSE) ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;
      a->zero = m->measure.total_angle; /* 方向变了重新取零 */
      a->zero_valid = 1;
    }
  }
  if (c->set_param && c->param_slot >= 0 && c->param_slot < LM_COUNT) {
    MotorLoadParamRaw(c->param_slot, c->param_id, c->param_value);
    c->set_param = 0;
  }
}

void MotorsTask(void) {
  if (s_motor == NULL) return;

  /* ★DWT 兜底: 调试器连接/断开可能清 TRCENA, 导致 DJI PID 的 dt=0 */
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  HandleRequests();

  Motor_Ctrl_Cmd_s* c = &s_motor->ctrl_cmd;
  for (int i = 0; i < LM_COUNT; i++) {
    MotorAxis* a = &s_motor->axis[i];
    DJIMotorInstance* m = a->inst;

    if (m->feed_cnt != a->last_feed) {
      a->last_feed = m->feed_cnt;
      a->online = 1;
    }
    if (m->feed_cnt > 0 && !a->zero_valid) {
      a->zero = m->measure.total_angle;
      a->zero_valid = 1;
    }

    /* ctrl 的持续设定 -> 应用态 */
    a->mode = (Launcher_MotorMode_e)c->mode[i];
    a->target = c->target[i];

    if (c->estop || a->mode == LMODE_STOP) {
      DJIMotorStop(m);
      continue;
    }
    DJIMotorEnable(m);

    if (a->mode == LMODE_SPEED) {
      DJIMotorOuterLoop(m, SPEED_LOOP);
      DJIMotorSetPIDRef(m, SignOf(m) * a->target * a->ratio * 6.0f);
    } else {
      DJIMotorOuterLoop(m, ANGLE_LOOP);
      float tgt = (a->mode == LMODE_TURNS) ? a->target * 360.0f : a->target;
      DJIMotorSetPIDRef(m, a->zero + SignOf(m) * tgt * a->ratio);
    }
  }
}
