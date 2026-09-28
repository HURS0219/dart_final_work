/*
 * dart_motor.c — 直接使用原始 DJIMotor 实例 (无自定义电机类)
 */
#include "dart_motor.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

/* ===== 四个原始 DJIMotor 实例 ===== */
DJIMotorInstance* MotorSpringA = NULL;
DJIMotorInstance* MotorSpringB = NULL;
DJIMotorInstance* MotorTrigger = NULL;
DJIMotorInstance* MotorYaw = NULL;

MotorAxis Axis[MOTOR_COUNT];

extern int g_estop;  // dart_fsm.c

static const float kRatio[MOTOR_COUNT] = {
    DART_M3508_GEAR_RATIO, DART_M3508_GEAR_RATIO, DART_M3508_GEAR_RATIO, DART_M2006_GEAR_RATIO};

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

void MotorsInit(void) {
  Motor_Init_Config_s cfg_a = DART_M3508_CONFIG(DART_SPRING_A_ID, DART_SPRING_A_REVERSE);
  Motor_Init_Config_s cfg_b = DART_M3508_CONFIG(DART_SPRING_B_ID, DART_SPRING_B_REVERSE);
  Motor_Init_Config_s cfg_t = DART_M3508_CONFIG(DART_TRIGGER_ID, DART_TRIGGER_REVERSE);
  Motor_Init_Config_s cfg_y = DART_M2006_CONFIG(DART_YAW_ID, DART_YAW_REVERSE);

  MotorSpringA = DJIMotorInit(&cfg_a);
  MotorSpringB = DJIMotorInit(&cfg_b);
  MotorTrigger = DJIMotorInit(&cfg_t);
  MotorYaw = DJIMotorInit(&cfg_y);

  memset(Axis, 0, sizeof(Axis));
  Axis[M_SPRING_A].inst = MotorSpringA;
  Axis[M_SPRING_B].inst = MotorSpringB;
  Axis[M_TRIGGER].inst = MotorTrigger;
  Axis[M_YAW].inst = MotorYaw;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Axis[i].ratio = kRatio[i];
    Axis[i].mode = MODE_STOP;
    DJIMotorStop(Axis[i].inst);
  }
}

float MotorOutAngle(int idx) {
  if (idx < 0 || idx >= MOTOR_COUNT) return 0.0f;
  MotorAxis* a = &Axis[idx];
  return SignOf(a->inst) * (a->inst->measure.total_angle - a->zero) / a->ratio;
}
float MotorOutTurns(int idx) { return MotorOutAngle(idx) / 360.0f; }
int MotorAtTurns(int idx, float turns, float tol) {
  return fabsf(MotorOutTurns(idx) - turns) < tol;
}

void MotorSet(int idx, int mode, float target) {
  if (idx < 0 || idx >= MOTOR_COUNT) return;
  Axis[idx].mode = (MotorMode_e)mode;
  Axis[idx].target = target;
  if (mode == MODE_STOP) DJIMotorStop(Axis[idx].inst);

  /* 拉簧 A/B 视为同一轴: 命令永远同步 */
  if (idx == M_SPRING_A || idx == M_SPRING_B) {
    int o = (idx == M_SPRING_A) ? M_SPRING_B : M_SPRING_A;
    Axis[o].mode = (MotorMode_e)mode;
    Axis[o].target = target;
    if (mode == MODE_STOP) DJIMotorStop(Axis[o].inst);
  }
}

void MotorsStop(void) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Axis[i].mode = MODE_STOP;
    Axis[i].target = 0.0f;
    DJIMotorStop(Axis[i].inst);
  }
}

void MotorZero(int idx) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (idx >= 0 && i != idx) {
      /* 拉簧 A/B 零点必须对齐: 取零时一起取 */
      int spring = (idx == M_SPRING_A || idx == M_SPRING_B);
      int ispring = (i == M_SPRING_A || i == M_SPRING_B);
      if (!(spring && ispring)) continue;
    }
    Axis[i].zero = Axis[i].inst->measure.total_angle;
    Axis[i].zero_valid = 1;
    if (Axis[i].mode != MODE_STOP) Axis[i].target = 0.0f;
  }
}

void MotorDirToggle(int idx) {
  if (idx < 0 || idx >= MOTOR_COUNT) return;
  DJIMotorInstance* m = Axis[idx].inst;
  Motor_Reverse_Flag_e r = (m->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_NORMAL)
                               ? MOTOR_DIRECTION_REVERSE
                               : MOTOR_DIRECTION_NORMAL;
  m->motor_settings.motor_reverse_flag = r;
  m->motor_settings.feedback_reverse_flag =
      (r == MOTOR_DIRECTION_REVERSE) ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;
  Axis[idx].zero = m->measure.total_angle;  // 方向变了重新取零
  Axis[idx].zero_valid = 1;
}

static PidDef_s DefOf(int idx) { return (idx == M_YAW) ? kDefM2006 : kDefM3508; }

void MotorLoadParamRaw(int idx, int id, float v) {
  if (idx < 0 || idx >= MOTOR_COUNT) return;
  PIDInstance* sp = &Axis[idx].inst->motor_controller.speed_PID;
  PIDInstance* ap = &Axis[idx].inst->motor_controller.angle_PID;
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
    case 11:
      if (v > 0.01f) Axis[idx].ratio = v;
      break;
    default: break;
  }
}

void MotorSetParam(int idx, int id, int value) { MotorLoadParamRaw(idx, id, value / 100.0f); }

void MotorReadParams(int idx, float* out) {
  if (idx < 0 || idx >= MOTOR_COUNT || out == NULL) return;
  PIDInstance* sp = &Axis[idx].inst->motor_controller.speed_PID;
  PIDInstance* ap = &Axis[idx].inst->motor_controller.angle_PID;
  out[0] = sp->Kp;
  out[1] = sp->Ki;
  out[2] = sp->Kd;
  out[3] = sp->IntegralLimit;
  out[4] = sp->MaxOut;
  out[5] = ap->Kp;
  out[6] = ap->Ki;
  out[7] = ap->Kd;
  out[8] = ap->DeadBand;
  out[9] = ap->MaxOut;
  out[10] = Axis[idx].ratio;
}

void MotorLoadZero(int idx, float zero, uint8_t valid) {
  if (idx < 0 || idx >= MOTOR_COUNT) return;
  Axis[idx].zero = zero;
  Axis[idx].zero_valid = valid;
}

void MotorLoadDir(int idx, uint8_t reverse) {
  if (idx < 0 || idx >= MOTOR_COUNT) return;
  Motor_Reverse_Flag_e r = reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
  Axis[idx].inst->motor_settings.motor_reverse_flag = r;
  Axis[idx].inst->motor_settings.feedback_reverse_flag =
      reverse ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;
}

void MotorResetParams(int idx) {
  if (idx < 0 || idx >= MOTOR_COUNT) return;
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

/* ===== 每周期: 直接用 DJI 接口设参考 ===== */
void MotorsTask(void) {
  /* DWT 兜底: 调试器断开可能清 TRCENA, 导致 DJI PID 的 dt=0 */
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    MotorAxis* a = &Axis[i];
    DJIMotorInstance* m = a->inst;

    if (m->feed_cnt != a->last_feed) {
      a->last_feed = m->feed_cnt;
      a->online = 1;
    }
    if (m->feed_cnt > 0 && !a->zero_valid) {
      a->zero = m->measure.total_angle;
      a->zero_valid = 1;
    }

    if (g_estop || a->mode == MODE_STOP) {
      DJIMotorStop(m);
      continue;
    }
    DJIMotorEnable(m);

    if (a->mode == MODE_SPEED) {
      DJIMotorOuterLoop(m, SPEED_LOOP);
      DJIMotorSetPIDRef(m, SignOf(m) * a->target * a->ratio * 6.0f);
    } else {
      DJIMotorOuterLoop(m, ANGLE_LOOP);
      float tgt = (a->mode == MODE_TURNS) ? a->target * 360.0f : a->target;
      DJIMotorSetPIDRef(m, a->zero + SignOf(m) * tgt * a->ratio);
    }
  }
}
