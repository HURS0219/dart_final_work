/*
 * dart_fsm.c — 自动化发射任务状态机
 */
#include "dart_fsm.h"

#include <math.h>

#include "dart_motor.h"
#include "dart_servo.h"
#include "dart_vision.h"
#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

float g_spring_turns = DART_SPRING_PREP_TURNS;
int g_yaw_mode = DART_YAW_MANUAL;
int g_estop = 0;
int g_task_step = DART_TASK_IDLE;
float g_yaw_aim_rpm = DART_YAW_AIM_RPM;

#define SERVO_SETTLE_MS 400u
#define YAW_AIM_MAX_RPM 60.0f  // 自瞄最大输出转速

static void SpringSetTurns(float turns) {
  MotorSet(M_SPRING_A, MODE_TURNS, turns);
  MotorSet(M_SPRING_B, MODE_TURNS, turns);
}

static int SpringAtTurns(float turns) {
  return MotorAtTurns(M_SPRING_A, turns, DART_POS_TOL_TURNS) &&
         MotorAtTurns(M_SPRING_B, turns, DART_POS_TOL_TURNS);
}

void DartFsmInit(void) {
  g_spring_turns = DART_SPRING_PREP_TURNS;
  g_yaw_mode = DART_YAW_MANUAL;
  g_estop = 0;
  g_task_step = DART_TASK_IDLE;
}

void DartFsmServoGo(int which) { DartServoGo(which); }
void DartFsmSpringGo(int which) { SpringSetTurns(which == 1 ? g_spring_turns : 0.0f); }

void DartFsmAutoStart(void) {
  if (g_estop) return;
  g_task_step = DART_TASK_SERVO_STD1;
}
void DartFsmAutoStop(void) { g_task_step = DART_TASK_IDLE; }

void DartFsmSetEstop(int on) {
  g_estop = on ? 1 : 0;
  if (g_estop) {
    MotorsStop();
    DartServoGo(DART_SERVO_STD);
    g_task_step = DART_TASK_IDLE;
  }
}

static void YawHandler(void) {
  static float aim_i = 0.0f;
  if (g_yaw_mode == DART_YAW_VISION) {
    if (g_vis_ok) {
      float err = (float)g_vis_err;
      if (fabsf(err) < DART_YAW_AIM_DEADBAND) {
        aim_i = 0.0f;
        MotorSet(M_YAW, MODE_SPEED, 0.0f);
      } else {
        /* PI: P 快速逼近, I 消除静差并加速; 允许过冲 */
        aim_i += err * 0.001f;
        VAL_LIMIT(aim_i, -DART_YAW_AIM_I_LIMIT, DART_YAW_AIM_I_LIMIT);
        float spd = DART_YAW_AIM_KP * err + DART_YAW_AIM_KI * aim_i;
        VAL_LIMIT(spd, -g_yaw_aim_rpm, g_yaw_aim_rpm);
        if (fabsf(spd) < DART_YAW_AIM_MIN_RPM)
          spd = (err > 0) ? DART_YAW_AIM_MIN_RPM : -DART_YAW_AIM_MIN_RPM;
        MotorSet(M_YAW, MODE_SPEED, spd);
      }
    } else {
      aim_i = 0.0f;
      MotorSet(M_YAW, MODE_SPEED, 0.0f);
    }
  } else if (g_yaw_mode == DART_YAW_GUIDE) {
    aim_i = 0.0f;
    MotorSet(M_YAW, MODE_ANGLE, DART_YAW_GUIDE_DEG);
  } else {
    aim_i = 0.0f;
  }
}

static void AutoHandler(void) {
  static uint32_t t0 = 0;
  uint32_t now = HAL_GetTick();

  switch (g_task_step) {
    case DART_TASK_IDLE:
    case DART_TASK_DONE:
      break;

    case DART_TASK_SERVO_STD1:
      DartServoGo(DART_SERVO_STD);
      t0 = now;
      g_task_step = DART_TASK_SPRING_STD1;
      break;

    case DART_TASK_SPRING_STD1:
      SpringSetTurns(0.0f);
      if (SpringAtTurns(0.0f) || now - t0 > DART_TASK_STEP_TIMEOUT_MS) {
        t0 = now;
        g_task_step = DART_TASK_SPRING_PREP;
      }
      break;

    case DART_TASK_SPRING_PREP:
      SpringSetTurns(g_spring_turns);
      if (SpringAtTurns(g_spring_turns) || now - t0 > DART_TASK_STEP_TIMEOUT_MS) {
        t0 = now;
        g_task_step = DART_TASK_SERVO_PREP;
      }
      break;

    case DART_TASK_SERVO_PREP:
      DartServoGo(DART_SERVO_PREP);
      if (now - t0 > SERVO_SETTLE_MS) {
        t0 = now;
        g_task_step = DART_TASK_SPRING_BACK;
      }
      break;

    case DART_TASK_SPRING_BACK:
      SpringSetTurns(0.0f);
      if (SpringAtTurns(0.0f) || now - t0 > DART_TASK_STEP_TIMEOUT_MS) {
        t0 = now;
        g_task_step = DART_TASK_SERVO_STD2;
      }
      break;

    case DART_TASK_SERVO_STD2:
      DartServoGo(DART_SERVO_STD);
      if (now - t0 > SERVO_SETTLE_MS) {
        g_task_step = DART_TASK_DONE;
      }
      break;

    default:
      g_task_step = DART_TASK_IDLE;
      break;
  }
}

void DartFsmTask(void) {
  if (g_estop) {
    MotorsStop();
    return;
  }
  YawHandler();
  AutoHandler();
}
