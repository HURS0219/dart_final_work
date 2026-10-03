/*
 * ctrl/ctrl.c — 大脑实现 (robot_cmd)
 * =============================================================================
 * 唯一决定"各机构该到哪"的地方:
 *   1) 从 link 取操作员指令 -> 填 motor/servo 的 ctrl_cmd;
 *   2) 自动发射时序(移植 dart_fsm.c 的 AutoHandler);
 *   3) 自瞄 yaw(移植 dart_fsm.c 的 YawHandler);
 *   4) 急停/失联保护(移植 dart_link.c 的超时停车语义)。
 * 不直接调用任何 DJIMotor/舵机接口。
 * =============================================================================
 */
#include "ctrl.h"

#include <math.h>

#include "link.h"
#include "main.h"
#include "motor.h"
#include "robot.h"
#include "robot_config.h"
#include "servo.h"
#include "store.h"
#include "user_lib.h"
#include "vision.h"

#define SERVO_SETTLE_MS 400u

static uint8_t s_estop = 0;
static uint8_t s_yaw_mode = LAUNCH_YAW_MANUAL;
static float s_aim_rpm = DART_YAW_AIM_RPM;
static float s_spring_turns = DART_SPRING_PREP_TURNS;
static uint8_t s_task_step = LAUNCH_TASK_IDLE;
static uint32_t s_step_t0 = 0;

/* 持久化的逐路目标(每周期整帧写入 motor->ctrl_cmd) */
static uint8_t s_mode[LM_COUNT];
static float s_target[LM_COUNT];

static uint8_t s_sv_state = LAUNCH_SERVO_STD;
static float s_sv_std = DART_SERVO_STD_DEG;
static float s_sv_prep = DART_SERVO_PREP_DEG;

static float s_aim_i = 0.0f;

void CtrlInit(void) {
  s_estop = 0;
  s_yaw_mode = LAUNCH_YAW_MANUAL;
  s_aim_rpm = DART_YAW_AIM_RPM;
  s_spring_turns = DART_SPRING_PREP_TURNS;
  s_task_step = LAUNCH_TASK_IDLE;
  s_step_t0 = 0;
  s_aim_i = 0.0f;
  for (int i = 0; i < LM_COUNT; i++) {
    s_mode[i] = LMODE_STOP;
    s_target[i] = 0.0f;
  }
  s_sv_state = LAUNCH_SERVO_STD;
  s_sv_std = DART_SERVO_STD_DEG;
  s_sv_prep = DART_SERVO_PREP_DEG;
}

float CtrlGetSpringTurns(void) { return s_spring_turns; }
void CtrlSetSpringTurns(float t) { s_spring_turns = t; }
uint8_t CtrlGetYawMode(void) { return s_yaw_mode; }
void CtrlSetYawMode(uint8_t m) { s_yaw_mode = m; }
float CtrlGetAimRpm(void) { return s_aim_rpm; }
void CtrlSetAimRpm(float r) { s_aim_rpm = r; }
uint8_t CtrlGetTaskStep(void) { return s_task_step; }
uint8_t CtrlGetEstop(void) { return s_estop; }

static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void SpringSetTurns(float turns) {
  s_mode[LM_SPRING_A] = LMODE_TURNS;
  s_target[LM_SPRING_A] = turns;
  s_mode[LM_SPRING_B] = LMODE_TURNS;
  s_target[LM_SPRING_B] = turns;
}

static int SpringAtTurns(float turns) {
  return (fabsf(MotorOutTurns(LM_SPRING_A) - turns) < DART_POS_TOL_TURNS) &&
         (fabsf(MotorOutTurns(LM_SPRING_B) - turns) < DART_POS_TOL_TURNS);
}

/* ---- 自动发射时序 ---- */
static void AutoHandler(void) {
  uint32_t now = HAL_GetTick();
  switch (s_task_step) {
    case LAUNCH_TASK_IDLE:
    case LAUNCH_TASK_DONE:
      break;
    case LAUNCH_TASK_SERVO_STD1:
      s_sv_state = LAUNCH_SERVO_STD;
      s_step_t0 = now;
      s_task_step = LAUNCH_TASK_SPRING_STD1;
      break;
    case LAUNCH_TASK_SPRING_STD1:
      SpringSetTurns(0.0f);
      if (SpringAtTurns(0.0f) || now - s_step_t0 > DART_TASK_STEP_TIMEOUT_MS) {
        s_step_t0 = now;
        s_task_step = LAUNCH_TASK_SPRING_PREP;
      }
      break;
    case LAUNCH_TASK_SPRING_PREP:
      SpringSetTurns(s_spring_turns);
      if (SpringAtTurns(s_spring_turns) || now - s_step_t0 > DART_TASK_STEP_TIMEOUT_MS) {
        s_step_t0 = now;
        s_task_step = LAUNCH_TASK_SERVO_PREP;
      }
      break;
    case LAUNCH_TASK_SERVO_PREP:
      s_sv_state = LAUNCH_SERVO_PREP;
      if (now - s_step_t0 > SERVO_SETTLE_MS) {
        s_step_t0 = now;
        s_task_step = LAUNCH_TASK_SPRING_BACK;
      }
      break;
    case LAUNCH_TASK_SPRING_BACK:
      SpringSetTurns(0.0f);
      if (SpringAtTurns(0.0f) || now - s_step_t0 > DART_TASK_STEP_TIMEOUT_MS) {
        s_step_t0 = now;
        s_task_step = LAUNCH_TASK_SERVO_STD2;
      }
      break;
    case LAUNCH_TASK_SERVO_STD2:
      s_sv_state = LAUNCH_SERVO_STD;
      if (now - s_step_t0 > SERVO_SETTLE_MS) s_task_step = LAUNCH_TASK_DONE;
      break;
    default:
      s_task_step = LAUNCH_TASK_IDLE;
      break;
  }
  if (s_task_step != LAUNCH_TASK_IDLE && s_task_step != LAUNCH_TASK_DONE) {
    /* 时序激活: 舵机去该去的位 */
    robot->servo->cmd.go_set = 1;
    robot->servo->cmd.go = s_sv_state;
  }
}

/* ---- 自瞄 yaw ---- */
static void YawHandler(void) {
  if (s_yaw_mode == LAUNCH_YAW_VISION) {
    if (robot->vision->ok) {
      float err = (float)robot->vision->err;
      if (fabsf(err) < DART_YAW_AIM_DEADBAND) {
        s_aim_i = 0.0f;
        s_mode[LM_YAW] = LMODE_SPEED;
        s_target[LM_YAW] = 0.0f;
      } else {
        s_aim_i += err * 0.001f;
        s_aim_i = Clamp(s_aim_i, -DART_YAW_AIM_I_LIMIT, DART_YAW_AIM_I_LIMIT);
        float spd = DART_YAW_AIM_KP * err + DART_YAW_AIM_KI * s_aim_i;
        spd = Clamp(spd, -s_aim_rpm, s_aim_rpm);
        if (fabsf(spd) < DART_YAW_AIM_MIN_RPM)
          spd = (err > 0) ? DART_YAW_AIM_MIN_RPM : -DART_YAW_AIM_MIN_RPM;
        s_mode[LM_YAW] = LMODE_SPEED;
        s_target[LM_YAW] = spd;
      }
    } else {
      s_aim_i = 0.0f;
      s_mode[LM_YAW] = LMODE_SPEED;
      s_target[LM_YAW] = 0.0f;
    }
  } else if (s_yaw_mode == LAUNCH_YAW_GUIDE) {
    s_aim_i = 0.0f;
    s_mode[LM_YAW] = LMODE_ANGLE;
    s_target[LM_YAW] = DART_YAW_GUIDE_DEG;
  }
  /* MANUAL: 由 M,3,... 直接控制, 此处不覆盖 */
}

/* ---- 操作员指令处理 ---- */
static void HandleCmd(const Launcher_Cmd_s* c) {
  /* 单次请求 -> motor ctrl_cmd 标志 */
  if (c->req_zero) {
    if (c->req_zero_slot < 0) {
      for (int i = 0; i < LM_COUNT; i++) robot->motor->ctrl_cmd.zero_req[i] = 1;
    } else if (c->req_zero_slot < LM_COUNT) {
      robot->motor->ctrl_cmd.zero_req[c->req_zero_slot] = 1;
    }
    StoreMarkDirty();
  }
  if (c->req_reset && c->req_reset_slot >= 0 && c->req_reset_slot < LM_COUNT) {
    robot->motor->ctrl_cmd.reset_req[c->req_reset_slot] = 1;
    StoreMarkDirty();
  }
  if (c->req_dir && c->req_dir_slot >= 0 && c->req_dir_slot < LM_COUNT) {
    robot->motor->ctrl_cmd.dir_req[c->req_dir_slot] = 1;
    StoreMarkDirty();
  }
  if (c->req_save) StoreMarkDirtyNow();

  /* M,slot,mode,value */
  if (c->set_motor && c->motor_slot >= 0 && c->motor_slot < LM_COUNT) {
    int slot = c->motor_slot;
    s_mode[slot] = c->motor_mode;
    s_target[slot] = c->motor_value;
    if (slot == LM_SPRING_A || slot == LM_SPRING_B) { /* 拉簧同步 */
      int o = (slot == LM_SPRING_A) ? LM_SPRING_B : LM_SPRING_A;
      s_mode[o] = c->motor_mode;
      s_target[o] = c->motor_value;
      if (c->motor_mode == LMODE_TURNS) s_spring_turns = c->motor_value;
    }
    StoreMarkDirty();
  }
  /* P,slot,id,value */
  if (c->set_param && c->param_slot >= 0 && c->param_slot < LM_COUNT) {
    robot->motor->ctrl_cmd.set_param = 1;
    robot->motor->ctrl_cmd.param_slot = c->param_slot;
    robot->motor->ctrl_cmd.param_id = c->param_id;
    robot->motor->ctrl_cmd.param_value = c->param_value;
    StoreMarkDirty();
  }
  /* W,turns100 */
  if (c->set_turns) {
    float t = Clamp(c->turns, 0.0f, DART_SPRING_MAX_TURNS);
    s_spring_turns = t;
    SpringSetTurns(t);
    StoreMarkDirty();
  }
  if (c->set_yaw_mode) {
    s_yaw_mode = c->yaw_mode;
    StoreMarkDirty();
  }
  if (c->set_aim_rpm) {
    s_aim_rpm = Clamp(c->aim_rpm, 1.0f, 300.0f);
    StoreMarkDirty();
  }
  if (c->vis_set) LauncherVisionSet(c->vis_x, c->vis_center);

  /* V,a[,b] 舵机 */
  if (c->servo_op) {
    switch (c->servo_op) {
      case LAUNCH_SV_OP_SET_STD:
        robot->servo->cmd.set_std = 1;
        robot->servo->cmd.std_deg = c->servo_arg1;
        s_sv_std = c->servo_arg1;
        StoreMarkDirty();
        break;
      case LAUNCH_SV_OP_SET_PREP:
        robot->servo->cmd.set_prep = 1;
        robot->servo->cmd.prep_deg = c->servo_arg1;
        s_sv_prep = c->servo_arg1;
        StoreMarkDirty();
        break;
      case LAUNCH_SV_OP_GO_STD:
        s_sv_state = LAUNCH_SERVO_STD;
        robot->servo->cmd.go_set = 1;
        robot->servo->cmd.go = LAUNCH_SERVO_STD;
        break;
      case LAUNCH_SV_OP_GO_PREP:
        s_sv_state = LAUNCH_SERVO_PREP;
        robot->servo->cmd.go_set = 1;
        robot->servo->cmd.go = LAUNCH_SERVO_PREP;
        break;
      case LAUNCH_SV_OP_SET_DEG:
        robot->servo->cmd.set_deg = 1;
        robot->servo->cmd.deg = c->servo_arg1;
        break;
      case LAUNCH_SV_OP_ZERO:
        robot->servo->cmd.zero = 1;
        StoreMarkDirty();
        break;
      default: break;
    }
  }

  /* G,cmd */
  switch (c->fsm_op) {
    case LAUNCH_FSM_OP_SPRING_ZERO: SpringSetTurns(0.0f); break;
    case LAUNCH_FSM_OP_SPRING_PREP: SpringSetTurns(s_spring_turns); break;
    case LAUNCH_FSM_OP_SERVO_STD:
      s_sv_state = LAUNCH_SERVO_STD;
      robot->servo->cmd.go_set = 1;
      robot->servo->cmd.go = LAUNCH_SERVO_STD;
      break;
    case LAUNCH_FSM_OP_SERVO_PREP:
      s_sv_state = LAUNCH_SERVO_PREP;
      robot->servo->cmd.go_set = 1;
      robot->servo->cmd.go = LAUNCH_SERVO_PREP;
      break;
    case LAUNCH_FSM_OP_AUTO_START:
      if (!s_estop) s_task_step = LAUNCH_TASK_SERVO_STD1;
      break;
    case LAUNCH_FSM_OP_AUTO_STOP: s_task_step = LAUNCH_TASK_IDLE; break;
    case LAUNCH_FSM_OP_ESTOP_ON:
      s_estop = 1;
      s_task_step = LAUNCH_TASK_IDLE;
      s_sv_state = LAUNCH_SERVO_STD;
      robot->servo->cmd.go_set = 1;
      robot->servo->cmd.go = LAUNCH_SERVO_STD;
      break;
    case LAUNCH_FSM_OP_ESTOP_OFF: s_estop = 0; s_task_step = LAUNCH_TASK_IDLE; break;
    default: break;
  }
}

void CtrlTask(void) {
  Launcher_Cmd_s c;
  if (LinkGetCmd(&c)) HandleCmd(&c);

  /* 失联保护: 收过数据后超时 -> 急停 + 停时序 */
  if (!LinkIsAlive()) {
    s_estop = 1;
    s_task_step = LAUNCH_TASK_IDLE;
  }

  if (s_estop) {
    for (int i = 0; i < LM_COUNT; i++) {
      s_mode[i] = LMODE_STOP;
      s_target[i] = 0.0f;
    }
  } else {
    AutoHandler();
    YawHandler();
  }

  /* 整帧写入 motor ctrl_cmd(持续型) */
  robot->motor->ctrl_cmd.estop = s_estop;
  for (int i = 0; i < LM_COUNT; i++) {
    robot->motor->ctrl_cmd.mode[i] = s_mode[i];
    robot->motor->ctrl_cmd.target[i] = s_target[i];
  }
}
