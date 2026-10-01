/*
 * cmd.c — 大脑 app 实现
 * =============================================================================
 * 数据流: launch_cmd(操作员指令) + 各 *_fb(反馈)
 *         -> 翻译成 motor_cmd / servo_cmd / yaw_cmd / fsm_cmd
 *         -> 汇总发布 launch_state(供 link 组遥测帧)
 *
 * 设计要点:
 *   1) 本 app 是**唯一**决定"某机构该到哪"的地方(除 fsm 的自动时序外);
 *   2) 所有角度取自 launcher_cfg.h, 不硬编码;
 *   3) 急停是"保持型": 一旦置位, 持续向 motor_cmd 传 estop=1, 直到显式解除。
 *      注意拉簧的急停策略在 app/motor 里(锁位保持+缓慢归零), 本 app 只传意图。
 * =============================================================================
 */
#include "cmd.h"

#include <string.h>

#include "bsp_dwt.h"
#include "launcher_cfg.h"
#include "message_center.h"

static Subscriber_t *s_sub_cmd = NULL;
static Subscriber_t *s_sub_mf = NULL;
static Subscriber_t *s_sub_sf = NULL;
static Subscriber_t *s_sub_yf = NULL;
static Subscriber_t *s_sub_ff = NULL;

static Publisher_t *s_pub_mcmd = NULL;
static Publisher_t *s_pub_scmd = NULL;
static Publisher_t *s_pub_ycmd = NULL;
static Publisher_t *s_pub_fcmd = NULL;
static Publisher_t *s_pub_state = NULL;
static Publisher_t *s_pub_vcmd = NULL;

/* ---- 状态 ---- */
static uint8_t s_estop = 0;
static uint8_t s_estop_clr_cnt = 0; /* >0 时连续发 estop_clr(防单周期脉冲丢失) */
static uint8_t s_yaw_mode = LAUNCH_YAW_MANUAL;
static float s_aim_rpm = LAUNCH_YAW_AIM_RPM;
static uint32_t s_last_cmd_ms = 0;
static uint8_t s_link_ok = 0;

/* 拉簧预备位设定(可由 N 命令逐路修改; 默认取 cfg) */
static int16_t s_prep_a = LAUNCH_SA_DEG_PREP;
static int16_t s_prep_b = LAUNCH_SB_DEG_PREP;

/* 舵机设定 */
static int16_t s_sv_std = LAUNCH_SV_DEG_STD;
static int16_t s_sv_prep = LAUNCH_SV_DEG_PREP;
static uint8_t s_sv_state = LAUNCH_SERVO_STD;

/* 时序反馈镜像 */
static uint8_t s_task_step = LAUNCH_TASK_IDLE;

/* 逐路"保持型"电机目标。
 * 【为什么需要】若 Cmd_Task 每周期都把 mcmd 重置为 STOP, 那么 N/M 命令设定的
 *   目标只生效一个周期, 下一周期就被本函数覆盖成 STOP —— 表现为
 *   "命令发了但电机不动"。故用 s_hold_* 记住"当前应保持的逐路目标",
 *   每周期整帧发出, 只在收到新指令时才更新。上电默认全部 STOP。 */
static uint8_t s_hold_mode[LAUNCH_M_COUNT];
static float s_hold_target[LAUNCH_M_COUNT];

static Launcher_AppStatus_s s_st;
static Launcher_State_s s_state;

void Cmd_Init(void) {
  memset(&s_st, 0, sizeof(s_st));
  memset(&s_state, 0, sizeof(s_state));

  s_estop = 0;
  s_estop_clr_cnt = 0;
  s_yaw_mode = LAUNCH_YAW_MANUAL;
  s_aim_rpm = LAUNCH_YAW_AIM_RPM;
  s_last_cmd_ms = 0;
  s_link_ok = 0;
  s_prep_a = LAUNCH_SA_DEG_PREP;
  s_prep_b = LAUNCH_SB_DEG_PREP;
  s_sv_std = LAUNCH_SV_DEG_STD;
  s_sv_prep = LAUNCH_SV_DEG_PREP;
  s_sv_state = LAUNCH_SERVO_STD;
  s_task_step = LAUNCH_TASK_IDLE;

  /* 逐路目标上电默认 STOP(不驱动); 之后由 N/M/G 命令接管 */
  for (int i = 0; i < LAUNCH_M_COUNT; i++) {
    s_hold_mode[i] = LAUNCH_MODE_STOP;
    s_hold_target[i] = 0.0f;
  }

  s_sub_cmd = SubRegister(TOPIC_LAUNCH_CMD, sizeof(Launcher_Cmd_s));
  s_sub_mf = SubRegister(TOPIC_MOTOR_FB, sizeof(Launcher_MotorFb_s));
  s_sub_sf = SubRegister(TOPIC_SERVO_FB, sizeof(Launcher_ServoFb_s));
  s_sub_yf = SubRegister(TOPIC_YAW_FB, sizeof(Launcher_YawFb_s));
  s_sub_ff = SubRegister(TOPIC_FSM_FB, sizeof(Launcher_FsmFb_s));

  s_pub_mcmd = PubRegister(TOPIC_MOTOR_CMD, sizeof(Launcher_MotorCmd_s));
  s_pub_scmd = PubRegister(TOPIC_SERVO_CMD, sizeof(Launcher_ServoCmd_s));
  s_pub_ycmd = PubRegister(TOPIC_YAW_CMD, sizeof(Launcher_YawCmd_s));
  s_pub_fcmd = PubRegister(TOPIC_FSM_CMD, sizeof(Launcher_FsmCmd_s));
  s_pub_state = PubRegister(TOPIC_LAUNCH_STATE, sizeof(Launcher_State_s));
  s_pub_vcmd = PubRegister(TOPIC_VISION_CMD, sizeof(Launcher_VisCmd_s));
}

/** @brief 把操作员指令翻译成一次性的电机请求(取零/复位/换向) */
static void HandleMotorRequests(const Launcher_Cmd_s *c, Launcher_MotorCmd_s *m) {
  if (c->req_zero) {
    if (c->req_zero_slot < 0) {
      for (int i = 0; i < LAUNCH_M_COUNT; i++) m->zero_req[i] = 1;
    } else if (c->req_zero_slot < LAUNCH_M_COUNT) {
      m->zero_req[(int)c->req_zero_slot] = 1;
    }
  }
  if (c->req_reset && c->req_reset_slot >= 0 && c->req_reset_slot < LAUNCH_M_COUNT) {
    m->reset_req[(int)c->req_reset_slot] = 1;
  }
  if (c->req_dir && c->req_dir_slot >= 0 && c->req_dir_slot < LAUNCH_M_COUNT) {
    m->dir_req[(int)c->req_dir_slot] = 1;
  }
}

void Cmd_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  uint16_t err = LAUNCH_ERR_NONE;

  Launcher_Cmd_s c;
  Launcher_MotorCmd_s mcmd;
  Launcher_ServoCmd_s scmd;
  Launcher_YawCmd_s ycmd;
  Launcher_FsmCmd_s fcmd;
  Launcher_VisCmd_s vcmd;

  memset(&mcmd, 0, sizeof(mcmd));
  memset(&scmd, 0, sizeof(scmd));
  memset(&ycmd, 0, sizeof(ycmd));
  memset(&fcmd, 0, sizeof(fcmd));
  memset(&vcmd, 0, sizeof(vcmd));

  /* 【重要】电机目标必须"保持型", 不能每周期重置为 STOP。
   * 原因: 若这里每次都填 STOP, 那么 N/M 命令设定的目标只生效一个周期, 下一周期
   *       就被本函数覆盖成 STOP, 表现为"命令发了但电机不动"。
   * 做法: 用 s_hold_* 记录"当前应保持的逐路目标", 每周期把它整帧发出;
   *       只有收到新指令时才更新它。 */
  for (int i = 0; i < LAUNCH_M_COUNT; i++) {
    mcmd.mode[i] = s_hold_mode[i];
    mcmd.target[i] = s_hold_target[i];
  }

  /* ---- 取反馈 ---- */
  {
    Launcher_MotorFb_s mf;
    Launcher_ServoFb_s sf;
    Launcher_YawFb_s yf;
    Launcher_FsmFb_s ff;
    if (SubGetMessage(s_sub_mf, &mf)) { /* 仅用于判活/透传, 本 app 不做闭环 */ }
    if (SubGetMessage(s_sub_sf, &sf)) {
      s_sv_std = sf.std_deg;
      s_sv_prep = sf.prep_deg;
    }
    if (SubGetMessage(s_sub_yf, &yf)) { /* 透传给 state */ }
    if (SubGetMessage(s_sub_ff, &ff)) s_task_step = ff.task_step;
  }

  /* ---- 指令处理 ---- */
  if (SubGetMessage(s_sub_cmd, &c)) {
    s_last_cmd_ms = now;
    s_link_ok = 1;

    HandleMotorRequests(&c, &mcmd);

    /* PING: 由 link 直接回 PONG, 这里无需处理(req_ping 仅供 link 判断) */

    /* ---- 逐路目标 ---- */
    /* M,slot,mode,value : 直接设定(角度模式下 value 为输出侧 deg) */
    if (c.set_motor && c.motor_slot >= 0 && c.motor_slot < LAUNCH_M_COUNT) {
      s_hold_mode[(int)c.motor_slot] = c.motor_mode;
      s_hold_target[(int)c.motor_slot] = c.motor_target;
    }

    /* N,slot,deg : 设某路目标角度(取代旧 W) */
    if (c.set_angle && c.angle_slot >= 0 && c.angle_slot < LAUNCH_M_COUNT) {
      int slot = c.angle_slot;
      s_hold_mode[slot] = LAUNCH_MODE_ANGLE;
      s_hold_target[slot] = (float)c.angle_deg;
      /* 同步更新"预备位"记忆, 便于 N 之后状态机沿用 */
      if (slot == LAUNCH_M_SPRING_A) s_prep_a = c.angle_deg;
      if (slot == LAUNCH_M_SPRING_B) s_prep_b = c.angle_deg;
    }

    /* P,slot,id,value : 参数设定 -> 经 motor_cmd 转发给 app/motor 执行 */
    if (c.set_param && c.param_slot >= 0 && c.param_slot < LAUNCH_M_COUNT) {
      mcmd.set_param = 1;
      mcmd.param_slot = c.param_slot;
      mcmd.param_id = c.param_id;
      mcmd.param_value = c.param_value;
    }

    /* Y,mode */
    if (c.set_yaw_mode) s_yaw_mode = c.yaw_mode;
    /* A,rpm */
    if (c.set_aim_rpm) s_aim_rpm = c.aim_rpm;

    /* V,a[,b] 舵机操作 -> 经 servo_cmd 转发给 app/trigger */
    if (c.servo_op) {
      scmd.op = c.servo_op;
      scmd.op_arg = (int16_t)c.servo_arg1;
    }

    /* C,x,center 视觉注入 -> 转给 vision */
    if (c.vis_op) {
      vcmd.inject = 1;
      vcmd.x = c.vis_x;
      vcmd.center = c.vis_center;
      vcmd.tick = now;
      PubPushMessage(s_pub_vcmd, &vcmd);
    }

    /* G,cmd 时序/急停 */
    if (c.fsm_op == LAUNCH_FSM_OP_ESTOP_ON || c.fsm_op == LAUNCH_FSM_OP_ESTOP_OFF ||
        c.fsm_op == LAUNCH_FSM_OP_AUTO_START || c.fsm_op == LAUNCH_FSM_OP_AUTO_STOP ||
        c.fsm_op == LAUNCH_FSM_OP_SPRING_ZERO || c.fsm_op == LAUNCH_FSM_OP_SPRING_PREP ||
        c.fsm_op == LAUNCH_FSM_OP_SERVO_STD || c.fsm_op == LAUNCH_FSM_OP_SERVO_PREP) {
      switch (c.fsm_op) {
        case LAUNCH_FSM_OP_ESTOP_ON:
          s_estop = 1;
          s_estop_clr_cnt = 0;
          break;
        case LAUNCH_FSM_OP_ESTOP_OFF:
          s_estop = 0;
          /* estop_clr 是单周期脉冲, 而 message_center 队列深度仅 1, 有被后续帧
           * 覆盖而丢失的风险(会导致急停解不掉) -> 连续发若干帧确保 motor 侧收到。 */
          s_estop_clr_cnt = 20;
          break;
        case LAUNCH_FSM_OP_AUTO_START: fcmd.start = 1; break;
        case LAUNCH_FSM_OP_AUTO_STOP: fcmd.stop = 1; break;
        case LAUNCH_FSM_OP_SPRING_ZERO:
          s_hold_mode[LAUNCH_M_SPRING_A] = LAUNCH_MODE_ANGLE;
          s_hold_target[LAUNCH_M_SPRING_A] = (float)LAUNCH_SA_DEG_ZERO;
          s_hold_mode[LAUNCH_M_SPRING_B] = LAUNCH_MODE_ANGLE;
          s_hold_target[LAUNCH_M_SPRING_B] = (float)LAUNCH_SB_DEG_ZERO;
          break;
        case LAUNCH_FSM_OP_SPRING_PREP:
          s_hold_mode[LAUNCH_M_SPRING_A] = LAUNCH_MODE_ANGLE;
          s_hold_target[LAUNCH_M_SPRING_A] = (float)s_prep_a;
          s_hold_mode[LAUNCH_M_SPRING_B] = LAUNCH_MODE_ANGLE;
          s_hold_target[LAUNCH_M_SPRING_B] = (float)s_prep_b;
          break;
        case LAUNCH_FSM_OP_SERVO_STD: s_sv_state = LAUNCH_SERVO_STD; break;
        case LAUNCH_FSM_OP_SERVO_PREP: s_sv_state = LAUNCH_SERVO_PREP; break;
        default: break;
      }
    }
  }

  /* ---- 链路超时: 失联降级 ---- */
  if (s_last_cmd_ms != 0 && (now - s_last_cmd_ms) > LAUNCH_LINK_TIMEOUT_MS) {
    if (s_link_ok) {
      s_link_ok = 0;
      fcmd.stop = 1; /* 停自动流程 */
      /* 注意: 不触发 estop, 因为 estop 会让拉簧进入"缓慢归零"(主动动作)。
       * 失联时更保守的做法是"就地保持": 拉簧继续顶住当前位置。
       * 通过把目标设为"不改变"无法表达, 故这里让 motor 维持最后目标即可
       * (不给新 motor_cmd 时, app/motor 内部会沿用上一次的目标/保持)。 */
    }
  }

  /* ---- 急停: 保持型, 每周期都传 ---- */
  if (s_estop) {
    mcmd.estop = 1;
    scmd.estop = 1;
    ycmd.estop = 1;
    /* 急停时不要再下发运动目标, 否则会与保持策略打架 */
    for (int i = 0; i < LAUNCH_M_COUNT; i++) mcmd.mode[i] = LAUNCH_MODE_STOP;
  }

  /* 解除急停: 连续若干帧重复发送, 防止单周期脉冲被队列覆盖丢失 */
  if (s_estop_clr_cnt > 0) {
    s_estop_clr_cnt--;
    mcmd.estop_clr = 1;
    fcmd.estop_clr = 1;
  }

  /* ---- 舵机 ---- */
  scmd.std_deg = s_sv_std;
  scmd.prep_deg = s_sv_prep;
  scmd.state = s_sv_state;
  scmd.target_deg = (s_sv_state == LAUNCH_SERVO_PREP) ? s_sv_prep : s_sv_std;
  if (scmd.op == 0) scmd.op = LAUNCH_SERVO_OP_NONE;

  /* ---- yaw ---- */
  ycmd.mode = s_yaw_mode;
  ycmd.manual_deg = 0;
  ycmd.guide_deg = LAUNCH_YAW_GUIDE_DEG;
  ycmd.aim_rpm = s_aim_rpm;
  ycmd.estop = s_estop;

  /* ---- 发布 ---- */
  mcmd.tick = now;
  scmd.tick = now;
  ycmd.tick = now;
  fcmd.tick = now;
  PubPushMessage(s_pub_mcmd, &mcmd);
  PubPushMessage(s_pub_scmd, &scmd);
  PubPushMessage(s_pub_ycmd, &ycmd);
  PubPushMessage(s_pub_fcmd, &fcmd);

  /* ---- 整机状态 ---- */
  s_state.estop = s_estop;
  s_state.task_step = s_task_step;
  s_state.yaw_mode = s_yaw_mode;
  s_state.spring_a_deg = s_prep_a;
  s_state.spring_b_deg = s_prep_b;
  s_state.aim_rpm = s_aim_rpm;
  s_state.link_ok = s_link_ok;
  s_state.tick = now;
  PubPushMessage(s_pub_state, &s_state);

  (void)err;
  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = (float)s_task_step; /* 关键量: 时序步 */
  s_st.err = LAUNCH_ERR_NONE;
}

const Launcher_AppStatus_s *Cmd_GetStatus(void) { return &s_st; }
