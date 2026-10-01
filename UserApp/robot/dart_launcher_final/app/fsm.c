/*
 * fsm.c — 时序 app 实现(自动发射流程状态机)
 * =============================================================================
 * 流程(与旧版 dart_fsm 一致, 便于对照):
 *
 *   IDLE ──start──> SERVO_STD1 ──> SPRING_STD1 ──> SPRING_PREP ──> SERVO_PREP
 *                                                       │
 *                                                       v
 *                                    DONE <── SERVO_STD2 <── SPRING_BACK
 *
 * 每步的"到位"判据:
 *   - 舵机步: 固定延时 LAUNCH_SERVO_SETTLE_MS(舵机无位置反馈)
 *   - 拉簧步: motor_fb 里 A/B 两路都 at_target==1 且且切到角度模式
 *   - 兜底  : 单步超过 LAUNCH_STEP_TIMEOUT_MS 强制推进(防止卡死)
 *
 * 【重要】本 app 发布 motor_cmd 时**只写拉簧 A/B 的目标**, 丝杆与 yaw 的
 *   字段填"不改变/停止"以避开争用 —— 实际上本 app 只负责把指令发出去,
 *   真正的仲裁在 app/motor 里按"谁最后写谁生效"。为避免与 app/cmd 抢写同一
 *   话题, 本 app 仅在自己的流程激活(非 IDLE/DONE)时发布。
 * =============================================================================
 */
#include "fsm.h"

#include <string.h>

#include "bsp_dwt.h"
#include "launcher_cfg.h"
#include "message_center.h"

static Subscriber_t *s_sub_cmd = NULL;
static Subscriber_t *s_sub_mf = NULL;
static Subscriber_t *s_sub_sf = NULL;
static Publisher_t *s_pub_mcmd = NULL;
static Publisher_t *s_pub_scmd = NULL;
static Publisher_t *s_pub_fb = NULL;

static uint8_t s_step = LAUNCH_TASK_IDLE;
static uint32_t s_step_t0 = 0;
static uint8_t s_active = 0; /* 1=流程进行中 */

/* 上一帧的边沿检测状态(见 Fsm_Task 中 spring_go 的说明) */
static uint8_t s_prev_start = 0;
static uint8_t s_prev_stop = 0;
static uint8_t s_prev_spring_go = 0;
static uint8_t s_prev_servo_go = LAUNCH_SERVO_STD;

/* 最近一次反馈 */
static Launcher_MotorFb_s s_mf;
static Launcher_ServoFb_s s_sf;
static uint8_t s_mf_valid = 0;

static Launcher_AppStatus_s s_st;
static Launcher_FsmFb_s s_fb_out;

/** @brief 拉簧 A/B 是否都已到位 */
static uint8_t SpringAtTarget(void) {
  if (!s_mf_valid) return 0;
  return (s_mf.at_target[LAUNCH_M_SPRING_A] && s_mf.at_target[LAUNCH_M_SPRING_B]) ? 1u : 0u;
}

/** @brief 发一帧拉簧目标(逐路独立角度, 丝杆/yaw 保持不动) */
static void PublishSpring(int16_t deg_a, int16_t deg_b) {
  Launcher_MotorCmd_s m;
  memset(&m, 0, sizeof(m));
  m.mode[LAUNCH_M_SPRING_A] = LAUNCH_MODE_ANGLE;
  m.target[LAUNCH_M_SPRING_A] = (float)deg_a;
  m.mode[LAUNCH_M_SPRING_B] = LAUNCH_MODE_ANGLE;
  m.target[LAUNCH_M_SPRING_B] = (float)deg_b;
  /* 丝杆与 yaw 不在本 app 职责内: 用 STOP 表示"不要动"会误伤, 故保持 ANGLE+当前值
   * 不可行(本 app 不知道其当前值)。折中: 仅发布时把它们标为 STOP, 由 app/motor
   * 的"最后写入生效"语义决定; 由于本 app 只在流程激活时发布, 影响可控。
   * 注: 丝杆的 STOP 在 app/motor 里被解释为"就地保持"(自锁丝杆会卸力, 见其策略)。 */
  m.mode[LAUNCH_M_SCREW] = LAUNCH_MODE_STOP;
  m.tick = (uint32_t)DWT_GetTimeline_ms();
  PubPushMessage(s_pub_mcmd, &m);
}

/** @brief 发一帧舵机目标 */
static void PublishServo(uint8_t state) {
  Launcher_ServoCmd_s s;
  memset(&s, 0, sizeof(s));
  s.state = state;
  s.std_deg = (int16_t)LAUNCH_STEP_SERVO_STD1_DEG;
  s.prep_deg = (int16_t)LAUNCH_STEP_SERVO_PREP_DEG;
  s.target_deg = (int16_t)((state == LAUNCH_SERVO_PREP) ? LAUNCH_STEP_SERVO_PREP_DEG
                                                        : LAUNCH_STEP_SERVO_STD1_DEG);
  s.tick = (uint32_t)DWT_GetTimeline_ms();
  PubPushMessage(s_pub_scmd, &s);
}

void Fsm_Init(void) {
  memset(&s_st, 0, sizeof(s_st));
  memset(&s_fb_out, 0, sizeof(s_fb_out));
  memset(&s_mf, 0, sizeof(s_mf));
  memset(&s_sf, 0, sizeof(s_sf));

  s_step = LAUNCH_TASK_IDLE;
  s_active = 0;

  s_sub_cmd = SubRegister(TOPIC_FSM_CMD, sizeof(Launcher_FsmCmd_s));
  s_sub_mf = SubRegister(TOPIC_MOTOR_FB, sizeof(Launcher_MotorFb_s));
  s_sub_sf = SubRegister(TOPIC_SERVO_FB, sizeof(Launcher_ServoFb_s));
  s_pub_mcmd = PubRegister(TOPIC_MOTOR_CMD, sizeof(Launcher_MotorCmd_s));
  s_pub_scmd = PubRegister(TOPIC_SERVO_CMD, sizeof(Launcher_ServoCmd_s));
  s_pub_fb = PubRegister(TOPIC_FSM_FB, sizeof(Launcher_FsmFb_s));
}

void Fsm_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  uint16_t err = LAUNCH_ERR_NONE;
  Launcher_FsmCmd_s fc;
  uint8_t got_cmd = 0;

  /* ---- 输入 ---- */
  {
    Launcher_MotorFb_s m;
    Launcher_ServoFb_s s;
    if (SubGetMessage(s_sub_mf, &m)) {
      s_mf = m;
      s_mf_valid = 1;
    }
    if (SubGetMessage(s_sub_sf, &s)) s_sf = s;
  }

  if (SubGetMessage(s_sub_cmd, &fc)) {
    got_cmd = 1;

    /* 急停: 立即停流程 */
    if (fc.estop) {
      s_active = 0;
      s_step = LAUNCH_TASK_IDLE;
    }
    if (fc.estop_clr) {
      s_step = LAUNCH_TASK_IDLE;
    }
    /* 启动/停止(边沿: 只在收到跳变时动作) */
    if (fc.start && !s_prev_start && !s_active) {
      s_active = 1;
      s_step = LAUNCH_TASK_SERVO_STD1;
      s_step_t0 = now;
    }
    if (fc.stop && !s_prev_stop) {
      s_active = 0;
      s_step = LAUNCH_TASK_IDLE;
    }
    /* 单步(手动): spring_go 是"边沿触发"请求 —— 只在 0->1 / 1->0 跳变时执行一次。
     * 【曾经的问题】若按电平判断(fc.spring_go 直接当条件), 由于 cmd.c 每周期都发布
     *   一帧 fsm_cmd(未设置时该字段为 0), 该分支会被反复触发, 进而每周期都发布
     *   motor_cmd 覆盖掉 app/cmd 设定的目标, 表现为"N/M 命令发了但电机不动"。
     *   用边沿判定即可避免与 cmd 抢写。 */
    if (!s_active && fc.spring_go == 1 && s_prev_spring_go == 0) {
      PublishSpring(LAUNCH_STEP_SPRING_PREP_A_DEG, LAUNCH_STEP_SPRING_PREP_B_DEG);
    } else if (!s_active && fc.spring_go == 0 && s_prev_spring_go == 1) {
      PublishSpring(LAUNCH_STEP_SPRING_STD1_A_DEG, LAUNCH_STEP_SPRING_STD1_B_DEG);
    }
    s_prev_start = fc.start;
    s_prev_stop = fc.stop;
    s_prev_spring_go = fc.spring_go;
    s_prev_servo_go = fc.servo_go;
  }

  /* ---- 推进 ---- */
  if (s_active) {
    uint8_t timeout = ((now - s_step_t0) > LAUNCH_STEP_TIMEOUT_MS) ? 1u : 0u;

    switch (s_step) {
      case LAUNCH_TASK_SERVO_STD1:
        PublishServo(LAUNCH_SERVO_STD);
        if (timeout) {
          s_step = LAUNCH_TASK_SPRING_STD1;
          s_step_t0 = now;
        } else if ((now - s_step_t0) > LAUNCH_SERVO_SETTLE_MS) {
          s_step = LAUNCH_TASK_SPRING_STD1;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SPRING_STD1:
        PublishSpring(LAUNCH_STEP_SPRING_STD1_A_DEG, LAUNCH_STEP_SPRING_STD1_B_DEG);
        if (SpringAtTarget() || timeout) {
          s_step = LAUNCH_TASK_SPRING_PREP;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SPRING_PREP:
        PublishSpring(LAUNCH_STEP_SPRING_PREP_A_DEG, LAUNCH_STEP_SPRING_PREP_B_DEG);
        if (SpringAtTarget() || timeout) {
          s_step = LAUNCH_TASK_SERVO_PREP;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SERVO_PREP:
        PublishServo(LAUNCH_SERVO_PREP);
        if ((now - s_step_t0) > LAUNCH_SERVO_SETTLE_MS || timeout) {
          s_step = LAUNCH_TASK_SPRING_BACK;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SPRING_BACK:
        PublishSpring(LAUNCH_STEP_SPRING_BACK_A_DEG, LAUNCH_STEP_SPRING_BACK_B_DEG);
        if (SpringAtTarget() || timeout) {
          s_step = LAUNCH_TASK_SERVO_STD2;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SERVO_STD2:
        PublishServo(LAUNCH_SERVO_STD);
        if ((now - s_step_t0) > LAUNCH_SERVO_SETTLE_MS || timeout) {
          s_step = LAUNCH_TASK_DONE;
          s_active = 0;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_DONE:
      case LAUNCH_TASK_IDLE:
      default:
        s_active = 0;
        break;
    }
  }

  /* ---- 发布状态 ---- */
  s_fb_out.task_step = s_step;
  s_fb_out.done = (s_step == LAUNCH_TASK_DONE) ? 1u : 0u;
  s_fb_out.error = 0;
  s_fb_out.tick = now;
  PubPushMessage(s_pub_fb, &s_fb_out);

  (void)got_cmd;
  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = (float)s_step; /* 关键量: 当前时序步 */
  s_st.err = err;
}

const Launcher_AppStatus_s *Fsm_GetStatus(void) { return &s_st; }
