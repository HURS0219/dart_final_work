/*
 * fsm.c — 发射时序状态机(直接驱动执行器版)
 * =============================================================================
 * 【为什么直接调用 Motortest_* 而不走 motor_cmd 话题】
 *   本文件原先把目标发到 "motor_cmd" 话题, 由 app/motor 消费。但当前调试阶段
 *   实际在跑的是 app/motortest(最小电机 app), 它不订阅该话题 —— 于是时序推了、
 *   电机却不动。为了让"状态机 + 电机"这条链最短、最可验证, 这里改为**直接调用
 *   Motortest_SetAngle/Stop/AtTarget**(与台架手动测试完全同一条代码路径)。
 *   舵机仍走 "servo_cmd" 话题(app/trigger 订阅它), 因为舵机侧一直是通的。
 *
 * 【流程(与旧版 dart_fsm.c 的 AutoHandler 一致)】
 *   SERVO_STD1   舵机->标准位          (等待 SERVO_SETTLE_MS)
 *   SPRING_STD1  拉簧A/B->零位          (等到位 或 超时)
 *   SPRING_PREP  拉簧A/B->预备位(上膛)  (等到位 或 超时)
 *   SERVO_PREP   舵机->预备位(待发)     (等待 SERVO_SETTLE_MS)
 *   SPRING_BACK  拉簧A/B->零位(释放)    (等到位 或 超时)
 *   SERVO_STD2   舵机->标准位(收尾)     (等待 SERVO_SETTLE_MS)
 *   DONE
 * 每一步都有 LAUNCH_STEP_TIMEOUT_MS 兜底超时; 急停可随时中断。
 *
 * 【安全】急停时不直接 DJIMotorStop 拉簧(会瞬间释放弹簧) —— 由 app/motortest
 *   的 Stop 语义决定; 当前 motortest 的 Stop 就是 DJIMotorStop, 所以急停路径
 *   只用于"确认弹簧已卸能"的场合。正式使用前需要按弹簧特性再定。
 * =============================================================================
 */
#include "fsm.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "launcher_cfg.h"
#include "message_center.h"
#include "motortest.h"
#include "robot_def.h"

/* ============================== 状态 ============================== */

static Subscriber_t *s_sub_cmd = NULL;   /* fsm_cmd:  start/stop/estop/单步 */
static Publisher_t *s_pub_scmd = NULL;   /* servo_cmd: 舵机目标 */
static Publisher_t *s_pub_fb = NULL;     /* fsm_fb:    当前步/完成标志 */

static uint8_t s_step = LAUNCH_TASK_IDLE;
static uint32_t s_step_t0 = 0;
static uint8_t s_active = 0; /* 1=自动流程进行中 */

/* 上一帧电平(边沿检测) */
static uint8_t s_prev_start = 0;
static uint8_t s_prev_stop = 0;
static uint8_t s_prev_spring_go = 0;

static Launcher_AppStatus_s s_st;
static Launcher_FsmFb_s s_fb_out;

/* ============================== 执行器抽象 ============================== */

/** @brief 拉簧 A/B 同时下发输出侧角度(deg) */
static void SpringSet(int16_t deg_a, int16_t deg_b) {
  Motortest_SetAngle(LAUNCH_M_SPRING_A, (float)deg_a);
  Motortest_SetAngle(LAUNCH_M_SPRING_B, (float)deg_b);
}

/** @brief 两路拉簧是否都已到位(容差判定在 motortest 内) */
static uint8_t SpringAtTarget(void) {
  return (Motortest_AtTarget(LAUNCH_M_SPRING_A) && Motortest_AtTarget(LAUNCH_M_SPRING_B)) ? 1u : 0u;
}

/** @brief 发一帧舵机目标(走话题, app/trigger 消费) */
static void ServoSet(uint8_t state) {
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

/** @brief 急停: 停自动流程 + 拉簧卸力 + 舵机回标准位
 * @note  仅在收到的 estop 电平为 1 时调用。 */
static void EnterEstop(void) {
  s_active = 0;
  s_step = LAUNCH_TASK_IDLE;
  Motortest_Stop(LAUNCH_M_SPRING_A);
  Motortest_Stop(LAUNCH_M_SPRING_B);
  ServoSet(LAUNCH_SERVO_STD);
}

/* ============================== 生命周期 ============================== */

void Fsm_Init(void) {
  memset(&s_st, 0, sizeof(s_st));
  memset(&s_fb_out, 0, sizeof(s_fb_out));

  s_step = LAUNCH_TASK_IDLE;
  s_active = 0;
  s_step_t0 = 0;
  s_prev_start = s_prev_stop = s_prev_spring_go = 0;

  s_sub_cmd = SubRegister(TOPIC_FSM_CMD, sizeof(Launcher_FsmCmd_s));
  s_pub_scmd = PubRegister(TOPIC_SERVO_CMD, sizeof(Launcher_ServoCmd_s));
  s_pub_fb = PubRegister(TOPIC_FSM_FB, sizeof(Launcher_FsmFb_s));
}

/* ============================== 周期任务 ============================== */

void Fsm_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  uint16_t err = LAUNCH_ERR_NONE;
  Launcher_FsmCmd_s fc;

  /* ---- 指令 ---- */
  if (SubGetMessage(s_sub_cmd, &fc)) {
    /* 急停: 电平触发(持续为 1 就持续停) */
    if (fc.estop) {
      EnterEstop();
    } else if (fc.estop_clr) {
      s_step = LAUNCH_TASK_IDLE;
    }

    /* 启动: 边沿(0->1) */
    if (fc.start && !s_prev_start && !s_active) {
      s_active = 1;
      s_step = LAUNCH_TASK_SERVO_STD1;
      s_step_t0 = now;
      LOGINFO("[fsm] AUTO START -> step=%d", (int)s_step);
    }
    /* 停止: 边沿(0->1) */
    if (fc.stop && !s_prev_stop) {
      s_active = 0;
      s_step = LAUNCH_TASK_IDLE;
    }
    /* 单步上膛/释放: spring_go 边沿触发(避开与 cmd 抢写) */
    if (!s_active && fc.spring_go == 1 && s_prev_spring_go == 0) {
      SpringSet(LAUNCH_STEP_SPRING_PREP_A_DEG, LAUNCH_STEP_SPRING_PREP_B_DEG);
    } else if (!s_active && fc.spring_go == 0 && s_prev_spring_go == 1) {
      SpringSet(LAUNCH_STEP_SPRING_STD1_A_DEG, LAUNCH_STEP_SPRING_STD1_B_DEG);
    }

    s_prev_start = fc.start;
    s_prev_stop = fc.stop;
    s_prev_spring_go = fc.spring_go;
  }

  /* ---- 流程推进 ---- */
  if (s_active) {
    uint8_t timeout = ((now - s_step_t0) > LAUNCH_STEP_TIMEOUT_MS) ? 1u : 0u;

    /* 【诊断】每次步号变化时打一条日志, 便于在 RTT 上确认时序真的在推进。
     * 用 static 记录上一步; 只在变化时打印, 不会刷屏。 */
    {
      static uint8_t last_step = 0xFF;
      if (s_step != last_step) {
        LOGINFO("[fsm] step %d -> %d (t0=%u now=%u)", last_step, s_step, s_step_t0, now);
        last_step = s_step;
      }
    }

    switch (s_step) {
      case LAUNCH_TASK_SERVO_STD1: /* 舵机 -> 标准位(先把发射机构让开) */
        ServoSet(LAUNCH_SERVO_STD);
        if (timeout || (now - s_step_t0) > LAUNCH_SERVO_SETTLE_MS) {
          s_step = LAUNCH_TASK_SPRING_STD1;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SPRING_STD1: /* 拉簧 -> 零位(确认起点) */
        SpringSet(LAUNCH_STEP_SPRING_STD1_A_DEG, LAUNCH_STEP_SPRING_STD1_B_DEG);
        if (SpringAtTarget() || timeout) {
          s_step = LAUNCH_TASK_SPRING_PREP;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SPRING_PREP: /* 拉簧 -> 预备位(上膛, 储能) */
        SpringSet(LAUNCH_STEP_SPRING_PREP_A_DEG, LAUNCH_STEP_SPRING_PREP_B_DEG);
        if (SpringAtTarget() || timeout) {
          s_step = LAUNCH_TASK_SERVO_PREP;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SERVO_PREP: /* 舵机 -> 预备位(待发) */
        ServoSet(LAUNCH_SERVO_PREP);
        if (timeout || (now - s_step_t0) > LAUNCH_SERVO_SETTLE_MS) {
          s_step = LAUNCH_TASK_SPRING_BACK;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SPRING_BACK: /* 拉簧 -> 零位(释放, 发射) */
        SpringSet(LAUNCH_STEP_SPRING_BACK_A_DEG, LAUNCH_STEP_SPRING_BACK_B_DEG);
        if (SpringAtTarget() || timeout) {
          s_step = LAUNCH_TASK_SERVO_STD2;
          s_step_t0 = now;
        }
        break;

      case LAUNCH_TASK_SERVO_STD2: /* 舵机 -> 标准位(收尾) */
        ServoSet(LAUNCH_SERVO_STD);
        if (timeout || (now - s_step_t0) > LAUNCH_SERVO_SETTLE_MS) {
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

    /* 单步超时告警(供 Monitor 汇总) */
    if (timeout && s_step != LAUNCH_TASK_DONE) err |= LAUNCH_ERR_FSM_OFF;
  }

  /* ---- 发布 ---- */
  s_fb_out.task_step = s_step;
  s_fb_out.done = (s_step == LAUNCH_TASK_DONE) ? 1u : 0u;
  s_fb_out.error = (uint8_t)(err & 0xFF);
  s_fb_out.tick = now;
  PubPushMessage(s_pub_fb, &s_fb_out);

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = (float)s_step;
  s_st.err = err;
}

const Launcher_AppStatus_s *Fsm_GetStatus(void) { return &s_st; }
