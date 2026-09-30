/*
 * trigger.c — 舵机 app 实现(复用 Modules/motor/servo_motor)
 * =============================================================================
 * 数据流: 订阅 "servo_cmd" -> 更新标准位/预备位设定 或 切换目标位
 *         -> ServoSetAngle() 给目标 -> ServoTask() 推进(速率限幅)
 *         -> 读回 当前角/目标角/脉宽 -> 发布 "servo_fb"
 *
 * 【分层】本 app 只 include servo_motor.h(module) 与 bsp_pwm.h(通过 module 间接),
 *   不 include dji_motor, 也不感知电机与状态机。
 * =============================================================================
 */
#include "trigger.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "launcher_cfg.h"
#include "message_center.h"
#include "servo_motor.h"
#include "tim.h" /* LAUNCH 舵机 PWM 定时器句柄(htim1) */

/* ============================== 舵机 PWM 通道 ============================== */
/* 与旧版 dart_launcher_web_v5_HIK 一致: TIM1 CH1, 50Hz(20ms) */
#define LAUNCH_SV_TIM (&htim1)
#define LAUNCH_SV_CHANNEL TIM_CHANNEL_1
#define LAUNCH_SV_PERIOD_S 0.02f

static ServoInstance *s_servo = NULL;

static Subscriber_t *s_sub_cmd = NULL;
static Publisher_t *s_pub_fb = NULL;

/* 设定值(可被上位机 V,0/V,1 修改) */
static float s_std_deg = LAUNCH_SV_DEG_STD;
static float s_prep_deg = LAUNCH_SV_DEG_PREP;
static uint8_t s_state = LAUNCH_SERVO_STD;

static Launcher_AppStatus_s s_st;
static Launcher_ServoFb_s s_fb;

/** @brief 该去的位对应的逻辑角 */
static float DegOfState(uint8_t state) {
  return (state == LAUNCH_SERVO_PREP) ? s_prep_deg : s_std_deg;
}

void Trigger_Init(void) {
  Servo_Init_Config_s cfg;
  memset(&cfg, 0, sizeof(cfg));

  /* PWM 通道 */
  cfg.pwm.htim = LAUNCH_SV_TIM;
  cfg.pwm.channel = LAUNCH_SV_CHANNEL;
  cfg.pwm.period = LAUNCH_SV_PERIOD_S;
  cfg.pwm.dutyratio = 0.0f;
  cfg.pwm.callback = NULL;
  cfg.pwm.id = NULL;

  /* 信号层: 由 servo_motor_cfg.h 的 SERVO_CFG_* 提供默认(PTK7350 规格) */
  cfg.center_us = SERVO_CFG_CENTER_US;
  cfg.half_us = SERVO_CFG_HALF_US;
  cfg.half_deg = SERVO_CFG_HALF_DEG;
  cfg.pulse_min_us = SERVO_CFG_PULSE_MIN_US;
  cfg.pulse_max_us = SERVO_CFG_PULSE_MAX_US;

  /* 逻辑层: 本工程的逐项标定(见 launcher_cfg.h) */
  cfg.scale = LAUNCH_SV_SCALE;
  cfg.trim_deg = LAUNCH_SV_TRIM_DEG;
  cfg.reverse = LAUNCH_SV_REVERSE;
  cfg.limit_deg = LAUNCH_SV_LIMIT_DEG;
  cfg.rate_limit_dps = LAUNCH_SV_RATE_DPS;
  cfg.zero_enable = 1;

  s_servo = ServoInit(&cfg);
  if (s_servo == NULL) {
    LOGERROR("[trigger] ServoInit FAIL");
  } else {
    LOGINFO("[trigger] servo ready (std=%.1f prep=%.1f)", s_std_deg, s_prep_deg);
  }

  s_state = LAUNCH_SERVO_STD;
  memset(&s_fb, 0, sizeof(s_fb));
  memset(&s_st, 0, sizeof(s_st));

  s_sub_cmd = SubRegister(TOPIC_SERVO_CMD, sizeof(Launcher_ServoCmd_s));
  s_pub_fb = PubRegister(TOPIC_SERVO_FB, sizeof(Launcher_ServoFb_s));
}

void Trigger_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  Launcher_ServoCmd_s cmd;
  uint16_t err = LAUNCH_ERR_NONE;

  if (s_servo == NULL) {
    s_st.hb++;
    s_st.err = LAUNCH_ERR_TRIGGER_OFF;
    return;
  }

  if (SubGetMessage(s_sub_cmd, &cmd)) {
    /* 更新标准位/预备位设定(V,0 / V,1) */
    if (cmd.set_std) s_std_deg = (float)cmd.std_deg;
    if (cmd.set_prep) s_prep_deg = (float)cmd.prep_deg;

    /* V,a[,b] 直接操作(标定/联调用) */
    switch (cmd.op) {
      case LAUNCH_SERVO_OP_SET_STD: s_std_deg = (float)cmd.op_arg; break;
      case LAUNCH_SERVO_OP_SET_PREP: s_prep_deg = (float)cmd.op_arg; break;
      case LAUNCH_SERVO_OP_GO_STD: s_state = LAUNCH_SERVO_STD; break;
      case LAUNCH_SERVO_OP_GO_PREP: s_state = LAUNCH_SERVO_PREP; break;
      case LAUNCH_SERVO_OP_SET_DEG:
        /* 直接给逻辑角: 绕过标准/预备位, 立即输出 */
        ServoSetAngle(s_servo, (float)cmd.op_arg);
        break;
      case LAUNCH_SERVO_OP_ZERO:
        if (!ServoZero(s_servo)) LOGWARNING("[trigger] ServoZero refused");
        break;
      default: break;
    }

    /* 调零请求(结构体标志位形式) */
    if (cmd.zero) {
      if (!ServoZero(s_servo)) LOGWARNING("[trigger] ServoZero refused");
    }

    if (cmd.estop) {
      /* 急停: 回标准位(舵机无"保持"概念, PWM 必须持续输出) */
      s_state = LAUNCH_SERVO_STD;
    } else if (cmd.state < 2) {
      s_state = cmd.state;
    }
  }

  /* 目标角 = 该位的设定值(急停时已强制为标准位)
   * 注意: V,4(直接给角) 已在上面直接输出, 此处仅处理标准/预备位。 */
  if (cmd.op != LAUNCH_SERVO_OP_SET_DEG) {
    ServoSetAngle(s_servo, DegOfState(s_state));
  }

  /* ★ 全局推进: 速率限幅在这里生效, 必须每周期调用 */
  ServoTask();

  /* ---- 反馈 ---- */
  s_fb.cur_deg = (int16_t)ServoGetAngle(s_servo);
  s_fb.target_deg = (int16_t)ServoGetTarget(s_servo);
  s_fb.pulse_us = (int16_t)ServoGetPulseUs(s_servo);
  s_fb.std_deg = (int16_t)s_std_deg;
  s_fb.prep_deg = (int16_t)s_prep_deg;
  s_fb.state = s_state;
  s_fb.tick = (uint32_t)DWT_GetTimeline_ms();
  PubPushMessage(s_pub_fb, &s_fb);

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = (float)s_fb.cur_deg; /* 关键量: 当前逻辑角 */
  s_st.err = err;
}

const Launcher_AppStatus_s *Trigger_GetStatus(void) { return &s_st; }
