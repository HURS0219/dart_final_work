/*
 * guidance.c — 制导+控制 app 实现
 * =============================================================================
 * 数据流:  订阅 "attitude"/"target"
 *          -> 由目标像素求水平视线角 λ_yaw = (x-CX)/focal
 *          -> 内部微分 + 一阶低通得到 dλ_yaw (png_ai 只吃 dλ)
 *          -> png_ai 解算横向加速度 a_cmd (m/s^2)
 *          -> 控制: mix.yaw = k*a_cmd;  mix.roll = roll_PID(姿态);  mix.pitch = 0
 *          -> 发布 "mix"
 *
 * 说明: 本机不控俯仰(目标水平移动), 只做“yaw 制导 + roll 稳定”。
 *       k 建议取物理值 1/PNG_MAX_OUT; “过偏”调 SERVO_MIX_MAX_DEG 而非 k (见 README)。
 * =============================================================================
 */
#include "guidance.h"

#include <string.h>

#include "bsp_dwt.h"
#include "controller.h"
#include "dart_final_cfg.h"
#include "message_center.h"
#include "png.h"
#include "robot_def.h"
#include "adrc_ai.h" /* 自抗扰 roll 控制器(adrc 分支) */

static PNGInstance s_png;
static PIDInstance s_roll_pid; /* 传统 PID 稳滚(ROLL_CTRL_MODE=0) */
static ADRCInstance s_adrc;    /* 自抗扰稳滚(adrc 分支, ROLL_CTRL_MODE=2) */

static Subscriber_t *s_sub_att = NULL;
static Subscriber_t *s_sub_tgt = NULL;
static Publisher_t *s_pub_mix = NULL;

static Dart_Attitude_s s_att;
static Dart_Target_s s_tgt;
static Dart_Mix_s s_mix;

static uint8_t s_los_started = 0;
static float s_los_prev = 0.0f; /* 上次视线角 */
static float s_dlambda = 0.0f;  /* 滤波后的视线角速率 */
static Dart_AppStatus_s s_st;

static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void Guidance_Init(void) {
  PNG_Init_Config_s pc = {
      .mode = PNG_DEFAULT_MODE,
      .vel_src = PNG_DEFAULT_VEL_SRC,
      .N = PNG_DEFAULT_N,
      .MaxOut = PNG_DEFAULT_MAX_OUT,
      .DeadBand = PNG_DEFAULT_DEADBAND,
      .a_o_gain = PNG_DEFAULT_AO_GAIN,
  };
  PID_Init_Config_s rc;

  PNGInit(&s_png, &pc);

  memset(&rc, 0, sizeof(rc));
  rc.Kp = ROLL_KP;
  rc.Ki = ROLL_KI;
  rc.Kd = ROLL_KD;
  rc.MaxOut = ROLL_CMD_LIMIT;
  PIDInit(&s_roll_pid, &rc);

  /* ADRC 初始化(adrc 分支): 默认值来自 adrc_cfg.h; 现场只需改 adrc_cfg.h */
  {
    ADRC_Init_Config_s ac = {.wo = ADRC_DEFAULT_WO,
                             .wc = ADRC_DEFAULT_WC,
                             .b0 = ADRC_DEFAULT_B0,
                             .max_out = ADRC_DEFAULT_MAXOUT,
                             .zeta = ADRC_DEFAULT_ZETA};
    ADRCInit(&s_adrc, &ac);
  }

  s_sub_att = SubRegister(TOPIC_ATTITUDE, sizeof(Dart_Attitude_s));
  s_sub_tgt = SubRegister(TOPIC_TARGET, sizeof(Dart_Target_s));
  s_pub_mix = PubRegister(TOPIC_MIX, sizeof(Dart_Mix_s));

  memset(&s_att, 0, sizeof(s_att));
  memset(&s_tgt, 0, sizeof(s_tgt));
  memset(&s_mix, 0, sizeof(s_mix));
  memset(&s_st, 0, sizeof(s_st));
}

void Guidance_Task(float dt, uint8_t guide_enable) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  Dart_Attitude_s a;
  Dart_Target_s t;
  float lambda = 0.0f;
  float d_raw = 0.0f;
  float a_cmd = 0.0f;
  float yaw = 0.0f;
  float roll = 0.0f;
  uint8_t vision_ok;
  uint8_t failsafe;
  PNG_Input_s in;

  /* 1) 取最新输入 */
  if (SubGetMessage(s_sub_att, &a)) s_att = a;
  if (SubGetMessage(s_sub_tgt, &t)) s_tgt = t;
  vision_ok = s_tgt.found ? 1u : 0u;

  /* 2) 目标像素 -> 水平视线角 -> 视线角速率(微分 + 一阶低通) */
  if (vision_ok) {
    float dx = (float)s_tgt.x - GUID_IMAGE_CX;
    lambda = dx / GUID_FOCAL_PX;
    if (s_los_started && dt > 0.0f) d_raw = (lambda - s_los_prev) / dt;
    s_los_prev = lambda;
    s_los_started = 1;
  } else {
    s_los_started = 0; /* 丢失目标后重置微分基准 */
  }
  s_dlambda += GUID_LOS_FILTER_ALPHA * (d_raw - s_dlambda);

  /* 3) 制导律: a_cmd = png(dλ, v_c) */
  memset(&in, 0, sizeof(in));
  in.d_lambda = s_dlambda;
  in.v_c = GUID_DLC_V_C;
  a_cmd = PNGCalculate(&s_png, &in);

  /* 4) 控制: yaw = k*a_cmd, pitch = 0; roll 由 PID 或 ADRC (ROLL_CTRL_MODE) */
  yaw = GUID_GAIN_K * a_cmd;
#if (ROLL_CTRL_MODE == 2)
  /* 自抗扰: 输入 roll 角(measure) 与 ref=0; 角速率由 ESO 估计 */
  roll = ADRCCalculate(&s_adrc, s_att.roll_deg, 0.0f, dt);
#else
  roll = PIDCalculate(&s_roll_pid, s_att.roll_deg, 0.0f);
#endif

  /* 5) 失效/未制导: 输出回中(全 0) */
  failsafe = (!guide_enable || !vision_ok || !s_att.valid) ? 1u : 0u;
  if (failsafe) {
    yaw = 0.0f;
    roll = 0.0f;
  }

  s_mix.pitch = 0.0f;
  s_mix.yaw = Clamp(yaw, -1.0f, 1.0f);
  s_mix.roll = Clamp(roll, -1.0f, 1.0f);
  s_mix.failsafe = failsafe;
  s_mix.tick = (uint32_t)DWT_GetTimeline_ms();
  PubPushMessage(s_pub_mix, &s_mix);

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = a_cmd; /* 关键量: a_cmd, 便于观察制导输出 */
  s_st.err = DART_ERR_NONE;
}

const Dart_AppStatus_s *Guidance_GetStatus(void) { return &s_st; }
