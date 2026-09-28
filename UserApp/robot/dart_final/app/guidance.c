/*
 * guidance.c — 制导+控制 app 实现  [roll_dec 分支: 旋转解耦(路线二)]
 * =============================================================================
 * 数据流:  订阅 "attitude"/"target"
 *          -> 由目标像素求水平视线角 λ = (x-CX)/focal
 *          -> 内部微分 + 一阶低通得到 dλ (png_ai 只吃 dλ)
 *          -> png_ai 解算横向加速度 a_cmd (m/s^2)
 *          -> 控制:
 *               [旋转解耦开启] 弹体自由滚转; 用滚转角 γ 把"空间指令"旋转到弹体坐标系:
 *                   yaw_body   =  ay_space*cosγ + az_space*sinγ
 *                   pitch_body = -ay_space*sinγ + az_space*cosγ
 *               [关闭] 传统: 仅控 yaw, 用 roll PID 稳滚(见 dart_final_cfg.h 开关)
 *          -> 发布 "mix"
 *
 * 说明: 目标水平移动, 垂直不控 => az_space = 0; 因此本质是"水平指令按 γ 分配到体轴"。
 *       旋转解耦让舵面随弹体一起滚也能产生正确的空间方向(不做滚转稳定)。
 *       详见本分支 `roll_dec.md`。k 仍取物理值 1/PNG_MAX_OUT; “过偏”调 SERVO_MIX_MAX_DEG。
 * =============================================================================
 */
#include "guidance.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "controller.h"
#include "dart_final_cfg.h"
#include "message_center.h"
#include "png.h"
#include "robot_def.h"

#define DEG2RAD (0.01745329252f)

static PNGInstance s_png;
static PIDInstance s_roll_pid; /* 仅在“关闭旋转解耦”的传统模式下使用 */

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
  float ay_space = 0.0f; /* 空间系横向指令(水平) */
  float az_space = 0.0f; /* 空间系横向指令(垂直), 本机不控高 => 0 */
  float yaw = 0.0f;
  float pitch = 0.0f; /* roll_dec 分支: 体轴俯仰指令(旋转解耦时非 0) */
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

  /* 3) 制导律: a_cmd = png(dλ, v_c) -> 空间系水平指令 */
  memset(&in, 0, sizeof(in));
  in.d_lambda = s_dlambda;
  in.v_c = GUID_DLC_V_C;
  a_cmd = PNGCalculate(&s_png, &in);
  ay_space = GUID_GAIN_K * a_cmd;

  /* 4) 控制 */
#if GUID_ROLL_DEC_ENABLE
  /* ---- 路线二: 旋转解耦(允许自旋, 用 γ 把空间指令旋转到弹体坐标系) ---- */
  {
    float g = (s_att.roll_deg + GUID_ROLL_OFFSET_DEG) * GUID_ROLL_SIGN * DEG2RAD;
    float c = cosf(g);
    float s = sinf(g);
    yaw = ay_space * c + az_space * s;   /* 体轴 yaw */
    pitch = -ay_space * s + az_space * c; /* 体轴 pitch */
    roll = 0.0f;                          /* 不再稳滚 */
  }
#else
  /* ---- 传统: 仅控 yaw + roll PID 稳滚 ---- */
  yaw = ay_space;
  pitch = 0.0f;
  roll = PIDCalculate(&s_roll_pid, s_att.roll_deg, 0.0f);
#endif

  /* 5) 失效/未制导: 输出回中(全 0) */
  failsafe = (!guide_enable || !vision_ok || !s_att.valid) ? 1u : 0u;
  if (failsafe) {
    yaw = 0.0f;
    pitch = 0.0f;
    roll = 0.0f;
  }

  s_mix.pitch = Clamp(pitch, -1.0f, 1.0f);
  s_mix.yaw = Clamp(yaw, -1.0f, 1.0f);
  s_mix.roll = Clamp(roll, -1.0f, 1.0f);
  s_mix.failsafe = failsafe;
  s_mix.tick = (uint32_t)DWT_GetTimeline_ms();
  PubPushMessage(s_pub_mix, &s_mix);

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = a_cmd; /* 关键量: a_cmd (调试亦可看 s_att.roll_deg=γ) */
  s_st.err = DART_ERR_NONE;
}

const Dart_AppStatus_s *Guidance_GetStatus(void) { return &s_st; }
