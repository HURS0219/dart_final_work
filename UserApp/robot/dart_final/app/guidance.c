/*
 * guidance.c — 制导+控制 app 实现  [roll_dec 分支: 旋转解耦(路线二)]
 * =============================================================================
 * 完整流水线(允许弹体自旋; 舵面随弹体滚, 算法补偿):
 *   1) 相机随机体滚 -> 像素(x,y)是"体轴坐标";
 *      [输入解旋] 用 R⁻¹(γ) 把体轴像素解旋成"空间视线" -> 取空间水平分量 λ = xs/focal;
 *      (关键: 必须先解旋再微分, 否则世界水平被旋转, 且 dλ 混入伪速率 γ̇·λ)
 *   2) dλ = 微分 + 一阶低通(在空间系);
 *   3) a_cmd = png_ai(dλ, v_c) -> 空间系横向指令 (ay_space = k*a_cmd, az_space = 0);
 *   4) [输出旋转 + 相位超前] 用 γ_eff = γ + ω·τ_lead 把空间指令旋到体轴:
 *         yaw_body   =  ay_space*cos(γ_eff) + az_space*sin(γ_eff)
 *         pitch_body = -ay_space*sin(γ_eff) + az_space*cos(γ_eff)
 *      (τ_lead 补偿舵机响应滞后; 不再做滚转稳定)
 *   5) 发布 "mix"
 *
 * 坐标/符号约定(可现场标定, 见 dart_final_cfg.h):
 *   纵轴 x; 横向面 (y=右, z=上)。γ 为正 = 绕 x 正方向滚转。
 * 详见本分支 `roll_dec.md`。k 仍取物理值 1/PNG_MAX_OUT; “过偏”调 SERVO_MIX_MAX_DEG。
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
static float s_los_prev = 0.0f; /* 上次“空间”视线角 */
static float s_dlambda = 0.0f;  /* 滤波后的“空间”视线角速率 */
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
  float gamma_deg;    /* 滚转角 γ(含零点偏置) */
  float omega_dps;    /* 滚转角速率 ω(deg/s) */
  float lambda = 0.0f;
  float d_raw = 0.0f;
  float a_cmd = 0.0f;
  float ay_space = 0.0f; /* 空间系横向指令(水平) */
  float az_space = 0.0f; /* 空间系横向指令(垂直), 本机不控高 => 0 */
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
  uint8_t vision_ok;
  uint8_t failsafe;
  PNG_Input_s in;

  /* 1) 取最新输入 */
  if (SubGetMessage(s_sub_att, &a)) s_att = a;
  if (SubGetMessage(s_sub_tgt, &t)) s_tgt = t;
  vision_ok = s_tgt.found ? 1u : 0u;

  gamma_deg = s_att.roll_deg + GUID_ROLL_OFFSET_DEG; /* γ */
  omega_dps = s_att.gx_dps;                          /* roll 角速率 */

  /* 2) [输入解旋] 体轴像素 -> 空间水平视线角 λ; 再微分 + 低通 */
  if (vision_ok) {
    float dxb = (float)s_tgt.x - GUID_IMAGE_CX; /* 体轴水平像素偏移 */
    float dxs = dxb;
#if GUID_DEROT_IN_ENABLE
    {
      float gi = gamma_deg * GUID_IN_ROT_SIGN * DEG2RAD; /* R⁻¹(γ) */
      float dyb = (float)s_tgt.y - GUID_IMAGE_CY;
      float c = cosf(gi);
      float s = sinf(gi);
      /* 空间 = R(-γ)·体轴: xs = dxb·c + dyb·s (仅取水平分量) */
      dxs = dxb * c + dyb * s;
    }
#endif
    lambda = dxs / GUID_FOCAL_PX;
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
  /* ---- 路线二: 输出旋转 + 相位超前(允许自旋, 不再稳滚) ---- */
  {
    float ge = gamma_deg + omega_dps * (GUID_LEAD_MS / 1000.0f); /* γ_eff (deg) */
    float g = ge * GUID_ROLL_SIGN * DEG2RAD;
    float c = cosf(g);
    float s = sinf(g);
    yaw = ay_space * c + az_space * s;    /* 体轴 yaw */
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
  s_st.key = a_cmd; /* 关键量: a_cmd; 调试可同时看 s_att.roll_deg=γ */
  s_st.err = DART_ERR_NONE;
}

const Dart_AppStatus_s *Guidance_GetStatus(void) { return &s_st; }
