/*
 * fin.c — 舵面 app 实现
 * =============================================================================
 * 依赖: Modules/algorithm/servo_mix_ai (四舵面混控, 内部驱动 4 路 servo_motor)。
 * 行为: 订阅 "mix"; 若 failsafe 或未制导, 指令置 0(=> 4 舵面回中);
 *       否则把 (pitch,yaw,roll) 交给 servo_mix_ai; 周期推进 ServoMixTask()。
 *       之后读回 4 路目标角/脉宽, 发布 "servo_fb"。
 * =============================================================================
 */
#include "fin.h"

#include <string.h>

#include "bsp_dwt.h"
#include "dart_final_cfg.h"
#include "message_center.h"
#include "robot_def.h"
#include "servo_mix_ai.h"

static Subscriber_t *s_sub_mix = NULL;
static Publisher_t *s_pub_fb = NULL;
static Dart_Mix_s s_cmd;         /* 最近一次收到的混控指令 */
static Dart_ServoFb_s s_fb;      /* 对外发布的舵面反馈 */
static Dart_AppStatus_s s_st;

void Fin_Init(void) {
  ServoMixInit();                       /* 注册 4 路 + 上电回中 */
  ServoMixSetMode(SERVO_MIX_MODE_MIX);  /* 固定混控模式(本飞控只用 MIX) */

  memset(&s_cmd, 0, sizeof(s_cmd));
  memset(&s_fb, 0, sizeof(s_fb));
  memset(&s_st, 0, sizeof(s_st));
  s_st.err = DART_ERR_NONE;

  s_sub_mix = SubRegister(TOPIC_MIX, sizeof(Dart_Mix_s));
  s_pub_fb = PubRegister(TOPIC_SERVO_FB, sizeof(Dart_ServoFb_s));
}

void Fin_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  Dart_Mix_s m;

  if (SubGetMessage(s_sub_mix, &m)) s_cmd = m; /* 更新为最新指令(无新消息则沿用上次) */

  if (s_cmd.failsafe) {
    ServoMixSetCmd(0.0f, 0.0f, 0.0f); /* 失效/未制导 -> 回中 */
  } else {
    ServoMixSetCmd(s_cmd.pitch, s_cmd.yaw, s_cmd.roll);
  }

  ServoMixTask(); /* 计算目标角 + 驱动 4 舵机 + 推进速率限幅 */

  ServoMixGetDeflDeg(s_fb.defl_deg); /* 4 路目标逻辑角 */
  ServoMixGetPulseUs(s_fb.pulse_us); /* 4 路当前脉宽 */
  s_fb.tick = (uint32_t)DWT_GetTimeline_ms();
  PubPushMessage(s_pub_fb, &s_fb);

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = s_fb.defl_deg[1]; /* 关键量: 1 路目标角, 便于调试 */
}

const Dart_AppStatus_s *Fin_GetStatus(void) { return &s_st; }

/* ---------------- 测试接口(薄封装 servo_mix_ai) ---------------- */

void Fin_SetMode(uint8_t manual) {
  ServoMixSetMode(manual ? SERVO_MIX_MODE_MANUAL : SERVO_MIX_MODE_MIX);
}

void Fin_SetManual(uint8_t ch, float deg) { ServoMixSetManual(ch, deg); }

uint8_t Fin_Zero(uint8_t ch) { return ServoMixZero(ch); }
