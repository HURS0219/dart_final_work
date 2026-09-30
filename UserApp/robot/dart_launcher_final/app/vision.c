/*
 * vision.c — 视觉 app 实现
 * =============================================================================
 * 数据流: 订阅 "vision_cmd"(link 的 C,x,center 注入)
 *         -> 更新 x / center, 置 ok=1, 刷新时间戳
 *         -> 算 err = x - center
 *         -> 发布 "aim_cmd"
 *         -> 超时未更新则 ok=0
 *
 * 说明: 现阶段 PC(hik_vision.py) 经 ESP32 把坐标透传进来, 将来 Jetson 同理。
 *       本 app 是"坐标的归一化层", 下游 yaw 只认 aim_cmd, 不关心来源。
 * =============================================================================
 */
#include "vision.h"

#include <string.h>

#include "bsp_dwt.h"
#include "launcher_cfg.h"
#include "message_center.h"

static Subscriber_t *s_sub_cmd = NULL;
static Publisher_t *s_pub_aim = NULL;

static int16_t s_x = 0;
static int16_t s_center = LAUNCH_VIS_CENTER_DEFAULT;
static uint8_t s_ok = 0;
static uint32_t s_last_rx_ms = 0;

static Launcher_AppStatus_s s_st;
static Launcher_Aim_s s_aim;

void Vision_Init(void) {
  memset(&s_st, 0, sizeof(s_st));
  memset(&s_aim, 0, sizeof(s_aim));

  s_x = 0;
  s_center = LAUNCH_VIS_CENTER_DEFAULT;
  s_ok = 0;
  s_last_rx_ms = 0;

  s_sub_cmd = SubRegister(TOPIC_VISION_CMD, sizeof(Launcher_VisCmd_s));
  s_pub_aim = PubRegister(TOPIC_AIM_CMD, sizeof(Launcher_Aim_s));
}

void Vision_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  Launcher_VisCmd_s vc;

  /* 1) 取注入坐标 */
  if (SubGetMessage(s_sub_cmd, &vc)) {
    if (vc.center > 0) s_center = vc.center;
    s_x = vc.x;
    s_ok = 1;
    s_last_rx_ms = now;
  }

  /* 2) 超时 -> 目标丢失 */
  if (s_ok && s_last_rx_ms != 0 && (now - s_last_rx_ms) > LAUNCH_VIS_TIMEOUT_MS) {
    s_ok = 0;
  }

  /* 3) 发布 */
  s_aim.x = s_x;
  s_aim.center = s_center;
  s_aim.err = (int16_t)(s_x - s_center);
  s_aim.ok = s_ok;
  s_aim.tick = now;
  PubPushMessage(s_pub_aim, &s_aim);

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = (float)s_aim.err; /* 关键量: 像素误差 */
  /* 视觉丢失不算致命(发射前无目标是常态), 仍上报便于观察 */
  s_st.err = s_ok ? LAUNCH_ERR_NONE : LAUNCH_ERR_VISION_OFF;
}

const Launcher_AppStatus_s *Vision_GetStatus(void) { return &s_st; }
