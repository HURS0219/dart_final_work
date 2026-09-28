/*
 * vision.c — 视觉 app 实现
 * =============================================================================
 * OpenMV 9 字节帧(定长, 大端):
 *   [0]0xAA [1]0x55 [2]X_hi [3]X_lo [4]Y_hi [5]Y_lo [6]W [7]H [8]CRC8
 *   CRC8: SHT75(poly=0x31, init=0), 对 [0..7] 求校验, 与 Modules/algorithm/crc8 一致。
 *   未识别到目标时 OpenMV 发全 0(X=Y=W=H=0)。
 * 分工: 中断里只做“收整帧 + 校验 + 解码到缓存并置标志”; 发布到话题在任务里做。
 * =============================================================================
 */
#include "vision.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "crc8.h"
#include "dart_final_cfg.h"
#include "message_center.h"
#include "robot_def.h"
#include "usart.h"  /* DART_USART_OPENMV(&huart3) 声明处 */

#define OPENMV_HEAD1 0xAA
#define OPENMV_HEAD2 0x55

static USARTInstance *s_usart = NULL;
static Publisher_t *s_pub = NULL;

static volatile uint8_t s_new_frame = 0;   /* 中断置 1, 任务清零 */
static Dart_Target_s s_rx;                  /* 中断里填入的原始解码结果 */
static Dart_Target_s s_tgt;                 /* 对外发布的最新目标 */
static uint32_t s_last_frame_ms = 0;        /* 最近一次有效帧时刻 */
static Dart_AppStatus_s s_st;

/** @brief 串口接收完成回调(中断上下文): 校验并解码一帧 */
static void Vision_RxCallback(void) {
  uint8_t *b = s_usart->recv_buff;

  if (b[0] != OPENMV_HEAD1 || b[1] != OPENMV_HEAD2) return;
  if (crc_8(b, 8) != b[8]) return; /* CRC8 校验失败丢弃 */

  s_rx.x = (int16_t)((b[2] << 8) | b[3]);
  s_rx.y = (int16_t)((b[4] << 8) | b[5]);
  s_rx.w = b[6];
  s_rx.h = b[7];
  /* OpenMV 丢失目标时全 0 */
  s_rx.found = !(s_rx.x == 0 && s_rx.y == 0 && s_rx.w == 0 && s_rx.h == 0);
  s_new_frame = 1;
}

void Vision_Init(void) {
  USART_Init_Config_s cfg;

  memset(&cfg, 0, sizeof(cfg));
  cfg.usart_handle = DART_USART_OPENMV; /* 引脚待定, 见 dart_final_cfg.h */
  cfg.recv_buff_size = OPENMV_RECV_SIZE;
  cfg.module_callback = Vision_RxCallback;
  s_usart = USARTRegister(&cfg);

  memset(&s_tgt, 0, sizeof(s_tgt));
  memset(&s_st, 0, sizeof(s_st));
  s_pub = PubRegister(TOPIC_TARGET, sizeof(Dart_Target_s));

  LOGINFO("[vision] usart reg %s", (s_usart != NULL) ? "OK" : "FAIL");
}

void Vision_Task(void) {
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();

  if (s_new_frame) {
    s_new_frame = 0;
    s_tgt = s_rx; /* 中断缓存的整帧 -> 对外发布 */
    s_tgt.tick = now;
    s_last_frame_ms = now;
    PubPushMessage(s_pub, &s_tgt);
  } else if (now - s_last_frame_ms > VISION_TIMEOUT_MS) {
    /* 超时: 明确告知下游“目标丢失”, 只在上次 found 时发一次 */
    if (s_tgt.found) {
      s_tgt.found = 0;
      s_tgt.tick = now;
      PubPushMessage(s_pub, &s_tgt);
    }
  }

  s_st.hb++;
  s_st.key = (float)s_tgt.x; /* 关键量: 目标 x 像素 */
  s_st.err = (now - s_last_frame_ms > VISION_TIMEOUT_MS) ? DART_ERR_VISION_OFF : DART_ERR_NONE;
}

const Dart_AppStatus_s *Vision_GetStatus(void) { return &s_st; }

void Vision_GetTarget(Dart_Target_s *out) {
  if (out != NULL) *out = s_tgt;
}
