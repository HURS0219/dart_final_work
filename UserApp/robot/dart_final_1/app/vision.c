/*
 * vision.c — 视觉 app 实现 (dart_final_1 沙盒版)
 * =============================================================================
 * 两种链路(由 robot_cfg.h 的 DF1_VISION_SPI 选择):
 *   - SPI2 从机 (DF1_VISION_SPI=1): OpenMV 做 SPI 主机, 发 7 字节帧; STM32 轮询收。
 *       SCK=PB13, MOSI=PB15, NSS(CS)=PB12, GND 共地 (MISO/PB14 不用)。
 *       OpenMV(machine.SPI 只有主机): SPI(id=2) sck=P2 mosi=P0, cs=Pin(P3,OUT)。
 *   - USART3  (DF1_VISION_SPI=0): 原 UART 方案(需与 OpenMV 波特率/格式一致)。
 *
 * 帧格式(7 字节, 大端): [0]0xAA [1]0x55 [2]X_hi [3]X_lo [4]Y_hi [5]Y_lo [6]CRC8
 *   CRC8: SHT75(poly=0x31, init=0), 对 [0..5]。未识别: X=Y=0。
 * =============================================================================
 */
#include "vision.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "crc8.h"
#include "dart_final_cfg.h"
#include "message_center.h"
#include "robot_cfg.h"
#include "robot_def.h"

#define OPENMV_HEAD1 0xAA
#define OPENMV_HEAD2 0x55

static Publisher_t *s_pub = NULL;
static Dart_Target_s s_tgt;
static uint32_t s_last_frame_ms = 0;
static Dart_AppStatus_s s_st;

#if DF1_VISION_SPI
#include "spi.h" /* hspi2 */

static uint8_t s_buf[OPENMV_RECV_SIZE]; /* SPI 从机接收缓冲(中断填) */

static void VisionSpiInit(void) {
  GPIO_InitTypeDef g = {0};
  /* PB12 <- SPI2_NSS(AF5), 从机硬件片选(由 OpenMV 主机拉低) */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin = GPIO_PIN_12;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &g);

  /* 复用 SPI2, 重配为从机 (CPOL=0, CPHA=0, 与 OpenMV 默认一致) */
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    LOGERROR("[vision] SPI2 slave init FAIL");
  }
  /* 非阻塞: 中断方式收 7 字节; SPI2_IRQHandler 已在 it.c 接入 */
  (void)HAL_SPI_Receive_IT(&hspi2, s_buf, OPENMV_RECV_SIZE);
}
#else
#include "bsp_usart.h"
#include "usart.h"

static USARTInstance *s_usart = NULL;
static volatile uint8_t s_new_frame = 0;
static Dart_Target_s s_rx;

/** @brief 串口接收完成回调(中断上下文): 校验并解码一帧 */
static void Vision_RxCallback(void) {
  uint8_t *b = s_usart->recv_buff;
  if (b[0] != OPENMV_HEAD1 || b[1] != OPENMV_HEAD2) return;
  if (crc_8(b, 6) != b[6]) return;
  s_rx.x = (int16_t)((b[2] << 8) | b[3]);
  s_rx.y = (int16_t)((b[4] << 8) | b[5]);
  s_rx.found = !(s_rx.x == 0 && s_rx.y == 0);
  s_new_frame = 1;
}
#endif

void Vision_Init(void) {
  memset(&s_tgt, 0, sizeof(s_tgt));
  memset(&s_st, 0, sizeof(s_st));
  s_pub = PubRegister(TOPIC_TARGET, sizeof(Dart_Target_s));

#if DF1_VISION_SPI
  VisionSpiInit();
  LOGINFO("[vision] SPI2 slave ready (SCK PB13/MOSI PB15/CS PB12)");
#else
  {
    USART_Init_Config_s cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.usart_handle = DART_USART_OPENMV;
    cfg.recv_buff_size = OPENMV_RECV_SIZE;
    cfg.module_callback = Vision_RxCallback;
    s_usart = USARTRegister(&cfg);
    LOGINFO("[vision] usart reg %s", (s_usart != NULL) ? "OK" : "FAIL");
  }
#endif
}

void Vision_Task(void) {
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  uint8_t got = 0;
  Dart_Target_s t;

#if DF1_VISION_SPI
  {
    HAL_SPI_StateTypeDef st = HAL_SPI_GetState(&hspi2);
    if (st == HAL_SPI_STATE_READY) { /* 一帧收完(中断方式) */
      if (s_buf[0] == OPENMV_HEAD1 && s_buf[1] == OPENMV_HEAD2 && crc_8(s_buf, 6) == s_buf[6]) {
        t.x = (int16_t)((s_buf[2] << 8) | s_buf[3]);
        t.y = (int16_t)((s_buf[4] << 8) | s_buf[5]);
        t.found = !(t.x == 0 && t.y == 0);
        got = 1;
      }
      (void)HAL_SPI_Receive_IT(&hspi2, s_buf, OPENMV_RECV_SIZE); /* 重新武装 */
    } else if (st == HAL_SPI_STATE_ERROR || st == HAL_SPI_STATE_ABORT) {
      (void)HAL_SPI_Abort(&hspi2);
      (void)HAL_SPI_Receive_IT(&hspi2, s_buf, OPENMV_RECV_SIZE);
    }
  }
#else
  if (s_new_frame) {
    s_new_frame = 0;
    t = s_rx;
    got = 1;
  }
#endif

  if (got) {
    t.tick = now;
    s_tgt = t;
    s_last_frame_ms = now;
    PubPushMessage(s_pub, &s_tgt);
  } else if (now - s_last_frame_ms > VISION_TIMEOUT_MS) {
    if (s_tgt.found) { /* 超时: 明确告知下游“目标丢失”, 只发一次 */
      s_tgt.found = 0;
      s_tgt.tick = now;
      PubPushMessage(s_pub, &s_tgt);
    }
  }

  s_st.hb++;
  s_st.key = (float)s_tgt.x;
  s_st.err = (now - s_last_frame_ms > VISION_TIMEOUT_MS) ? DART_ERR_VISION_OFF : DART_ERR_NONE;
}

const Dart_AppStatus_s *Vision_GetStatus(void) { return &s_st; }

void Vision_GetTarget(Dart_Target_s *out) {
  if (out != NULL) *out = s_tgt;
}
