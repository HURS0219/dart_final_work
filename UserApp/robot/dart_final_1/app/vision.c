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
static uint32_t s_rx_ok = 0, s_rx_bad = 0; /* 收帧统计 */

#if DF1_VISION_SPI
#include "spi.h" /* hspi2 */

static uint8_t s_buf[OPENMV_RECV_SIZE]; /* SPI 从机接收缓冲(中断填) */

static void VisionSpiInit(void) {
  /* 复用 SPI2, 重配为从机 (CPOL=0, CPHA=0, 与 OpenMV 默认一致)
   * NSS 软件管理 + SSI=1: 从机“常被选中”, 只要 SCK 有脉冲就接收 —— 不强依赖 CS 接线。 */
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    LOGERROR("[vision] SPI2 slave init FAIL");
  }
  hspi2.Instance->CR1 &= ~SPI_CR1_SSI; /* 软件 NSS: 从机需 NSS=低(SSI=0)才被选中 */
  /* 非阻塞: 中断方式收 7 字节; SPI2_IRQHandler 已在 it.c 接入 */
  (void)HAL_SPI_Receive_IT(&hspi2, s_buf, OPENMV_RECV_SIZE);
}
#else
#include "usart.h"

static volatile uint8_t s_new_frame = 0;
static Dart_Target_s s_rx;
static uint8_t s_uart_buf[OPENMV_RECV_SIZE]; /* 定长 7 字节中断接收缓冲 */

#if DF1_UART_SCAN
/* ===== 诊断: USART1/3/6 各挂 1 字节中断接收, 统计各口收到字节数 ===== */
static uint8_t s_b1[1], s_b3[1], s_b6[1];
static volatile uint32_t s_c1 = 0, s_c3 = 0, s_c6 = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) { s_c1++; (void)HAL_UART_Receive_IT(&huart1, s_b1, 1); }
  else if (huart->Instance == USART3) { s_c3++; (void)HAL_UART_Receive_IT(&huart3, s_b3, 1); }
  else if (huart->Instance == USART6) { s_c6++; (void)HAL_UART_Receive_IT(&huart6, s_b6, 1); }
}
void Vision_GetScan(uint32_t *c1, uint32_t *c3, uint32_t *c6) { if (c1) *c1 = s_c1; if (c3) *c3 = s_c3; if (c6) *c6 = s_c6; }
#else
/** @brief 串口接收完成回调(中断上下文): 校验并解码一帧, 随即重新武装接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    uint8_t *b = s_uart_buf;
    if (b[0] == OPENMV_HEAD1 && b[1] == OPENMV_HEAD2 && crc_8(b, 6) == b[6]) {
      s_rx.x = (int16_t)((b[2] << 8) | b[3]);
      s_rx.y = (int16_t)((b[4] << 8) | b[5]);
      s_rx.found = !(s_rx.x == 0 && s_rx.y == 0);
      s_new_frame = 1;
      s_rx_ok++;
    } else {
      s_rx_bad++;
    }
    (void)HAL_UART_Receive_IT(&huart3, s_uart_buf, OPENMV_RECV_SIZE);
  }
}
#endif
#endif

void Vision_Init(void) {
  memset(&s_tgt, 0, sizeof(s_tgt));
  memset(&s_st, 0, sizeof(s_st));
  s_pub = PubRegister(TOPIC_TARGET, sizeof(Dart_Target_s));

#if DF1_VISION_SPI
  VisionSpiInit();
  LOGINFO("[vision] SPI2 slave ready (SCK PB13/MOSI PB15/CS PB12)");
#else
#if DF1_UART_SCAN
  (void)HAL_UART_Receive_IT(&huart1, s_b1, 1);
  (void)HAL_UART_Receive_IT(&huart3, s_b3, 1);
  (void)HAL_UART_Receive_IT(&huart6, s_b6, 1);
  LOGINFO("[vision] UART scan armed on USART1/3/6");
#else
  {
    /* 统一为 115200 8N1 (与 OpenMV 默认一致), 定长 7 字节中断接收 */
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.StopBits = UART_STOPBITS_1;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
      LOGERROR("[vision] huart3 re-init FAIL");
    }
    if (HAL_UART_Receive_IT(&huart3, s_uart_buf, OPENMV_RECV_SIZE) == HAL_OK) {
      LOGINFO("[vision] USART3 RX-IT armed (115200 8N1)");
    } else {
      LOGERROR("[vision] USART3 RX-IT arm FAIL");
    }
  }
#endif
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
        s_rx_ok++;
      } else {
        s_rx_bad++;
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

void Vision_GetStats(uint32_t *ok, uint32_t *bad) {
  if (ok != NULL) *ok = s_rx_ok;
  if (bad != NULL) *bad = s_rx_bad;
}

#if !DF1_UART_SCAN
void Vision_GetScan(uint32_t *c1, uint32_t *c3, uint32_t *c6) {
  if (c1) *c1 = 0;
  if (c3) *c3 = 0;
  if (c6) *c6 = 0;
}
#endif

/* 自检: 往视觉串口发一帧有效帧(x=160,y=120)。把 TX<->RX 短接即可自收, 验证串口链。
 * 返回 HAL 发送结果(HAL_OK=0)。 */
int Vision_TxTest(void) {
#if DF1_VISION_SPI
  return -1;
#else
  uint8_t f[7];
  f[0] = OPENMV_HEAD1;
  f[1] = OPENMV_HEAD2;
  f[2] = 0x00;
  f[3] = 0xA0; /* x=160 */
  f[4] = 0x00;
  f[5] = 0x78; /* y=120 */
  f[6] = crc_8(f, 6);
  return (int)HAL_UART_Transmit(DART_USART_OPENMV, f, 7, 100);
#endif
}
