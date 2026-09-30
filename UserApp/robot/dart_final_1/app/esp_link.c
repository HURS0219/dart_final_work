/*
 * esp_link.c — ESP32 <-> STM32 串口链路 app (dart_final_1 测试用)
 * =============================================================================
 * 底层只经由 bsp_usart(USARTRegister/USARTSend) 收发行协议, 与 dart_fc 一致。
 * 端口: ESP=UART0 -> STM32 USART1(PA9 TX/PA10 RX), 115200 8N1。
 * =============================================================================
 */
#include "esp_link.h"

#include <string.h>

#include "bsp_log.h"
#include "bsp_usart.h"
#include "robot_cfg.h"
#include "usart.h"

#ifndef DF1_ESP_UART
#define DF1_ESP_UART (&huart1)
#endif
#ifndef DF1_ESP_RX_SIZE
#define DF1_ESP_RX_SIZE 64
#endif
#ifndef DF1_ESP_PERIOD_MS
#define DF1_ESP_PERIOD_MS 500u
#endif
#ifndef DF1_ESP_TIMEOUT_MS
#define DF1_ESP_TIMEOUT_MS 1500u
#endif

static USARTInstance *s_uart = NULL;
static EspLink_Stats_s s_st;
static uint32_t s_last_ping = 0;

static void EspLink_Decode(void) {
  char *buf = (char *)s_uart->recv_buff;
  char *p = buf;
  size_t n = strlen(buf);

  if (n == 0) return;
  s_st.rx_bytes += (uint32_t)n;
  s_st.last_rx_ms = HAL_GetTick();

  while (*p) {
    char *nl = strchr(p, '\n');
    size_t len;
    if (nl != NULL) *nl = '\0';
    len = strlen(p);
    while (len > 0 && (p[len - 1] == '\r' || p[len - 1] == ' ')) p[--len] = '\0';
    if (len > 0) {
      s_st.rx_lines++;
      if (strncmp(p, "PONG", 4) == 0) {
        s_st.pong++;
      } else if (strncmp(p, "F,", 2) == 0) {
        s_st.tele++;
      } else if (strcmp(p, "H") == 0) {
        s_st.hb++;
      }
      strncpy(s_st.last_line, p, sizeof(s_st.last_line) - 1);
      s_st.last_line[sizeof(s_st.last_line) - 1] = '\0';
    }
    if (nl == NULL) break;
    p = nl + 1;
  }
}

void EspLink_Init(void) {
  USART_Init_Config_s cfg;

  memset(&s_st, 0, sizeof(s_st));
  s_last_ping = 0;

  cfg.recv_buff_size = DF1_ESP_RX_SIZE;
  cfg.usart_handle = DF1_ESP_UART;
  cfg.module_callback = EspLink_Decode;
  s_uart = USARTRegister(&cfg);

  LOGINFO("[esp] USART1 link reg %s (115200 8N1)", (s_uart != NULL) ? "OK" : "FAIL");
}

void EspLink_Task(void) {
  uint32_t now = HAL_GetTick();

  if (s_uart == NULL) return;

  if (now - s_last_ping >= DF1_ESP_PERIOD_MS) {
    s_last_ping = now;
    EspLink_Send("PING\n");
    s_st.tx_frames++;
  }

  s_st.link_ok = (s_st.last_rx_ms != 0 && (now - s_st.last_rx_ms) <= DF1_ESP_TIMEOUT_MS) ? 1u : 0u;
}

void EspLink_Send(const char *s) {
  if (s_uart == NULL || s == NULL) return;
  USARTSend(s_uart, (uint8_t *)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}

void EspLink_GetStats(EspLink_Stats_s *out) {
  if (out != NULL) *out = s_st;
}

/* 弱符号覆盖: 统计 USART1 发送完成中断(证明字节确实移出引脚) */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (s_uart != NULL && huart == s_uart->usart_handle) {
    s_st.tx_done++;
  }
}
