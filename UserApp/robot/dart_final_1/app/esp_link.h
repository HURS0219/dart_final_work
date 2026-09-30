/*
 * esp_link.h — ESP32 <-> STM32 串口链路 app (dart_final_1 测试用)
 * =============================================================================
 * 用途: 检测 743 板(STM32H743, USART1) 与 ESP32(UART0) 的 UART 通信是否成功。
 * 协议: 沿用 dart_fc 的 ASCII 行协议 —— MCU 周期发 "PING", ESP 回 "PONG";
 *       ESP 周期发心跳 "H"; MCU 遥测帧 "F,..." 由 ESP 侧解析。
 * 判定: 最近 DF1_ESP_TIMEOUT_MS 内收到过任意字节 -> link_ok=1。
 * =============================================================================
 */
#pragma once

#include <stdint.h>

typedef struct {
  uint32_t tx_frames;  /* 已发 PING 帧数 */
  uint32_t tx_done;    /* TX 完成中断次数(证明字节已移出) */
  uint32_t rx_bytes;   /* 累计收到字节 */
  uint32_t rx_lines;   /* 累计收到完整行 */
  uint32_t pong;       /* 收到 "PONG" 次数 */
  uint32_t hb;         /* 收到心跳 "H" 次数 */
  uint32_t tele;       /* 收到遥测 "F,.." 次数 */
  uint32_t last_rx_ms; /* 最近一次收到数据的时刻 */
  uint8_t link_ok;     /* 1=通信正常 */
  char last_line[48];  /* 最近一行原文 */
} EspLink_Stats_s;

void EspLink_Init(void);
void EspLink_Task(void);
void EspLink_Send(const char *s);
void EspLink_GetStats(EspLink_Stats_s *out);
