/*
 * wifi_link.c — 解析 ESP32 发来的 ASCII 指令帧: "C,<mode>,<value>\n"
 *
 * bsp_usart 使用空闲中断+DMA 接收变长帧, 回调时 recv_buff 已含整帧数据,
 * 且缓冲尾部为零(接收结束会被清零), 故可直接按字符串解析。
 */
#include "wifi_link.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_usart.h"
#include "robot_config.h"
#include "wifi_motor.h"

static USARTInstance* wifi_usart = NULL;

static bool ParseCommand(const char* s, int* mode, int* value) {
  if (s == NULL || s[0] != 'C' || s[1] != ',') return false;

  char* end = NULL;
  long m = strtol(s + 2, &end, 10);
  if (end == NULL || *end != ',') return false;

  long v = strtol(end + 1, &end, 10);
  *mode = (int)m;
  *value = (int)v;
  return true;
}

/* 解析参数设置帧: "P,<id>,<value>" */
static bool ParseParam(const char* s, int* id, int* value) {
  if (s == NULL || s[0] != 'P' || s[1] != ',') return false;
  char* end = NULL;
  long a = strtol(s + 2, &end, 10);
  if (end == NULL || *end != ',') return false;
  long b = strtol(end + 1, &end, 10);
  *id = (int)a;
  *value = (int)b;
  return true;
}

/* 解析 "<letter>,<a>[,<b>]" */
static bool Parse2(const char* s, char letter, int* a, int* b) {
  if (s == NULL || s[0] != letter || s[1] != ',') return false;
  char* end = NULL;
  long x = strtol(s + 2, &end, 10);
  *a = (int)x;
  if (b != NULL) {
    *b = 0;
    if (end != NULL && *end == ',') *b = (int)strtol(end + 1, &end, 10);
  }
  return true;
}

static void WifiDecode(void) {
  char* buf = (char*)wifi_usart->recv_buff;

  /* 串口链路自检: 收到 "PING" 则回 "PONG" (IT 发送, 避免与 DMA 接收冲突) */
  if (buf[0] == 'P' && buf[1] == 'I' && buf[2] == 'N' && buf[3] == 'G') {
    USARTSend(wifi_usart, (uint8_t*)"PONG\n", 5, USART_TRANSFER_IT);
    return;
  }

  /* 复位(设零): 收到 "Z" 把当前方向设为角度模式 0 度 */
  if (buf[0] == 'Z') {
    WifiMotorZeroHere();
    return;
  }

  /* 扫描电机: 收到 "S" */
  if (buf[0] == 'S') {
    WifiMotorScanRequest();
    return;
  }

  /* 清空电机列表: "X" */
  if (buf[0] == 'X') {
    WifiMotorClearMotors();
    return;
  }

  /* 添加电机: "A,<type>,<id>" */
  int type = 0, mid = 0;
  if (Parse2(buf, 'A', &type, &mid)) {
    WifiMotorAddMotor(type, mid);
    return;
  }

  /* 选择调参电机(槽位): "U,<idx>" */
  if (Parse2(buf, 'U', &type, NULL)) {
    WifiMotorSetSelect(type);
    return;
  }

  int id = 0, value = 0;
  if (ParseParam(buf, &id, &value)) {
    WifiMotorSetParam(id, value);
    return;
  }

  int mode = 0;
  if (ParseCommand(buf, &mode, &value)) {
    WifiMotorSetCommand(mode, value);
  }
}

void WifiLinkInit(void) {
  USART_Init_Config_s cfg = {
      .recv_buff_size = WIFI_RECV_SIZE,
      .usart_handle = WIFI_UART_HANDLE,
      .module_callback = WifiDecode,
  };
  wifi_usart = USARTRegister(&cfg);
}

void WifiLinkSend(const char* s) {
  if (wifi_usart == NULL || s == NULL) return;
  USARTSend(wifi_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}

void WifiLinkSendBlocking(const char* s) {
  if (wifi_usart == NULL || s == NULL) return;
  /* 等上一次发送完成(周期反馈用 IT), 否则阻塞发送会因 UART 忙而失败 */
  uint32_t t0 = HAL_GetTick();
  while (wifi_usart->usart_handle->gState != HAL_UART_STATE_READY) {
    if (HAL_GetTick() - t0 > 50) break;
  }
  USARTSend(wifi_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_BLOCKING);
}
