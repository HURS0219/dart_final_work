/*
 * wifi_link.h — ESP32 <-> STM32 串口指令接收
 */
#ifndef WIFI_GM6020_LINK_H
#define WIFI_GM6020_LINK_H

void WifiLinkInit(void);
/* 通过 ESP32 串口发一行 (给网页显示) */
void WifiLinkSend(const char* s);          // IT 异步(用于周期性反馈)
void WifiLinkSendBlocking(const char* s);  // 阻塞(用于一次性结果, 不会被丢弃)

#endif  // WIFI_GM6020_LINK_H
