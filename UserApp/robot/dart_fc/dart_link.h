/*
 * dart_link.h — 制导飞镖 <-> ESP32 串口协议 (USART6, 115200, ASCII 行协议)
 *
 * 用于手机网页无线调参/调试 4 路舵面。命令见 dart_link.c 顶部注释。
 */
#ifndef DART_FC_LINK_H
#define DART_FC_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

void DartLinkInit(void);
void DartLinkTask(void);

/* 1 = 链路正常 (从未收到命令时也视为正常, 便于脱机测试) */
int DartLinkIsOk(void);

void DartLinkSend(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* DART_FC_LINK_H */
