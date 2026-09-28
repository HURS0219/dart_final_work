/*
 * dart_link.h — ESP32 <-> C板 串口协议 (v3)
 */
#ifndef DART_LAUNCHER_WEB_V3_DART_LINK_H
#define DART_LAUNCHER_WEB_V3_DART_LINK_H

void DartLinkInit(void);
void DartLinkTask(void);
void DartLinkSend(const char* s);
void DartLinkSendBlocking(const char* s);

#endif  // DART_LAUNCHER_WEB_V3_DART_LINK_H
