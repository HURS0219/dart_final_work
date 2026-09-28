/*
 * dart_store.h — 参数掉电保存 (STM32F407 内部 Flash, sector 11)
 */
#ifndef DART_LAUNCHER_WEB_V3_DART_STORE_H
#define DART_LAUNCHER_WEB_V3_DART_STORE_H

#include <stdint.h>

void DartStoreInit(void);
void DartStoreTask(void);
void DartStoreMarkDirty(void);
void DartStoreMarkDirtyNow(void);  // 立即保存(跳过去抖)
int DartStoreTakeSaved(void);

#endif  // DART_LAUNCHER_WEB_V3_DART_STORE_H
