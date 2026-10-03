/*
 * launcher/store.h — 参数掉电保存 (STM32F407 内部 Flash, 双 bank 交替+校验)
 */
#pragma once

#include <stdint.h>

void StoreInit(void);
void StoreTask(void);
void StoreMarkDirty(void);
void StoreMarkDirtyNow(void);
int StoreTakeSaved(void);
