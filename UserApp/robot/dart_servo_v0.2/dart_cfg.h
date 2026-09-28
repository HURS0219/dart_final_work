/*
 * dart_cfg.h — 舵面标定掉电保存 (内部 Flash sector 10)
 * 只在收到 SAVE 时写一次; 结果通过 DartCfgTakeResult() 上报。
 */
#ifndef DART_V2_CFG_H
#define DART_V2_CFG_H

#include <stdint.h>

void DartCfgInit(void);          /* 上电加载 */
void DartCfgTask(void);          /* 处理保存 (显式请求 + 标定改动自动保存) */
void DartCfgRequestSave(void);   /* 显式请求立即保存 (可在中断中调用) */
void DartCfgMarkDirty(void);     /* 标定已改动, 去抖后自动保存 (可在中断中调用) */
uint8_t DartCfgTakeResult(void); /* 0=无 1=成功 2=失败 */

#endif /* DART_V2_CFG_H */
