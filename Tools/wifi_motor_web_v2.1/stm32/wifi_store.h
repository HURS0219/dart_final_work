/*
 * wifi_store.h — 参数/零点 掉电保存 (STM32 内部 Flash)
 *
 * 保存内容: 角度零点 + 全部可调参数(FF/Kp/Ki/积分限幅/角度Kp/角度Kd/角度斜率/限幅)。
 * 写 Flash 会阻塞(擦除约 1s), 因此只在“停止”状态真正写入; 运行中只置脏标记。
 */
#ifndef WIFI_GM6020_STORE_H
#define WIFI_GM6020_STORE_H

void WifiStoreInit(void);      // 上电从 Flash 读取(无有效数据则用默认)
void WifiStoreMarkDirty(void); // 标记需要保存(可在中断里调用)
void WifiStoreTask(void);      // 任务里调用: 停止且脏时写 Flash

#endif  // WIFI_GM6020_STORE_H
