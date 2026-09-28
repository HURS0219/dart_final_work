/*
 * st7735_ui.h — ST7735 可视化界面 (飞镖制导)
 *
 * 三个界面, 用板载 KEY (PA0) 切换:
 *   1) 系统状态: 4 舵面角度 + 制导状态机 + PID 参数 + 加速度指令
 *   2) OpenMV   : 相机取景 (目标框) / 真图像流 / 无信号花屏
 *   3) 惯导     : 姿态人工地平仪 + 欧拉角 + 角速率/加速度
 *
 * 数据来自 dart_app (dart_fc 移植的 control/guidance/attitude)。
 */
#ifndef ST7735_UI_H
#define ST7735_UI_H

#include <stdint.h>

/* 初始化: 初始化屏幕 + 按键, 进入界面 1 */
void ST7735_UI_Init(void);

/* 周期调用 (建议 >=200Hz): 扫键 + 按帧率刷新当前界面 */
void ST7735_UI_Task(void);

/* 真图像流接口 (OpenMV 整帧 RGB565): 传一帧即显示; 传 NULL 回到演示/花屏。
 * 图像按比例缩放到取景区。可在接收完一帧后调用。 */
void ST7735_UI_SetFrame(const uint16_t *rgb565, uint16_t w, uint16_t h);

#endif /* ST7735_UI_H */
