/*
 * launcher/vision.h — 视觉坐标组件 (绿光中心 x -> 像素误差)
 */
#pragma once

#include <stdint.h>

typedef struct {
  int x;
  int center;
  int err;
  int ok;
  uint32_t last_ms;
} VisionInstance;

VisionInstance* LauncherVisionInit(void);
void LauncherVisionTask(void);
void LauncherVisionSet(int x, int center); /* 上位机 C,x,center 注入; ctrl 调用 */
