/*
 * dart_vision.h — 绿光中心坐标 (OpenMV -> C板; v2 先支持串口注入联调)
 */
#ifndef DART_LAUNCHER_WEB_V2_DART_VISION_H
#define DART_LAUNCHER_WEB_V2_DART_VISION_H

#include <stdint.h>

extern int g_vis_x;
extern int g_vis_center;
extern int g_vis_ok;
extern int g_vis_err;

void DartVisionInit(void);
void DartVisionSet(int x, int center);
void DartVisionLost(void);
void DartVisionTask(void);

#endif  // DART_LAUNCHER_WEB_V2_DART_VISION_H
