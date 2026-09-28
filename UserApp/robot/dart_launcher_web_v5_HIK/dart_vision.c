/*
 * dart_vision.c — 绿光中心坐标
 */
#include "dart_vision.h"

#include "main.h"
#include "robot_config.h"

int g_vis_x = DART_VIS_CENTER;
int g_vis_center = DART_VIS_CENTER;
int g_vis_ok = 0;
int g_vis_err = 0;

static uint32_t s_last_rx = 0;

void DartVisionInit(void) {
  g_vis_x = DART_VIS_CENTER;
  g_vis_center = DART_VIS_CENTER;
  g_vis_ok = 0;
  g_vis_err = 0;
  s_last_rx = 0;
}

void DartVisionSet(int x, int center) {
  if (center > 0) g_vis_center = center;
  g_vis_x = x;
  g_vis_err = x - g_vis_center;
  g_vis_ok = 1;
  s_last_rx = HAL_GetTick();
}

void DartVisionLost(void) {
  g_vis_ok = 0;
  s_last_rx = 0;
}

void DartVisionTask(void) {
  if (g_vis_ok && s_last_rx != 0 && HAL_GetTick() - s_last_rx > DART_VIS_TIMEOUT_MS) g_vis_ok = 0;
  g_vis_err = g_vis_x - g_vis_center;
}
