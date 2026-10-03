/*
 * launcher/vision.c — 视觉坐标组件实现 (移植 dart_launcher_web_v5_HIK/dart_vision.c)
 */
#include "vision.h"

#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

static VisionInstance* s_vis = NULL;

VisionInstance* LauncherVisionInit(void) {
  s_vis = (VisionInstance*)zmalloc(sizeof(VisionInstance));
  if (s_vis == NULL) return NULL;
  s_vis->x = DART_VIS_CENTER;
  s_vis->center = DART_VIS_CENTER;
  s_vis->err = 0;
  s_vis->ok = 0;
  s_vis->last_ms = 0;
  return s_vis;
}

void LauncherVisionSet(int x, int center) {
  if (s_vis == NULL) return;
  if (center > 0) s_vis->center = center;
  s_vis->x = x;
  s_vis->err = x - s_vis->center;
  s_vis->ok = 1;
  s_vis->last_ms = HAL_GetTick();
}

void LauncherVisionTask(void) {
  if (s_vis == NULL) return;
  if (s_vis->ok && s_vis->last_ms != 0 &&
      HAL_GetTick() - s_vis->last_ms > DART_VIS_TIMEOUT_MS)
    s_vis->ok = 0;
  s_vis->err = s_vis->x - s_vis->center;
}
