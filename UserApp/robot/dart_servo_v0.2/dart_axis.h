/*
 * dart_axis.h — 4 路舵面: 标定 (方向/零点/增益) + 状态机 + 速率限幅
 *
 * 模式 (互斥, 显式切换, 无隐藏覆盖):
 *   IDLE   四舵面回中立 0°
 *   MANUAL 每路跟随各自手动角
 *   MIX    由 pitch/yaw/roll (-1..1) 经解耦矩阵输出
 *   TEST   缓慢正弦扫舵
 *
 * 逻辑角范围 = ±139.5° (279° 行程), 0° = 1500us。
 * 施加角 applied = angle*scale + trim, 方向反则取 -applied。
 */
#ifndef DART_V2_AXIS_H
#define DART_V2_AXIS_H

#include <stdint.h>

typedef enum {
  DART_MODE_IDLE = 0,
  DART_MODE_MANUAL = 1,
  DART_MODE_MIX = 2,
  DART_MODE_TEST = 3,
} DartMode_e;

void DartAxisInit(void);
void DartAxisTask(void);

void DartAxisSetMode(DartMode_e m);
DartMode_e DartAxisGetMode(void);

void DartAxisSetManual(uint8_t ch, float deg);
float DartAxisGetManual(uint8_t ch);

void DartAxisSetMix(float pitch, float yaw, float roll);
void DartAxisGetMix(float out[3]);

void DartAxisSetTrim(uint8_t ch, float deg);
void DartAxisSetScale(uint8_t ch, float scale);
void DartAxisSetDir(uint8_t ch, uint8_t reversed);
void DartAxisZero(uint8_t ch);
void DartAxisResetCal(uint8_t ch);

float DartAxisGetAngle(uint8_t ch);   /* 当前逻辑角 (速率限幅后) */
float DartAxisGetPulse(uint8_t ch);
uint8_t DartAxisGetDir(uint8_t ch);
float DartAxisGetTrim(uint8_t ch);
float DartAxisGetScale(uint8_t ch);

#endif /* DART_V2_AXIS_H */
