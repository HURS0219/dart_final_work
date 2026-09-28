/*
 * dart_axis.c — 4 路舵面: 标定 + 状态机 + 速率限幅
 */
#include "dart_axis.h"

#include <math.h>

#include "bsp_dwt.h"
#include "dart_pwm.h"
#include "robot_config.h"

#define DART_TWO_PI 6.2831853f

typedef struct {
  float manual;      /* 手动目标逻辑角 */
  float out;         /* 速率限幅后的逻辑角 (对外显示) */
  float trim;        /* 零点 */
  float scale;       /* 增益 */
  uint8_t reversed;  /* 方向反转 */
} Axis_t;

static const float kMix[DART_AXIS_N][3] = {
    {DART_MIX_PITCH_0, DART_MIX_YAW_0, DART_MIX_ROLL_0},
    {DART_MIX_PITCH_1, DART_MIX_YAW_1, DART_MIX_ROLL_1},
    {DART_MIX_PITCH_2, DART_MIX_YAW_2, DART_MIX_ROLL_2},
    {DART_MIX_PITCH_3, DART_MIX_YAW_3, DART_MIX_ROLL_3},
};

static Axis_t s_ax[DART_AXIS_N];
static DartMode_e s_mode = DART_MODE_IDLE;
static float s_mix[3];
static float s_last_t;
static uint8_t s_inited;

static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float LogToPulse(float applied_deg) {
  return DART_PULSE_CENTER_US +
         applied_deg * (DART_PULSE_HALF_US / DART_TRAVEL_HALF_DEG);
}

/* 计算某路当前实际施加角 (含标定/方向), 用于调零 */
static float AppliedOf(const Axis_t *a) {
  float applied = a->out * a->scale + a->trim;
  if (a->reversed) applied = -applied;
  return Clamp(applied, -DART_TRAVEL_HALF_DEG, DART_TRAVEL_HALF_DEG);
}

void DartAxisInit(void) {
  uint8_t i;

  if (s_inited) return;
  DartPwmInit();
  for (i = 0; i < DART_AXIS_N; i++) {
    s_ax[i].manual = 0.0f;
    s_ax[i].out = 0.0f;
    s_ax[i].trim = 0.0f;
    s_ax[i].scale = 1.0f;
    s_ax[i].reversed = 0u;
  }
  s_mix[0] = s_mix[1] = s_mix[2] = 0.0f;
  s_mode = DART_MODE_IDLE;
  s_last_t = DWT_GetTimeline_s();
  s_inited = 1u;
}

void DartAxisTask(void) {
  float now, dt, max_delta, lim;
  float target[DART_AXIS_N];
  uint8_t i;

  if (!s_inited) DartAxisInit();

  now = DWT_GetTimeline_s();
  dt = now - s_last_t;
  s_last_t = now;
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  max_delta = DART_RATE_LIMIT_DPS * dt;

  switch (s_mode) {
    case DART_MODE_MANUAL:
      for (i = 0; i < DART_AXIS_N; i++) target[i] = s_ax[i].manual;
      break;
    case DART_MODE_MIX:
      for (i = 0; i < DART_AXIS_N; i++) {
        float u = kMix[i][0] * s_mix[0] + kMix[i][1] * s_mix[1] +
                  kMix[i][2] * s_mix[2];
        target[i] = u * DART_MIX_MAX_DEG;
      }
      break;
    case DART_MODE_TEST: {
      float sweep = DART_TEST_SWEEP_DEG *
                    sinf(DART_TWO_PI * DART_TEST_FREQ_HZ * now);
      for (i = 0; i < DART_AXIS_N; i++) target[i] = sweep;
      break;
    }
    case DART_MODE_IDLE:
    default:
      for (i = 0; i < DART_AXIS_N; i++) target[i] = 0.0f;
      break;
  }

  /* 混控模式每片舵面限制在机械偏角内, 手动/自检允许全行程 */
  lim = (s_mode == DART_MODE_MIX) ? DART_MIX_MAX_DEG : DART_TRAVEL_HALF_DEG;

  for (i = 0; i < DART_AXIS_N; i++) {
    float t = Clamp(target[i], -lim, lim);
    float applied;

    if (t > s_ax[i].out + max_delta) t = s_ax[i].out + max_delta;
    if (t < s_ax[i].out - max_delta) t = s_ax[i].out - max_delta;
    s_ax[i].out = t;

    applied = t * s_ax[i].scale + s_ax[i].trim;
    if (s_ax[i].reversed) applied = -applied;
    applied = Clamp(applied, -DART_TRAVEL_HALF_DEG, DART_TRAVEL_HALF_DEG);
    DartPwmSetPulseUs(i, LogToPulse(applied));
  }
}

void DartAxisSetMode(DartMode_e m) {
  if (m > DART_MODE_TEST) m = DART_MODE_IDLE;
  s_mode = m;
}
DartMode_e DartAxisGetMode(void) { return s_mode; }

void DartAxisSetManual(uint8_t ch, float deg) {
  if (ch >= DART_AXIS_N) return;
  s_ax[ch].manual = Clamp(deg, -DART_TRAVEL_HALF_DEG, DART_TRAVEL_HALF_DEG);
}
float DartAxisGetManual(uint8_t ch) {
  return (ch < DART_AXIS_N) ? s_ax[ch].manual : 0.0f;
}

void DartAxisSetMix(float pitch, float yaw, float roll) {
  s_mix[0] = Clamp(pitch, -1.0f, 1.0f);
  s_mix[1] = Clamp(yaw, -1.0f, 1.0f);
  s_mix[2] = Clamp(roll, -1.0f, 1.0f);
}
void DartAxisGetMix(float out[3]) {
  out[0] = s_mix[0];
  out[1] = s_mix[1];
  out[2] = s_mix[2];
}

void DartAxisSetTrim(uint8_t ch, float deg) {
  if (ch >= DART_AXIS_N) return;
  s_ax[ch].trim = Clamp(deg, -DART_TRAVEL_HALF_DEG, DART_TRAVEL_HALF_DEG);
}
void DartAxisSetScale(uint8_t ch, float scale) {
  if (ch >= DART_AXIS_N) return;
  if (scale <= 0.0f) return;
  s_ax[ch].scale = scale;
}
void DartAxisSetDir(uint8_t ch, uint8_t reversed) {
  if (ch >= DART_AXIS_N) return;
  s_ax[ch].reversed = reversed ? 1u : 0u;
}

void DartAxisZero(uint8_t ch) {
  float applied;
  if (ch >= DART_AXIS_N) return;
  /* 让逻辑 0° 对应"当前位置": 反推 trim (方向反转在 trim 之后) */
  applied = AppliedOf(&s_ax[ch]);
  s_ax[ch].trim = s_ax[ch].reversed ? -applied : applied;
  s_ax[ch].out = 0.0f;
  s_ax[ch].manual = 0.0f;
}

void DartAxisResetCal(uint8_t ch) {
  if (ch >= DART_AXIS_N) return;
  s_ax[ch].trim = 0.0f;
  s_ax[ch].scale = 1.0f;
}

float DartAxisGetAngle(uint8_t ch) {
  return (ch < DART_AXIS_N) ? s_ax[ch].out : 0.0f;
}
float DartAxisGetPulse(uint8_t ch) { return DartPwmGetPulseUs(ch); }
uint8_t DartAxisGetDir(uint8_t ch) {
  return (ch < DART_AXIS_N) ? s_ax[ch].reversed : 0u;
}
float DartAxisGetTrim(uint8_t ch) {
  return (ch < DART_AXIS_N) ? s_ax[ch].trim : 0.0f;
}
float DartAxisGetScale(uint8_t ch) {
  return (ch < DART_AXIS_N) ? s_ax[ch].scale : 1.0f;
}
