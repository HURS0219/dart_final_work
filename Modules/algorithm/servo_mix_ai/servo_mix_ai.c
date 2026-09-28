/**
 * @file servo_mix_ai.c
 * @author ai
 * @brief 四舵面混控实现 (仅 MIX / MANUAL 两种模式; 上电回中)
 * @version 1.0
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention 详细说明见 servo_mix_ai.md
 *
 * 数据流:
 *   ServoMixTask() 每周期:
 *       MANUAL: target[ch] = manual[ch]
 *       MIX   : u[ch] = Σ matrix[ch][j]*cmd[j];  target[ch] = clamp(u*MAX_DEG, ±MAX_DEG)
 *   然后 target[ch] -> ServoSetAngle(servo[ch])  (限位/限速由 servo_motor 处理)
 *   末尾 ServoTask() 推进速率限幅。
 */
#include "servo_mix_ai.h"

#include "string.h"

/* 4 路舵机实例(由 ServoInit 注册, 本模块持有并驱动) */
static ServoInstance *s_servo[SERVO_MIX_N];

static ServoMixMode_e s_mode = SERVO_MIX_MODE_MIX;  // 当前模式
static float s_cmd[3] = {0.0f, 0.0f, 0.0f};         // 混控指令 pitch/yaw/roll(-1..1)
static float s_manual[SERVO_MIX_N] = {0};           // 手动逐路逻辑角
static uint8_t s_inited = 0;

/* 解耦矩阵(行=舵面, 列=pitch/yaw/roll), 取自 cfg */
static const float kMix[SERVO_MIX_N][3] = SERVO_MIX_MATRIX;

/* 数值限幅 */
static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void ServoMixInit(void) {
  /* 依 cfg 逐路构造 servo_motor 配置; 数组下标 = 舵面编号 0..3 */
  const uint32_t ch[SERVO_MIX_N] = SERVO_MIX_CH;
  const float scale[SERVO_MIX_N] = SERVO_MIX_SCALE;
  const float trim[SERVO_MIX_N] = SERVO_MIX_TRIM_DEG;
  const uint8_t rev[SERVO_MIX_N] = SERVO_MIX_REVERSE;
  const float lim[SERVO_MIX_N] = SERVO_MIX_LIMIT_DEG;
  const float rate[SERVO_MIX_N] = SERVO_MIX_RATE_DPS;
  const uint8_t zen[SERVO_MIX_N] = SERVO_MIX_ZERO_ENABLE;
  uint8_t i;

  if (s_inited) return;

  for (i = 0; i < SERVO_MIX_N; i++) {
    Servo_Init_Config_s cfg;

    memset(&cfg, 0, sizeof(cfg));
    /* 硬件通道 */
    cfg.pwm.htim = SERVO_MIX_TIM;
    cfg.pwm.channel = ch[i];
    cfg.pwm.period = 0.02f;        /* 50Hz */
    cfg.pwm.dutyratio = 0.075f;    /* 初始 1.5ms 中位 */
    /* 信号层(型号规格, 统一取自 servo_motor_cfg.h) */
    cfg.center_us = SERVO_CFG_CENTER_US;
    cfg.half_us = SERVO_CFG_HALF_US;
    cfg.half_deg = SERVO_CFG_HALF_DEG;
    cfg.pulse_min_us = SERVO_CFG_PULSE_MIN_US;
    cfg.pulse_max_us = SERVO_CFG_PULSE_MAX_US;
    /* 逻辑层(逐路装机标定, 取自本模块 cfg) */
    cfg.scale = scale[i];
    cfg.trim_deg = trim[i];
    cfg.limit_deg = lim[i];
    cfg.rate_limit_dps = rate[i];
    cfg.reverse = rev[i];
    cfg.zero_enable = zen[i];

    s_servo[i] = ServoInit(&cfg);  /* 注册后立即输出中位 */
  }

  s_mode = (SERVO_MIX_DEFAULT_MODE == 1) ? SERVO_MIX_MODE_MANUAL : SERVO_MIX_MODE_MIX;
  s_cmd[0] = s_cmd[1] = s_cmd[2] = 0.0f;
  for (i = 0; i < SERVO_MIX_N; i++) s_manual[i] = 0.0f;
  s_inited = 1;

  ServoMixTask();  /* 立即算一次并回中(默认 MIX + 零指令 -> 逻辑 0°) */
}

void ServoMixSetMode(ServoMixMode_e mode) {
  if (mode != SERVO_MIX_MODE_MIX && mode != SERVO_MIX_MODE_MANUAL) return;
  s_mode = mode;
}

ServoMixMode_e ServoMixGetMode(void) { return s_mode; }

void ServoMixSetCmd(float pitch, float yaw, float roll) {
  s_cmd[0] = Clamp(pitch, -1.0f, 1.0f);
  s_cmd[1] = Clamp(yaw, -1.0f, 1.0f);
  s_cmd[2] = Clamp(roll, -1.0f, 1.0f);
}

void ServoMixSetManual(uint8_t ch, float deg) {
  if (ch >= SERVO_MIX_N) return;
  s_manual[ch] = deg;  /* 最终限位由 ServoSetAngle 按该路 limit_deg 处理 */
}

void ServoMixSetManualAll(const float deg[SERVO_MIX_N]) {
  uint8_t i;
  if (deg == NULL) return;
  for (i = 0; i < SERVO_MIX_N; i++) s_manual[i] = deg[i];
}

void ServoMixTask(void) {
  uint8_t i;

  if (!s_inited) ServoMixInit();

  for (i = 0; i < SERVO_MIX_N; i++) {
    float target;

    if (s_mode == SERVO_MIX_MODE_MANUAL) {
      target = s_manual[i];
    } else {
      /* 混控: 解耦矩阵 . 指令, 再乘满偏角并限幅 */
      float u = kMix[i][0] * s_cmd[0] + kMix[i][1] * s_cmd[1] + kMix[i][2] * s_cmd[2];
      target = Clamp(u * SERVO_MIX_MAX_DEG, -SERVO_MIX_MAX_DEG, SERVO_MIX_MAX_DEG);
    }

    if (s_servo[i] != NULL) ServoSetAngle(s_servo[i], target);  // 逻辑角(servo_motor 再做 ±limit 限位)
  }

  ServoTask();  // 推进速率限幅(仅对 rate_limit_dps>0 的实例)
}

uint8_t ServoMixZero(uint8_t ch) {
  if (!s_inited || ch >= SERVO_MIX_N || s_servo[ch] == NULL) return 0;
  return ServoZero(s_servo[ch]);
}

void ServoMixEnable(void) {
  uint8_t i;
  for (i = 0; i < SERVO_MIX_N; i++) ServoEnable(s_servo[i]);
}

void ServoMixDisable(void) {
  uint8_t i;
  for (i = 0; i < SERVO_MIX_N; i++) ServoDisable(s_servo[i]);
}

void ServoMixGetDeflDeg(float out[SERVO_MIX_N]) {
  uint8_t i;
  if (out == NULL) return;
  for (i = 0; i < SERVO_MIX_N; i++) out[i] = ServoGetTarget(s_servo[i]);
}

void ServoMixGetPulseUs(float out[SERVO_MIX_N]) {
  uint8_t i;
  if (out == NULL) return;
  for (i = 0; i < SERVO_MIX_N; i++) out[i] = ServoGetPulseUs(s_servo[i]);
}
