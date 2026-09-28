/*
 * dart_pwm.c — 4 路 50Hz 舵机 PWM 输出 (TIM1 CH1~4)
 */
#include "dart_pwm.h"

#include "main.h"
#include "robot_config.h"
#include "tim.h"

#define DART_PWM_PERIOD_US 20000.0f

static const uint32_t kCh[DART_AXIS_N] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

static float s_tick_us = 1.0f;
static float s_pulse[DART_AXIS_N];
static uint8_t s_inited;

/* 1 个计数对应多少微秒 (F4: TIM1/8/9/10/11 挂 APB2, 分频!=1 时 x2) */
static float TickUs(TIM_HandleTypeDef *h) {
  uintptr_t inst = (uintptr_t)h->Instance;
  RCC_ClkInitTypeDef clk;
  uint32_t lat = 0, pclk, tclk;

  HAL_RCC_GetClockConfig(&clk, &lat);
  if (inst == (uintptr_t)TIM1 || inst == (uintptr_t)TIM8 ||
      inst == (uintptr_t)TIM9 || inst == (uintptr_t)TIM10 ||
      inst == (uintptr_t)TIM11) {
    pclk = HAL_RCC_GetPCLK2Freq();
    tclk = (clk.APB2CLKDivider == RCC_HCLK_DIV1) ? pclk : (pclk * 2u);
  } else {
    pclk = HAL_RCC_GetPCLK1Freq();
    tclk = (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk : (pclk * 2u);
  }
  if (tclk == 0u) return 1.0f;
  return (float)(h->Init.Prescaler + 1u) * 1000000.0f / (float)tclk;
}

void DartPwmInit(void) {
  uint32_t arr;
  uint8_t i;

  if (s_inited) return;

  s_tick_us = TickUs(DART_PWM_TIM);
  if (s_tick_us <= 0.0f) s_tick_us = 1.0f;

  arr = (uint32_t)(DART_PWM_PERIOD_US / s_tick_us + 0.5f);
  if (arr > 0u) arr -= 1u;
  if (arr > 0xFFFFu) arr = 0xFFFFu;
  __HAL_TIM_SET_AUTORELOAD(DART_PWM_TIM, arr);

  for (i = 0; i < DART_AXIS_N; i++) {
    s_pulse[i] = DART_PULSE_CENTER_US;
    __HAL_TIM_SET_COMPARE(DART_PWM_TIM, kCh[i],
                          (uint32_t)(s_pulse[i] / s_tick_us + 0.5f));
    HAL_TIM_PWM_Start(DART_PWM_TIM, kCh[i]);
  }
  s_inited = 1u;
}

void DartPwmSetPulseUs(uint8_t ch, float us) {
  uint32_t ccr;

  if (ch >= DART_AXIS_N) return;
  if (us < DART_PULSE_MIN_US) us = DART_PULSE_MIN_US;
  if (us > DART_PULSE_MAX_US) us = DART_PULSE_MAX_US;
  s_pulse[ch] = us;
  ccr = (uint32_t)(us / s_tick_us + 0.5f);
  __HAL_TIM_SET_COMPARE(DART_PWM_TIM, kCh[ch], ccr);
}

float DartPwmGetPulseUs(uint8_t ch) {
  return (ch < DART_AXIS_N) ? s_pulse[ch] : 0.0f;
}
