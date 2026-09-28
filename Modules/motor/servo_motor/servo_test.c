#include "servo_test.h"

#if defined(STM32F407xx)

#include <math.h>

#include "bsp_dwt.h"
#include "ptk7350.h"
#include "ptk7645.h"
#include "tim.h"

#define SERVO_TEST_CENTER_DEG 90.0f
#define SERVO_TEST_SWEEP_DEG  20.0f
#define SERVO_TEST_FREQ_HZ    0.25f

static PTK7350_t s_ptk7350; /* PWM2 = TIM1_CH2 = PE11 */
static PTK7645_t s_ptk7645; /* PWM1 = TIM1_CH1 = PE9  */
static uint8_t s_inited;

void ServoTestInit(void) {
  if (s_inited) return;
  PTK7350_Init(&s_ptk7350, &htim1, TIM_CHANNEL_2);
  PTK7645_Init(&s_ptk7645, &htim1, TIM_CHANNEL_1);
  s_inited = 1u;
}

void ServoTestTask(void) {
  float t;
  float angle;

  if (!s_inited) {
    ServoTestInit();
  }

  t = DWT_GetTimeline_s();
  angle = SERVO_TEST_CENTER_DEG + SERVO_TEST_SWEEP_DEG * sinf(6.2831853f * SERVO_TEST_FREQ_HZ * t);

  PTK7350_SetAngle(&s_ptk7350, angle);
  PTK7645_SetAngle(&s_ptk7645, angle);
}

#else

void ServoTestInit(void) {}
void ServoTestTask(void) {}

#endif /* STM32F407xx */
