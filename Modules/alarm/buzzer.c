#include "buzzer.h"

#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "string.h"

static PWMInstance *buzzer;
// static uint8_t idx;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

#define BUZZER_BEEP_QUEUE_LEN 8
#define BUZZER_BEEP_ON_TICKS 8
#define BUZZER_BEEP_OFF_TICKS 8
#define BUZZER_BEEP_LOUDNESS 0.3f

typedef struct {
  uint16_t frequency;
  uint8_t count;
  float loudness;
} BuzzerBeepRequest_s;

static BuzzerBeepRequest_s beep_queue[BUZZER_BEEP_QUEUE_LEN];
static uint8_t beep_queue_head = 0;
static uint8_t beep_queue_tail = 0;
static uint8_t beep_queue_count = 0;
static BuzzerBeepRequest_s active_beep = {
    .frequency = SoFreq,
    .count = 0,
    .loudness = BUZZER_BEEP_LOUDNESS,
};
static uint8_t beep_remaining = 0;
static uint8_t beep_tick = 0;
static uint8_t beep_is_on = 0;

static void BuzzerSetBeepOutput(uint8_t on) {
  if (buzzer == NULL) return;

  if (on) {
    PWMSetDutyRatio(buzzer, active_beep.loudness);
    PWMSetPeriod(buzzer, (float)1 / active_beep.frequency);
  } else {
    PWMSetDutyRatio(buzzer, 0);
  }
}

static uint8_t BuzzerPopBeepRequest(BuzzerBeepRequest_s *request) {
  if (beep_queue_count == 0) {
    return 0;
  }

  *request = beep_queue[beep_queue_head];
  beep_queue_head = (uint8_t)((beep_queue_head + 1) % BUZZER_BEEP_QUEUE_LEN);
  beep_queue_count--;
  return 1;
}

static uint8_t BuzzerBeepTask(void) {
  if (buzzer == NULL) return 0;

  if (beep_remaining == 0) {
    if (!BuzzerPopBeepRequest(&active_beep)) {
      return 0;
    }

    beep_remaining = active_beep.count;
    beep_is_on = 1;
    beep_tick = BUZZER_BEEP_ON_TICKS;
    BuzzerSetBeepOutput(1);
    return 1;
  }

  if (beep_tick > 0) {
    beep_tick--;
  }

  if (beep_tick == 0) {
    if (beep_is_on) {
      beep_is_on = 0;
      beep_tick = BUZZER_BEEP_OFF_TICKS;
      BuzzerSetBeepOutput(0);
    } else {
      beep_remaining--;
      if (beep_remaining == 0) {
        BuzzerSetBeepOutput(0);
      } else {
        beep_is_on = 1;
        beep_tick = BUZZER_BEEP_ON_TICKS;
        BuzzerSetBeepOutput(1);
      }
    }
  }

  return 1;
}

/**
 * @brief 蜂鸣器初始化
 *
 */
void BuzzerInit() {
#ifdef STM32F407xx
  PWM_Init_Config_s buzzer_config = {
      .htim = &htim10,
      .channel = TIM_CHANNEL_3,
      .dutyratio = 0,
      .period = 0.001,
  };
#elifdef STM32H723xx
  PWM_Init_Config_s buzzer_config = {
      .htim = &htim12,
      .channel = TIM_CHANNEL_2,
      .dutyratio = 0,
      .period = 0.001,
  };
#endif
  buzzer = PWMRegister(&buzzer_config);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config) {
  if (config->alarm_level > BUZZER_DEVICE_CNT)  // 超过最大实例数,考虑增加或查看是否有内存泄漏
    while (1);
  BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
  memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

  buzzer_temp->alarm_level = config->alarm_level;
  buzzer_temp->loudness = config->loudness;
  buzzer_temp->octave = config->octave;
  buzzer_temp->alarm_state = ALARM_OFF;

  buzzer_list[config->alarm_level] = buzzer_temp;
  return buzzer_temp;
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state) { buzzer->alarm_state = state; }

uint8_t BuzzerBeep(uint8_t count) {
  return BuzzerBeepWithFreq(SoFreq, count);
}

uint8_t BuzzerBeepWithFreq(uint16_t frequency, uint8_t count) {
  Buzzer_Beep_Config_s config = {
      .frequency = frequency,
      .count = count,
      .loudness = BUZZER_BEEP_LOUDNESS,
  };
  return BuzzerBeepWithConfig(&config);
}

uint8_t BuzzerBeepWithConfig(const Buzzer_Beep_Config_s *config) {
  if (config == NULL || config->frequency == 0 || config->count == 0) return 0;

  BuzzerBeepRequest_s request = {
      .frequency = config->frequency,
      .count = config->count,
      .loudness = config->loudness,
  };
  if (request.count > 6) request.count = 6;
  if (request.loudness <= 0.0f) request.loudness = BUZZER_BEEP_LOUDNESS;
  if (request.loudness > 1.0f) request.loudness = 1.0f;

  if (beep_queue_count >= BUZZER_BEEP_QUEUE_LEN) {
    return 0;
  }

  beep_queue[beep_queue_tail] = request;
  beep_queue_tail = (uint8_t)((beep_queue_tail + 1) % BUZZER_BEEP_QUEUE_LEN);
  beep_queue_count++;
  return 1;
}

void BuzzerTask() {
  BuzzzerInstance *buzz;
  if (BuzzerBeepTask()) {
    return;
  }

  for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i) {
    buzz = buzzer_list[i];
    if (buzz == NULL) {
      continue;
    }
    if (buzz->alarm_level > ALARM_LEVEL_LOW) {
      continue;
    }
    if (buzz->alarm_state == ALARM_OFF) {
      PWMSetDutyRatio(buzzer, 0);
    } else {
      PWMSetDutyRatio(buzzer, buzz->loudness);
      switch (buzz->octave) {
        case OCTAVE_1:
          PWMSetPeriod(buzzer, (float)1 / DoFreq);
          break;
        case OCTAVE_2:
          PWMSetPeriod(buzzer, (float)1 / ReFreq);
          break;
        case OCTAVE_3:
          PWMSetPeriod(buzzer, (float)1 / MiFreq);
          break;
        case OCTAVE_4:
          PWMSetPeriod(buzzer, (float)1 / FaFreq);
          break;
        case OCTAVE_5:
          PWMSetPeriod(buzzer, (float)1 / SoFreq);
          break;
        case OCTAVE_6:
          PWMSetPeriod(buzzer, (float)1 / LaFreq);
          break;
        case OCTAVE_7:
          PWMSetPeriod(buzzer, (float)1 / SiFreq);
          break;
        default:
          break;
      }
      break;
    }
  }
}
