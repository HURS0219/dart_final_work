/**
 * @file servo_motor.c
 * @author ai (extracted & merged from UserApp/robot/dart_servo_v0.2)
 * @brief 舵机通用模块实现: PWM 舵机(PTK7350/SG90...) + 串口总线舵机
 * @version 1.1
 * @date 2026-09-28
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention 详细说明见 servo_motor.md
 */
#include "servo_motor.h"

#include "bsp_dwt.h"
#include "bsp_flash.h"
#include "bsp_log.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"

/* 总线舵机命令模板(索引 8/9 为角度; TODO: 多舵机下需按 servo_id 组帧) */
static const uint8_t kBusMoveTpl[16] = {0x55, 0x55, 0x08, 0x03, 0x01, 0xF4, 0x01,
                                        0x01, 0x20, 0x03, 0x55, 0x55, 0x04, 0x15, 0x01, 0x01};
static const uint8_t kBusRead[6] = {0x55, 0x55, 0x04, 0x15, 0x01, 0x01};
static const uint8_t kBusUnload[6] = {0x55, 0x55, 0x04, 0x14, 0x01, 0x01};

static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];  // 所有实例
static uint8_t servo_idx = 0;                                 // 已注册数量

static void DecodeServo(void);

/* ============================== 掉电保存(标定) ============================== */
/* magic 低字节为版本号: 标定结构布局变化时 +1 (旧数据自动失效) */
#define SERVO_CALIB_MAGIC 0x53525601u /* 'SRV' + version 0x01 */
#define SERVO_CALIB_VERSION 0x01u

/* 持久化镜像: 显式 int32 方向 + 无隐式 padding, 避免跨编译器 CRC 误判 */
typedef struct {
  float pulse_min_us;
  float pulse_max_us;
  float range_deg;
  float center_deg;
  float trim_deg;
  float scale;
  float rate_limit_dps;
  float limit_deg;
  int32_t reverse;
} Servo_Calib_Persist_s;

typedef struct {
  uint32_t magic;
  uint32_t crc;
  Servo_Calib_Persist_s calib[SERVO_MOTOR_CNT];
} Servo_Calib_Blob_s;

/* FNV-1a 校验 */
static uint32_t ServoCalibCrc(const Servo_Calib_Persist_s *c) {
  const uint8_t *p = (const uint8_t *)c;
  uint32_t s = 0x811C9DC5u;
  for (uint32_t i = 0; i < sizeof(Servo_Calib_Persist_s) * SERVO_MOTOR_CNT; i++) {
    s ^= p[i];
    s *= 16777619u;
  }
  return s;
}

/* 清除悬挂的 Flash 错误标志(否则擦除可能静默失败) */
static void FlashClearErrors(void) {
#ifdef STM32F407xx
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
}

/* ============================== 内部工具 ============================== */

static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* 逻辑角限位(对称): 返回 ±limit_deg; 未设置时默认 range/2 */
static float LimitDeg(const ServoInstance *servo) {
  float lim = servo->calib.limit_deg;
  if (lim <= 0.0f) lim = servo->calib.range_deg * 0.5f;
  return lim;
}

/* PWM 周期(us), 从底层实例读取, 异常时回落到 20000us(50Hz) */
static float PeriodUs(const ServoInstance *servo) {
  float period_s = servo->pwm_instance ? servo->pwm_instance->period : 0.0f;
  if (period_s <= 0.0f) period_s = 0.02f;
  return period_s * 1000000.0f;
}

/* 机械角 -> 脉宽 */
static float PulseFromRaw(const ServoInstance *servo, float raw_deg) {
  float range = servo->calib.range_deg > 0.0f ? servo->calib.range_deg : 1.0f;
  float span = servo->calib.pulse_max_us - servo->calib.pulse_min_us;
  return servo->calib.pulse_min_us + (raw_deg / range) * span;
}

/* 脉宽 -> 机械角 (PulseFromRaw 的逆运算) */
static float RawFromPulse(const ServoInstance *servo, float pulse_us) {
  float range = servo->calib.range_deg > 0.0f ? servo->calib.range_deg : 1.0f;
  float span = servo->calib.pulse_max_us - servo->calib.pulse_min_us;
  if (span <= 0.0f) return 0.0f;
  return (pulse_us - servo->calib.pulse_min_us) / span * range;
}

/* 逻辑角 -> 机械角 (含 center/trim/scale/reverse 并限幅) */
static float RawFromLogical(const ServoInstance *servo, float logical_deg) {
  float raw = servo->calib.center_deg + servo->calib.trim_deg +
              (float)servo->calib.reverse * servo->calib.scale * logical_deg;
  return Clamp(raw, 0.0f, servo->calib.range_deg);
}

/* 输出一个脉宽(限幅后写入 PWM 占空比) */
static void OutputPwm(ServoInstance *servo, float pulse_us) {
  if (servo->pwm_instance == NULL) return;
  pulse_us = Clamp(pulse_us, servo->calib.pulse_min_us, servo->calib.pulse_max_us);
  servo->pulse_us = pulse_us;
  PWMSetDutyRatio(servo->pwm_instance, pulse_us / PeriodUs(servo));
}

/* 按当前逻辑角 angle 计算并输出 (先做对称限位) */
static void ApplyLogical(ServoInstance *servo) {
  float lim = LimitDeg(servo);
  servo->angle = Clamp(servo->angle, -lim, lim);
  servo->raw_deg = RawFromLogical(servo, servo->angle);
  OutputPwm(servo, PulseFromRaw(servo, servo->raw_deg));
}

/* ============================== 注册 ============================== */

ServoInstance *ServoInit(Servo_Init_Config_s *config) {
  if (config == NULL) return NULL;
  if (servo_idx >= SERVO_MOTOR_CNT) {
    LOGERROR("[servo] exceed max instance count %d", SERVO_MOTOR_CNT);
    return NULL;
  }

  ServoInstance *servo = (ServoInstance *)malloc(sizeof(ServoInstance));
  memset(servo, 0, sizeof(ServoInstance));
  servo->servo_id = config->servo_id;
  servo->servo_type = config->servo_type;

  switch (config->servo_type) {
    case Bus_Servo: {
      USART_Init_Config_s usart_config;
      usart_config.module_callback = DecodeServo;
      usart_config.recv_buff_size = Servo_MAX_BUFF;
      usart_config.usart_handle = config->_handle;
      servo->usart_instance = USARTRegister(&usart_config);
      memcpy(servo->bus_tx, kBusMoveTpl, sizeof(servo->bus_tx));  // 每实例独立缓冲
      break;
    }
    case PWM_Servo: {
      servo->pwm_instance = PWMRegister(&config->pwm_init_config);
      servo->calib = config->calib;
      /* 未填写/非法参数回落默认, 避免除零或方向反转 */
      if (servo->calib.range_deg <= 0.0f) servo->calib.range_deg = 180.0f;
      if (servo->calib.pulse_max_us <= servo->calib.pulse_min_us) {
        servo->calib.pulse_min_us = 500.0f;
        servo->calib.pulse_max_us = 2500.0f;
      }
      if (servo->calib.scale <= 0.0f) servo->calib.scale = 1.0f;
      if (servo->calib.reverse == 0) servo->calib.reverse = 1;
      if (servo->calib.center_deg <= 0.0f) servo->calib.center_deg = servo->calib.range_deg * 0.5f;
      if (servo->calib.limit_deg <= 0.0f) servo->calib.limit_deg = servo->calib.range_deg * 0.5f;

      servo->target_deg = 0.0f;
      servo->angle = 0.0f;
      servo->last_time_s = DWT_GetTimeline_s();
      ApplyLogical(servo);  // 上电即输出中立位
      break;
    }
    default:
      LOGERROR("[servo] unknown servo type %d", (int)config->servo_type);
      free(servo);
      return NULL;
  }

  servo_motor_instance[servo_idx++] = servo;
  return servo;
}

/* ============================== 角度控制 ============================== */

void ServoSetAngle(ServoInstance *servo, float angle) {
  if (servo == NULL) return;

  if (servo->servo_type == Bus_Servo) {
    if (servo->usart_instance == NULL) return;
    servo->angle = angle;
    int16_t raw = (int16_t)angle;
    servo->bus_tx[8] = (uint8_t)(raw & 0xFF);
    servo->bus_tx[9] = (uint8_t)((raw >> 8) & 0xFF);
    USARTSend(servo->usart_instance, servo->bus_tx, sizeof(servo->bus_tx), USART_TRANSFER_DMA);
    return;
  }

  if (servo->servo_type != PWM_Servo) return;
  angle = Clamp(angle, -LimitDeg(servo), LimitDeg(servo));  // 对称限位
  servo->target_deg = angle;
  if (servo->calib.rate_limit_dps <= 0.0f) {  // 不限速: 立即到位
    servo->angle = angle;
    ApplyLogical(servo);
  }
}

void ServoSetPulseUs(ServoInstance *servo, float pulse_us) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  OutputPwm(servo, pulse_us);
  /* 反推逻辑角/机械角, 保持状态一致(并做限位) */
  servo->raw_deg = RawFromPulse(servo, servo->pulse_us);
  float denom = (float)servo->calib.reverse * servo->calib.scale;
  if (denom != 0.0f) {
    float logical = (servo->raw_deg - servo->calib.center_deg - servo->calib.trim_deg) / denom;
    servo->angle = Clamp(logical, -LimitDeg(servo), LimitDeg(servo));
    servo->target_deg = servo->angle;
  }
}

void ServoTask(void) {
  float now = DWT_GetTimeline_s();

  for (uint8_t i = 0; i < servo_idx; i++) {
    ServoInstance *servo = servo_motor_instance[i];
    if (servo == NULL || servo->servo_type != PWM_Servo) continue;

    float dt = now - servo->last_time_s;
    servo->last_time_s = now;
    if (servo->calib.rate_limit_dps <= 0.0f) continue;  // 未启用限速

    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
    float max_delta = servo->calib.rate_limit_dps * dt;

    float err = servo->target_deg - servo->angle;
    if (err > max_delta) err = max_delta;
    if (err < -max_delta) err = -max_delta;
    servo->angle += err;

    ApplyLogical(servo);
  }
}

void ServoUnload(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != Bus_Servo || servo->usart_instance == NULL) return;
  USARTSend(servo->usart_instance, (uint8_t *)kBusUnload, sizeof(kBusUnload), USART_TRANSFER_DMA);
}

void ServoRequestAngle(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != Bus_Servo || servo->usart_instance == NULL) return;
  USARTSend(servo->usart_instance, (uint8_t *)kBusRead, sizeof(kBusRead), USART_TRANSFER_DMA);
}

/* ============================== 标定 ============================== */

void ServoSetNeutral(ServoInstance *servo, float center_deg) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  servo->calib.center_deg = Clamp(center_deg, 0.0f, servo->calib.range_deg);
  ApplyLogical(servo);
}

void ServoSetTrim(ServoInstance *servo, float trim_deg) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  servo->calib.trim_deg = trim_deg;
  ApplyLogical(servo);
}

void ServoSetScale(ServoInstance *servo, float scale) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  if (scale <= 0.0f) return;
  servo->calib.scale = scale;
  ApplyLogical(servo);
}

void ServoSetReverse(ServoInstance *servo, int8_t reverse) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  servo->calib.reverse = reverse < 0 ? -1 : 1;
  ApplyLogical(servo);
}

void ServoSetRateLimit(ServoInstance *servo, float dps) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  servo->calib.rate_limit_dps = dps;
}

float ServoGetRateLimit(ServoInstance *servo) {
  return (servo != NULL) ? servo->calib.rate_limit_dps : 0.0f;
}

void ServoSetRange(ServoInstance *servo, float range_deg) {
  if (servo == NULL || servo->servo_type != PWM_Servo || range_deg <= 0.0f) return;
  servo->calib.range_deg = range_deg;
  if (servo->calib.center_deg > range_deg) servo->calib.center_deg = range_deg * 0.5f;
  ApplyLogical(servo);
}

void ServoSetPulseRange(ServoInstance *servo, float pulse_min_us, float pulse_max_us) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  if (pulse_max_us <= pulse_min_us) return;
  servo->calib.pulse_min_us = pulse_min_us;
  servo->calib.pulse_max_us = pulse_max_us;
  ApplyLogical(servo);
}

void ServoSetLimit(ServoInstance *servo, float limit_deg) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  servo->calib.limit_deg = (limit_deg > 0.0f) ? limit_deg : 0.0f;
  ApplyLogical(servo);
}

/* 内部: 调零安全窗口检查(当前机械角是否落在 机械中位 ±SERVO_ZERO_WINDOW_DEG) */
static uint8_t ZeroInWindow(const ServoInstance *servo, float raw) {
  float mid = servo->calib.range_deg * 0.5f;
  float d = (raw > mid) ? (raw - mid) : (mid - raw);
  return (d <= SERVO_ZERO_WINDOW_DEG) ? 1u : 0u;
}

/* 内部: 单实例调零(不落盘); 返回 1 成功, 0 被窗口拒绝 */
static uint8_t ZeroOne(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return 0;
  /* 让逻辑 0° 对应"当前位置": center = 当前机械角 - trim */
  float raw = RawFromLogical(servo, servo->angle);
  if (!ZeroInWindow(servo, raw)) return 0;  // 不在机械中位附近, 拒绝调零
  servo->calib.center_deg = Clamp(raw - servo->calib.trim_deg, 0.0f, servo->calib.range_deg);
  servo->target_deg = 0.0f;
  servo->angle = 0.0f;
  ApplyLogical(servo);  // 脉宽不变(center+trim == 当前机械角)
  return 1;
}

uint8_t ServoZero(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return 0;
  if (!ZeroOne(servo)) return 0;  // 被窗口拒绝 -> 不落盘
  ServoSaveCalib();               // 仅在成功调零后立即写 Flash(掉电保存; 阻塞约 1s)
  return 1;
}

uint8_t ServoZeroAll(void) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < servo_idx; i++) n += ZeroOne(servo_motor_instance[i]);
  if (n) ServoSaveCalib();  // 至少一个成功才落盘
  return n;
}

void ServoResetCal(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  servo->calib.center_deg = servo->calib.range_deg * 0.5f;
  servo->calib.trim_deg = 0.0f;
  servo->calib.scale = 1.0f;
  servo->calib.reverse = 1;
  ApplyLogical(servo);
}

/* ============================== Flash 保存/读回 ============================== */

/* Config -> Persist */
static void CalibToPersist(const Servo_Calib_Config_s *c, Servo_Calib_Persist_s *p) {
  p->pulse_min_us = c->pulse_min_us;
  p->pulse_max_us = c->pulse_max_us;
  p->range_deg = c->range_deg;
  p->center_deg = c->center_deg;
  p->trim_deg = c->trim_deg;
  p->scale = c->scale;
  p->rate_limit_dps = c->rate_limit_dps;
  p->limit_deg = c->limit_deg;
  p->reverse = c->reverse;
}

/* Persist -> Config */
static void PersistToCalib(const Servo_Calib_Persist_s *p, Servo_Calib_Config_s *c) {
  c->pulse_min_us = p->pulse_min_us;
  c->pulse_max_us = p->pulse_max_us;
  c->range_deg = p->range_deg;
  c->center_deg = p->center_deg;
  c->trim_deg = p->trim_deg;
  c->scale = p->scale;
  c->rate_limit_dps = p->rate_limit_dps;
  c->limit_deg = p->limit_deg;
  c->reverse = (int8_t)p->reverse;
}

void ServoSaveCalib(void) {
  Servo_Calib_Blob_s blob;
  memset(&blob, 0, sizeof(blob));
  blob.magic = SERVO_CALIB_MAGIC;
  for (uint8_t i = 0; i < servo_idx && i < SERVO_MOTOR_CNT; i++) {
    if (servo_motor_instance[i] != NULL)
      CalibToPersist(&servo_motor_instance[i]->calib, &blob.calib[i]);
  }
  blob.crc = ServoCalibCrc(blob.calib);

  FlashClearErrors();
  flash_erase_address(SERVO_CALIB_FLASH_ADDR, 1);
  flash_write_single_address(SERVO_CALIB_FLASH_ADDR, (uint32_t *)&blob,
                             (uint32_t)(sizeof(blob) / 4));
}

void ServoLoadCalib(void) {
  Servo_Calib_Blob_s blob;
  flash_read(SERVO_CALIB_FLASH_ADDR, (uint32_t *)&blob, (uint32_t)(sizeof(blob) / 4));
  if (blob.magic != SERVO_CALIB_MAGIC) return;   // 无有效数据 / 版本不符
  if (blob.crc != ServoCalibCrc(blob.calib)) return;  // 校验失败
  for (uint8_t i = 0; i < servo_idx && i < SERVO_MOTOR_CNT; i++) {
    if (servo_motor_instance[i] != NULL) {
      PersistToCalib(&blob.calib[i], &servo_motor_instance[i]->calib);
      ApplyLogical(servo_motor_instance[i]);
    }
  }
}

/* ============================== 使能/失能 ============================== */

void ServoEnable(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != PWM_Servo || servo->pwm_instance == NULL) return;
  PWMStart(servo->pwm_instance);
  ApplyLogical(servo);
}

void ServoDisable(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != PWM_Servo || servo->pwm_instance == NULL) return;
  PWMStop(servo->pwm_instance);
}

/* ============================== 查询 ============================== */

float ServoGetAngle(ServoInstance *servo) { return (servo != NULL) ? servo->angle : 0.0f; }

float ServoGetRawDeg(ServoInstance *servo) { return (servo != NULL) ? servo->raw_deg : 0.0f; }

float ServoGetPulseUs(ServoInstance *servo) { return (servo != NULL) ? servo->pulse_us : 0.0f; }

uint16_t ServoGetRecvAngle(ServoInstance *servo) { return (servo != NULL) ? servo->recv_angle : 0; }

void ServoGetCalib(ServoInstance *servo, Servo_Calib_Config_s *out) {
  if (servo == NULL || out == NULL) return;
  *out = servo->calib;
}

/* ============================== 总线舵机解析 ============================== */

/* 中断上下文: 只解析位置回读帧(0x55 0x55 ... 0x15) */
static void DecodeServo(void) {
  for (uint8_t i = 0; i < servo_idx; i++) {
    ServoInstance *servo = servo_motor_instance[i];
    if (servo == NULL || servo->servo_type != Bus_Servo || servo->usart_instance == NULL) continue;

    uint8_t *buf = servo->usart_instance->recv_buff;
    if (buf[0] == Servo_Frame_First && buf[1] == Servo_Frame_Second) {
      if (buf[3] == SERVO_POS_READ_CMD) {  // 位置回读应答
        servo->recv_angle = (uint16_t)((buf[7] << 8) | buf[6]);
      }
    }
  }
}
