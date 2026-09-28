/**
 * @file servo_motor.c
 * @author ai (extracted & merged from UserApp/robot/dart_servo_v0.2)
 * @brief 舵机通用模块实现: PWM 舵机(PTK7350/SG90...) + 串口总线舵机
 * @version 1.0
 * @date 2026-09-28
 *
 * @attention 详细说明见 servo_motor.md
 */
#include "servo_motor.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "memory.h"
#include "stdlib.h"

/* 总线舵机固定命令帧(与旧实现保持一致, 索引 8/9 为角度) */
static uint8_t servo_angle_read[6] = {0x55, 0x55, 0x04, 0x15, 0x01, 0x01};
static uint8_t servo_angle_write[16] = {0x55, 0x55, 0x08, 0x03, 0x01, 0xF4, 0x01,
                                        0x01, 0x20, 0x03, 0x55, 0x55, 0x04, 0x15, 0x01, 0x01};
static uint8_t servo_unload[6] = {0x55, 0x55, 0x04, 0x14, 0x01, 0x01};

static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];  // 所有实例
static uint8_t servo_idx = 0;                                 // 已注册数量

static void DecodeServo(void);

/* ============================== 内部工具 ============================== */

static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
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

/* 按当前逻辑角 angle 计算并输出 */
static void ApplyLogical(ServoInstance *servo) {
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

      servo->target_deg = 0.0f;
      servo->angle = 0.0f;
      servo->last_time_s = DWT_GetTimeline_s();
      ApplyLogical(servo);  // 上电即输出中立位
      break;
    }
    default:
      LOGERROR("[servo] unknown servo type %d", (int)config->servo_type);
      break;
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
    servo_angle_write[8] = (uint8_t)(raw & 0xFF);
    servo_angle_write[9] = (uint8_t)((raw >> 8) & 0xFF);
    USARTSend(servo->usart_instance, servo_angle_write, 16, USART_TRANSFER_DMA);
    return;
  }

  if (servo->servo_type != PWM_Servo) return;
  servo->target_deg = angle;
  if (servo->calib.rate_limit_dps <= 0.0f) {  // 不限速: 立即到位
    servo->angle = angle;
    ApplyLogical(servo);
  }
}

void ServoSetPulseUs(ServoInstance *servo, float pulse_us) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  OutputPwm(servo, pulse_us);
  /* 反推逻辑角/机械角, 保持状态一致 */
  servo->raw_deg = RawFromPulse(servo, servo->pulse_us);
  float denom = (float)servo->calib.reverse * servo->calib.scale;
  if (denom != 0.0f) {
    servo->angle = (servo->raw_deg - servo->calib.center_deg - servo->calib.trim_deg) / denom;
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
  USARTSend(servo->usart_instance, servo_unload, 6, USART_TRANSFER_DMA);
}

void ServoRequestAngle(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != Bus_Servo || servo->usart_instance == NULL) return;
  USARTSend(servo->usart_instance, servo_angle_read, 6, USART_TRANSFER_DMA);
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

void ServoZero(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  /* 让逻辑 0° 对应"当前位置": center = 当前机械角 - trim (reverse 量为 0) */
  float raw = RawFromLogical(servo, servo->angle);
  servo->calib.center_deg = Clamp(raw - servo->calib.trim_deg, 0.0f, servo->calib.range_deg);
  servo->target_deg = 0.0f;
  servo->angle = 0.0f;
  ApplyLogical(servo);  // 脉宽不变(center+trim == 当前机械角)
}

void ServoResetCal(ServoInstance *servo) {
  if (servo == NULL || servo->servo_type != PWM_Servo) return;
  servo->calib.center_deg = servo->calib.range_deg * 0.5f;
  servo->calib.trim_deg = 0.0f;
  servo->calib.scale = 1.0f;
  servo->calib.reverse = 1;
  ApplyLogical(servo);
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

float ServoGetAngle(ServoInstance *servo) {
  return (servo != NULL) ? servo->angle : 0.0f;
}

float ServoGetRawDeg(ServoInstance *servo) {
  return (servo != NULL) ? servo->raw_deg : 0.0f;
}

float ServoGetPulseUs(ServoInstance *servo) {
  return (servo != NULL) ? servo->pulse_us : 0.0f;
}

uint16_t ServoGetRecvAngle(ServoInstance *servo) {
  return (servo != NULL) ? servo->recv_angle : 0;
}

/* ============================== 总线舵机解析 ============================== */

/* 中断上下文: 只解析位置回读帧(0x55 0x55 ... 0x15) */
static void DecodeServo(void) {
  for (uint8_t i = 0; i < servo_idx; i++) {
    ServoInstance *servo = servo_motor_instance[i];
    if (servo == NULL || servo->servo_type != Bus_Servo || servo->usart_instance == NULL) continue;

    uint8_t *buf = servo->usart_instance->recv_buff;
    if (buf[0] == Servo_Frame_First && buf[1] == Servo_Frame_Second) {
      if (buf[3] == 21) {  // 0x15 位置回读应答
        servo->recv_angle = (uint16_t)((buf[7] << 8) | buf[6]);
      }
    }
  }
}
