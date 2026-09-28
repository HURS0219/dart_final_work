/**
 * @file servo_motor.c
 * @author ai
 * @brief PWM 舵机(PTK7350 等)底层驱动实现
 * @version 2.0
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026 SHU SRM all rights reserved
 *
 * @attention 详细说明见 servo_motor.md
 *
 * 标定链路(逻辑角 -> 脉宽), 全部在 ApplyLogical() 内完成:
 *   1) 逻辑角限幅到 ±limit_deg;
 *   2) 加增益与零点偏置:  applied = 逻辑角 * scale + trim_deg   (机械偏角, 单位 deg);
 *   3) 方向取反(可选):    reverse 时 applied = -applied;
 *   4) 机械偏角限幅到 ±half_deg (防止越程);
 *   5) 线性映射成脉宽:    pulse = center_us + applied * (half_us / half_deg);
 *   6) 脉宽硬限幅到 [pulse_min_us, pulse_max_us] 后换算占空比写入底层 PWM。
 *
 * 例: center=1500us, half_us=1000, half_deg=139.5, scale=1, trim=0, reverse=0
 *     逻辑角 30° -> applied=30° -> pulse = 1500 + 30*(1000/139.5) ≈ 1715us
 */
#include "servo_motor.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"

/* 所有已注册的舵机实例(静态数组 + 自增索引, 与 dji_motor 的注册方式一致) */
static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];
static uint8_t servo_idx = 0;

/* ============================== 内部工具 ============================== */

/**
 * @brief 数值限幅: 把 v 限制在 [lo, hi] 内
 */
static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/**
 * @brief 逻辑角对称上限(deg)
 * @note  未显式设置(limit_deg <= 0)时默认取 half_deg, 即允许满行程
 */
static float LimitDeg(const ServoInstance *servo) {
  return (servo->cfg.limit_deg > 0.0f) ? servo->cfg.limit_deg : servo->cfg.half_deg;
}

/**
 * @brief 取 PWM 周期并换算为微秒(us)
 * @note  周期从底层 PWM 实例读取(单位秒); 异常/未设置时回落 20000us(=50Hz)
 */
static float PeriodUs(const ServoInstance *servo) {
  float period_s = (servo->pwm != NULL) ? servo->pwm->period : 0.0f;
  if (period_s <= 0.0f) period_s = 0.02f;
  return period_s * 1000000.0f;
}

/**
 * @brief 计算"当前机械偏角"(相对中位, 单位 deg)
 * @note  applied = 逻辑角*scale + trim, reverse 时取负, 再限幅到 ±half_deg。
 *        仅用于调零判断(是否需要把当前位置记为逻辑 0°)。
 */
static float AppliedOf(const ServoInstance *servo) {
  float applied = servo->angle_deg * servo->cfg.scale + servo->cfg.trim_deg;
  if (servo->cfg.reverse) applied = -applied;
  return Clamp(applied, -servo->cfg.half_deg, servo->cfg.half_deg);
}

/**
 * @brief 把逻辑角换算成脉宽并输出(驱动核心)
 * @param servo        舵机实例
 * @param logical_deg  目标逻辑角(deg)
 * @note  流程见文件头"标定链路"注释, 纯计算 + 写一次占空比, 不做限速
 */
static void ApplyLogical(ServoInstance *servo, float logical_deg) {
  float lim = LimitDeg(servo);
  float applied, pulse;

  /* 1) 逻辑角对称限位: ±limit_deg */
  logical_deg = Clamp(logical_deg, -lim, lim);

  /* 2) 标定: 加增益 scale 与零点 trim; 3) 方向 reverse 时整体取负 */
  applied = logical_deg * servo->cfg.scale + servo->cfg.trim_deg;
  if (servo->cfg.reverse) applied = -applied;

  /* 4) 机械偏角限幅到 ±half_deg, 避免超出舵机行程 */
  applied = Clamp(applied, -servo->cfg.half_deg, servo->cfg.half_deg);

  /* 5) 机械偏角 -> 脉宽: 以中位 center_us 为基准, 每度对应 (half_us/half_deg) 微秒 */
  pulse = servo->cfg.center_us + applied * (servo->cfg.half_us / servo->cfg.half_deg);

  /* 6) 脉宽硬限幅(超出会损坏/翻转), 再换算占空比写入 PWM */
  pulse = Clamp(pulse, servo->cfg.pulse_min_us, servo->cfg.pulse_max_us);

  servo->angle_deg = logical_deg;
  servo->pulse_us = pulse;
  if (servo->pwm != NULL) PWMSetDutyRatio(servo->pwm, pulse / PeriodUs(servo));
}

/* ============================== 对外接口 ============================== */

/**
 * @brief 注册一路 PWM 舵机
 * @param config 初始化配置
 * @return 成功返回实例指针, 失败(参数为空/超过实例上限)返回 NULL
 * @note  未填写的关键参数会回落到宏默认值, 避免后续除零或方向异常; 注册后立即输出中位。
 */
ServoInstance *ServoInit(Servo_Init_Config_s *config) {
  ServoInstance *servo;

  if (config == NULL) return NULL;
  if (servo_idx >= SERVO_MOTOR_CNT) {
    LOGERROR("[servo] exceed max instance count %d", SERVO_MOTOR_CNT);
    return NULL;
  }

  servo = (ServoInstance *)malloc(sizeof(ServoInstance));
  memset(servo, 0, sizeof(ServoInstance));
  servo->cfg = *config;

  /* 缺省/非法参数回落: half_deg 必须 >0(后面要做除数), 脉宽上界须大于下界 */
  if (servo->cfg.center_us <= 0.0f) servo->cfg.center_us = SERVO_CFG_CENTER_US;
  if (servo->cfg.half_us <= 0.0f) servo->cfg.half_us = SERVO_CFG_HALF_US;
  if (servo->cfg.half_deg <= 0.0f) servo->cfg.half_deg = SERVO_CFG_HALF_DEG;
  if (servo->cfg.pulse_max_us <= servo->cfg.pulse_min_us) {
    servo->cfg.pulse_min_us = SERVO_CFG_PULSE_MIN_US;
    servo->cfg.pulse_max_us = SERVO_CFG_PULSE_MAX_US;
  }
  if (servo->cfg.scale <= 0.0f) servo->cfg.scale = SERVO_CFG_SCALE;

  servo->pwm = PWMRegister(&servo->cfg.pwm);  // 底层注册 PWM 通道
  servo->last_time_s = DWT_GetTimeline_s();
  ApplyLogical(servo, 0.0f);  // 上电即输出中位(逻辑 0°)

  servo_motor_instance[servo_idx++] = servo;
  return servo;
}

/**
 * @brief 设置逻辑角(deg)
 * @note  先按 ±limit_deg 限位:
 *        - 未启用速率限幅(rate_limit_dps<=0): 立即输出;
 *        - 启用速率限幅: 只更新 target_deg, 由 ServoTask() 平滑逼近。
 */
void ServoSetAngle(ServoInstance *servo, float angle) {
  float lim;
  if (servo == NULL) return;

  lim = LimitDeg(servo);
  angle = Clamp(angle, -lim, lim);
  servo->target_deg = angle;
  if (servo->cfg.rate_limit_dps <= 0.0f) ApplyLogical(servo, angle);  // 不限速: 立即到位
}

/**
 * @brief 直接设置脉宽(us), 用于标定/开环测试
 * @note  自动按 pulse_min/max 硬限幅; 并反推逻辑角, 保持 angle_deg/target_deg 与输出一致
 *        (反向的标定链路: pulse -> applied -> before-reverse -> 逻辑角)。
 */
void ServoSetPulseUs(ServoInstance *servo, float pulse_us) {
  float applied, logical;
  if (servo == NULL) return;

  pulse_us = Clamp(pulse_us, servo->cfg.pulse_min_us, servo->cfg.pulse_max_us);
  servo->pulse_us = pulse_us;
  if (servo->pwm != NULL) PWMSetDutyRatio(servo->pwm, pulse_us / PeriodUs(servo));

  /* 反推: 脉宽 -> 机械偏角(反向若开启则取负) -> 去掉 trim/scale 得到逻辑角 */
  applied = (pulse_us - servo->cfg.center_us) * (servo->cfg.half_deg / servo->cfg.half_us);
  if (servo->cfg.reverse) applied = -applied;
  logical = Clamp((applied - servo->cfg.trim_deg) / servo->cfg.scale, -LimitDeg(servo), LimitDeg(servo));
  servo->angle_deg = logical;
  servo->target_deg = logical;
}

/**
 * @brief 舵机周期任务: 对启用速率限幅的实例推进输出
 * @note  dt 由 DWT 时间戳差分得到; 每周期最多移动 rate_limit_dps*dt 度,
 *        限幅到 [1ms, 50ms] 防止时间抖动。未启用限速的实例直接跳过。
 *        建议放入 app 周期任务(100Hz~1kHz)。
 */
void ServoTask(void) {
  float now = DWT_GetTimeline_s();
  uint8_t i;

  for (i = 0; i < servo_idx; i++) {
    ServoInstance *servo = servo_motor_instance[i];
    float dt, max_delta, err;

    if (servo == NULL) continue;

    dt = now - servo->last_time_s;
    servo->last_time_s = now;
    if (servo->cfg.rate_limit_dps <= 0.0f) continue;  // 未启用限速

    if (dt <= 0.0f) dt = 0.001f;   // 时间抖动兜底
    if (dt > 0.05f) dt = 0.05f;
    max_delta = servo->cfg.rate_limit_dps * dt;  // 本周期允许的最大角度增量

    err = servo->target_deg - servo->angle_deg;  // 剩余角度, 限幅后累加
    if (err > max_delta) err = max_delta;
    if (err < -max_delta) err = -max_delta;
    ApplyLogical(servo, servo->angle_deg + err);
  }
}

/**
 * @brief 把"当前位置"记为逻辑 0°(调零, 物理不动)
 * @return 1 成功; 0 未启用调零/超窗口/无效
 * @note  两道门槛:
 *        1) zero_enable 必须为 1(可用可不用, 由 config 决定);
 *        2) 当前机械偏角 |applied| 必须 <= SERVO_CFG_ZERO_WINDOW_DEG, 防止在极端位置误调零。
 *        通过后令 trim 抵消当前偏角, 使 out=0 时机械位置不变, 逻辑角归零 (reverse 在 trim 之后)。
 */
uint8_t ServoZero(ServoInstance *servo) {
  float applied;

  if (servo == NULL) return 0;
  if (!servo->cfg.zero_enable) return 0;  // 调零功能未启用

  applied = AppliedOf(servo);
  if (applied < -SERVO_CFG_ZERO_WINDOW_DEG || applied > SERVO_CFG_ZERO_WINDOW_DEG) return 0;  // 超出调零窗口

  /* 让逻辑 0° 对应当前位置: out=0 时 applied 不变 -> trim = ±applied (reverse 在 trim 之后) */
  servo->cfg.trim_deg = servo->cfg.reverse ? -applied : applied;
  servo->target_deg = 0.0f;
  servo->angle_deg = 0.0f;
  ApplyLogical(servo, 0.0f);
  return 1;
}

/**
 * @brief 设置逻辑角对称限位 ±limit_deg (<=0 表示恢复默认 half_deg), 并立即按新限位重发
 */
void ServoSetLimit(ServoInstance *servo, float limit_deg) {
  if (servo == NULL) return;
  servo->cfg.limit_deg = (limit_deg > 0.0f) ? limit_deg : 0.0f;
  ApplyLogical(servo, servo->angle_deg);
}

/* ------------------------------ 标定(在线调参) ------------------------------ */

/**
 * @brief 批量写入标定并立即生效
 * @note  逐字段合法性检查: 非法值(如 half_deg<=0 / scale<=0 / 脉宽上下界颠倒)保持原值,
 *        避免除零或方向异常; 最后按当前"目标角"重发(遵守限速/限位)。
 */
void ServoSetCalib(ServoInstance *servo, const Servo_Calib_s *calib) {
  if (servo == NULL || calib == NULL) return;

  if (calib->center_us > 0.0f) servo->cfg.center_us = calib->center_us;
  if (calib->half_us > 0.0f) servo->cfg.half_us = calib->half_us;
  if (calib->half_deg > 0.0f) servo->cfg.half_deg = calib->half_deg;
  if (calib->pulse_min_us > 0.0f && calib->pulse_max_us > calib->pulse_min_us) {
    servo->cfg.pulse_min_us = calib->pulse_min_us;
    servo->cfg.pulse_max_us = calib->pulse_max_us;
  }
  if (calib->scale > 0.0f) servo->cfg.scale = calib->scale;
  servo->cfg.trim_deg = calib->trim_deg;
  servo->cfg.limit_deg = (calib->limit_deg > 0.0f) ? calib->limit_deg : 0.0f;
  servo->cfg.rate_limit_dps = calib->rate_limit_dps;
  servo->cfg.reverse = (calib->reverse != 0) ? 1 : 0;
  servo->cfg.zero_enable = (calib->zero_enable != 0) ? 1 : 0;

  ServoSetAngle(servo, servo->target_deg);  // 立即生效(遵守限位/限速)
}

/**
 * @brief 取当前标定快照, 可直接交给 flash_store_save 持久化
 */
void ServoGetCalib(ServoInstance *servo, Servo_Calib_s *out) {
  if (servo == NULL || out == NULL) return;
  out->center_us = servo->cfg.center_us;
  out->half_us = servo->cfg.half_us;
  out->half_deg = servo->cfg.half_deg;
  out->pulse_min_us = servo->cfg.pulse_min_us;
  out->pulse_max_us = servo->cfg.pulse_max_us;
  out->scale = servo->cfg.scale;
  out->trim_deg = servo->cfg.trim_deg;
  out->limit_deg = servo->cfg.limit_deg;
  out->rate_limit_dps = servo->cfg.rate_limit_dps;
  out->reverse = servo->cfg.reverse;
  out->zero_enable = servo->cfg.zero_enable;
}

void ServoSetScale(ServoInstance *servo, float scale) {
  if (servo == NULL || scale <= 0.0f) return;
  servo->cfg.scale = scale;
  ServoSetAngle(servo, servo->target_deg);
}

void ServoSetTrim(ServoInstance *servo, float trim_deg) {
  if (servo == NULL) return;
  servo->cfg.trim_deg = trim_deg;
  ServoSetAngle(servo, servo->target_deg);
}

void ServoSetReverse(ServoInstance *servo, uint8_t reverse) {
  if (servo == NULL) return;
  servo->cfg.reverse = reverse ? 1 : 0;
  ServoSetAngle(servo, servo->target_deg);
}

void ServoSetRateLimit(ServoInstance *servo, float dps) {
  if (servo == NULL) return;
  servo->cfg.rate_limit_dps = dps;
}

/**
 * @brief 使能: 启动 PWM 输出并按当前逻辑角重发一次
 */
void ServoEnable(ServoInstance *servo) {
  if (servo == NULL || servo->pwm == NULL) return;
  PWMStart(servo->pwm);
  ApplyLogical(servo, servo->angle_deg);
}

/**
 * @brief 失能: 停止 PWM 输出(舵机失去保持力, 原地自由)
 */
void ServoDisable(ServoInstance *servo) {
  if (servo == NULL || servo->pwm == NULL) return;
  PWMStop(servo->pwm);
}

/* ============================== 查询 ============================== */

float ServoGetAngle(ServoInstance *servo) { return (servo != NULL) ? servo->angle_deg : 0.0f; }
float ServoGetTarget(ServoInstance *servo) { return (servo != NULL) ? servo->target_deg : 0.0f; }
float ServoGetPulseUs(ServoInstance *servo) { return (servo != NULL) ? servo->pulse_us : 0.0f; }
