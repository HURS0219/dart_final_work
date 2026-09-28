/*
 * wifi_motor.c — 多电机同时控制 (每个电机一套独立参数)
 */
#include "wifi_motor.h"

#include <math.h>
#include <stdio.h>

#include "main.h"
#include "robot_config.h"
#include "user_lib.h"
#include "wifi_link.h"
#include "wifi_store.h"

WifiMotorInstance wifi_motor;

MotorParamSet_s g_motor_params[SIMPLE_MOTOR_MAX];
int g_sel = -1;

/* 活动参数 (g_motor_params[g_sel] 的镜像) */
float g_speed_ff;
float g_speed_kp;
float g_speed_ki;
float g_speed_i_limit;
float g_angle_kp;
float g_angle_kd;
float g_angle_slew;
float g_gear_ratio;
float g_angle_deadband;
float g_angle_speed_max;
int g_max_value;

/* 各类型的出厂默认 (用于新加电机时的初始参数) */
static const MotorParamSet_s kDefaults[3] = {
    /* GM6020: ff,kp,ki,ilim, akp, akd, slew, ratio, dead, angVmax, max */
    {90.0f, 30.0f, 10.0f, 5000.0f, 5.0f, 0.0f, 0.0f, 1.0f, 0.3f, 100.0f, 10000},
    /* M3508 */ {2.0f, 8.0f, 2.0f, 3000.0f, 5.0f, 0.0f, 0.0f, 19.2f, 1.0f, 60.0f, 6000},
    /* M2006 */ {3.0f, 10.0f, 2.0f, 3000.0f, 5.0f, 0.0f, 0.0f, 36.0f, 1.0f, 60.0f, 5000},
};

void WifiMotorStoreActive(void) {
  if (g_sel < 0 || g_sel >= SIMPLE_MOTOR_MAX) return;
  MotorParamSet_s* p = &g_motor_params[g_sel];
  p->speed_ff = g_speed_ff;
  p->speed_kp = g_speed_kp;
  p->speed_ki = g_speed_ki;
  p->speed_i_limit = g_speed_i_limit;
  p->angle_kp = g_angle_kp;
  p->angle_kd = g_angle_kd;
  p->angle_slew = g_angle_slew;
  p->gear_ratio = g_gear_ratio;
  p->angle_deadband = g_angle_deadband;
  p->angle_speed_max = g_angle_speed_max;
  p->max_value = g_max_value;
}

void WifiMotorLoadActive(void) {
  if (g_sel < 0 || g_sel >= SIMPLE_MOTOR_MAX) return;
  MotorParamSet_s* p = &g_motor_params[g_sel];
  g_speed_ff = p->speed_ff;
  g_speed_kp = p->speed_kp;
  g_speed_ki = p->speed_ki;
  g_speed_i_limit = p->speed_i_limit;
  g_angle_kp = p->angle_kp;
  g_angle_kd = p->angle_kd;
  g_angle_slew = p->angle_slew;
  g_gear_ratio = p->gear_ratio;
  g_angle_deadband = p->angle_deadband;
  g_angle_speed_max = p->angle_speed_max;
  g_max_value = p->max_value;
}

void WifiMotorSetSelect(int idx) {
  if (idx < 0 || idx >= SIMPLE_MOTOR_MAX || !g_motors[idx].used) return;
  WifiMotorStoreActive();
  g_sel = idx;
  WifiMotorLoadActive();
}

void WifiMotorSetParam(int id, int value) {
  if (g_sel < 0) return;
  if (id == 6) {
    if (value < 0) value = 0;
    if (value > 30000) value = 30000;
    g_max_value = value;
    WifiMotorStoreActive();
    WifiStoreMarkDirty();
    return;
  }
  if (wifi_motor.mode != WIFI_MODE_STOP) return;  // 其余参数: 只在停止时允许改
  switch (id) {
    case 1: g_speed_ff = value / 100.0f; break;
    case 2: g_speed_kp = value / 100.0f; break;
    case 3: g_speed_ki = value / 100.0f; break;
    case 4: g_speed_i_limit = value / 100.0f; break;
    case 5: g_angle_kp = value / 100.0f; break;
    case 7: g_angle_kd = value / 100.0f; break;
    case 8: g_angle_slew = value / 100.0f; break;
    case 10: g_gear_ratio = value / 100.0f; break;
    case 11: g_angle_deadband = value / 100.0f; break;
    case 12: g_angle_speed_max = value / 100.0f; break;
    default: return;
  }
  WifiMotorStoreActive();
  WifiStoreMarkDirty();
}

void WifiMotorInit(void) {
  wifi_motor.mode = WIFI_MODE_STOP;
  wifi_motor.speed_rpm = 0.0f;
  wifi_motor.angle_deg = 0.0f;
  wifi_motor.last_cmd_tick = HAL_GetTick();
  g_sel = -1;

  SimpleMotorBusInit();
  WifiStoreInit();  // 恢复电机列表(含各自参数/零点)与选择
  if (g_sel >= 0) WifiMotorLoadActive();
  SimpleMotorSendAll();
}

void WifiMotorAddMotor(int type, int id) {
  if (wifi_motor.mode != WIFI_MODE_STOP) return;
  if (type < 0 || type > 2) return;
  int idx = SimpleMotorAdd((SimpleMotorType_e)type, (uint8_t)id);
  if (idx < 0) return;
  g_motor_params[idx] = kDefaults[type];  // 初始化该电机的参数
  g_sel = idx;
  WifiMotorLoadActive();
  WifiStoreMarkDirty();
}

void WifiMotorClearMotors(void) {
  if (wifi_motor.mode != WIFI_MODE_STOP) return;
  SimpleMotorClear();
  g_sel = -1;
  WifiStoreMarkDirty();
}

void WifiMotorZeroHere(void) {
  for (int i = 0; i < SIMPLE_MOTOR_MAX; i++)
    if (g_motors[i].used) g_motors[i].zero = g_motors[i].total_angle;
  WifiStoreMarkDirty();
}

/* ---------------- 扫描 ---------------- */
static uint8_t s_scan_active = 0;
static uint32_t s_scan_t0 = 0;

void WifiMotorScanRequest(void) {
  if (wifi_motor.mode == WIFI_MODE_STOP) s_scan_active = 1;
}

int WifiMotorIsScanning(void) { return s_scan_active; }

void WifiMotorScanTask(void) {
  if (!s_scan_active) return;
  if (wifi_motor.mode != WIFI_MODE_STOP) { s_scan_active = 0; return; }

  if (s_scan_t0 == 0) {
    s_scan_t0 = HAL_GetTick();
    SimpleMotorScanBegin();
    return;
  }
  SimpleMotorScanTask();

  if (HAL_GetTick() - s_scan_t0 >= 1000u) {
    uint16_t ids[8];
    uint8_t n = SimpleMotorScanResult(ids, 8);
    static char buf[96];
    int len = snprintf(buf, sizeof(buf), "S,%d", n);
    for (uint8_t i = 0; i < n && len < (int)sizeof(buf) - 8; i++)
      len += snprintf(buf + len, sizeof(buf) - len, ",%d", ids[i]);
    buf[len++] = '\n';
    buf[len] = 0;
    WifiLinkSendBlocking(buf);
    SimpleMotorReconfigFilters();
    s_scan_active = 0;
    s_scan_t0 = 0;
  }
}

void WifiMotorSetCommand(int mode, int value) {
  wifi_motor.last_cmd_tick = HAL_GetTick();
  switch (mode) {
    case WIFI_MODE_STOP:
      wifi_motor.mode = WIFI_MODE_STOP;
      break;
    case WIFI_MODE_SPEED: {
      float rpm = (float)value;
      VAL_LIMIT(rpm, -GM6020_MAX_RPM, GM6020_MAX_RPM);
      wifi_motor.speed_rpm = rpm;
      wifi_motor.mode = WIFI_MODE_SPEED;
      break;
    }
    case WIFI_MODE_ANGLE: {
      float deg = (float)value / WIFI_ANGLE_SCALE;
      VAL_LIMIT(deg, GM6020_ANGLE_MIN, GM6020_ANGLE_MAX);
      wifi_motor.angle_deg = deg;
      wifi_motor.mode = WIFI_MODE_ANGLE;
      break;
    }
    default: break;
  }
}

void WifiMotorTask(void) {
  if (HAL_GetTick() - wifi_motor.last_cmd_tick > WIFI_CMD_TIMEOUT_MS) {
    wifi_motor.mode = WIFI_MODE_STOP;
  }

  if (!s_scan_active) SimpleMotorReadAll();

  for (int i = 0; i < SIMPLE_MOTOR_MAX; i++) {
    SimpleMotor_t* m = &g_motors[i];
    if (!m->used) continue;
    MotorParamSet_s* p = &g_motor_params[i];
    if (p->gear_ratio < 0.01f) p->gear_ratio = 1.0f;

    switch (wifi_motor.mode) {
      case WIFI_MODE_SPEED: {
        float tgt_motor = wifi_motor.speed_rpm * p->gear_ratio;
        float err = tgt_motor - (float)m->rpm;
        m->speed_integral += err * SPEED_CTRL_DT;
        VAL_LIMIT(m->speed_integral, -p->speed_i_limit, p->speed_i_limit);
        float value = p->speed_ff * tgt_motor + p->speed_kp * err + p->speed_ki * m->speed_integral;
        VAL_LIMIT(value, -(float)p->max_value, (float)p->max_value);
        SimpleMotorSet(i, (int16_t)value);
        break;
      }
      case WIFI_MODE_ANGLE: {
        float target = m->zero + wifi_motor.angle_deg * p->gear_ratio;
        float err_out = (target - m->total_angle) / p->gear_ratio;
        float speed_cmd;
        if (fabsf(err_out) < p->angle_deadband) speed_cmd = 0.0f;
        else {
          speed_cmd = p->angle_kp * err_out;
          VAL_LIMIT(speed_cmd, -p->angle_speed_max, p->angle_speed_max);
        }
        float tgt_motor = speed_cmd * p->gear_ratio;
        float err = tgt_motor - (float)m->rpm;
        m->speed_integral += err * SPEED_CTRL_DT;
        VAL_LIMIT(m->speed_integral, -p->speed_i_limit, p->speed_i_limit);
        float value = p->speed_ff * tgt_motor + p->speed_kp * err + p->speed_ki * m->speed_integral;
        VAL_LIMIT(value, -(float)p->max_value, (float)p->max_value);
        SimpleMotorSet(i, (int16_t)value);
        break;
      }
      case WIFI_MODE_STOP:
      default:
        m->speed_integral = 0.0f;
        SimpleMotorSet(i, 0);
        break;
    }
  }

  SimpleMotorSendAll();
}
