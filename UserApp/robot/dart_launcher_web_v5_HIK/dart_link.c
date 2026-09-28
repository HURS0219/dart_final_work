/*
 * dart_link.c — ESP32 <-> C板 串口协议 (v3)
 *
 * 命令同 v2:
 *   PING / S / Z[,slot] / M,slot,mode,value / P,slot,id,value / R,slot / D,slot
 *   SAVE / W,turns100 / Y,mode / C,x,center / V,a,b / G,cmd / H
 * 遥测:
 *   F,<n>,<每电机12项>*n,<舵机4>,<任务4>,<视觉4>,<每电机11参数*100>*n
 *     电机12: slot,type,id,online,dir,mode,target100,rpm,angle100,turns100,temp,cur
 */
#include "dart_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_usart.h"
#include "bsp_dwt.h"
#include "dart_fsm.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "dart_store.h"
#include "dart_vision.h"
#include "main.h"
#include "robot_config.h"

static USARTInstance* s_usart = NULL;
static uint32_t s_last_cmd = 0;

static int SplitInts(const char* s, long* out, int max) {
  int n = 0;
  const char* p = s;
  while (*p && n < max) {
    char* end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) break;
    out[n++] = v;
    p = end;
    if (*p == ',') p++;
    else break;
  }
  return n;
}

static void ProcessCmd(char* buf) {
  if (buf[0] == '\0') return;

  if (buf[0] == 'P' && buf[1] == 'I' && buf[2] == 'N' && buf[3] == 'G') {
    DartLinkSendBlocking("PONG\n");
    return;
  }
  if (buf[0] == 'S' && buf[1] == 'A' && buf[2] == 'V' && buf[3] == 'E') {
    DartStoreMarkDirtyNow();
    return;
  }

  char cmd = buf[0];
  long a[4] = {0};
  int n = (buf[1] == ',') ? SplitInts(buf + 2, a, 4) : 0;

  switch (cmd) {
    case 'Z':
      MotorZero(n > 0 ? (int)a[0] : -1);
      DartStoreMarkDirty();
      break;
    case 'M':
      if (n >= 3) {
        float v = 0.0f;
        if (a[1] == 1) v = (float)a[2] / 10.0f;
        else if (a[1] == 2) v = (float)a[2] / 10.0f;
        else if (a[1] == 3) v = (float)a[2] / 100.0f;
        MotorSet((int)a[0], (int)a[1], v);
      }
      break;
    case 'P':
      if (n >= 3) {
        MotorSetParam((int)a[0], (int)a[1], (int)a[2]);
        DartStoreMarkDirty();
      }
      break;
    case 'R':
      if (n >= 1) {
        MotorResetParams((int)a[0]);
        DartStoreMarkDirty();
      }
      break;
    case 'D':
      if (n >= 1) {
        MotorDirToggle((int)a[0]);
        DartStoreMarkDirty();
      }
      break;
    case 'W':
      if (n >= 1) {
        g_spring_turns = (float)a[0] / 100.0f;
        if (g_spring_turns < 0) g_spring_turns = 0;
        if (g_spring_turns > DART_SPRING_MAX_TURNS) g_spring_turns = DART_SPRING_MAX_TURNS;
        DartStoreMarkDirty();
      }
      break;
    case 'Y':
      if (n >= 1) {
        g_yaw_mode = (int)a[0];
        DartStoreMarkDirty();
      }
      break;
    case 'C':
      if (n >= 1) DartVisionSet((int)a[0], n >= 2 ? (int)a[1] : 0);
      break;
    case 'A':
      if (n >= 1) {
        g_yaw_aim_rpm = (float)a[0] / 100.0f;
        if (g_yaw_aim_rpm < 1.0f) g_yaw_aim_rpm = 1.0f;
        if (g_yaw_aim_rpm > 300.0f) g_yaw_aim_rpm = 300.0f;
        DartStoreMarkDirty();
      }
      break;
    case 'V':
      if (n >= 1) {
        switch (a[0]) {
          case 0: DartServoSetPos(DART_SERVO_STD, (float)a[1] / 10.0f); DartStoreMarkDirty(); break;
          case 1: DartServoSetPos(DART_SERVO_PREP, (float)a[1] / 10.0f); DartStoreMarkDirty(); break;
          case 2: DartServoGo(DART_SERVO_STD); break;
          case 3: DartServoGo(DART_SERVO_PREP); break;
          case 4: DartServoSetDeg((float)a[1] / 10.0f); break;
          case 5: DartServoZero(); DartStoreMarkDirty(); break;
          default: break;
        }
      }
      break;
    case 'G':
      if (n >= 1) {
        switch (a[0]) {
          case 0: DartFsmSpringGo(0); break;
          case 1: DartFsmSpringGo(1); break;
          case 2: DartFsmServoGo(DART_SERVO_STD); break;
          case 3: DartFsmServoGo(DART_SERVO_PREP); break;
          case 10: DartFsmAutoStart(); break;
          case 11: DartFsmAutoStop(); break;
          case 12: DartFsmSetEstop(1); break;
          case 13: DartFsmSetEstop(0); break;
          default: break;
        }
      }
      break;
    default:
      break;
  }
}

/* 一个空闲帧里可能含多条 '\n' 分隔的命令, 逐条解析 */
static void DartDecode(void) {
  char* buf = (char*)s_usart->recv_buff;
  s_last_cmd = HAL_GetTick();

  char* p = buf;
  while (*p) {
    char* nl = strchr(p, '\n');
    if (nl != NULL) *nl = '\0';
    size_t len = strlen(p);
    while (len > 0 && (p[len - 1] == '\r' || p[len - 1] == ' ')) p[--len] = '\0';
    ProcessCmd(p);
    if (nl == NULL) break;
    p = nl + 1;
  }
}

static void SendTelemetry(void) {
  static char buf[1100];
  int len = snprintf(buf, sizeof(buf), "F,%d", MOTOR_COUNT);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    MotorAxis* m = &Axis[i];
    DJIMotorInstance* inst = m->inst;
    float rpm_out = inst->measure.speed_aps / 6.0f / (m->ratio > 0.01f ? m->ratio : 1.0f);
    int mode = (int)m->mode;
    int target100 = (mode == MODE_STOP) ? 0 : (int)(m->target * 100.0f);
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", i,
                    (int)inst->motor_type, (int)inst->motor_can_instance->tx_id, (int)m->online,
                    (inst->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? 1 : 0,
                    mode, target100, (int)rpm_out, (int)(MotorOutAngle(i) * 100.0f),
                    (int)(MotorOutTurns(i) * 100.0f), (int)inst->measure.temperature,
                    (int)inst->measure.real_current);
  }

  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", (int)(g_servo_cur_deg * 10.0f),
                  g_servo_state, (int)(g_servo_std_deg * 10.0f), (int)(g_servo_prep_deg * 10.0f));
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d", (int)(g_spring_turns * 100.0f),
                  g_task_step, g_yaw_mode, g_estop, (int)(g_yaw_aim_rpm * 100.0f));
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", g_vis_x, g_vis_ok, g_vis_center,
                  g_vis_err);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    float p[11];
    MotorReadParams(i, p);
    for (int j = 0; j < 11; j++)
      len += snprintf(buf + len, sizeof(buf) - len, ",%d", (int)(p[j] * 100.0f));
  }

  buf[len++] = '\n';
  buf[len] = 0;
  DartLinkSend(buf);
}

void DartLinkInit(void) {
  USART_Init_Config_s cfg = {
      .recv_buff_size = DART_RECV_SIZE,
      .usart_handle = DART_UART_HANDLE,
      .module_callback = DartDecode,
  };
  s_usart = USARTRegister(&cfg);
}

void DartLinkTask(void) {
  if (DartStoreTakeSaved()) DartLinkSendBlocking("SAVED\n");

  static uint8_t timed_out = 0;
  if (s_last_cmd != 0 && HAL_GetTick() - s_last_cmd > DART_CMD_TIMEOUT_MS) {
    if (!timed_out) {
      timed_out = 1;
      MotorsStop();
      DartFsmAutoStop();
    }
  } else {
    timed_out = 0;
  }

  static uint32_t last_fb = 0;
  if (HAL_GetTick() - last_fb >= DART_FB_PERIOD_MS) {
    last_fb = HAL_GetTick();
    SendTelemetry();
  }
}

void DartLinkSend(const char* s) {
  if (s_usart == NULL || s == NULL) return;
  USARTSend(s_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}

void DartLinkSendBlocking(const char* s) {
  if (s_usart == NULL || s == NULL) return;
  uint32_t t0 = HAL_GetTick();
  while (s_usart->usart_handle->gState != HAL_UART_STATE_READY) {
    if (HAL_GetTick() - t0 > 50) break;
  }
  USARTSend(s_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_BLOCKING);
}
