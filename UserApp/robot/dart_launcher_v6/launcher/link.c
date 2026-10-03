/*
 * launcher/link.c — 上位机通信组件实现
 * =============================================================================
 * 移植 dart_launcher_web_v5_HIK/dart_link.c 的协议与遥测格式(保证 esp32/www 与
 * Tools/dart_launcher_web_pc(J-Link RTT) 都无需改), 但**只解析成 Launcher_Cmd_s**,
 * 不再直接调用电机/舵机/状态机(那是 ctrl 的职责)。
 * =============================================================================
 */
#include "link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "main.h"
#include "robot_config.h"
#include "usart.h"

/* RTT 专用通道(与 bsp_log 日志通道 0 分开) */
#define DART_RTT_CH 1
static char s_rtt_up[4096];
static char s_rtt_down[512];
static int s_rtt_inited = 0;

static USARTInstance* s_usart = NULL;
static uint32_t s_last_rx = 0;

static Launcher_Cmd_s s_parse; /* 累积本次解析结果 */
static uint8_t s_ready = 0;

static Launcher_Telemetry_s s_telem; /* 遥测快照(robot 提供) */

/* ============================ 下行解析 ============================ */
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

static void Reply(const char* s) { LinkSend(s); }

static void ScanReply(void) {
  char out[64];
  int cnt = 0;
  for (int i = 0; i < s_telem.n && i < 4; i++)
    if (s_telem.motor[i].online) cnt++;
  int len = snprintf(out, sizeof(out), "S,%d", cnt);
  for (int i = 0; i < s_telem.n && i < 4; i++)
    if (s_telem.motor[i].online) len += snprintf(out + len, sizeof(out) - len, ",%d", i);
  len += snprintf(out + len, sizeof(out) - len, "\n");
  Reply(out);
}

static void ProcessOne(char* buf) {
  if (buf[0] == '\0') return;

  if (buf[0] == 'P' && buf[1] == 'I' && buf[2] == 'N' && buf[3] == 'G') {
    Reply("PONG\n");
    return;
  }
  if (buf[0] == 'S' && buf[1] == 'A' && buf[2] == 'V' && buf[3] == 'E') {
    s_parse.req_save = 1;
    s_ready = 1;
    return;
  }
  if (buf[0] == 'H' && buf[1] == '\0') return; /* 心跳: 活性已在收包时刷新 */
  if (buf[0] == 'S' && buf[1] == '\0') {
    ScanReply();
    return;
  }

  char cmd = buf[0];
  long a[4] = {0};
  int n = (buf[1] == ',') ? SplitInts(buf + 2, a, 4) : 0;

  switch (cmd) {
    case 'Z':
      s_parse.req_zero = 1;
      s_parse.req_zero_slot = (n > 0) ? (int8_t)a[0] : -1;
      break;
    case 'M':
      if (n >= 3) {
        s_parse.set_motor = 1;
        s_parse.motor_slot = (int8_t)a[0];
        s_parse.motor_mode = (uint8_t)a[1];
        if (a[1] == 3)
          s_parse.motor_value = (float)a[2] / 100.0f; /* 圈 */
        else
          s_parse.motor_value = (float)a[2] / 10.0f; /* rpm / deg */
      }
      break;
    case 'P':
      if (n >= 3) {
        s_parse.set_param = 1;
        s_parse.param_slot = (int8_t)a[0];
        s_parse.param_id = (uint8_t)a[1];
        s_parse.param_value = (float)a[2] / 100.0f;
      }
      break;
    case 'R':
      if (n >= 1) {
        s_parse.req_reset = 1;
        s_parse.req_reset_slot = (int8_t)a[0];
      }
      break;
    case 'D':
      if (n >= 1) {
        s_parse.req_dir = 1;
        s_parse.req_dir_slot = (int8_t)a[0];
      }
      break;
    case 'W':
      if (n >= 1) {
        s_parse.set_turns = 1;
        s_parse.turns = (float)a[0] / 100.0f;
      }
      break;
    case 'Y':
      if (n >= 1) {
        s_parse.set_yaw_mode = 1;
        s_parse.yaw_mode = (uint8_t)a[0];
      }
      break;
    case 'A':
      if (n >= 1) {
        s_parse.set_aim_rpm = 1;
        s_parse.aim_rpm = (float)a[0] / 100.0f;
      }
      break;
    case 'C':
      if (n >= 1) {
        s_parse.vis_set = 1;
        s_parse.vis_x = (int16_t)a[0];
        s_parse.vis_center = (n >= 2) ? (int16_t)a[1] : 0;
      }
      break;
    case 'V':
      if (n >= 1) {
        s_parse.servo_op = (uint8_t)a[0];
        s_parse.servo_arg1 = (n >= 2) ? (float)a[1] / 10.0f : 0.0f;
        s_parse.servo_arg2 = (n >= 3) ? (float)a[2] / 10.0f : 0.0f;
      }
      break;
    case 'G':
      if (n >= 1) s_parse.fsm_op = (uint8_t)a[0];
      break;
    default:
      Reply("ERR\n");
      return;
  }
  s_ready = 1;
}

/* 一段(可能多行)文本 -> 逐行 ProcessOne */
static void ProcessLines(char* buf) {
  char* p = buf;
  while (*p) {
    char* nl = strchr(p, '\n');
    if (nl != NULL) *nl = '\0';
    size_t len = strlen(p);
    while (len > 0 && (p[len - 1] == '\r' || p[len - 1] == ' ')) p[--len] = '\0';
    if (len > 0) ProcessOne(p);
    if (nl == NULL) break;
    p = nl + 1;
  }
}

static void LinkRxCallback(void) {
  s_last_rx = HAL_GetTick();
  ProcessLines((char*)s_usart->recv_buff);
}

/* ---- J-Link RTT 链路(可选, 与串口并存) ---- */
static char s_rtt_line[128];
static int s_rtt_len = 0;
static void LinkRttPoll(void) {
  char b[64];
  unsigned n = SEGGER_RTT_Read(DART_RTT_CH, b, sizeof(b));
  for (unsigned i = 0; i < n; i++) {
    char c = b[i];
    if (c == '\n' || c == '\r') {
      if (s_rtt_len > 0) {
        s_rtt_line[s_rtt_len] = 0;
        s_last_rx = HAL_GetTick();
        ProcessOne(s_rtt_line);
        s_rtt_len = 0;
      }
    } else if (s_rtt_len < (int)sizeof(s_rtt_line) - 1) {
      s_rtt_line[s_rtt_len++] = c;
    } else {
      s_rtt_len = 0;
    }
  }
}

/* ============================ 上行遥测 ============================ */
static void SendTelemetry(void) {
  static char buf[1100];
  int len = snprintf(buf, sizeof(buf), "F,%d", s_telem.n);

  for (int i = 0; i < s_telem.n && i < 4; i++) {
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    s_telem.motor[i].slot, s_telem.motor[i].type, s_telem.motor[i].id,
                    s_telem.motor[i].online, s_telem.motor[i].dir, s_telem.motor[i].mode,
                    s_telem.motor[i].target100, s_telem.motor[i].rpm, s_telem.motor[i].angle100,
                    s_telem.motor[i].turns100, s_telem.motor[i].temp, s_telem.motor[i].cur);
  }
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", s_telem.servo.cur10,
                  s_telem.servo.state, s_telem.servo.std10, s_telem.servo.prep10);
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d", s_telem.task.spring100,
                  s_telem.task.step, s_telem.task.yaw, s_telem.task.estop, s_telem.task.aim100);
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", s_telem.vis.x, s_telem.vis.ok,
                  s_telem.vis.center, s_telem.vis.err);
  for (int i = 0; i < s_telem.n && i < 4; i++)
    for (int j = 0; j < 11; j++)
      len += snprintf(buf + len, sizeof(buf) - len, ",%d", s_telem.param[i][j]);

  buf[len++] = '\n';
  buf[len] = 0;
  LinkSend(buf);
}

/* ============================ 生命周期 ============================ */
void LinkInit(void) {
  memset(&s_parse, 0, sizeof(s_parse));
  memset(&s_telem, 0, sizeof(s_telem));
  s_ready = 0;
  s_last_rx = 0;

  if (!s_rtt_inited) {
    SEGGER_RTT_ConfigUpBuffer(DART_RTT_CH, "DARTLNK", s_rtt_up, sizeof(s_rtt_up),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    SEGGER_RTT_ConfigDownBuffer(DART_RTT_CH, "DARTLNK", s_rtt_down, sizeof(s_rtt_down),
                                SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    s_rtt_inited = 1;
  }

  USART_Init_Config_s cfg = {
      .recv_buff_size = DART_RECV_SIZE,
      .usart_handle = DART_UART_HANDLE,
      .module_callback = LinkRxCallback,
  };
  s_usart = USARTRegister(&cfg);
}

void LinkTask(void) {
  LinkRttPoll();

  static uint32_t last_fb = 0;
  if (HAL_GetTick() - last_fb >= DART_FB_PERIOD_MS) {
    last_fb = HAL_GetTick();
    SendTelemetry();
  }
}

uint8_t LinkGetCmd(Launcher_Cmd_s* out) {
  if (!s_ready) return 0;
  if (out != NULL) {
    s_parse.tick = HAL_GetTick();
    *out = s_parse;
  }
  memset(&s_parse, 0, sizeof(s_parse));
  s_ready = 0;
  return 1;
}

uint8_t LinkIsAlive(void) {
  if (s_last_rx == 0) return 1; /* 从未收到 -> 视为正常(脱机调试) */
  return (HAL_GetTick() - s_last_rx <= DART_CMD_TIMEOUT_MS) ? 1u : 0u;
}

void LinkUpdateTelemetry(const Launcher_Telemetry_s* t) {
  if (t != NULL) s_telem = *t;
}

void LinkSend(const char* s) {
  if (s == NULL) return;
  SEGGER_RTT_WriteString(DART_RTT_CH, s);
  if (s_usart == NULL) return;
  USARTSend(s_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}
