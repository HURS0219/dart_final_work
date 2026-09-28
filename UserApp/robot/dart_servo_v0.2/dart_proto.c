/*
 * dart_proto.c — ESP32 <-> C板 串口协议 (ASCII)
 */
#include "dart_proto.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_usart.h"
#include "dart_axis.h"
#include "dart_cfg.h"
#include "main.h"
#include "robot_config.h"

static USARTInstance *s_usart;
static volatile uint32_t s_last_rx;
static volatile uint8_t s_pong_req;
static uint8_t s_link_ok;

static int SplitInts(const char *s, long *out, int max) {
  int n = 0;
  const char *p = s;
  while (*p && n < max) {
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) break;
    out[n++] = v;
    p = end;
    if (*p == ',') p++;
    else break;
  }
  return n;
}

/* 中断上下文: 只做解析与置位, 不阻塞发送 */
static void ProcessCmd(char *buf) {
  char *comma = strchr(buf, ',');
  char *rest = NULL;
  long a[4] = {0};
  int n = 0;

  if (buf[0] == '\0') return;
  s_last_rx = HAL_GetTick();

  if (comma != NULL) {
    *comma = '\0';
    rest = comma + 1;
    n = SplitInts(rest, a, 4);
  }

  if (strcmp(buf, "PING") == 0) {
    s_pong_req = 1u;
  } else if (strcmp(buf, "SAVE") == 0) {
    DartCfgRequestSave();
  } else if (strcmp(buf, "H") == 0) {
    /* 心跳, 仅刷新超时 */
  } else if (strcmp(buf, "MODE") == 0) {
    if (n >= 1) DartAxisSetMode((DartMode_e)a[0]);
  } else if (strcmp(buf, "ANG") == 0) {
    if (n >= 2) DartAxisSetManual((uint8_t)a[0], (float)a[1] / 10.0f);
  } else if (strcmp(buf, "MIX") == 0) {
    if (n >= 3) {
      DartAxisSetMix((float)a[0] / 1000.0f, (float)a[1] / 1000.0f,
                     (float)a[2] / 1000.0f);
    }
  } else if (strcmp(buf, "TRIM") == 0) {
    if (n >= 2) {
      DartAxisSetTrim((uint8_t)a[0], (float)a[1] / 10.0f);
      DartCfgMarkDirty();
    }
  } else if (strcmp(buf, "SCALE") == 0) {
    if (n >= 2) {
      DartAxisSetScale((uint8_t)a[0], (float)a[1] / 1000.0f);
      DartCfgMarkDirty();
    }
  } else if (strcmp(buf, "DIR") == 0) {
    if (n >= 1) {
      uint8_t ch = (uint8_t)a[0];
      DartAxisSetDir(ch, DartAxisGetDir(ch) ? 0u : 1u);
      DartCfgMarkDirty();
    }
  } else if (strcmp(buf, "ZERO") == 0) {
    if (n >= 1) {
      DartAxisZero((uint8_t)a[0]);
      DartCfgMarkDirty();
    }
  } else if (strcmp(buf, "RSTCAL") == 0) {
    if (n >= 1) {
      DartAxisResetCal((uint8_t)a[0]);
      DartCfgMarkDirty();
    }
  }
}

static void DartOnRx(void) {
  char *buf = (char *)s_usart->recv_buff;
  char *p = buf;

  while (*p) {
    char *nl = strchr(p, '\n');
    if (nl != NULL) *nl = '\0';
    {
      size_t len = strlen(p);
      while (len > 0 && (p[len - 1] == '\r' || p[len - 1] == ' ')) {
        p[--len] = '\0';
      }
    }
    ProcessCmd(p);
    if (nl == NULL) break;
    p = nl + 1;
  }
}

void DartProtoInit(void) {
  USART_Init_Config_s cfg = {
      .recv_buff_size = DART_RECV_SIZE,
      .usart_handle = DART_UART_HANDLE,
      .module_callback = DartOnRx,
  };
  s_usart = USARTRegister(&cfg);
  s_last_rx = HAL_GetTick();
}

void DartProtoSend(const char *s) {
  if (s_usart == NULL || s == NULL) return;
  USARTSend(s_usart, (uint8_t *)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}

static void SendTelemetry(void) {
  static char buf[320];
  float mix[3];
  int len;
  uint8_t i;

  DartAxisGetMix(mix);
  len = snprintf(buf, sizeof(buf), "T,%d,%d", (int)DartAxisGetMode(),
                 (int)s_link_ok);
  for (i = 0; i < DART_AXIS_N; i++) {
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d",
                    (int)(DartAxisGetAngle(i) * 10.0f),
                    (int)(DartAxisGetPulse(i) + 0.5f), (int)DartAxisGetDir(i),
                    (int)(DartAxisGetTrim(i) * 10.0f),
                    (int)(DartAxisGetScale(i) * 1000.0f));
  }
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d",
                  (int)(mix[0] * 1000.0f), (int)(mix[1] * 1000.0f),
                  (int)(mix[2] * 1000.0f));
  buf[len++] = '\n';
  buf[len] = '\0';
  DartProtoSend(buf);
}

void DartProtoTask(void) {
  static uint32_t last_tx;

  if (s_pong_req) {
    s_pong_req = 0u;
    DartProtoSend("PONG\n");
  }

  switch (DartCfgTakeResult()) {
    case 1u:
      DartProtoSend("SAVED\n");
      break;
    case 2u:
      DartProtoSend("SAVEERR\n");
      break;
    default:
      break;
  }

  if ((HAL_GetTick() - s_last_rx) > DART_LINK_TIMEOUT_MS) {
    s_link_ok = 0u;
    if (DartAxisGetMode() != DART_MODE_IDLE) DartAxisSetMode(DART_MODE_IDLE);
  } else {
    s_link_ok = 1u;
  }

  if ((HAL_GetTick() - last_tx) >= DART_TELEM_PERIOD_MS) {
    last_tx = HAL_GetTick();
    SendTelemetry();
  }
}
