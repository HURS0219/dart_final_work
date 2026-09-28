/*
 * dart_link.c — 制导飞镖 <-> ESP32 串口协议 (USART6)
 *
 * 命令 (ESP32 -> C 板, '\n' 结尾):
 *   PING                  -> PONG
 *   SAVE                  -> 立即保存参数
 *   T,<state>             设置舵面状态机 (0 BOOT 1 NEUTRAL 2 ACTIVE 3 TEST 4 MANUAL 5 FAULT)
 *   X,<p*100>,<y*100>,<r*100>  设置混控指令 (-1..1 -> -100..100)
 *   V,<idx>,<deg*10>      手动设置第 idx 路逻辑偏角
 *   Z,<idx>               第 idx 路取零点 (当前位置设为中立)
 *   P,<id>,<val*100>      设置运行参数 (DartCfg, id 见 dart_cfg.h)
 *   R                     参数恢复默认
 *   C,<x>,<y>[,<w>,<h>]   注入视觉目标坐标
 *   H                     心跳 (刷新链路超时)
 *
 * 遥测 (C 板 -> ESP32, 150ms):
 *   F,<state>,<failsafe>,<link>,<visok>,
 *     <defl0..3 *10>,<raw0..3 *10>,<mix*100>,<roll/pitch/yaw *10>,
 *     <visx>,<visy>,<NP>,<param0..NP-1 *100>
 */
#include "dart_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_usart.h"
#include "dart_app.h"
#include "dart_cfg.h"
#include "dart_config.h"
#include "dart_surface.h"
#include "main.h"
#include "usart.h"

static USARTInstance *s_usart = NULL;
static uint32_t s_last_cmd = 0;
static int s_link_ok = 1;

static int SplitInts(const char *s, long *out, int max)
{
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

static void ProcessCmd(char *buf)
{
    char cmd;
    long a[6] = {0};
    int n;

    if (buf[0] == '\0') return;
    s_last_cmd = HAL_GetTick();

    if (strncmp(buf, "PING", 4) == 0) {
        DartLinkSend("PONG\n");
        return;
    }
    if (strncmp(buf, "SAVE", 4) == 0) {
        DartCfgSaveNow();
        return;
    }

    cmd = buf[0];
    n = (buf[1] == ',') ? SplitInts(buf + 2, a, 6) : 0;

    switch (cmd) {
    case 'T':
        if (n >= 1) DartSurfaceSetState((surface_state_t)a[0]);
        break;
    case 'X':
        if (n >= 3) DartSurfaceSetMix((float)a[0] / 100.0f, (float)a[1] / 100.0f,
                                      (float)a[2] / 100.0f);
        break;
    case 'V':
        if (n >= 2) DartSurfaceSetManualOne((int)a[0], (float)a[1] / 10.0f);
        break;
    case 'Z':
        if (n >= 1) DartSurfaceZero((int)a[0]);
        break;
    case 'P':
        if (n >= 2) DartCfgSetParam((int)a[0], (float)a[1] / 100.0f);
        break;
    case 'R':
        DartCfgReset();
        break;
    case 'C':
        if (n >= 2) DartAppInjectVision((int)a[0], (int)a[1], n >= 3 ? (int)a[2] : 24,
                                        n >= 4 ? (int)a[3] : 18);
        break;
    case 'H':
        /* 心跳: 仅刷新 s_last_cmd */
        break;
    default:
        break;
    }
}

/* 一个空闲帧里可能含多条 '\n' 分隔的命令, 逐条解析 */
static void DartDecode(void)
{
    char *buf = (char *)s_usart->recv_buff;
    char *p = buf;

    while (*p) {
        char *nl = strchr(p, '\n');
        size_t len;
        if (nl != NULL) *nl = '\0';
        len = strlen(p);
        while (len > 0 && (p[len - 1] == '\r' || p[len - 1] == ' ')) p[--len] = '\0';
        ProcessCmd(p);
        if (nl == NULL) break;
        p = nl + 1;
    }
}

static void SendTelemetry(void)
{
    static char buf[900];
    const dart_state_t *s = DartAppGetState();
    int len;
    int i;

    len = snprintf(buf, sizeof(buf), "F,%d,%d,%d,%d",
                   s->surface_state, s->surface_failsafe, DartLinkIsOk(), s->vision_online);

    for (i = 0; i < 4; i++)
        len += snprintf(buf + len, sizeof(buf) - len, ",%d", (int)(s->servo_deg[i] * 10.0f));
    for (i = 0; i < 4; i++)
        len += snprintf(buf + len, sizeof(buf) - len, ",%d", (int)(s->servo_raw[i] * 10.0f));

    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d",
                    (int)(s->mix_pitch * 100.0f), (int)(s->mix_yaw * 100.0f),
                    (int)(s->mix_roll * 100.0f));
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d",
                    (int)(s->att.roll_deg * 10.0f), (int)(s->att.pitch_deg * 10.0f),
                    (int)(s->att.yaw_deg * 10.0f));
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d",
                    s->target_x, s->target_y, DART_P_COUNT);

    for (i = 0; i < DART_P_COUNT; i++)
        len += snprintf(buf + len, sizeof(buf) - len, ",%d", (int)(DartCfgGetParam(i) * 100.0f));

    buf[len++] = '\n';
    buf[len] = '\0';
    DartLinkSend(buf);
}

void DartLinkInit(void)
{
    USART_Init_Config_s cfg = {
        .recv_buff_size = DART_RECV_SIZE,
        .usart_handle = &huart6,
        .module_callback = DartDecode,
    };
    s_usart = USARTRegister(&cfg);
    s_last_cmd = 0;
    s_link_ok = 1;
}

int DartLinkIsOk(void)
{
    return s_link_ok;
}

void DartLinkTask(void)
{
    static uint32_t last_fb = 0;

    if (s_last_cmd != 0 && (HAL_GetTick() - s_last_cmd) > DART_CMD_TIMEOUT_MS) {
        s_link_ok = 0;
    } else {
        s_link_ok = 1;
    }

    if (HAL_GetTick() - last_fb >= DART_FB_PERIOD_MS) {
        last_fb = HAL_GetTick();
        SendTelemetry();
    }
}

void DartLinkSend(const char *s)
{
    if (s_usart == NULL || s == NULL) return;
    USARTSend(s_usart, (uint8_t *)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}
