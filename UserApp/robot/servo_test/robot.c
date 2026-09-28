/**
 * @file robot.c
 * @author ai
 * @brief servo_motor 驱动测试 app (servo_test)
 * @version 1.1
 * @date 2026-09-28
 *
 * 命令通道: 同时支持
 *   1) J-Link RTT (推荐, 只需 JLink): 用 J-Link RTT Viewer 在 Channel 0 输入框发命令;
 *   2) USB-CDC 虚拟串口(若插了板载 USB -> COM 口)
 * ASCII 行协议(以 \r 或 \n 结尾), 命令:
 *   PING             -> PONG
 *   A,<deg> / ANG,<deg>   设逻辑角(deg)
 *   U,<us>          直接设脉宽(us)
 *   Z               当前位置记为逻辑 0°(调零)
 *   M               把当前位置标定为 90°(按比例反算 scale, 物理不动)
 *   N,<deg>         设逻辑 0° 对应机械角(center)
 *   T,<deg>         设微调 trim
 *   S,<scale>       设比例 scale
 *   D               方向翻转
 *   R,<dps>         设速率限幅(deg/s, <=0 不限速)
 *   L,<deg>         设逻辑角对称限位 ±deg(如 L,90)
 *   G,<deg>         设机械量程    PL,<us>/PH,<us> 设脉宽两端
 *   C               清除标定
 *   E / X           使能 / 失能(PWM 输出)
 *   I 或 ?          回读状态
 * 回复(USB + RTT 同时): OK a=.. raw=.. pulse=.. | cal c=.. t=.. s=.. d=.. r=..
 */
#include "robot.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usb.h"
#include "robot_config.h"
#include "servo_motor.h"
#include "tim.h"
#include "user_lib.h"

RobotInstance *robot = NULL;

static ServoInstance *g_servo = NULL;
static uint8_t *g_usb_rx = NULL;

/* USB 中断 -> 任务的环形缓冲 */
#define RX_RING_SZ 64
static volatile char s_ring[RX_RING_SZ];
static volatile uint8_t s_rh = 0;
static volatile uint8_t s_wh = 0;

/* 行解析缓存(仅任务上下文使用) */
static char s_line[64];
static uint8_t s_len = 0;

static void ParseByte(char c);
static void HandleLine(char *line);

/* 回复: 同时走 USB-CDC 与 RTT */
static void Reply(const char *s) {
  if (s == NULL) return;
  if (g_usb_rx != NULL) USBTransmit((uint8_t *)s, (uint16_t)strlen(s));
  SEGGER_RTT_WriteString(0, s);
}

static void ReportState(void) {
  Servo_Calib_Config_s c;
  ServoGetCalib(g_servo, &c);
  char buf[200];
  snprintf(buf, sizeof(buf),
           "OK a=%.1f raw=%.1f pulse=%.0f | cal c=%.1f t=%.1f s=%.3f d=%d r=%.0f | rg=%.1f p=%.0f/%.0f lim=%.1f\r\n",
           ServoGetAngle(g_servo), ServoGetRawDeg(g_servo), ServoGetPulseUs(g_servo),
           c.center_deg, c.trim_deg, c.scale, (int)c.reverse, c.rate_limit_dps,
           c.range_deg, c.pulse_min_us, c.pulse_max_us, c.limit_deg);
  Reply(buf);
}

/* USB 接收回调(中断): 只入环形缓冲 */
static void UsbRxCallback(uint16_t len) {
  if (g_usb_rx == NULL) return;
  for (uint16_t i = 0; i < len; i++) {
    uint8_t next = (uint8_t)((s_wh + 1) % RX_RING_SZ);
    if (next != s_rh) {
      s_ring[s_wh] = (char)g_usb_rx[i];
      s_wh = next;
    }
  }
}

/* 单字节喂给行解析器(任务上下文) */
static void ParseByte(char c) {
  if (c == '\r' || c == '\n') {
    if (s_len > 0) {
      s_line[s_len] = '\0';
      HandleLine(s_line);
      s_len = 0;
    }
  } else if (s_len < sizeof(s_line) - 1) {
    s_line[s_len++] = c;
  }
}

static void HandleLine(char *line) {
  for (char *p = line; *p != '\0'; p++) *p = (char)toupper((unsigned char)*p);

  char *cmd = strtok(line, ",");
  if (cmd == NULL) return;
  char *arg1 = strtok(NULL, ",");
  float v = (arg1 != NULL) ? strtof(arg1, NULL) : 0.0f;

  if (strcmp(cmd, "PING") == 0) {
    Reply("PONG\r\n");
    return;
  } else if (strcmp(cmd, "A") == 0 || strcmp(cmd, "ANG") == 0) {
    ServoSetAngle(g_servo, v);
  } else if (strcmp(cmd, "U") == 0) {
    ServoSetPulseUs(g_servo, v);
  } else if (strcmp(cmd, "Z") == 0) {
    if (ServoZero(g_servo)) {
      Reply("OK: zeroed & saved\r\n");
    } else {
      Reply("ERR: zero refused (raw must be within mid 135 +/-30, i.e. 105~165)\r\n");
    }
  } else if (strcmp(cmd, "N") == 0) {
    ServoSetNeutral(g_servo, v);
  } else if (strcmp(cmd, "T") == 0) {
    ServoSetTrim(g_servo, v);
  } else if (strcmp(cmd, "S") == 0) {
    ServoSetScale(g_servo, v);
  } else if (strcmp(cmd, "D") == 0) {
    Servo_Calib_Config_s c;
    ServoGetCalib(g_servo, &c);
    ServoSetReverse(g_servo, c.reverse > 0 ? -1 : 1);
  } else if (strcmp(cmd, "R") == 0) {
    ServoSetRateLimit(g_servo, v);
  } else if (strcmp(cmd, "L") == 0) {
    ServoSetLimit(g_servo, v);
  } else if (strcmp(cmd, "G") == 0) {
    ServoSetRange(g_servo, v);
  } else if (strcmp(cmd, "PL") == 0) {
    Servo_Calib_Config_s c;
    ServoGetCalib(g_servo, &c);
    ServoSetPulseRange(g_servo, v, c.pulse_max_us);
  } else if (strcmp(cmd, "PH") == 0) {
    Servo_Calib_Config_s c;
    ServoGetCalib(g_servo, &c);
    ServoSetPulseRange(g_servo, c.pulse_min_us, v);
  } else if (strcmp(cmd, "M") == 0) {
    /* 记 90°: 把当前逻辑角 L 标定为 90°, 按比例反算 scale(物理位置不动)。
     * 新 scale = 旧 scale * |L| / 90, 之后逻辑角改为 ±90(与 L 同号)。 */
    float L = ServoGetAngle(g_servo);
    if (fabsf(L) < 1.0e-3f) {
      Reply("ERR: L~0, move to the 90 mark first\r\n");
      return;
    }
    Servo_Calib_Config_s c;
    ServoGetCalib(g_servo, &c);
    ServoSetScale(g_servo, c.scale * fabsf(L) / 90.0f);
    ServoSetAngle(g_servo, (L >= 0.0f) ? 90.0f : -90.0f);
  } else if (strcmp(cmd, "C") == 0) {
    ServoResetCal(g_servo);
  } else if (strcmp(cmd, "SAVE") == 0) {
    ServoSaveCalib();
  } else if (strcmp(cmd, "LOAD") == 0) {
    ServoLoadCalib();
  } else if (strcmp(cmd, "E") == 0) {
    ServoEnable(g_servo);
  } else if (strcmp(cmd, "X") == 0) {
    ServoDisable(g_servo);
  } else if (strcmp(cmd, "I") == 0 || strcmp(cmd, "?") == 0) {
    /* 落到下面回状态 */
  } else {
    Reply("ERR: unknown cmd\r\n");
    return;
  }
  ReportState();
}

void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  /* 注册一路 PWM 舵机: PWM1 = TIM1_CH1 = PE9 */
  Servo_Init_Config_s servo_cfg;
  memset(&servo_cfg, 0, sizeof(servo_cfg));
  servo_cfg.servo_type = PWM_Servo;
  servo_cfg.servo_id = SERVO_TEST_ID;
  servo_cfg.pwm_init_config.htim = SERVO_TEST_TIM;
  servo_cfg.pwm_init_config.channel = SERVO_TEST_CHANNEL;
  servo_cfg.pwm_init_config.period = SERVO_TEST_PERIOD_S;
  servo_cfg.pwm_init_config.dutyratio = SERVO_TEST_DUTY_INIT;
  servo_cfg.calib.pulse_min_us = SERVO_TEST_PULSE_MIN_US;
  servo_cfg.calib.pulse_max_us = SERVO_TEST_PULSE_MAX_US;
  servo_cfg.calib.range_deg = SERVO_TEST_RANGE_DEG;
  servo_cfg.calib.center_deg = SERVO_TEST_CENTER_DEG;
  servo_cfg.calib.trim_deg = SERVO_TEST_TRIM_DEG;
  servo_cfg.calib.scale = SERVO_TEST_SCALE;
  servo_cfg.calib.reverse = SERVO_TEST_REVERSE;
  servo_cfg.calib.rate_limit_dps = SERVO_TEST_RATE_DPS;
  servo_cfg.calib.limit_deg = SERVO_TEST_LIMIT_DEG;
  g_servo = ServoInit(&servo_cfg);

  ServoLoadCalib();  // 从 Flash 读回标定(掉电保存; 无有效数据则用源码默认值)

  /* 初始化 USB-CDC 命令通道(即使没插 USB 也无害) */
  USB_Init_Config_s usb_cfg;
  usb_cfg.tx_cbk = NULL;
  usb_cfg.rx_cbk = UsbRxCallback;
  g_usb_rx = USBInit(usb_cfg);

  /* 上电后转到指定逻辑角 */
  ServoSetAngle(g_servo, SERVO_TEST_INIT_DEG);

  LOGINFO("[servo_test] ready: PWM1/TIM1_CH1/PE9");
  Reply("\r\nservo_test ready (RTT ch0 / USB-CDC). cmds: PING A,<deg> U,<us> Z M N T S D R G L PL PH C SAVE LOAD E X I\r\n");
  ReportState();
}

void RobotTask(void) {
  ServoTask();  // 速率限幅推进(>0 时生效)

  /* 1) 排空 USB 环形缓冲 */
  while (s_rh != s_wh) {
    char c = s_ring[s_rh];
    s_rh = (uint8_t)((s_rh + 1) % RX_RING_SZ);
    ParseByte(c);
  }

  /* 2) 轮询 J-Link RTT 输入(channel 0) */
  if (SEGGER_RTT_HasData(0)) {
    char buf[32];
    unsigned n = SEGGER_RTT_Read(0, buf, sizeof(buf));
    for (unsigned i = 0; i < n; i++) ParseByte(buf[i]);
  }
}
