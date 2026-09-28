/**
 * @file robot.c
 * @author ai
 * @brief servo_motor 驱动测试 app (servo_test)
 * @version 2.1
 * @date 2026-09-28
 *
 * 命令通道: 同时支持
 *   1) J-Link RTT (Channel 0 输入框发命令);
 *   2) USB-CDC 虚拟串口(插了板载 USB -> COM 口)。
 *
 * ASCII 行协议(以 \r 或 \n 结尾, 命令大写不敏感), 命令:
 *   PING                 -> PONG
 *   A,<deg> / ANG,<deg>  设逻辑角(deg)
 *   U,<us>               直接设脉宽(us)
 *   Z                    当前位置记为逻辑 0°(调零; 受 zero_enable 与 ±30° 窗口限制)并保存
 *   L,<deg>              逻辑角对称限位 ±deg
 *   S,<scale>            逻辑角->机械角 增益
 *   T,<deg>              零点微调 trim
 *   D                    方向翻转
 *   R,<dps>              速率限幅(deg/s, <=0 不限速)
 *   ZE,<0|1>             调零功能开关
 *   C                    清除标定(回默认)
 *   SAVE / LOAD          立即写 Flash / 从 Flash 读回(双 Bank)
 *   E / X                使能 / 失能(PWM 输出)
 *   I 或 ?               回读状态
 *
 * 标定读写: app 只依赖 Servo_Calib_s(无 padding), 持久化直接交给 bsp_flash 的
 *           flash_store_save/load; 不复制模块内部字段, 模块不碰 Flash。
 */
#include "robot.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "bsp_dwt.h"
#include "bsp_flash.h"
#include "bsp_log.h"
#include "bsp_usb.h"
#include "robot_config.h"
#include "servo_motor.h"
#include "tim.h"
#include "user_lib.h"

RobotInstance *robot = NULL;

static ServoInstance *g_servo = NULL;  // 唯一测试舵机
static uint8_t *g_usb_rx = NULL;       // USB-CDC 接收缓冲

/* 掉电保存: 使用 bsp_flash 的双 Bank(两个扇区) */
#define SERVO_TEST_CFG_BANK_A ADDR_FLASH_SECTOR_10 /* 0x080C0000 */
#define SERVO_TEST_CFG_BANK_B ADDR_FLASH_SECTOR_11 /* 0x080E0000 */

/* USB 接收回调在中断上下文, 只把字节塞进环形缓冲, 由任务上下文解析 */
#define RX_RING_SZ 64
static volatile char s_ring[RX_RING_SZ];
static volatile uint8_t s_rh = 0;  // 读指针
static volatile uint8_t s_wh = 0;  // 写指针

/* 行解析缓存(仅任务上下文使用) */
static char s_line[64];
static uint8_t s_len = 0;

static void ParseByte(char c);
static void HandleLine(char *line);

/**
 * @brief 回复一行: 同时走 USB-CDC 与 RTT(方便任选其一观察)
 */
static void Reply(const char *s) {
  if (s == NULL) return;
  if (g_usb_rx != NULL) USBTransmit((uint8_t *)s, (uint16_t)strlen(s));
  SEGGER_RTT_WriteString(0, s);
}

/**
 * @brief 把当前标定(Servo_Calib_s 快照)保存到 Flash(双 Bank), 并回复结果
 */
static void SaveCfg(void) {
  Servo_Calib_s c;
  ServoGetCalib(g_servo, &c);
  if (flash_store_save(SERVO_TEST_CFG_BANK_A, SERVO_TEST_CFG_BANK_B, &c, sizeof(c)) == 0) {
    Reply("SAVED\r\n");
  } else {
    Reply("SAVEERR\r\n");
  }
}

/**
 * @brief 从 Flash 读回标定并应用
 * @return true 读到有效数据; false 无有效数据(保持当前默认)
 */
static bool LoadCfg(void) {
  Servo_Calib_s c;
  if (flash_store_load(SERVO_TEST_CFG_BANK_A, SERVO_TEST_CFG_BANK_B, &c, sizeof(c)) != 1) return false;
  ServoSetCalib(g_servo, &c);
  return true;
}

/**
 * @brief 回读并打印当前状态(逻辑角/目标角/脉宽 + 全部标定参数)
 */
static void ReportState(void) {
  Servo_Calib_s c;
  char buf[200];
  ServoGetCalib(g_servo, &c);
  snprintf(buf, sizeof(buf),
           "OK a=%.1f tgt=%.1f pulse=%.0f | cu=%.0f hu=%.0f hd=%.1f s=%.3f t=%.1f d=%d lim=%.1f r=%.0f z=%d\r\n",
           ServoGetAngle(g_servo), ServoGetTarget(g_servo), ServoGetPulseUs(g_servo),
           c.center_us, c.half_us, c.half_deg, c.scale, c.trim_deg, (int)c.reverse,
           c.limit_deg, c.rate_limit_dps, (int)c.zero_enable);
  Reply(buf);
}

/**
 * @brief USB 接收回调(中断上下文): 只入环形缓冲, 不做解析
 */
static void UsbRxCallback(uint16_t len) {
  if (g_usb_rx == NULL) return;
  for (uint16_t i = 0; i < len; i++) {
    uint8_t next = (uint8_t)((s_wh + 1) % RX_RING_SZ);
    if (next != s_rh) {  // 未满才写入(满了丢弃, 简单可靠)
      s_ring[s_wh] = (char)g_usb_rx[i];
      s_wh = next;
    }
  }
}

/**
 * @brief 单字节喂给行解析器(任务上下文); 遇到 \r 或 \n 触发一次 HandleLine
 */
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

/**
 * @brief 解析并执行一行命令
 * @note  命令先整体转大写; 以 ',' 分割出命令字与第一个数值参数
 */
static void HandleLine(char *line) {
  for (char *p = line; *p != '\0'; p++) *p = (char)toupper((unsigned char)*p);  // 统一大写

  char *cmd = strtok(line, ",");
  if (cmd == NULL) return;
  char *arg1 = strtok(NULL, ",");
  float v = (arg1 != NULL) ? strtof(arg1, NULL) : 0.0f;

  if (strcmp(cmd, "PING") == 0) {
    Reply("PONG\r\n");  // 测通道
    return;
  } else if (strcmp(cmd, "A") == 0 || strcmp(cmd, "ANG") == 0) {
    ServoSetAngle(g_servo, v);  // 设逻辑角
  } else if (strcmp(cmd, "U") == 0) {
    ServoSetPulseUs(g_servo, v);  // 直给脉宽(标定/开环)
  } else if (strcmp(cmd, "Z") == 0) {
    if (ServoZero(g_servo)) {  // 调零成功 -> 立即落盘
      SaveCfg();
      Reply("OK: zeroed & saved\r\n");
    } else {
      Reply("ERR: zero refused (need zero_enable=1 & |applied|<=30)\r\n");
    }
  } else if (strcmp(cmd, "L") == 0) {
    ServoSetLimit(g_servo, v);  // 逻辑角对称限位
  } else if (strcmp(cmd, "S") == 0) {
    ServoSetScale(g_servo, v);  // 增益(内部校验 >0)
  } else if (strcmp(cmd, "T") == 0) {
    ServoSetTrim(g_servo, v);  // 零点微调
  } else if (strcmp(cmd, "D") == 0) {
    Servo_Calib_s c;
    ServoGetCalib(g_servo, &c);
    ServoSetReverse(g_servo, c.reverse ? 0 : 1);  // 方向翻转
  } else if (strcmp(cmd, "R") == 0) {
    ServoSetRateLimit(g_servo, v);  // 速率限幅(<=0 不限速)
  } else if (strcmp(cmd, "ZE") == 0) {
    Servo_Calib_s c;
    ServoGetCalib(g_servo, &c);
    c.zero_enable = (v != 0.0f) ? 1 : 0;  // 调零功能开关
    ServoSetCalib(g_servo, &c);
  } else if (strcmp(cmd, "C") == 0) {
    /* 清除标定: 构造一份默认标定并写回 */
    Servo_Calib_s d = {
        .center_us = SERVO_TEST_CENTER_US,
        .half_us = SERVO_TEST_HALF_US,
        .half_deg = SERVO_TEST_HALF_DEG,
        .pulse_min_us = SERVO_TEST_PULSE_MIN_US,
        .pulse_max_us = SERVO_TEST_PULSE_MAX_US,
        .scale = SERVO_TEST_SCALE,
        .trim_deg = SERVO_TEST_TRIM_DEG,
        .limit_deg = SERVO_TEST_LIMIT_DEG,
        .rate_limit_dps = SERVO_TEST_RATE_DPS,
        .reverse = SERVO_TEST_REVERSE,
        .zero_enable = SERVO_TEST_ZERO_ENABLE,
    };
    ServoSetCalib(g_servo, &d);
  } else if (strcmp(cmd, "SAVE") == 0) {
    SaveCfg();
  } else if (strcmp(cmd, "LOAD") == 0) {
    if (LoadCfg()) {
      Reply("LOADED\r\n");
    } else {
      Reply("LOAD: no valid data\r\n");
    }
  } else if (strcmp(cmd, "E") == 0) {
    ServoEnable(g_servo);  // 启动 PWM
  } else if (strcmp(cmd, "X") == 0) {
    ServoDisable(g_servo);  // 停止 PWM
  } else if (strcmp(cmd, "I") == 0 || strcmp(cmd, "?") == 0) {
    /* 回读状态: 落到下面统一 ReportState */
  } else {
    Reply("ERR: unknown cmd\r\n");
    return;
  }
  ReportState();  // 任何有效命令执行后都回读一次状态
}

void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  /* 注册一路 PWM 舵机: PWM1 = TIM1_CH1 = PE9 */
  Servo_Init_Config_s servo_cfg;
  memset(&servo_cfg, 0, sizeof(servo_cfg));
  servo_cfg.pwm.htim = SERVO_TEST_TIM;             // TIM1
  servo_cfg.pwm.channel = SERVO_TEST_CHANNEL;      // CH1
  servo_cfg.pwm.period = SERVO_TEST_PERIOD_S;      // 20ms -> 50Hz
  servo_cfg.pwm.dutyratio = SERVO_TEST_DUTY_INIT;  // 初始占空比(1.5ms)
  servo_cfg.center_us = SERVO_TEST_CENTER_US;      // 中位 1500us
  servo_cfg.half_us = SERVO_TEST_HALF_US;          // 半程 1000us
  servo_cfg.half_deg = SERVO_TEST_HALF_DEG;        // 半行程 139.5°
  servo_cfg.pulse_min_us = SERVO_TEST_PULSE_MIN_US;
  servo_cfg.pulse_max_us = SERVO_TEST_PULSE_MAX_US;
  servo_cfg.scale = SERVO_TEST_SCALE;
  servo_cfg.trim_deg = SERVO_TEST_TRIM_DEG;
  servo_cfg.limit_deg = SERVO_TEST_LIMIT_DEG;
  servo_cfg.rate_limit_dps = SERVO_TEST_RATE_DPS;
  servo_cfg.reverse = SERVO_TEST_REVERSE;
  servo_cfg.zero_enable = SERVO_TEST_ZERO_ENABLE;
  g_servo = ServoInit(&servo_cfg);

  LoadCfg();  // 从 Flash 读回标定(掉电保存; 无有效数据则用源码默认值)

  /* 初始化 USB-CDC 命令通道(即使没插 USB 也无害) */
  USB_Init_Config_s usb_cfg;
  usb_cfg.tx_cbk = NULL;
  usb_cfg.rx_cbk = UsbRxCallback;
  g_usb_rx = USBInit(usb_cfg);

  /* 上电后转到指定逻辑角 */
  ServoSetAngle(g_servo, SERVO_TEST_INIT_DEG);

  LOGINFO("[servo_test] ready: PWM1/TIM1_CH1/PE9");
  Reply("\r\nservo_test ready (RTT ch0 / USB-CDC). cmds: PING A,<deg> U,<us> Z L,<deg> S,<s> T,<d> D R,<dps> ZE,<0|1> C SAVE LOAD E X I\r\n");
  ReportState();
}

void RobotTask(void) {
  ServoTask();  // 速率限幅推进(>0 时生效)

  /* 1) 排空 USB 环形缓冲(中断写入) */
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
