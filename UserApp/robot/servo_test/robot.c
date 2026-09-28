/**
 * @file robot.c
 * @author ai
 * @brief servo_motor 驱动测试 app (servo_test)
 * @version 2.0
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
 * 状态回读格式:
 *   OK a=<逻辑角> tgt=<目标角> pulse=<脉宽> | cu=<中位> hu=<半脉宽> hd=<半行程> \
 *      s=<增益> t=<trim> d=<方向> lim=<限位> r=<限速> z=<调零开关>
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

/* 掉电保存: 使用 bsp_flash 的双 Bank(两个扇区), 模块本身不碰 Flash */
#define SERVO_TEST_CFG_BANK_A ADDR_FLASH_SECTOR_10 /* 0x080C0000 */
#define SERVO_TEST_CFG_BANK_B ADDR_FLASH_SECTOR_11 /* 0x080E0000 */

/*
 * 持久化镜像: 只保存"标定参数", 不保存运行状态。
 * 用固定宽度类型(9 个 float + 2 个 int32 = 44B)保证无隐式 padding, 跨编译器 CRC 一致。
 */
typedef struct {
  float center_us, half_us, half_deg, pulse_min_us, pulse_max_us;  // 信号标定
  float scale, trim_deg, limit_deg, rate_limit_dps;                // 逻辑标定
  int32_t reverse, zero_enable;                                    // 方向 / 调零开关
} ServoTestCfg_s;

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
 * @brief 修改标定参数后, 按当前目标角重新输出一次(否则要等下次 ServoSetAngle 才生效)
 */
static void Reapply(void) { ServoSetAngle(g_servo, ServoGetTarget(g_servo)); }

/**
 * @brief 把当前标定参数打包保存到 Flash(双 Bank), 并回复结果
 */
static void SaveCfg(void) {
  ServoTestCfg_s p;
  p.center_us = g_servo->cfg.center_us;
  p.half_us = g_servo->cfg.half_us;
  p.half_deg = g_servo->cfg.half_deg;
  p.pulse_min_us = g_servo->cfg.pulse_min_us;
  p.pulse_max_us = g_servo->cfg.pulse_max_us;
  p.scale = g_servo->cfg.scale;
  p.trim_deg = g_servo->cfg.trim_deg;
  p.limit_deg = g_servo->cfg.limit_deg;
  p.rate_limit_dps = g_servo->cfg.rate_limit_dps;
  p.reverse = g_servo->cfg.reverse;
  p.zero_enable = g_servo->cfg.zero_enable;
  if (flash_store_save(SERVO_TEST_CFG_BANK_A, SERVO_TEST_CFG_BANK_B, &p, sizeof(p)) == 0) {
    Reply("SAVED\r\n");
  } else {
    Reply("SAVEERR\r\n");
  }
}

/**
 * @brief 从 Flash 读回标定参数并应用
 * @return true 读到有效数据; false 无有效数据(保持当前默认)
 */
static bool LoadCfg(void) {
  ServoTestCfg_s p;
  if (flash_store_load(SERVO_TEST_CFG_BANK_A, SERVO_TEST_CFG_BANK_B, &p, sizeof(p)) != 1) return false;
  g_servo->cfg.center_us = p.center_us;
  g_servo->cfg.half_us = p.half_us;
  g_servo->cfg.half_deg = p.half_deg;
  g_servo->cfg.pulse_min_us = p.pulse_min_us;
  g_servo->cfg.pulse_max_us = p.pulse_max_us;
  g_servo->cfg.scale = p.scale;
  g_servo->cfg.trim_deg = p.trim_deg;
  g_servo->cfg.limit_deg = p.limit_deg;
  g_servo->cfg.rate_limit_dps = p.rate_limit_dps;
  g_servo->cfg.reverse = (uint8_t)p.reverse;
  g_servo->cfg.zero_enable = (uint8_t)p.zero_enable;
  Reapply();
  return true;
}

/**
 * @brief 回读并打印当前状态(逻辑角/目标角/脉宽 + 全部标定参数)
 */
static void ReportState(void) {
  char buf[200];
  snprintf(buf, sizeof(buf),
           "OK a=%.1f tgt=%.1f pulse=%.0f | cu=%.0f hu=%.0f hd=%.1f s=%.3f t=%.1f d=%d lim=%.1f r=%.0f z=%d\r\n",
           ServoGetAngle(g_servo), ServoGetTarget(g_servo), ServoGetPulseUs(g_servo),
           g_servo->cfg.center_us, g_servo->cfg.half_us, g_servo->cfg.half_deg,
           g_servo->cfg.scale, g_servo->cfg.trim_deg, (int)g_servo->cfg.reverse,
           g_servo->cfg.limit_deg, g_servo->cfg.rate_limit_dps, (int)g_servo->cfg.zero_enable);
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
  for (char *p = line; *p != '\0'; p++) *p = (char)toupper((unsigned char)*p);  // 统一大写, 大小写不敏感

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
    if (v > 0.0f) {  // 增益必须为正
      g_servo->cfg.scale = v;
      Reapply();
    }
  } else if (strcmp(cmd, "T") == 0) {
    g_servo->cfg.trim_deg = v;  // 零点微调
    Reapply();
  } else if (strcmp(cmd, "D") == 0) {
    g_servo->cfg.reverse = g_servo->cfg.reverse ? 0 : 1;  // 方向翻转
    Reapply();
  } else if (strcmp(cmd, "R") == 0) {
    g_servo->cfg.rate_limit_dps = v;  // 速率限幅(<=0 不限速)
  } else if (strcmp(cmd, "ZE") == 0) {
    g_servo->cfg.zero_enable = (v != 0.0f) ? 1 : 0;  // 调零功能开关
  } else if (strcmp(cmd, "C") == 0) {
    /* 清除标定: 全部回默认值 */
    g_servo->cfg.center_us = SERVO_TEST_CENTER_US;
    g_servo->cfg.half_us = SERVO_TEST_HALF_US;
    g_servo->cfg.half_deg = SERVO_TEST_HALF_DEG;
    g_servo->cfg.pulse_min_us = SERVO_TEST_PULSE_MIN_US;
    g_servo->cfg.pulse_max_us = SERVO_TEST_PULSE_MAX_US;
    g_servo->cfg.scale = SERVO_TEST_SCALE;
    g_servo->cfg.trim_deg = SERVO_TEST_TRIM_DEG;
    g_servo->cfg.limit_deg = SERVO_TEST_LIMIT_DEG;
    g_servo->cfg.rate_limit_dps = SERVO_TEST_RATE_DPS;
    g_servo->cfg.reverse = SERVO_TEST_REVERSE;
    Reapply();
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
