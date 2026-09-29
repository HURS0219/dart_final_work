/*
 * test.c — dart_final_test_app 入口 (代替 robot.c)
 * =============================================================================
 * 用途: 逐个黑盒测试 dart_final 的各 app 服务, 自底向上:
 *       ① fin/舵机 (FIN 逐路 / FMIX 混控) ② guidance (ATT+TGT 注入 -> 看 mix)
 *       ③ vision / ④ imu (真实 或 注入对照)。
 *
 * 机制:
 *   - 注入: 本文件持 Publisher, 往 "attitude"/"target"/"mix" 话题发模拟数据;
 *   - 监视(方案①): 本文件 SubRegister 订阅全部话题, 实时显示 producer/sub/data/值/时间戳;
 *   - 日志: 每 LOG_PERIOD_MS 打一条(供 MATLAB); fs=1 时附一行 fsreason;
 *   - 控制台: 经 J-Link RTT(下行通道0) 输入命令; 与 Tools/scripts/rtt_send.ps1 配合。
 *
 * 命令:
 *   PING                      -> PONG
 *   ATT,<roll>,<pitch>,<yaw>  注入姿态(发布 attitude)
 *   TGT,<x>,<y>               注入目标(发布 target, found=1)
 *   TGTN                      注入“目标丢失”(found=0)
 *   FMIX,<p>,<y>,<r>          注入混控指令(发布 mix)
 *   FIN,<ch>,<deg>            fin 切 MANUAL, 给第 ch 路逻辑角
 *   RESET                     软复位测试系统(全使能 + 清零监视)
 *   RESET,<NAME>              单独关掉某 app; NAME in IMU/VISION/GUIDANCE/FIN
 *   STAT                      立即打印一次状态表
 * =============================================================================
 */
#include "robot.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dart_final_cfg.h"
#include "message_center.h"
#include "robot_def.h"
#include "test_cfg.h"
#include "user_lib.h"

#include "fin.h"
#include "guidance.h"
#include "imu.h"
#include "vision.h"

RobotInstance *robot = NULL;

/* ===================== app 使能 / app 名 ===================== */
static uint8_t s_app_en[A_CNT] = {1, 1, 1, 1};
static const char *kAppName[A_CNT] = {"IMU", "VISION", "GUIDANCE", "FIN"};
static const char *kTopicName[T_CNT] = TOPIC_NAME;
static const int kProducer[T_CNT] = TOPIC_PRODUCER;
static const int kConsumer[T_CNT] = TOPIC_CONSUMER;

/* ===================== 话题发布者(注入) / 订阅者(监视) ===================== */
static Publisher_t *s_pub[T_CNT];  /* attitude/target/mix 用于注入 */
static Subscriber_t *s_sub[T_CNT];

/* ===================== 监视缓存(最近一次收到的各话题值) ===================== */
static Dart_Attitude_s s_m_att;
static Dart_Target_s s_m_tgt;
static Dart_Mix_s s_m_mix;
static Dart_ServoFb_s s_m_fb;
static uint32_t s_t_att, s_t_tgt, s_t_mix, s_t_fb; /* 收到时刻(ms) */
static uint8_t s_has_att, s_has_tgt, s_has_mix, s_has_fb;

/* ===================== app 心跳监测 ===================== */
static uint32_t s_hb_last[A_CNT];
static uint32_t s_hb_stale[A_CNT];

/* ===================== 控制台行缓冲(RTT) ===================== */
static char s_line[64];
static uint8_t s_len = 0;

/* ===================== 工具 ===================== */
static void Reply(const char *s) {
  if (s != NULL) SEGGER_RTT_WriteString(0, s);
}

static uint32_t NowMs(void) { return (uint32_t)DWT_GetTimeline_ms(); }

/* app 是否在运行: 其 hb 在最近 100ms 内更新过 */
static uint8_t AppAlive(int a) {
  if (a < 0 || a >= A_CNT) return 0;
  return (s_hb_stale[a] <= 100u) ? 1u : 0u;
}

/* ===================== 监视更新 ===================== */
static void MonitorUpdate(void) {
  Dart_Attitude_s a;
  Dart_Target_s t;
  Dart_Mix_s m;
  Dart_ServoFb_s f;
  const Dart_AppStatus_s *st[A_CNT];
  uint32_t dt_ms = (robot != NULL) ? (uint32_t)(robot->dt * 1000.0f) : 1u;
  int i;

  /* 1) 拉取各话题最新消息(监视器自己的订阅) */
  if (SubGetMessage(s_sub[T_ATTITUDE], &a)) { s_m_att = a; s_t_att = NowMs(); s_has_att = 1; }
  if (SubGetMessage(s_sub[T_TARGET], &t)) { s_m_tgt = t; s_t_tgt = NowMs(); s_has_tgt = 1; }
  if (SubGetMessage(s_sub[T_MIX], &m)) { s_m_mix = m; s_t_mix = NowMs(); s_has_mix = 1; }
  if (SubGetMessage(s_sub[T_SERVO_FB], &f)) { s_m_fb = f; s_t_fb = NowMs(); s_has_fb = 1; }

  /* 2) app 心跳 */
  st[A_IMU] = Imu_GetStatus();
  st[A_VISION] = Vision_GetStatus();
  st[A_GUID] = Guidance_GetStatus();
  st[A_FIN] = Fin_GetStatus();
  for (i = 0; i < A_CNT; i++) {
    if (st[i]->hb != s_hb_last[i]) {
      s_hb_last[i] = st[i]->hb;
      s_hb_stale[i] = 0;
    } else {
      s_hb_stale[i] += dt_ms;
    }
  }
}

/* fs 原因(由监视器从输入侧推断, 与 guidance 内部判定一致) */
static const char *FsReason(void) {
  static char buf[48];
  uint8_t no_en = !s_app_en[A_GUID];
  uint8_t no_vis = !(s_has_tgt && s_m_tgt.found);
  uint8_t no_att = !(s_has_att && s_m_att.valid);
  buf[0] = '\0';
  if (no_en) strcat(buf, "NO_ENABLE ");
  if (no_vis) strcat(buf, "NO_VISION ");
  if (no_att) strcat(buf, "NO_ATTITUDE ");
  if (buf[0] == '\0') strcat(buf, "NONE");
  return buf;
}

/* 打印一条话题状态行。注: nano.specs 默认禁用 %f, 故浮点用“整数定标”输出(x10 / x100)。 */
static void PrintTopic(uint32_t sec, int topic) {
  char buf[240];
  uint32_t now = NowMs();
  int prod = kProducer[topic], cons = kConsumer[topic];
  uint8_t producer_alive = (prod >= 0 && s_app_en[prod]) ? AppAlive(prod) : 0;
  uint8_t sub_alive = (cons >= 0 && s_app_en[cons]) ? AppAlive(cons) : 0;
  uint8_t data = 0;

  switch (topic) {
    case T_ATTITUDE:
      data = (s_has_att && (now - s_t_att) <= DATA_FRESH_MS) ? 1 : 0;
      snprintf(buf, sizeof(buf),
               "[t=%04us] %-8s prod=%-7s:%u sub=%-8s:%u data=%u | roll10=%d pitch10=%d yaw10=%d "
               "gx10=%d gy10=%d gz10=%d valid=%u\n",
               sec, kTopicName[topic], prod >= 0 ? kAppName[prod] : "-", producer_alive,
               cons >= 0 ? kAppName[cons] : "-", sub_alive, data, (int)(s_m_att.roll_deg * 10.0f),
               (int)(s_m_att.pitch_deg * 10.0f), (int)(s_m_att.yaw_deg * 10.0f),
               (int)(s_m_att.gx_dps * 10.0f), (int)(s_m_att.gy_dps * 10.0f),
               (int)(s_m_att.gz_dps * 10.0f), s_m_att.valid);
      break;
    case T_TARGET:
      data = (s_has_tgt && (now - s_t_tgt) <= DATA_FRESH_MS) ? 1 : 0;
      snprintf(buf, sizeof(buf),
               "[t=%04us] %-8s prod=%-7s:%u sub=%-8s:%u data=%u | x=%d y=%d found=%u\n", sec,
               kTopicName[topic], prod >= 0 ? kAppName[prod] : "-", producer_alive,
               cons >= 0 ? kAppName[cons] : "-", sub_alive, data, s_m_tgt.x, s_m_tgt.y, s_m_tgt.found);
      break;
    case T_MIX:
      data = (s_has_mix && (now - s_t_mix) <= DATA_FRESH_MS) ? 1 : 0;
      snprintf(buf, sizeof(buf),
               "[t=%04us] %-8s prod=%-7s:%u sub=%-8s:%u data=%u | p100=%d y100=%d r100=%d fs=%u\n",
               sec, kTopicName[topic], prod >= 0 ? kAppName[prod] : "-", producer_alive,
               cons >= 0 ? kAppName[cons] : "-", sub_alive, data, (int)(s_m_mix.pitch * 100.0f),
               (int)(s_m_mix.yaw * 100.0f), (int)(s_m_mix.roll * 100.0f), s_m_mix.failsafe);
      break;
    default: /* T_SERVO_FB */
      data = (s_has_fb && (now - s_t_fb) <= DATA_FRESH_MS) ? 1 : 0;
      snprintf(buf, sizeof(buf),
               "[t=%04us] %-8s prod=%-7s:%u sub=%-8s:%u data=%u | d10=%d,%d,%d,%d p=%d,%d,%d,%d\n",
               sec, kTopicName[topic], prod >= 0 ? kAppName[prod] : "-", producer_alive,
               cons >= 0 ? kAppName[cons] : "-", sub_alive, data, (int)(s_m_fb.defl_deg[0] * 10.0f),
               (int)(s_m_fb.defl_deg[1] * 10.0f), (int)(s_m_fb.defl_deg[2] * 10.0f),
               (int)(s_m_fb.defl_deg[3] * 10.0f), (int)s_m_fb.pulse_us[0], (int)s_m_fb.pulse_us[1],
               (int)s_m_fb.pulse_us[2], (int)s_m_fb.pulse_us[3]);
      break;
  }
  Reply(buf);
}

/* 打印整表 */
static void PrintTable(uint8_t with_app, uint32_t sec) {
  char buf[160];
  if (with_app) {
    snprintf(buf, sizeof(buf),
             "[t=%04us] apps: IMU:%u VISION:%u GUIDANCE:%u FIN:%u\n", sec,
             AppAlive(A_IMU), AppAlive(A_VISION), AppAlive(A_GUID), AppAlive(A_FIN));
    Reply(buf);
  }
  PrintTopic(sec, T_ATTITUDE);
  PrintTopic(sec, T_TARGET);
  PrintTopic(sec, T_MIX);
  PrintTopic(sec, T_SERVO_FB);
  if (s_has_mix && s_m_mix.failsafe) {
    snprintf(buf, sizeof(buf), "[t=%04us]   fsreason=%s\n", sec, FsReason());
    Reply(buf);
  }
}

/* ===================== 控制台命令 ===================== */
/* 解析逗号分隔的数值参数(最多 n 个), 返回个数 */
static int ParseArgs(char *line, float *out, int n) {
  int cnt = 0;
  char *p = line;
  while (p != NULL && cnt < n) {
    char *comma = strchr(p, ',');
    if (comma) *comma = '\0';
    out[cnt++] = strtof(p, NULL);
    p = comma ? (comma + 1) : NULL;
  }
  return cnt;
}

static void HandleLine(char *line) {
  char *p;
  for (p = line; *p; p++) *p = (char)toupper((unsigned char)*p);

  if (strcmp(line, "PING") == 0) {
    Reply("PONG\n");
    return;
  }
  if (strcmp(line, "STAT") == 0) {
    PrintTable(1, NowMs() / 1000u);
    return;
  }
  if (strncmp(line, "RESET", 5) == 0) {
    char *arg = (line[5] == ',') ? (line + 6) : NULL;
    if (arg == NULL || *arg == '\0') {
      int i;
      for (i = 0; i < A_CNT; i++) s_app_en[i] = 1;
      s_has_att = s_has_tgt = s_has_mix = s_has_fb = 0;
      memset(&s_hb_last, 0, sizeof(s_hb_last));
      memset(&s_hb_stale, 0, sizeof(s_hb_stale));
    } else {
      int i, hit = 0;
      for (i = 0; i < A_CNT; i++) {
        if (strcmp(arg, kAppName[i]) == 0) {
          s_app_en[i] = 0;
          hit = 1;
        }
      }
      if (!hit) Reply("RESET: unknown app name\n");
    }
    return;
  }
  if (strncmp(line, "ATT,", 4) == 0) {
    float v[3] = {0};
    ParseArgs(line + 4, v, 3);
    memset(&s_m_att, 0, sizeof(s_m_att));
    s_m_att.roll_deg = v[0];
    s_m_att.pitch_deg = v[1];
    s_m_att.yaw_deg = v[2];
    s_m_att.valid = 1;
    s_m_att.tick = NowMs();
    PubPushMessage(s_pub[T_ATTITUDE], &s_m_att);
    return;
  }
  if (strncmp(line, "TGT,", 4) == 0) {
    float v[2] = {0};
    ParseArgs(line + 4, v, 2);
    memset(&s_m_tgt, 0, sizeof(s_m_tgt));
    s_m_tgt.x = (int16_t)v[0];
    s_m_tgt.y = (int16_t)v[1];
    s_m_tgt.found = 1;
    s_m_tgt.tick = NowMs();
    PubPushMessage(s_pub[T_TARGET], &s_m_tgt);
    return;
  }
  if (strcmp(line, "TGTN") == 0) {
    s_m_tgt.found = 0;
    s_m_tgt.tick = NowMs();
    PubPushMessage(s_pub[T_TARGET], &s_m_tgt);
    return;
  }
  if (strncmp(line, "FMIX,", 5) == 0) {
    float v[3] = {0};
    ParseArgs(line + 5, v, 3);
    memset(&s_m_mix, 0, sizeof(s_m_mix));
    s_m_mix.pitch = v[0];
    s_m_mix.yaw = v[1];
    s_m_mix.roll = v[2];
    s_m_mix.failsafe = 0;
    s_m_mix.tick = NowMs();
    PubPushMessage(s_pub[T_MIX], &s_m_mix);
    return;
  }
  if (strncmp(line, "FIN,", 4) == 0) {
    float v[2] = {0};
    ParseArgs(line + 4, v, 2);
    Fin_SetMode(1); /* MANUAL */
    Fin_SetManual((uint8_t)v[0], v[1]);
    return;
  }
  Reply("ERR unknown cmd\n");
}

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

static void ConsolePoll(void) {
  if (SEGGER_RTT_HasData(0)) {
    char buf[32];
    unsigned n = SEGGER_RTT_Read(0, buf, sizeof(buf));
    unsigned i;
    for (i = 0; i < n; i++) ParseByte(buf[i]);
  }
}

/* ===================== 生命周期 ===================== */
void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  /* 被测服务(始终初始化) */
#if TEST_IMU_ENABLE
  Imu_Init();
#endif
#if TEST_VISION_ENABLE
  Vision_Init();
#endif
  Guidance_Init();
  Fin_Init();

  /* 注入发布者(attitude/target/mix) */
  s_pub[T_ATTITUDE] = PubRegister(TOPIC_ATTITUDE, sizeof(Dart_Attitude_s));
  s_pub[T_TARGET] = PubRegister(TOPIC_TARGET, sizeof(Dart_Target_s));
  s_pub[T_MIX] = PubRegister(TOPIC_MIX, sizeof(Dart_Mix_s));

  /* 监视订阅者(全部话题) */
  s_sub[T_ATTITUDE] = SubRegister(TOPIC_ATTITUDE, sizeof(Dart_Attitude_s));
  s_sub[T_TARGET] = SubRegister(TOPIC_TARGET, sizeof(Dart_Target_s));
  s_sub[T_MIX] = SubRegister(TOPIC_MIX, sizeof(Dart_Mix_s));
  s_sub[T_SERVO_FB] = SubRegister(TOPIC_SERVO_FB, sizeof(Dart_ServoFb_s));

  (void)DWT_GetDeltaT(&robot->DWT_CNT); /* 初始化 dt 基准 */

  LOGINFO("[dart_test] init done (IMU_EN=%d VISION_EN=%d)", TEST_IMU_ENABLE, TEST_VISION_ENABLE);
  Reply("\r\n=== dart_final_test_app ready ===\r\n"
        "cmds: PING | ATT,<r>,<p>,<y> | TGT,<x>,<y> | TGTN | FMIX,<p>,<y>,<r> | FIN,<ch>,<deg> | "
        "RESET[,<APP>] | STAT\r\n");
}

void RobotTask(void) {
  static uint32_t last_log = 0;
  uint32_t now;

  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  now = NowMs();

  /* 1) 跑被测 app(受使能控制; guidance 恒 guide_enable=1, 本 app 无状态机) */
  if (s_app_en[A_IMU]) Imu_Task();
  if (s_app_en[A_VISION]) Vision_Task();
  if (s_app_en[A_GUID]) Guidance_Task(robot->dt, 1u);
  if (s_app_en[A_FIN]) Fin_Task();

  /* 2) 监视 + 控制台(只收命令, 不刷屏) */
  MonitorUpdate();
  ConsolePoll();

  /* 3) 每秒日志(写出一条状态表, 经 RTT 存档到 Debug\)。
   *    无实时 UI 刷屏; 想要“完全静默, 只在 STAT 时打印一次” -> 把 test_cfg.h 里
   *    LOG_PERIOD_MS 设为 0。 */
  if (LOG_PERIOD_MS > 0u && (now - last_log) >= LOG_PERIOD_MS) {
    last_log = now;
    PrintTable(1, now / 1000u);
  }
}
