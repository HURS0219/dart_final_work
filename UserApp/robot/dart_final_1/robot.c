/*
 * robot.c — dart_final_1 (dart_final 的沙盒测试版)
 * =============================================================================
 * 由 dart_final 拷贝而来, 在**不动生产版**的前提下, 加了一个
 * 【RTT 测试控制台 + 话题注入】, 用于在硬件上把状态机(IDLE/ARMED/GUIDING/FAULT)
 * 全部跑通。逻辑主体(状态机/Monitor/各 app)与 dart_final 一致。
 *
 * 命令(RTT 下行通道0; ASCII 行, 以 \r 或 \n 结尾; 大写不敏感):
 *   PING                       -> PONG
 *   EN,<0|1>                   使能(0=回 IDLE, 1=ARMED)
 *   IMUEN,<0|1>                IMU app 开关(0=不跑真实 IMU, 改用 ATT 注入)
 *   VISEN,<0|1>                视觉 app 开关(0=不注册 OpenMV, 改用 TGT 注入)
 *   ATT,<roll>,<pitch>,<yaw>   注入姿态(发布 attitude, valid=1)
 *   TGT,<x>,<y>                注入目标(发布 target, found=1)
 *   TGTN                       注入目标丢失(found=0)
 *   FAULT,<hex>                注入故障位(OR 进 Monitor 汇总), 如 FAULT,1
 *   CLRFAULT                   清除注入的故障位
 *   STAT                       立即打印一次状态
 *
 * 注: 关掉的 app 不参与“卡死”判定(否则关 IMU 会立刻 FAULT)。
 * =============================================================================
 */
#include "robot.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dart_final_cfg.h"
#include "message_center.h"
#include "robot_cfg.h"
#include "robot_def.h"
#include "user_lib.h"

#include "fin.h"
#include "guidance.h"
#include "imu.h"
#include "vision.h"

RobotInstance *robot = NULL;

/* ============================ 飞行状态机 ============================ */
typedef enum {
  DART_IDLE = 0,
  DART_ARMED,
  DART_GUIDING,
  DART_FAULT,
} DartState_e;

static DartState_e s_state = DART_IDLE;
static uint16_t s_fault_bits = 0; /* Monitor 汇总(含注入) */
static uint16_t s_inj_fault = 0;  /* 【测试】注入的故障位 */
static uint8_t s_enable = 1;      /* 【测试】使能(Dart_IsEnabled 用) */
static float s_log_period_ms = 1000.0f; /* 【测试】遥测周期; 0=关闭 */
static uint8_t s_txauto = 0;      /* 【测试】自动发串口自检帧(配合 STM32 TX<->RX 短接) */

#define DART_FAULT_MASK (DART_ERR_IMU_OFF | DART_ERR_GUID_OFF | DART_ERR_FIN_OFF | DART_ERR_SERVO)

/* app 索引 / 使能 */
#define A_IMU 0
#define A_VISION 1
#define A_GUID 2
#define A_FIN 3
#define A_CNT 4
static uint8_t s_app_en[A_CNT] = {1, 1, 1, 1};

/* ===================== 注入 / 观察 ===================== */
static Publisher_t *s_pub_att = NULL, *s_pub_tgt = NULL;
static Subscriber_t *s_sub_att = NULL, *s_sub_tgt = NULL;
static Dart_Attitude_s s_inj_att, s_att_latest;
static Dart_Target_s s_inj_tgt, s_tgt_latest;
static Subscriber_t *s_sub_mix = NULL, *s_sub_fb = NULL;
static Dart_Mix_s s_mix_latest;
static Dart_ServoFb_s s_fb_latest;

static uint32_t NowMs(void) { return (uint32_t)DWT_GetTimeline_ms(); }

/* ======================= 【待配置接口】 ======================= */
static bool Dart_IsEnabled(void) { return s_enable ? true : false; }

static bool Dart_ShouldGuide(void) {
  return (s_tgt_latest.found && s_att_latest.valid && (s_att_latest.pitch_deg < DIVE_PITCH_DEG));
}

/* 拉取最新 attitude/target 用于 ShouldGuide 判定(真实或注入皆可) */
static void PollInputs(void) {
  Dart_Attitude_s a;
  Dart_Target_s t;
  Dart_Mix_s m;
  Dart_ServoFb_s f;
  if (SubGetMessage(s_sub_att, &a)) s_att_latest = a;
  if (SubGetMessage(s_sub_tgt, &t)) s_tgt_latest = t;
  if (SubGetMessage(s_sub_mix, &m)) s_mix_latest = m;
  if (SubGetMessage(s_sub_fb, &f)) s_fb_latest = f;
}

/* ============================== Monitor ============================== */
static uint32_t s_mon_hb[A_CNT];
static uint32_t s_mon_stale[A_CNT];
static uint32_t s_mon_app_stuck_ms = 50u;

static uint16_t MonErrOf(int idx) {
  switch (idx) {
    case A_IMU: return DART_ERR_IMU_OFF;
    case A_VISION: return DART_ERR_VISION_OFF;
    case A_GUID: return DART_ERR_GUID_OFF;
    case A_FIN: return DART_ERR_FIN_OFF;
    default: return DART_ERR_NONE;
  }
}

static void Dart_Monitor(float dt_ms) {
  const Dart_AppStatus_s *st[A_CNT];
  uint16_t bits = DART_ERR_NONE;
  int i;

  st[A_IMU] = Imu_GetStatus();
  st[A_VISION] = Vision_GetStatus();
  st[A_GUID] = Guidance_GetStatus();
  st[A_FIN] = Fin_GetStatus();

  for (i = 0; i < A_CNT; i++) {
    if (!s_app_en[i]) { /* 关掉的 app 不判卡死, 也不计它自报的错 */
      s_mon_hb[i] = st[i]->hb;
      s_mon_stale[i] = 0;
      continue;
    }
    if (st[i]->hb != s_mon_hb[i]) {
      s_mon_hb[i] = st[i]->hb;
      s_mon_stale[i] = 0;
    } else {
      s_mon_stale[i] += (uint32_t)dt_ms;
      if (s_mon_stale[i] > s_mon_app_stuck_ms) bits |= MonErrOf(i);
    }
    bits |= st[i]->err;
  }
  s_fault_bits = (uint16_t)(bits | s_inj_fault);
}

/* ============================== 状态机 ============================== */
static void Dart_StateMachine(void) {
  bool enabled = Dart_IsEnabled();
  bool fault = ((s_fault_bits & DART_FAULT_MASK) != 0);

  switch (s_state) {
    case DART_IDLE:
      if (enabled) s_state = DART_ARMED;
      break;
    case DART_ARMED:
      if (!enabled)
        s_state = DART_IDLE;
      else if (fault)
        s_state = DART_FAULT;
      else if (Dart_ShouldGuide())
        s_state = DART_GUIDING;
      break;
    case DART_GUIDING:
      if (!enabled)
        s_state = DART_IDLE;
      else if (fault)
        s_state = DART_FAULT;
      break;
    case DART_FAULT:
      if (!enabled)
        s_state = DART_IDLE;
      else if (!fault)
        s_state = DART_ARMED;
      break;
    default:
      s_state = DART_IDLE;
      break;
  }
}

/* ============================ 遥测 / 控制台 ============================ */
static char s_line[64];
static uint8_t s_len = 0;

static void Reply(const char *s) {
  if (s != NULL) SEGGER_RTT_WriteString(0, s);
}

static void PrintState(void) {
  char buf[260];
  uint32_t vok = 0, vbad = 0;
  Vision_GetStats(&vok, &vbad);
  snprintf(buf, sizeof(buf),
           "[dart1] state=%d fault=0x%04X en=%u imuen=%u visen=%u | hb imu=%u vis=%u guid=%u fin=%u | vis ok=%u bad=%u\r\n",
           (int)s_state, s_fault_bits, s_enable, s_app_en[A_IMU], s_app_en[A_VISION],
           (unsigned)Imu_GetStatus()->hb, (unsigned)Vision_GetStatus()->hb,
           (unsigned)Guidance_GetStatus()->hb, (unsigned)Fin_GetStatus()->hb,
           (unsigned)vok, (unsigned)vbad);
  Reply(buf);
  snprintf(buf, sizeof(buf),
           "[dart1] mix y100=%d r100=%d fs=%u | d10=%d,%d,%d,%d p=%d,%d,%d,%d\r\n",
           (int)(s_mix_latest.yaw * 100.0f), (int)(s_mix_latest.roll * 100.0f), s_mix_latest.failsafe,
           (int)(s_fb_latest.defl_deg[0] * 10.0f), (int)(s_fb_latest.defl_deg[1] * 10.0f),
           (int)(s_fb_latest.defl_deg[2] * 10.0f), (int)(s_fb_latest.defl_deg[3] * 10.0f),
           (int)s_fb_latest.pulse_us[0], (int)s_fb_latest.pulse_us[1],
           (int)s_fb_latest.pulse_us[2], (int)s_fb_latest.pulse_us[3]);
  Reply(buf);
  {
    uint32_t c1, c3, c6;
    Vision_GetScan(&c1, &c3, &c6);
    snprintf(buf, sizeof(buf), "[dart1] uartscan c1=%u c3=%u c6=%u\r\n",
             (unsigned)c1, (unsigned)c3, (unsigned)c6);
    Reply(buf);
  }
}

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
    Reply("PONG\r\n");
    return;
  }
  if (strncmp(line, "EN,", 3) == 0) {
    s_enable = (strtof(line + 3, NULL) != 0.0f) ? 1u : 0u;
    return;
  }
  if (strncmp(line, "IMUEN,", 6) == 0) {
    s_app_en[A_IMU] = (strtof(line + 6, NULL) != 0.0f) ? 1u : 0u;
    return;
  }
  if (strncmp(line, "VISEN,", 6) == 0) {
    s_app_en[A_VISION] = (strtof(line + 6, NULL) != 0.0f) ? 1u : 0u;
    return;
  }
  if (strncmp(line, "ATT,", 4) == 0) {
    float v[3] = {0};
    ParseArgs(line + 4, v, 3);
    memset(&s_inj_att, 0, sizeof(s_inj_att));
    s_inj_att.roll_deg = v[0];
    s_inj_att.pitch_deg = v[1];
    s_inj_att.yaw_deg = v[2];
    s_inj_att.valid = 1;
    s_inj_att.tick = NowMs();
    PubPushMessage(s_pub_att, &s_inj_att);
    return;
  }
  if (strncmp(line, "TGT,", 4) == 0) {
    float v[2] = {0};
    ParseArgs(line + 4, v, 2);
    memset(&s_inj_tgt, 0, sizeof(s_inj_tgt));
    s_inj_tgt.x = (int16_t)v[0];
    s_inj_tgt.y = (int16_t)v[1];
    s_inj_tgt.found = 1;
    s_inj_tgt.tick = NowMs();
    PubPushMessage(s_pub_tgt, &s_inj_tgt);
    return;
  }
  if (strcmp(line, "TGTN") == 0) {
    s_inj_tgt.found = 0;
    s_inj_tgt.tick = NowMs();
    PubPushMessage(s_pub_tgt, &s_inj_tgt);
    return;
  }
  if (strncmp(line, "FAULT,", 6) == 0) {
    s_inj_fault = (uint16_t)strtoul(line + 6, NULL, 16);
    return;
  }
  if (strcmp(line, "CLRFAULT") == 0) {
    s_inj_fault = 0;
    return;
  }
  if (strcmp(line, "STAT") == 0) {
    PrintState();
    return;
  }
  if (strcmp(line, "TXTEST") == 0) {
    char b[32];
    int st = Vision_TxTest();
    snprintf(b, sizeof(b), "TXTEST st=%d (0=OK)\r\n", st);
    Reply(b);
    return;
  }
  if (strncmp(line, "TXAUTO,", 7) == 0) {
    s_txauto = (strtof(line + 7, NULL) != 0.0f) ? 1u : 0u;
    return;
  }
  if (strncmp(line, "LOG,", 4) == 0) {
    s_log_period_ms = strtof(line + 4, NULL);
    return;
  }
  Reply("ERR\r\n");
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

/* ============================== 生命周期 ============================== */
void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#if DF1_IMU_ENABLE
  Imu_Init();
#endif
#if DF1_VISION_ENABLE
  Vision_Init();
#endif
  Guidance_Init();
  Fin_Init();

  s_app_en[A_IMU] = DF1_IMU_ENABLE ? 1u : 0u;
  s_app_en[A_VISION] = DF1_VISION_ENABLE ? 1u : 0u;

  /* 注入发布者 + 观察订阅者(attitude/target) */
  s_pub_att = PubRegister(TOPIC_ATTITUDE, sizeof(Dart_Attitude_s));
  s_pub_tgt = PubRegister(TOPIC_TARGET, sizeof(Dart_Target_s));
  s_sub_att = SubRegister(TOPIC_ATTITUDE, sizeof(Dart_Attitude_s));
  s_sub_tgt = SubRegister(TOPIC_TARGET, sizeof(Dart_Target_s));
  s_sub_mix = SubRegister(TOPIC_MIX, sizeof(Dart_Mix_s));
  s_sub_fb = SubRegister(TOPIC_SERVO_FB, sizeof(Dart_ServoFb_s));

  memset(s_mon_hb, 0, sizeof(s_mon_hb));
  memset(s_mon_stale, 0, sizeof(s_mon_stale));
  memset(&s_att_latest, 0, sizeof(s_att_latest));
  memset(&s_tgt_latest, 0, sizeof(s_tgt_latest));

  s_fault_bits = DART_ERR_NONE;
  s_inj_fault = 0;
  s_state = DART_IDLE;
  s_enable = 1;

  (void)DWT_GetDeltaT(&robot->DWT_CNT);

  LOGINFO("[dart1] init done (IMU_EN=%d VIS_EN=%d)", DF1_IMU_ENABLE, DF1_VISION_ENABLE);
  Reply("\r\n=== dart_final_1 ready ===\r\n"
        "cmds: PING | EN,<0|1> | IMUEN,<0|1> | VISEN,<0|1> | ATT,<r>,<p>,<y> | TGT,<x>,<y> | TGTN | "
        "FAULT,<hex> | CLRFAULT | LOG,<ms> | STAT\r\n");
}

void RobotTask(void) {
  static float acc = 0.0f;
  static float txacc = 0.0f;
  float dt_ms;

  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  dt_ms = robot->dt * 1000.0f;

  /* 自检: 自动发串口帧(每200ms), 配合 STM32 TX<->RX 短接验证接收链 */
  if (s_txauto) {
    txacc += dt_ms;
    if (txacc >= 200.0f) {
      txacc = 0.0f;
      Vision_TxTest();
    }
  }

  /* 1) 输入 app */
  if (s_app_en[A_IMU]) Imu_Task();
  if (s_app_en[A_VISION]) Vision_Task();

  /* 2) 看最新输入 -> 监控 -> 状态机 */
  PollInputs();
  Dart_Monitor(dt_ms);
  Dart_StateMachine();

  /* 3) 制导(仅 GUIDING 使能) + 舵面输出 */
  Guidance_Task(robot->dt, (s_state == DART_GUIDING) ? 1u : 0u);
  Fin_Task();

  /* 4) 控制台 */
  ConsolePoll();

  /* 5) 周期遥测(可用 LOG,<ms> 调整; 0=关闭) */
  acc += dt_ms;
  if (s_log_period_ms > 0.0f && acc >= s_log_period_ms) {
    acc = 0.0f;
    PrintState();
  }
}
