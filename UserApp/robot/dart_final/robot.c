/*
 * robot.c — dart_final 应用层入口 / 编排
 * =============================================================================
 * 本文件只做“编排”, 不含具体业务:
 *   1) 初始化各 app(imu/vision/guidance/fin) 并建立话题;
 *   2) RobotTask 周期执行: 各 app 任务 -> Monitor -> 状态机 -> 制导 -> 舵面;
 *   3) 状态机(IDLE/ARMED/GUIDING/FAULT), FAULT 时舵面回中;
 *   4) Monitor: 各 app 心跳(卡死)与错误位汇总, RTT 打印; VOFA 预留接口。
 *
 * 【重要】两处“待配置接口”(赛事规则出来后在此修改, 不要散落到别处):
 *   Dart_IsEnabled()   —— 使能: 现在=上电即使能;
 *   Dart_ShouldGuide() —— 制导时机: 现在=识别到绿光 且 姿态俯冲(Pitch<DIVE_PITCH_DEG)。
 *
 * 数据流:  imu(姿态) ┐
 *                   ├─> guidance ─> (mix) ─> fin ─> servo_mix_ai ─> 4 舵机
 *          vision(目标)┘
 * =============================================================================
 */
#include "robot.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dart_final_cfg.h"
#include "message_center.h"
#include "robot_def.h"
#include "user_lib.h"

#include "fin.h"
#include "guidance.h"
#include "imu.h"
#include "vision.h"

RobotInstance *robot = NULL;

/* ============================ 飞行状态机 ============================ */
typedef enum {
  DART_IDLE = 0,  /* 未使能: 舵面回中, 不制导 */
  DART_ARMED,     /* 已使能, 待制导触发: 舵面回中 */
  DART_GUIDING,   /* 制导中: 按 png_ai 驱动舵面 */
  DART_FAULT,     /* 失效: 舵面回中, 等待恢复 */
} DartState_e;

static DartState_e s_state = DART_IDLE;
static uint16_t s_fault_bits = 0; /* 汇总的故障位(见 robot_def.h DART_ERR_*) */

/* 使“视觉掉线”不致整机 FAULT(它是发射前的正常状态, 由 guidance 的 failsafe 回中即可);
 * 致命故障 = app 卡死 或 IMU 无效 或 舵机失败。 */
#define DART_FAULT_MASK (DART_ERR_IMU_OFF | DART_ERR_GUID_OFF | DART_ERR_FIN_OFF | DART_ERR_SERVO)

/* ======================= 【待配置接口】 ======================= */
/**
 * @brief 使能判定
 * @note  【待配置接口 1】现在: 由 cfg 的 DART_ENABLE_ON_BOOT 决定(默认上电即使能)。
 *        赛事规则确定后, 在此接入真实“使能”信号(如来自发射架的 IO/串口/按键)。
 *        —— 只改这里, 不要改动状态机其它地方!
 */
static bool Dart_IsEnabled(void) {
  return DART_ENABLE_ON_BOOT ? true : true; /* 当前恒使能; 以后替换为真实条件 */
}

/**
 * @brief 制导触发判定
 * @note  【待配置接口 2】现在: 识别到绿光(target.found) 且 姿态俯冲(Pitch < DIVE_PITCH_DEG)。
 *        以后可改为其它判据(如 IMU 检测发射过载)。—— 只改这里。
 */
static bool Dart_ShouldGuide(void) {
  Dart_Attitude_s a;
  Dart_Target_s t;
  Imu_GetAttitude(&a);
  Vision_GetTarget(&t);
  return (t.found && a.valid && (a.pitch_deg < DIVE_PITCH_DEG));
}

/* ============================== Monitor ============================== */
/* 每个 app 的“卡死”检测: 其任务每周期会自增 hb; 若 hb 连续 APP_STUCK_MS 未变, 判为卡死。*/
#define MON_IMU 0
#define MON_VISION 1
#define MON_GUID 2
#define MON_FIN 3
#define MON_N 4

static uint32_t s_mon_hb[MON_N];    /* 上次记录的心跳值 */
static uint32_t s_mon_stale[MON_N]; /* 心跳未更新累计时长(ms) */
static uint32_t s_mon_app_stuck_ms = 50u; /* 卡死阈值(ms) */

static uint16_t MonErrOf(int idx) {
  switch (idx) {
    case MON_IMU: return DART_ERR_IMU_OFF;
    case MON_VISION: return DART_ERR_VISION_OFF;
    case MON_GUID: return DART_ERR_GUID_OFF;
    case MON_FIN: return DART_ERR_FIN_OFF;
    default: return DART_ERR_NONE;
  }
}

/**
 * @brief 汇总各 app 健康状态(心跳 + 自身错误位) -> s_fault_bits
 * @param dt_ms 本周期时间(ms)
 */
static void Dart_Monitor(float dt_ms) {
  const Dart_AppStatus_s *st[MON_N];
  uint16_t bits = DART_ERR_NONE;
  int i;

  st[MON_IMU] = Imu_GetStatus();
  st[MON_VISION] = Vision_GetStatus();
  st[MON_GUID] = Guidance_GetStatus();
  st[MON_FIN] = Fin_GetStatus();

  for (i = 0; i < MON_N; i++) {
    if (st[i]->hb != s_mon_hb[i]) {
      s_mon_hb[i] = st[i]->hb;
      s_mon_stale[i] = 0;
    } else {
      s_mon_stale[i] += (uint32_t)dt_ms;
      if (s_mon_stale[i] > s_mon_app_stuck_ms) bits |= MonErrOf(i); /* 卡死 */
    }
    bits |= st[i]->err; /* app 自报错误位 */
  }
  s_fault_bits = bits;
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
        s_state = DART_ARMED; /* 故障消失 -> 回到待制导 */
      break;
    default:
      s_state = DART_IDLE;
      break;
  }
}

/* ============================ 遥测 / 日志 ============================ */
/**
 * @brief [VOFA 预留] 把关键量发到 VOFA+ (JustFloat)。当前未启用。
 * @note  本次只用 RTT; 将来接 VOFA 时: 取消下面注释并填端口/通道。
 */
static void Dart_Vofa_Push(void) {
  /* ==== VOFA 预留接口 (本次不启用) ====
   * 需先在 RobotInit 里调用 VOFAInit(DART_USART_VOFA);
   * 然后把各 app 关键量填入 float 数组后:
   *   float buf[8] = { attitude.roll, mix.yaw, fb.defl_deg[0], ... };
   *   VOFAJustFloatSend(buf, 8);
   */
}

/**
 * @brief 每秒用 RTT 打印一次整机状态(状态机 + 故障位 + 各 app 心跳)
 */
static void Dart_Telemetry(float dt_ms) {
  static float acc = 0.0f;
  acc += dt_ms;
  if (acc < 1000.0f) return;
  acc = 0.0f;

  LOGINFO("[dart] state=%d fault=0x%04X | hb imu=%u vis=%u guid=%u fin=%u",
          (int)s_state, s_fault_bits,
          (unsigned)Imu_GetStatus()->hb, (unsigned)Vision_GetStatus()->hb,
          (unsigned)Guidance_GetStatus()->hb, (unsigned)Fin_GetStatus()->hb);

  Dart_Vofa_Push(); /* 预留, 当前空实现 */
}

/* ============================== 生命周期 ============================== */
void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  Imu_Init();       /* 姿态(含 INS 任务, 阻塞约 1s) */
  Vision_Init();    /* 视觉串口 */
  Guidance_Init();  /* 制导+控制 */
  Fin_Init();       /* 4 舵面(上电回中) */

  memset(s_mon_hb, 0, sizeof(s_mon_hb));
  memset(s_mon_stale, 0, sizeof(s_mon_stale));
  s_fault_bits = DART_ERR_NONE;
  s_state = DART_IDLE;

  /* 各 app 的“卡死”阈值可在此按需调整(默认 50ms) */
  s_mon_app_stuck_ms = 50u;

  (void)DWT_GetDeltaT(&robot->DWT_CNT); /* 初始化 dt 基准 */
  LOGINFO("[dart_final] init done");
}

void RobotTask(void) {
  float dt_ms;

  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  dt_ms = robot->dt * 1000.0f;

  /* 1) 输入 */
  Imu_Task();
  Vision_Task();

  /* 2) 监控 + 状态机(决定是否制导) */
  Dart_Monitor(dt_ms);
  Dart_StateMachine();

  /* 3) 制导(仅 GUIDING 时使能, 否则输出回中) + 舵面输出 */
  Guidance_Task(robot->dt, (s_state == DART_GUIDING) ? 1u : 0u);
  Fin_Task();

  /* 4) 遥测 */
  Dart_Telemetry(dt_ms);
}
