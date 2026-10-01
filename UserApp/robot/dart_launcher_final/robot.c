/*
 * robot.c — dart_launcher_final 应用层入口 / 编排
 * =============================================================================
 * 本文件只做"编排", 不含任何业务:
 *   1) 初始化各 app(link/cmd/fsm/motor/trigger/vision/yaw);
 *   2) RobotTask 周期执行: 各 app 任务 -> Monitor -> 顶层状态机;
 *   3) Monitor: 各 app 心跳(卡死)与错误位汇总, RTT 打印; VOFA 预留接口。
 *
 * 【顶层状态机】只有 4 态, 只表达"整机处于什么阶段", 不含机构细节:
 *   IDLE  —— 上电/停止: 各机构静止(拉簧保持, 不卸力)
 *   READY —— 待命: 允许接受运动指令与自动流程
 *   RUN   —— 自动流程进行中(由 app/fsm 推进)
 *   FAULT —— 致命故障: 交由各机构按自身急停策略处理
 *
 * 【与 app/fsm 的区别】fsm 是"发射流程"的步骤机(上膛/待发/释放...);
 *   本文件的状态机是"整机阶段"(是否允许动作), 两者层次不同、不重复。
 *
 * 数据流:  link(上位机) ─> cmd(大脑) ─> fsm(时序) ─> motor/trigger/yaw(机构)
 *                                   └────────────────> launch_state ─> link(遥测)
 * =============================================================================
 */
#include "robot.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "launcher_cfg.h"
#include "message_center.h"
#include "robot_def.h"
#include "user_lib.h"

#include "cmd.h"
#include "fsm.h"
#include "link.h"
#include "motortest.h"
#include "trigger.h"
#include "vision.h"
#include "yaw.h"

RobotInstance *robot = NULL;

/* ============================ 顶层状态机 ============================ */
typedef enum {
  SYS_IDLE = 0, /* 上电初始: 机构静止 */
  SYS_READY,    /* 待命: 接受指令 */
  SYS_RUN,      /* 自动流程进行中 */
  SYS_FAULT,    /* 致命故障 */
} SysState_e;

static SysState_e s_state = SYS_IDLE;
static uint16_t s_fault_bits = 0;

/* ============================== Monitor ============================== */
/* 每个 app 的"卡死"检测: 其任务每周期自增 hb; 若 hb 连续阈值未变, 判为卡死 */
#define MON_LINK 0
#define MON_CMD 1
#define MON_FSM 2
#define MON_MOTOR 3
#define MON_TRIGGER 4
#define MON_VISION 5
#define MON_YAW 6
#define MON_N 7

static uint32_t s_mon_hb[MON_N];
static uint32_t s_mon_stale[MON_N];
static uint32_t s_mon_stuck_ms = 50u;

static uint16_t MonErrOf(int idx) {
  switch (idx) {
    case MON_LINK: return LAUNCH_ERR_LINK_OFF;
    case MON_CMD: return LAUNCH_ERR_CMD_OFF;
    case MON_FSM: return LAUNCH_ERR_FSM_OFF;
    case MON_MOTOR: return LAUNCH_ERR_MOTOR_OFF;
    case MON_TRIGGER: return LAUNCH_ERR_TRIGGER_OFF;
    case MON_VISION: return LAUNCH_ERR_VISION_OFF;
    case MON_YAW: return LAUNCH_ERR_YAW_OFF;
    default: return LAUNCH_ERR_NONE;
  }
}

/**
 * @brief 汇总各 app 健康状态(心跳 + 自身错误位) -> s_fault_bits
 * @param dt_ms 本周期时间(ms)
 */
static void SysMonitor(float dt_ms) {
  const Launcher_AppStatus_s *st[MON_N];
  uint16_t bits = LAUNCH_ERR_NONE;
  int i;

  st[MON_LINK] = Link_GetStatus();
  st[MON_CMD] = Cmd_GetStatus();
  st[MON_FSM] = Fsm_GetStatus();
  st[MON_MOTOR] = Motortest_GetStatus();
  st[MON_TRIGGER] = Trigger_GetStatus();
  st[MON_VISION] = Vision_GetStatus();
  st[MON_YAW] = Yaw_GetStatus();

  for (i = 0; i < MON_N; i++) {
    if (st[i]->hb != s_mon_hb[i]) {
      s_mon_hb[i] = st[i]->hb;
      s_mon_stale[i] = 0;
    } else {
      s_mon_stale[i] += (uint32_t)dt_ms;
      if (s_mon_stale[i] > s_mon_stuck_ms) bits |= MonErrOf(i);
    }
    bits |= st[i]->err;
  }
  s_fault_bits = bits;
}

/* ============================== 顶层状态机 ============================== */
/**
 * @brief 整机阶段推进
 * @note  只依据"是否致命故障"与"自动流程是否在跑"两个条件;
 *        急停等细节由 app/cmd 与各机构 app 各自处理, 此处不越权。
 */
static void SysStateMachine(void) {
  bool fault = ((s_fault_bits & LAUNCH_FATAL_MASK) != 0);

  switch (s_state) {
    case SYS_IDLE:
      /* 上电后等一小段时间让各 app 自检(取零/在线判定), 再进 READY。
       * 简单起见: 无致命故障即进 READY。 */
      if (!fault) s_state = SYS_READY;
      break;

    case SYS_READY:
      if (fault) {
        s_state = SYS_FAULT;
      } else if (Fsm_GetStatus()->key >= (float)LAUNCH_TASK_SERVO_STD1 &&
                 Fsm_GetStatus()->key < (float)LAUNCH_TASK_DONE) {
        s_state = SYS_RUN;
      }
      break;

    case SYS_RUN:
      if (fault) {
        s_state = SYS_FAULT;
      } else if (Fsm_GetStatus()->key == (float)LAUNCH_TASK_IDLE ||
                 Fsm_GetStatus()->key == (float)LAUNCH_TASK_DONE) {
        s_state = SYS_READY;
      }
      break;

    case SYS_FAULT:
      if (!fault) s_state = SYS_READY; /* 故障消除 -> 回待命 */
      break;

    default:
      s_state = SYS_IDLE;
      break;
  }
}

/* ============================ 遥测 / 日志 ============================ */
/**
 * @brief [VOFA 预留] 把关键量发到 VOFA+ (JustFloat)。当前未启用。
 */
static void Dart_Vofa_Push(void) {
  /* ==== VOFA 预留接口 (本次不启用) ====
   * 需先在 RobotInit 里调用 VOFAInit(LAUNCH_USART_VOFA);
   * 然后把各 app 关键量填入 float 数组后:
   *   float buf[8] = { 拉簧A角度, 拉簧B角度, yaw转速, ... };
   *   VOFAJustFloatSend(buf, 8);
   */
}

/**
 * @brief 每秒打印一次整机状态(阶段 + 故障位 + 各 app 心跳)
 */
static void SysTelemetry(float dt_ms) {
  static float acc = 0.0f;
  acc += dt_ms;
  if (acc < 1000.0f) return;
  acc = 0.0f;

  LOGINFO("[lch] sys=%d fault=0x%04X | hb link=%u cmd=%u fsm=%u mot=%u trg=%u vis=%u yaw=%u",
          (int)s_state, s_fault_bits, (unsigned)Link_GetStatus()->hb,
          (unsigned)Cmd_GetStatus()->hb, (unsigned)Fsm_GetStatus()->hb,
          (unsigned)Motortest_GetStatus()->hb, (unsigned)Trigger_GetStatus()->hb,
          (unsigned)Vision_GetStatus()->hb, (unsigned)Yaw_GetStatus()->hb);

  /* 机构关键量: 无网页时靠这条在 RTT 观察机构状态(角度为输出侧 deg)
   * 【注意】格式串的 %d 个数必须与参数个数**严格相等**, 否则参数错位、读到的
   *   是别的变量(曾因此把 B 的 717 度显示成 0, 掩盖了真实状态)。
   *   本行 11 个 %d <=> 11 个参数, 一一对应。 */
  LOGINFO("[lch] sys=%d fault=0x%04X step=%d | A deg=%d on=%d at=%d | B deg=%d on=%d at=%d",
          (int)s_state, s_fault_bits, (int)Fsm_GetStatus()->key,
          (int)Motortest_GetAngle(0), (int)Motortest_IsOnline(0), (int)Motortest_AtTarget(0),
          (int)Motortest_GetAngle(1), (int)Motortest_IsOnline(1), (int)Motortest_AtTarget(1));

  Dart_Vofa_Push();
}

/* ============================== 生命周期 ============================== */
void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  /* 顺序说明:
   *   先建"机构"app(它们注册话题并初始化硬件), 再建"逻辑"app(它们订阅话题)。
   *   message_center 允许订阅者晚于发布者注册, 但按此顺序更直观。 */
  /*
   * 【当前调试阶段】只跑最小电机 app(motortest), 不启用原 app/motor。
   * 原因: 两套 app 都会注册同一批 CAN 电机(ID 2/3/4), 同时启用会在
   *   MotorSenderGrouping 里触发 "ID crash" 死循环, 且互相覆盖目标。
   * 待 motortest 把方向/整定验证完毕, 再决定是把它并入 motor.c 还是反之。
   */
  Motortest_Init(); /* 最小电机 app: 只做"转到指定角度" */
  Trigger_Init();   /* PWM 舵机 */
  Yaw_Init();       /* 自瞄 yaw 轴 */
  Vision_Init();    /* 视觉坐标归一化 */
  Fsm_Init();       /* 时序状态机 */
  Cmd_Init();       /* 大脑: 指令 -> 目标 */
  Link_Init();      /* 上位机串口(唯一感知网页的地方) */

  memset(s_mon_hb, 0, sizeof(s_mon_hb));
  memset(s_mon_stale, 0, sizeof(s_mon_stale));
  s_fault_bits = LAUNCH_ERR_NONE;
  s_state = SYS_IDLE;

  (void)DWT_GetDeltaT(&robot->DWT_CNT); /* 初始化 dt 基准 */
  LOGINFO("[dart_launcher_final] init done");
}

void RobotTask(void) {
  float dt_ms;

  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  dt_ms = robot->dt * 1000.0f;

  /* 1) 通信 + 输入 app */
  Link_Task();    /* 收上位机指令(发 launch_cmd) + 发遥测 */
  Vision_Task();  /* 坐标 -> aim_cmd */

  /* 2) 逻辑 app: 大脑 -> 时序 */
  Cmd_Task(); /* launch_cmd -> motor_cmd/servo_cmd/yaw_cmd/fsm_cmd */
  Fsm_Task(); /* 自动流程推进 -> motor_cmd/servo_cmd */

  /* 3) 机构 app(执行) */
  Motortest_Task();
  Trigger_Task();
  Yaw_Task();

  /* 4) 监控 + 顶层状态机 */
  SysMonitor(dt_ms);
  SysStateMachine();

  /* 5) 遥测 */
  SysTelemetry(dt_ms);
}
