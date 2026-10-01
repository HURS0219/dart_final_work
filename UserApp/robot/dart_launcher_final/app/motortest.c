/*
 * motortest.c — 最小可用电机测试 app(只用 DJIMotor module 接口)
 * =============================================================================
 * 【设计原则: 最少的概念、最少的状态、最少的方向变换】
 *
 * 一、闭环组合(照抄实机跑通过的 dart_launcher_web_v5_HIK)
 *      outer_loop_type = ANGLE_LOOP      -> 下发时角度环参与
 *      close_loop_type = SPEED|ANGLE     -> 角度环 -> 速度环 -> 电流(串级)
 *      angle_PID = {Kp=5.0, Ki=0.0, Kd=0.3, MaxOut=13826, DeadBand=8,
 *                   Improve = PID_Integral_Limit | PID_Derivative_On_Measurement}
 *      speed_PID = {Kp=2.5, Ki=0.1, Kd=0.0, MaxOut=16384, DeadBand=10,
 *                   IntegralLimit=800,
 *                   Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle}
 *    【必须两个环都配】串级时最终下发的是**速度环输出**; 若速度环漏配(Kp=Ki=0),
 *      输出恒为 0, 现象是"目标与 pid_ref 都对但电机不动"(踩过)。
 *
 * 二、方向(只有一处符号, 且与库的字段一致)
 *    【本文件采用: 方向只在一处生效】
 *      库的两个字段永远 NORMAL(不翻):
 *          motor_reverse_flag    = MOTOR_DIRECTION_NORMAL
 *          feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL
 *      方向完全由 cfg 的 LAUNCH_S?_DIR(+1/-1) 决定, 且在 OutDeg / RotorRef
 *      **各乘一次**(见 DirOf):
 *          OutDeg   = DIR * (total_angle - zero) / ratio
 *          RotorRef = zero + DIR * deg * ratio
 *      闭环内 DIR 乘两次 => 净 +1(负反馈, 稳定); 而"命令正角对应物理哪个方向"
 *      完全由 DIR 决定 —— 翻它一定能反转。
 *
 *    【血泪教训: 为什么以前方向反复改不对】
 *      过去把 motor_reverse_flag 与 feedback_reverse_flag 一起跟随 REVERSE, 又让
 *      换算函数乘 SignOf, 闭环里符号一共翻了 4 次(偶数) => **净效果为零**:
 *      表现为"改了 REVERSE 但物理转向没变"; 更糟的组合会让目标与执行方向相反,
 *      形成**正反馈** —— 电机朝反方向一路冲到底、满电流死顶(曾把弹簧拉到底弹射)。
 *      根因不是"运气", 而是符号散落在多处、从未数清奇偶性。现在只留一处。
 *
 *    【调试铁律】改方向后必须先把 MaxOut 降到几百、目标给几度, 确认转向正确
 *      再逐步加大; 绝不可满电流试方向。
 * 三、零点: 上电首次收到 CAN 反馈时把当前位置记为 0°(也可用 Motortest_Zero 重取)。
 * =============================================================================
 */
#include "motortest.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dji_motor.h"
#include "launcher_cfg.h"
#include "robot_def.h"

/* ============================== 静态配置 ============================== */

/* 本 app 管的电机: 索引与 Launcher_MotorSlot_e 对齐(0=拉簧A, 1=拉簧B, 2=丝杆) */
typedef struct {
  uint8_t tx_id;   /* CAN 发送 ID */
  int8_t dir;      /* ★方向: +1 / -1(唯一方向开关) */
  float ratio;     /* 减速比(转子:输出) */
  float tol_deg;   /* 到位容差(输出侧 deg) */
} MotorCfg_s;

static const MotorCfg_s kCfg[LAUNCH_M_COUNT] = {
    [LAUNCH_M_SPRING_A] = {LAUNCH_SA_ID, LAUNCH_SA_DIR, LAUNCH_SA_RATIO, LAUNCH_SA_TOL_DEG},
    [LAUNCH_M_SPRING_B] = {LAUNCH_SB_ID, LAUNCH_SB_DIR, LAUNCH_SB_RATIO, LAUNCH_SB_TOL_DEG},
    [LAUNCH_M_SCREW] = {LAUNCH_SC_ID, LAUNCH_SC_DIR, LAUNCH_SC_RATIO, LAUNCH_SC_TOL_DEG},
};

/* ============================== 运行状态 ============================== */

typedef struct {
  DJIMotorInstance *inst;   /* module 返回的实例; NULL=未初始化 */
  float zero;               /* 零点(转子侧 total_angle 快照) */
  uint8_t zero_valid;       /* 1=已取零 */
  uint32_t last_feed;       /* 上次 feed_cnt, 用于判在线 */
  uint8_t online;           /* 1=CAN 有回传 */
  uint8_t run;              /* 1=已使能并使能角度环 */
  float target;             /* 目标输出角度(deg) */
} MotorRt_s;

static MotorRt_s s_rt[LAUNCH_M_COUNT];

/* 健康状态(定义在文件末尾, 这里前置声明供 Task 调用) */
static Launcher_AppStatus_s s_st;
static void ReportStatus(uint16_t err);

/* 构造初始化配置(定义在后面; NaN 看门狗要用到它, 故前置声明) */
static Motor_Init_Config_s MakeCfg(int i);

/* ============================== 工具函数 ============================== */

/** @brief 方向系数 —— **全工程唯一的方向开关**
 * @note  取值来自 cfg 的 LAUNCH_S?_DIR(+1 / -1)。
 *   OutDeg 与 RotorRef **都乘它**, 于是闭环内符号净为 (+1)*(+1)=+1(负反馈, 稳定),
 *   而"命令正角对应物理哪个方向"完全由它决定 —— 翻它就一定能反转, 不会出现
 *   "翻了等于没翻"的抵消(那是过去把符号散落在库字段+换算函数里造成的)。 */
static float DirOf(int i) { return (float)kCfg[i].dir; }

/** @brief 输出侧角度(deg, 相对零点, 已含方向) */
static float OutDeg(int i) {
  const MotorRt_s *r = &s_rt[i];
  if (!r->zero_valid) return 0.0f;
  return DirOf(i) * (r->inst->measure.total_angle - r->zero) / kCfg[i].ratio;
}

/** @brief 输出侧转速(rpm); speed_aps 单位 deg/s */
static float OutRpm(int i) {
  const MotorRt_s *r = &s_rt[i];
  float ratio = (kCfg[i].ratio > 0.01f) ? kCfg[i].ratio : 1.0f;
  return DirOf(i) * r->inst->measure.speed_aps / 6.0f / ratio;
}

/** @brief 输出侧角度 -> 转子侧目标(角度环设定值); 与 OutDeg 同乘 DirOf */
static float RotorRef(int i, float deg) {
  const MotorRt_s *r = &s_rt[i];
  return r->zero + DirOf(i) * deg * kCfg[i].ratio;
}

/* ======================= 数值看门狗(修 NaN 污染) ======================= * 【问题】controller.c 的 PIDCalculate 里有一行**无条件**执行的微分项:
 *     pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
 *   当某次调用恰好落在同一个 DWT 周期内时 dt == 0 -> x/0 = ±inf ->
 *   Kd(即使是 0) * inf = **NaN**。NaN 一旦写入 Iout/Last_Err 就永久保留,
 *   于是 final_output 永远为 nan, 电机彻底不动(实测: outer=ANGLE_LOOP 时必现)。
 *
 * 【修法】应用层每周期检查: 若 final_output 是 NaN, 就按 cfg 重新 PIDInit
 *   该实例的三个 PID(等价于清掉运行时状态)。这是应用层能做的最干净的处理,
 *   不需要改动 Modules/。
 * 【注意】只重建 PID, 不动 measure/zero, 所以不会丢零点。 */
static void PidNaNCheck(int i) {
  MotorRt_s *r = &s_rt[i];
  DJIMotorInstance *m = r->inst;
  float out = m->motor_controller.final_output;

  if (out == out) return; /* 不是 NaN, 正常 */

  /* 三个 PID 全部按同一套 cfg 重建(保持与 MakeCfg 一致) */
  Motor_Init_Config_s cfg = MakeCfg(i);
  PIDInit(&m->motor_controller.current_PID, &cfg.controller_param_init_config.current_PID);
  PIDInit(&m->motor_controller.speed_PID, &cfg.controller_param_init_config.speed_PID);
  PIDInit(&m->motor_controller.angle_PID, &cfg.controller_param_init_config.angle_PID);
  m->motor_controller.pid_ref = 0.0f;
  m->motor_controller.final_output = 0.0f;
  LOGWARNING("[mt] slot=%d NaN detected -> PID re-init", i);
}

/* ============================== 初始化 ============================== */

/** @brief 构造一路 M3508 的初始化配置(参数照抄实机验证过的旧版) */
static Motor_Init_Config_s MakeCfg(int i) {
  const MotorCfg_s *c = &kCfg[i];
  Motor_Init_Config_s cfg;
  memset(&cfg, 0, sizeof(cfg));

  cfg.motor_type = M3508;
  cfg.can_init_config.can_handle = &hcan1;
  cfg.can_init_config.tx_id = c->tx_id;

  /* --- 控制设置 --- */
  cfg.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  cfg.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  cfg.controller_setting_init_config.outer_loop_type = ANGLE_LOOP; /* 每次下发也会设, 这里定上电默认 */
  cfg.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
  /* --- 方向: 库的两个字段**永远 NORMAL**(彻底消除抵消) ---
   * 方向完全由 kCfg[].dir 在 OutDeg/RotorRef 里乘一次来实现(见 DirOf)。
   * 【为什么要这样】过去把 motor_reverse_flag / feedback_reverse_flag 也一起翻,
   *   再叠加换算函数的符号, 闭环里符号一共翻了 4 次(偶数) => 净效果为零,
   *   表现为"改了 REVERSE 但物理转向没变", 甚至目标与执行方向相反形成正反馈,
   *   导致电机朝反方向一路冲到底。现在符号只在一处, 奇偶性永远清晰。 */
  cfg.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  cfg.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;

  /* --- 速度环 PID(串级内环, 必须配!) --- */
  cfg.controller_param_init_config.speed_PID.Kp = LAUNCH_SA_SPEED_KP;
  cfg.controller_param_init_config.speed_PID.Ki = LAUNCH_SA_SPEED_KI;
  cfg.controller_param_init_config.speed_PID.Kd = LAUNCH_SA_SPEED_KD;
  cfg.controller_param_init_config.speed_PID.MaxOut = LAUNCH_SA_SPEED_MAXOUT;
  cfg.controller_param_init_config.speed_PID.DeadBand = LAUNCH_SA_SPEED_DEADBAND;
  cfg.controller_param_init_config.speed_PID.IntegralLimit = LAUNCH_SA_SPEED_ILIMIT;
  cfg.controller_param_init_config.speed_PID.Improve =
      PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle;

  /* --- 角度环 PID(外环) --- */
  cfg.controller_param_init_config.angle_PID.Kp = LAUNCH_SA_ANGLE_KP;
  cfg.controller_param_init_config.angle_PID.Ki = LAUNCH_SA_ANGLE_KI;
  cfg.controller_param_init_config.angle_PID.Kd = LAUNCH_SA_ANGLE_KD;
  cfg.controller_param_init_config.angle_PID.MaxOut = LAUNCH_SA_ANGLE_MAXOUT;
  cfg.controller_param_init_config.angle_PID.DeadBand = LAUNCH_SA_ANGLE_DEADBAND;
  cfg.controller_param_init_config.angle_PID.IntegralLimit = LAUNCH_SA_ANGLE_ILIMIT;
  cfg.controller_param_init_config.angle_PID.Improve =
      PID_Integral_Limit | PID_Derivative_On_Measurement;

  return cfg;
}

void Motortest_Init(void) {
  for (int i = 0; i < LAUNCH_M_COUNT; i++) {
    MotorRt_s *r = &s_rt[i];
    memset(r, 0, sizeof(*r));

    Motor_Init_Config_s cfg = MakeCfg(i);
    r->inst = DJIMotorInit(&cfg);
    if (r->inst == NULL) {
      LOGERROR("[mt] init FAIL slot=%d id=%d", i, kCfg[i].tx_id);
      continue;
    }
    /* 上电先停, 等首次反馈取零后由指令接管 */
    DJIMotorStop(r->inst);
    r->run = 0;
    r->target = 0.0f;
    LOGINFO("[mt] slot=%d id=%d ready", i, kCfg[i].tx_id);
  }
  LOGINFO("[mt] init done (%d slots)", LAUNCH_M_COUNT);
}

/* ============================== 周期任务 ============================== */

/** @brief 刷新在线/取零 */
static void Refresh(int i) {
  MotorRt_s *r = &s_rt[i];
  DJIMotorInstance *m = r->inst;

  if (m->feed_cnt != r->last_feed) {
    r->last_feed = m->feed_cnt;
    r->online = 1;
  }
  /* 首次收到反馈自动取零 */
  if (m->feed_cnt > 0 && !r->zero_valid) {
    r->zero = m->measure.total_angle;
    r->zero_valid = 1;
    LOGINFO("[mt] slot=%d zero latched", i);
  }
}

void Motortest_Task(void) {
  uint16_t err = LAUNCH_ERR_NONE;
  int inited = 0, online = 0;

  for (int i = 0; i < LAUNCH_M_COUNT; i++) {
    MotorRt_s *r = &s_rt[i];
    if (r->inst == NULL) continue;
    inited++;

    Refresh(i);
    if (r->online) online++;
    if (!r->zero_valid) continue; /* 还没收到反馈, 不能闭环 */

    /* NaN 看门狗: 一次 dt=0 就会污染 Iout, 必须每周期检查(见 PidNaNCheck 注释) */
    PidNaNCheck(i);

    if (!r->run) continue;        /* 未启动 -> 保持 stop, 不动 */

    /* 角度环下发(每次都要设 OuterLoop: 库会记住上一次的设置) */
    DJIMotorEnable(r->inst);
    DJIMotorOuterLoop(r->inst, ANGLE_LOOP);
    DJIMotorSetPIDRef(r->inst, RotorRef(i, r->target));
  }

  /* 错误位: 一路都没 init 或 全部掉线 -> 报 MOTOR_OFF(供顶层 Monitor 汇总) */
  if (inited == 0 || online == 0) err |= LAUNCH_ERR_MOTOR_OFF;
  ReportStatus(err);
}

/* ============================== 对外接口 ============================== */

void Motortest_SetAngle(int slot, float deg) {
  if (slot < 0 || slot >= LAUNCH_M_COUNT) return;
  MotorRt_s *r = &s_rt[slot];
  if (r->inst == NULL) return;
  r->target = deg;
  r->run = 1;
}

void Motortest_Stop(int slot) {
  if (slot < 0 || slot >= LAUNCH_M_COUNT) return;
  MotorRt_s *r = &s_rt[slot];
  if (r->inst == NULL) return;
  r->run = 0;
  DJIMotorStop(r->inst);
}

void Motortest_Zero(int slot) {
  if (slot < 0 || slot >= LAUNCH_M_COUNT) return;
  MotorRt_s *r = &s_rt[slot];
  if (r->inst == NULL) return;
  r->zero = r->inst->measure.total_angle;
  r->zero_valid = 1;
}

float Motortest_GetAngle(int slot) {
  if (slot < 0 || slot >= LAUNCH_M_COUNT) return 0.0f;
  if (s_rt[slot].inst == NULL) return 0.0f;
  return OutDeg(slot);
}

float Motortest_GetRpm(int slot) {
  if (slot < 0 || slot >= LAUNCH_M_COUNT) return 0.0f;
  if (s_rt[slot].inst == NULL) return 0.0f;
  return OutRpm(slot);
}

uint8_t Motortest_IsOnline(int slot) {
  if (slot < 0 || slot >= LAUNCH_M_COUNT) return 0;
  return s_rt[slot].online;
}

uint8_t Motortest_AtTarget(int slot) {
  if (slot < 0 || slot >= LAUNCH_M_COUNT) return 0;
  if (s_rt[slot].inst == NULL || !s_rt[slot].zero_valid) return 0;
  return (fabsf(OutDeg(slot) - s_rt[slot].target) < kCfg[slot].tol_deg) ? 1u : 0u;
}

float Motortest_GetOutput(int slot) {
  if (slot < 0 || slot >= LAUNCH_M_COUNT) return 0.0f;
  if (s_rt[slot].inst == NULL) return 0.0f;
  return s_rt[slot].inst->motor_controller.final_output;
}

/* ============================== 健康状态 ============================== */

const Launcher_AppStatus_s *Motortest_GetStatus(void) { return &s_st; }

/** @brief 由 Task 末尾调用, 刷新心跳与错误位(与其它 app 同构) */
static void ReportStatus(uint16_t err) {
  s_st.hb++;
  s_st.err = err;
}
