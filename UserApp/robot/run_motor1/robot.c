/*
 * robot.c — run_motor1: 台架"转一个电机"最小 app
 * =============================================================================
 * 目标: 只调用 motor module(DJImotor)的接口, 让一个 M3508 按角度环连续旋转。
 *
 * 【数据流(全部走 module 提供的接口, 不碰底层)】
 *   RobotInit(): DJIMotorInit(&cfg)          -> 注册电机到 CAN
 *   RobotTask(): DJIMotorSetPIDRef(m, ref)   -> 算 PID, 得电流输出
 *   motor_task.c -> DJIMotorTask()           -> 打包并发送 CAN(在 os_task 的 MOTOR 任务里)
 *
 * 【角度环怎么"转起来"】
 *   给一个**连续递增**的目标角度(等于"给定角度序列"), 角度环就会持续跟踪,
 *   于是输出轴匀速旋转。目标 = 输出轴 deg; 送入库前换算成转子侧:
 *       RotorRef = zero + Sign * out_deg * ratio
 *
 * 【方向(只有一处符号)】
 *   库: motor_reverse_flag 决定发送电流符号(物理转向);
 *       feedback_reverse_flag 决定 total_angle/speed_aps 是否取负。
 *   两者必须一起跟随 RUN_MOTOR_REVERSE, 否则闭环会变正反馈(电机冲到底)。
 *   本文件用 SignOf() 统一取 motor_reverse_flag, 保证"命令正角度 -> 物理固定方向"。
 *
 * 【零点】上电首次收到 CAN 反馈时, 把当前 total_angle 记为 0°, 之后目标从 0 起算。
 * =============================================================================
 */
#include "robot.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dji_motor.h"
#include "message_center.h"
#include "robot_def.h"
#include "run_motor_cfg.h"

/* ============================== 全局实例 ============================== */
static RobotInstance s_robot;
RobotInstance *robot = &s_robot; /* 供 robot.h 的 extern 使用 */

/* ============================== 电机 / 话题 ============================== */
static DJIMotorInstance *s_motor = NULL;
static Publisher_t *s_pub_fb = NULL;
static Subscriber_t *s_sub_cmd = NULL;

/* ============================== 运行状态 ============================== */
static float s_zero = 0.0f;        /* 上电零点(转子侧 total_angle 快照) */
static uint8_t s_zero_valid = 0;   /* 1=已取零, 可以闭环 */
static uint8_t s_pid_primed = 0;   /* 1=PID 运行时状态已清过一次(规避首拍除零 NaN) */
static uint32_t s_last_feed = 0;   /* 上次 feed_cnt, 判在线 */
static uint8_t s_online = 0;       /* 1=CAN 有回传 */

static float s_target_deg = 0.0f;                      /* 角度环目标(输出侧 deg) */
static float s_target_dps = RUN_MOTOR_TARGET_DPS;      /* 输出轴目标转速(deg/s) */
static uint8_t s_enable = 1;                           /* 1=运行, 0=停止 */

static RunMotor_Fb_s s_fb; /* 反馈缓存 */

/* ============================== 工具函数 ============================== */
/* 方向符号: 取自库里的 motor_reverse_flag(与 feedback_reverse_flag 保持一致) */
static float SignOf(void) {
  return (s_motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
}

/* 输出轴角度(deg, 相对零点, 已含方向) */
static float OutDeg(void) {
  if (!s_zero_valid) return 0.0f;
  return SignOf() * (s_motor->measure.total_angle - s_zero) / RUN_MOTOR_RATIO;
}

/* 输出轴转速(deg/s, 已含方向); speed_aps 单位 deg/s(转子侧) */
static float OutDps(void) {
  return SignOf() * s_motor->measure.speed_aps / RUN_MOTOR_RATIO;
}

/* 输出侧目标角度 -> 转子侧角度环设定值 */
static float RotorRef(float out_deg) {
  return s_zero + SignOf() * out_deg * RUN_MOTOR_RATIO;
}

/* ============================== 初始化 ============================== */
static Motor_Init_Config_s MakeCfg(void) {
  Motor_Init_Config_s cfg;
  memset(&cfg, 0, sizeof(cfg));

  /* --- 型号 + CAN --- */
  cfg.motor_type = RUN_MOTOR_TYPE;
  cfg.can_init_config.can_handle = &RUN_MOTOR_CAN; /* hcan1 */
  cfg.can_init_config.tx_id = RUN_MOTOR_TX_ID;

  /* --- 控制设置: 角度环(外) -> 速度环(内) -> 电流 --- */
  cfg.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  cfg.controller_setting_init_config.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
  cfg.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  cfg.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  cfg.controller_setting_init_config.motor_reverse_flag = RUN_MOTOR_REVERSE;
  cfg.controller_setting_init_config.feedback_reverse_flag =
      (RUN_MOTOR_REVERSE == MOTOR_DIRECTION_REVERSE) ? FEEDBACK_DIRECTION_REVERSE
                                                     : FEEDBACK_DIRECTION_NORMAL;

  /* --- 角度环 PID(外环) --- */
  cfg.controller_param_init_config.angle_PID.Kp = RUN_MOTOR_ANGLE_KP;
  cfg.controller_param_init_config.angle_PID.Ki = RUN_MOTOR_ANGLE_KI;
  cfg.controller_param_init_config.angle_PID.Kd = RUN_MOTOR_ANGLE_KD;
  cfg.controller_param_init_config.angle_PID.MaxOut = RUN_MOTOR_ANGLE_MAXOUT;
  cfg.controller_param_init_config.angle_PID.DeadBand = 0.0f;
  cfg.controller_param_init_config.angle_PID.Improve =
      PID_Integral_Limit | PID_Derivative_On_Measurement;

  /* --- 速度环 PID(内环, 必须配置!) --- */
  cfg.controller_param_init_config.speed_PID.Kp = RUN_MOTOR_SPEED_KP;
  cfg.controller_param_init_config.speed_PID.Ki = RUN_MOTOR_SPEED_KI;
  cfg.controller_param_init_config.speed_PID.Kd = RUN_MOTOR_SPEED_KD;
  cfg.controller_param_init_config.speed_PID.MaxOut = RUN_MOTOR_SPEED_MAXOUT;
  cfg.controller_param_init_config.speed_PID.DeadBand = RUN_MOTOR_SPEED_DEADBAND;
  cfg.controller_param_init_config.speed_PID.IntegralLimit = RUN_MOTOR_SPEED_ILIMIT;
  cfg.controller_param_init_config.speed_PID.Improve =
      PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle;

  return cfg;
}

void RobotInit(void) {
  /* 1) 注册电机到 motor module */
  Motor_Init_Config_s cfg = MakeCfg();
  s_motor = DJIMotorInit(&cfg);
  if (s_motor == NULL) {
    LOGERROR("[run_motor1] DJIMotorInit FAIL (id=%d)", RUN_MOTOR_TX_ID);
  } else {
    /* 上电先停, 等首次反馈取零后再由 RobotTask 使能 */
    DJIMotorStop(s_motor);
    LOGINFO("[run_motor1] motor id=%d ready, waiting CAN feedback...", RUN_MOTOR_TX_ID);
  }

  /* 2) 注册话题(遵循 application.md: app 间只用 pub-sub, 不相互包含) */
  s_pub_fb = PubRegister(TOPIC_RUN_MOTOR_FB, sizeof(RunMotor_Fb_s));
  s_sub_cmd = SubRegister(TOPIC_RUN_MOTOR_CMD, sizeof(RunMotor_Cmd_s));

  (void)DWT_GetDeltaT(&s_robot.DWT_CNT); /* 初始化 dt 基准 */
  LOGINFO("[run_motor1] init done (target=%.1f deg/s)", RUN_MOTOR_TARGET_DPS);
}

/* ============================== 周期任务 ============================== */
void RobotTask(void) {
  uint32_t now;
  RunMotor_Cmd_s cmd;

  s_robot.dt = DWT_GetDeltaT(&s_robot.DWT_CNT);
  now = (uint32_t)DWT_GetTimeline_ms();

  if (s_motor == NULL) return;

  /* 1) 在线检测: feed_cnt 在变说明持续收到 CAN 反馈 */
  if (s_motor->feed_cnt != s_last_feed) {
    s_last_feed = s_motor->feed_cnt;
    s_online = 1;
  }

  /* 2) 首次收到反馈 -> 取零, 之后才允许闭环 */
  if (s_motor->feed_cnt > 0 && !s_zero_valid) {
    s_zero = s_motor->measure.total_angle;
    s_zero_valid = 1;
    s_pid_primed = 0; /* 取零后重新清一次 PID 运行时状态 */
    LOGINFO("[run_motor1] zero latched (total_angle=%.1f)", s_zero);
  }

  /* 3) 接收上层指令(可选): 没有 cmd app 时保持 cfg 默认自动旋转 */
  while (SubGetMessage(s_sub_cmd, &cmd)) {
    s_enable = cmd.enable ? 1u : 0u;
    if (s_enable) s_target_dps = cmd.target_dps;
  }

  /* 4) 生成角度目标并下发(角度环) */
  if (s_zero_valid && s_enable) {
    if (RUN_MOTOR_MODE == 1) {
      s_target_deg = RUN_MOTOR_TARGET_DEG; /* 模式1: 转到给定角度并保持 */
    } else {
      float limit_deg = RUN_MOTOR_MAX_ROUNDS * 360.0f;
      s_target_deg += s_target_dps * s_robot.dt; /* 模式0: 连续递增 = 给定角度序列 */
      if (limit_deg > 0.0f) {
        if (s_target_deg > limit_deg) s_target_deg = limit_deg;
        if (s_target_deg < -limit_deg) s_target_deg = -limit_deg;
      }
    }

    /* 首次下发前清一次 PID 运行时状态: controller 在 dt=0 时会算出 0/0=NaN,
     * 且 NaN 会粘在积分项里, 导致输出恒 0、电机不转(实测踩坑)。 */
    if (!s_pid_primed) {
      PIDClear(&s_motor->motor_controller.current_PID);
      PIDClear(&s_motor->motor_controller.speed_PID);
      PIDClear(&s_motor->motor_controller.angle_PID);
      s_pid_primed = 1;
    }

    DJIMotorEnable(s_motor);
    DJIMotorOuterLoop(s_motor, ANGLE_LOOP); /* 每次下发都设, 防止被改 */
    DJIMotorSetPIDRef(s_motor, RotorRef(s_target_deg));

    /* 自愈: 若输出仍非有限, 清一次让下一拍重算(避免 NaN 永久卡死) */
    if (!isfinite(s_motor->motor_controller.final_output)) {
      PIDClear(&s_motor->motor_controller.current_PID);
      PIDClear(&s_motor->motor_controller.speed_PID);
      PIDClear(&s_motor->motor_controller.angle_PID);
      s_pid_primed = 0;
    }
  } else {
    DJIMotorStop(s_motor); /* 未取零/未使能: 直接停止输出(安全) */
  }

  /* 5) 发布反馈 */
  s_fb.angle_deg = OutDeg();
  s_fb.speed_dps = OutDps();
  s_fb.target_deg = s_target_deg;
  s_fb.current = s_motor->measure.real_current;
  s_fb.online = s_online;
  s_fb.enabled = (s_motor->stop_flag == MOTOR_ENALBED) ? 1u : 0u;
  s_fb.tick = now;
  PubPushMessage(s_pub_fb, &s_fb);
}
