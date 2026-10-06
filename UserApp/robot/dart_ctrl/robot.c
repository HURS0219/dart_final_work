/*
 * robot.c — dart_ctrl · 制导飞镖整机飞控 (单文件, 组合根直连, 无 PubSub)
 * =============================================================================
 * 由 dart_final 改写: 去掉 message_center 发布订阅; imu/vision/guidance/fin 的
 * 逻辑全部收进本文件, 由 RobotTask 顺序调用, 数据经 robot-> 直连。
 *
 * 数据流:  ImuStep(姿态) ┐
 *                          ├─> GuidStep ─> robot->mix ─> FinStep ─> 4 舵机 PWM
 *         VisionStep(目标)┘
 *
 * 依赖模块(不改框架): ins_task / controller(PID) / servo_motor(PWM) / bsp_usart
 *                     / crc8 / bsp_dwt / bsp_log / user_lib
 *
 * 【待配置接口】(赛事规则出来后只改这里): Dart_IsEnabled() / Dart_ShouldGuide()
 * =============================================================================
 */
#include "robot.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "controller.h"
#include "crc8.h"
#include "ins_task.h"
#include "robot_config.h"
#include "servo_motor.h"
#include "tim.h"
#include "usart.h"
#include "user_lib.h"

RobotInstance* robot = NULL;

/* ============================== 子模块运行态 ============================== */
static INS_t* s_ins = NULL;              /* 姿态(INS)实例 */
static USARTInstance* s_vis = NULL;      /* 视觉串口实例 */
static ServoInstance* s_servo[DART_SERVO_N] = {NULL};
static PIDInstance s_roll_pid;           /* roll 稳定 PID */

/* 视觉(中断里填) */
static volatile uint8_t s_new_frame = 0;
static Dart_Target_s s_rx;
static uint32_t s_last_frame_ms = 0;

/* 制导 */
static uint8_t s_los_started = 0;
static float s_los_prev = 0.0f;
static float s_dlambda = 0.0f;

/* 舵机(逐路现行逻辑角, 用于速率限幅) */
static float s_deg[DART_SERVO_N] = {0};

/* 逐路配置(由 robot_config.h 展开) */
static const uint32_t kCh[DART_SERVO_N] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
                                           TIM_CHANNEL_4};
static const float kScale[DART_SERVO_N] = DART_SERVO_SCALE;
static const float kTrim[DART_SERVO_N] = DART_SERVO_TRIM;
static const uint8_t kRev[DART_SERVO_N] = DART_SERVO_REVERSE;
static const float kRate[DART_SERVO_N] = DART_SERVO_RATE;
static const float kMix[DART_SERVO_N][3] = DART_MIX_MATRIX;

/* 状态机 / 故障 */
static DartState_e s_state = DART_IDLE;
static uint16_t s_fault_bits = 0;
/* 致命故障 = 姿态无效 或 舵机注册失败(vision 掉线非致命, 由 failsafe 回中即可) */
#define DART_FAULT_MASK (DART_ERR_IMU_OFF | DART_ERR_SERVO)

/* =============================== 工具函数 =============================== */
static float Clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* ======================= 【待配置接口】(只改这里) ======================= */
/** 【接口1】使能判定: 现在=按 cfg(默认上电即使能); 以后接入真实“使能”信号 */
static bool Dart_IsEnabled(void) { return DART_ENABLE_ON_BOOT ? true : true; }

/** 【接口2】制导时机: 现在=识别到绿光 且 姿态俯冲(Pitch<DIVE_PITCH_DEG) */
static bool Dart_ShouldGuide(void) {
  return (robot->target.found && robot->attitude.valid &&
          (robot->attitude.pitch_deg < DIVE_PITCH_DEG));
}

/* =============================== 视觉 =============================== */
/* OpenMV 7 字节帧: AA 55 X_hi X_lo Y_hi Y_lo CRC8; 中断里只解析+置标志 */
static void Vision_RxCallback(void) {
  if (s_vis == NULL) return;
  uint8_t* b = s_vis->recv_buff;
  if (b[0] != OPENMV_HEAD1 || b[1] != OPENMV_HEAD2) return;
  if (crc_8(b, 6) != b[6]) return;
  s_rx.x = (int16_t)((b[2] << 8) | b[3]);
  s_rx.y = (int16_t)((b[4] << 8) | b[5]);
  s_rx.found = !(s_rx.x == 0 && s_rx.y == 0); /* OpenMV 丢目标时全 0 */
  s_new_frame = 1;
}

/* =============================== 制导律(PNG) =============================== */
/* PPN: a_cmd = N * v * d_lambda; 可选 APN 增广项; 死区 + 输出限幅 */
static float PngCalc(float dlambda, float v_c) {
  float a = PNG_N * v_c * dlambda;
#if PNG_MODE_APN
  a += PNG_AO_GAIN * PNG_N * PNG_AO;
#endif
  if (fabsf(a) < PNG_DEADBAND) return 0.0f;
  if (a > PNG_MAX_OUT) a = PNG_MAX_OUT;
  if (a < -PNG_MAX_OUT) a = -PNG_MAX_OUT;
  return a;
}

/* =============================== 各步骤 =============================== */
/* 1) IMU: 读 INS 实例(ins_task 自带 1kHz 任务, 更新 INS_t) */
static void ImuStep(void) {
  Dart_Attitude_s* a = &robot->attitude;
  if (s_ins != NULL && s_ins->init) {
    a->roll_deg = s_ins->Roll;
    a->pitch_deg = s_ins->Pitch;
    a->yaw_deg = s_ins->Yaw;
    a->gx_dps = s_ins->Gyro[1]; /* Roll 轴 */
    a->gy_dps = s_ins->Gyro[0]; /* Pitch 轴 */
    a->gz_dps = s_ins->Gyro[2]; /* Yaw 轴 */
    a->valid = 1;
  } else {
    a->valid = 0;
  }
  a->tick = (uint32_t)DWT_GetTimeline_ms();
}

/* 2) 视觉: 把中断里解好的整帧发布到 robot->target; 超时置 found=0 */
static void VisionStep(void) {
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  Dart_Target_s* t = &robot->target;
  if (s_new_frame) {
    s_new_frame = 0;
    t->x = s_rx.x;
    t->y = s_rx.y;
    t->found = s_rx.found;
    t->tick = now;
    s_last_frame_ms = now;
  } else if (now - s_last_frame_ms > VISION_TIMEOUT_MS) {
    t->found = 0;
    t->tick = now;
  }
}

/* 3) 监控: 汇总健康/错误位 */
static void MonitorStep(void) {
  uint16_t bits = DART_ERR_NONE;
  if (!robot->attitude.valid) bits |= DART_ERR_IMU_OFF;
  if ((uint32_t)DWT_GetTimeline_ms() - s_last_frame_ms > VISION_TIMEOUT_MS)
    bits |= DART_ERR_VISION_OFF;
  for (int i = 0; i < DART_SERVO_N; i++)
    if (s_servo[i] == NULL) bits |= DART_ERR_SERVO;
  s_fault_bits = bits;
}

/* 4) 状态机: IDLE/ARMED/GUIDING/FAULT */
static void StateStep(void) {
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

/* 5) 制导: 目标像素 -> 视线角 -> dλ(微分+低通) -> png -> mix.yaw; roll=PID; pitch=0 */
static void GuidStep(float dt, uint8_t guide_enable) {
  Dart_Attitude_s a = robot->attitude;
  Dart_Target_s t = robot->target;
  float lambda = 0.0f, d_raw = 0.0f, a_cmd, yaw, roll;
  uint8_t vision_ok = t.found ? 1u : 0u;
  uint8_t failsafe;

  if (vision_ok) {
    lambda = ((float)t.x - GUID_IMAGE_CX) / GUID_FOCAL_PX;
    if (s_los_started && dt > 0.0f) d_raw = (lambda - s_los_prev) / dt;
    s_los_prev = lambda;
    s_los_started = 1;
  } else {
    s_los_started = 0; /* 丢目标后重置微分基准 */
  }
  s_dlambda += GUID_LOS_FILTER_ALPHA * (d_raw - s_dlambda);

  a_cmd = PngCalc(s_dlambda, GUID_DLC_V_C);
  yaw = GUID_GAIN_K * a_cmd;
  roll = PIDCalculate(&s_roll_pid, a.roll_deg, 0.0f);

  failsafe = (!guide_enable || !vision_ok || !a.valid) ? 1u : 0u;
  if (failsafe) {
    yaw = 0.0f;
    roll = 0.0f;
  }

  robot->mix.pitch = 0.0f;
  robot->mix.yaw = Clampf(yaw, -1.0f, 1.0f);
  robot->mix.roll = Clampf(roll, -1.0f, 1.0f);
  robot->mix.failsafe = failsafe;
  robot->mix.tick = (uint32_t)DWT_GetTimeline_ms();
}

/* 逻辑角 -> 占空比(干净库 servo_motor 的 PWM 接口直接吃占空比 0..1) */
static float AngleToDuty(float logic_deg, int ch) {
  float mech = logic_deg * kScale[ch] + kTrim[ch];
  if (kRev[ch]) mech = -mech;
  float pulse = SERVO_CENTER_US + mech * (SERVO_HALF_US / SERVO_HALF_DEG);
  if (pulse < SERVO_PULSE_MIN_US) pulse = SERVO_PULSE_MIN_US;
  if (pulse > SERVO_PULSE_MAX_US) pulse = SERVO_PULSE_MAX_US;
  return pulse / (DART_SERVO_PERIOD_S * 1000000.0f);
}

/* 6) 舵面: 混控矩阵 -> 4 路目标角(限幅) -> 速率限幅 -> 角度->占空比 -> 驱动 */
static void FinStep(float dt) {
  Dart_Mix_s m = robot->mix;
  for (int i = 0; i < DART_SERVO_N; i++) {
    float target;
    if (m.failsafe) {
      target = 0.0f; /* 失效/未制导 -> 回中 */
    } else {
      float u = kMix[i][0] * m.pitch + kMix[i][1] * m.yaw + kMix[i][2] * m.roll;
      target = Clampf(u * DART_MIX_MAX_DEG, -DART_MIX_MAX_DEG, DART_MIX_MAX_DEG);
    }

    /* 速率限幅(逐路) */
    if (kRate[i] <= 0.0f) {
      s_deg[i] = target;
    } else {
      float step = kRate[i] * dt;
      float diff = target - s_deg[i];
      if (diff > step)
        s_deg[i] += step;
      else if (diff < -step)
        s_deg[i] -= step;
      else
        s_deg[i] = target;
    }

    float duty = AngleToDuty(s_deg[i], i);
    if (s_servo[i] != NULL) ServoSetAngle(s_servo[i], duty);

    robot->servo_fb.defl_deg[i] = s_deg[i];
    robot->servo_fb.pulse_us[i] = duty * DART_SERVO_PERIOD_S * 1000000.0f;
  }
  robot->servo_fb.tick = (uint32_t)DWT_GetTimeline_ms();
}

/* 7) 遥测: 每秒 RTT 打印状态机 + 故障位 + 主要量 */
static void TelemetryStep(float dt_ms) {
  static float acc = 0.0f;
  acc += dt_ms;
  if (acc < 1000.0f) return;
  acc = 0.0f;
  LOGINFO("[dart] state=%d fault=0x%04X | roll=%.1f pitch=%.1f yaw=%.1f | mix(y=%.2f r=%.2f)",
          (int)s_state, s_fault_bits, robot->attitude.roll_deg, robot->attitude.pitch_deg,
          robot->attitude.yaw_deg, robot->mix.yaw, robot->mix.roll);
}

/* ============================== 生命周期 ============================== */
void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  memset(robot, 0, sizeof(RobotInstance));

  /* 1) IMU(ins_task 内部自建 1kHz INS 任务) */
#if DART_IMU_ENABLE
  {
    IMU_Init_Config_s cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.scale[X] = 1.0f;
    cfg.scale[Y] = 1.0f;
    cfg.scale[Z] = 1.0f;
    s_ins = INS_Init(&cfg);
    LOGINFO("[dart_ctrl] INS init %s", (s_ins != NULL) ? "OK" : "FAIL");
  }
#else
  s_ins = NULL;
  LOGINFO("[dart_ctrl] DART_IMU_ENABLE=0: skip INS");
#endif

  /* 2) 视觉串口(回调在中断里解析帧) */
  {
    USART_Init_Config_s cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.usart_handle = DART_USART_OPENMV;
    cfg.recv_buff_size = OPENMV_RECV_SIZE;
    cfg.module_callback = Vision_RxCallback;
    s_vis = USARTRegister(&cfg);
    LOGINFO("[dart_ctrl] vision usart %s", (s_vis != NULL) ? "OK" : "FAIL");
  }

  /* 3) 4 路 PWM 舵机(干净库 servo_motor, PWM 模式; 上电输出中位) */
  for (int i = 0; i < DART_SERVO_N; i++) {
    Servo_Init_Config_s cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.servo_type = PWM_Servo;
    cfg.servo_id = (uint8_t)i;
    cfg.pwm_init_config.htim = DART_SERVO_TIM;
    cfg.pwm_init_config.channel = kCh[i];
    cfg.pwm_init_config.period = DART_SERVO_PERIOD_S;
    cfg.pwm_init_config.dutyratio = 0.075f; /* 1.5ms @20ms 中位(占位, 随后由 FinStep 覆盖) */
    s_servo[i] = ServoInit(&cfg);
  }

  /* 4) roll PID */
  {
    PID_Init_Config_s rc;
    memset(&rc, 0, sizeof(rc));
    rc.Kp = ROLL_KP;
    rc.Ki = ROLL_KI;
    rc.Kd = ROLL_KD;
    rc.MaxOut = ROLL_CMD_LIMIT;
    rc.Improve = PID_Integral_Limit | PID_Derivative_On_Measurement;
    PIDInit(&s_roll_pid, &rc);
  }

  memset(&s_rx, 0, sizeof(s_rx));
  s_state = DART_IDLE;
  s_fault_bits = DART_ERR_NONE;

  (void)DWT_GetDeltaT(&robot->DWT_CNT); /* 初始化 dt 基准 */
  LOGINFO("[dart_ctrl] init done");
}

void RobotTask(void) {
  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  float dt_ms = robot->dt * 1000.0f;

  ImuStep();                              /* 输入: 姿态 */
  VisionStep();                           /* 输入: 目标 */
  MonitorStep();                          /* 健康汇总 */
  StateStep();                            /* 状态机 */
  GuidStep(robot->dt, (s_state == DART_GUIDING) ? 1u : 0u); /* 制导 -> mix */
  FinStep(robot->dt);                     /* 混控 -> 4 舵机 -> servo_fb */
  TelemetryStep(dt_ms);                   /* 每秒 RTT */
}
