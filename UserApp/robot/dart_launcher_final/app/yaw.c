/*
 * yaw.c — 自瞄 yaw 轴 app 实现
 * =============================================================================
 * 数据流: 订阅 "aim_cmd"(视觉误差) + "yaw_cmd"(模式/上限/急停)
 *         -> 按模式算目标转速:
 *              自瞄(VISION): 应用层 PI(像素误差 -> rpm), 带死区/最小转速
 *              手动(MANUAL): 由上位机直接给目标(角度环)
 *              引导(GUIDE) : 固定角度(角度环)
 *         -> 速度环/角度环驱动 M2006
 *         -> 发布 "yaw_fb"
 *
 * 【单位换算】DJIMotor 的 speed_aps 是转子侧 deg/s, 故:
 *   转子 ref(deg/s) = sign * 输出rpm * ratio * 6
 *   输出 rpm        = speed_aps / 6 / ratio
 * =============================================================================
 */
#include "yaw.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "can.h" /* hcan1 */
#include "dji_motor.h"
#include "launcher_cfg.h"
#include "message_center.h"
#include "motor_def.h"

static DJIMotorInstance *s_motor = NULL;
static float s_zero = 0.0f;
static uint8_t s_zero_valid = 0;
static uint32_t s_last_feed = 0;

static Subscriber_t *s_sub_aim = NULL;
static Subscriber_t *s_sub_cmd = NULL;
static Publisher_t *s_pub_fb = NULL;

/* 指令镜像 */
static uint8_t s_mode = LAUNCH_YAW_MANUAL;
static float s_manual_deg = 0.0f;
static float s_guide_deg = (float)LAUNCH_YAW_GUIDE_DEG;
static float s_aim_rpm = LAUNCH_YAW_AIM_RPM;
static uint8_t s_estop = 0;

/* 视觉 */
static int16_t s_err = 0;
static uint8_t s_vis_ok = 0;

/* 应用层 PI 积分 */
static float s_aim_i = 0.0f;

static Launcher_AppStatus_s s_st;
static Launcher_YawFb_s s_fb;

static float SignOf(void) {
  return (s_motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
}

static float OutDeg(void) {
  if (!s_zero_valid) return 0.0f;
  return SignOf() * (s_motor->measure.total_angle - s_zero) / LAUNCH_YAW_RATIO;
}

static float OutRpm(void) {
  return s_motor->measure.speed_aps / 6.0f / LAUNCH_YAW_RATIO;
}

static float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void Yaw_Init(void) {
  Motor_Init_Config_s cfg;
  memset(&cfg, 0, sizeof(cfg));

  cfg.motor_type = M2006;
  cfg.can_init_config.can_handle = &hcan1;
  cfg.can_init_config.tx_id = LAUNCH_YAW_ID;

  cfg.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  cfg.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  cfg.controller_setting_init_config.outer_loop_type = SPEED_LOOP;
  cfg.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
  cfg.controller_setting_init_config.motor_reverse_flag = LAUNCH_YAW_REVERSE;
  cfg.controller_setting_init_config.feedback_reverse_flag =
      (LAUNCH_YAW_REVERSE == MOTOR_DIRECTION_REVERSE) ? FEEDBACK_DIRECTION_REVERSE
                                                      : FEEDBACK_DIRECTION_NORMAL;

  cfg.controller_param_init_config.speed_PID.Kp = LAUNCH_YAW_SPEED_KP;
  cfg.controller_param_init_config.speed_PID.Ki = LAUNCH_YAW_SPEED_KI;
  cfg.controller_param_init_config.speed_PID.Kd = LAUNCH_YAW_SPEED_KD;
  cfg.controller_param_init_config.speed_PID.IntegralLimit = LAUNCH_YAW_SPEED_ILIMIT;
  cfg.controller_param_init_config.speed_PID.MaxOut = LAUNCH_YAW_SPEED_MAXOUT;
  cfg.controller_param_init_config.speed_PID.DeadBand = LAUNCH_YAW_SPEED_DEADBAND;
  cfg.controller_param_init_config.speed_PID.Improve =
      PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle;

  cfg.controller_param_init_config.angle_PID.Kp = LAUNCH_YAW_ANGLE_KP;
  cfg.controller_param_init_config.angle_PID.Ki = LAUNCH_YAW_ANGLE_KI;
  cfg.controller_param_init_config.angle_PID.Kd = LAUNCH_YAW_ANGLE_KD;
  cfg.controller_param_init_config.angle_PID.MaxOut = LAUNCH_YAW_ANGLE_MAXOUT;
  cfg.controller_param_init_config.angle_PID.DeadBand = LAUNCH_YAW_ANGLE_DEADBAND;
  cfg.controller_param_init_config.angle_PID.Improve =
      PID_Integral_Limit | PID_Derivative_On_Measurement;

  s_motor = DJIMotorInit(&cfg);
  if (s_motor == NULL) {
    LOGERROR("[yaw] DJIMotorInit FAIL");
  } else {
    DJIMotorStop(s_motor); /* 上电先停, 等指令 */
    LOGINFO("[yaw] M2006 id=%d ready", LAUNCH_YAW_ID);
  }

  s_mode = LAUNCH_YAW_MANUAL;
  s_aim_rpm = LAUNCH_YAW_AIM_RPM;
  s_aim_i = 0.0f;
  memset(&s_fb, 0, sizeof(s_fb));
  memset(&s_st, 0, sizeof(s_st));

  s_sub_aim = SubRegister(TOPIC_AIM_CMD, sizeof(Launcher_Aim_s));
  s_sub_cmd = SubRegister(TOPIC_YAW_CMD, sizeof(Launcher_YawCmd_s));
  s_pub_fb = PubRegister(TOPIC_YAW_FB, sizeof(Launcher_YawFb_s));
}

/** @brief 自瞄: 应用层 PI 把像素误差算成目标输出转速(rpm) */
static float AimStep(float dt) {
  if (!s_vis_ok) {
    s_aim_i = 0.0f;
    return 0.0f; /* 丢目标: 停转(不动比乱动安全) */
  }

  float err = (float)s_err;
  if (fabsf(err) < (float)LAUNCH_YAW_AIM_DEADBAND) {
    s_aim_i = 0.0f;
    return 0.0f;
  }

  s_aim_i += err * dt;
  s_aim_i = Clamp(s_aim_i, -LAUNCH_YAW_AIM_I_LIMIT, LAUNCH_YAW_AIM_I_LIMIT);

  float spd = LAUNCH_YAW_AIM_KP * err + LAUNCH_YAW_AIM_KI * s_aim_i;
  spd = Clamp(spd, -s_aim_rpm, s_aim_rpm);

  /* 最小转速: 克服静摩擦, 防止末端蠕动(误差存在时至少给一点速度) */
  if (fabsf(spd) < LAUNCH_YAW_AIM_MIN_RPM) {
    spd = (err > 0.0f) ? LAUNCH_YAW_AIM_MIN_RPM : -LAUNCH_YAW_AIM_MIN_RPM;
  }
  return spd;
}

void Yaw_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  static uint32_t last_ms = 0;
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  float dt = (now - last_ms) * 0.001f;
  if (last_ms == 0 || dt <= 0.0f || dt > 0.5f) dt = 0.001f;
  last_ms = now;

  uint16_t err_bits = LAUNCH_ERR_NONE;
  Launcher_Aim_s aim;
  Launcher_YawCmd_s yc;

  if (s_motor == NULL) {
    s_st.hb++;
    s_st.err = LAUNCH_ERR_YAW_OFF;
    return;
  }

  /* ---- 输入 ---- */
  if (SubGetMessage(s_sub_aim, &aim)) {
    s_err = aim.err;
    s_vis_ok = aim.ok;
  }
  if (SubGetMessage(s_sub_cmd, &yc)) {
    s_mode = yc.mode;
    s_manual_deg = (float)yc.manual_deg;
    s_guide_deg = (float)yc.guide_deg;
    s_aim_rpm = yc.aim_rpm;
    s_estop = yc.estop;
  }

  /* ---- 在线判定 + 首次取零 ---- */
  if (s_motor->feed_cnt != s_last_feed) {
    s_last_feed = s_motor->feed_cnt;
    if (!s_zero_valid) {
      s_zero = s_motor->measure.total_angle;
      s_zero_valid = 1;
    }
  }

  /* ---- 控制 ---- */
  if (s_estop) {
    /* 急停: 无储能负载 -> 直接卸力 */
    DJIMotorStop(s_motor);
    s_aim_i = 0.0f;
  } else {
    DJIMotorEnable(s_motor);
    switch (s_mode) {
      case LAUNCH_YAW_VISION: {
        float rpm = AimStep(dt);
        DJIMotorOuterLoop(s_motor, SPEED_LOOP);
        DJIMotorSetPIDRef(s_motor, SignOf() * rpm * LAUNCH_YAW_RATIO * 6.0f);
        break;
      }
      case LAUNCH_YAW_GUIDE:
        DJIMotorOuterLoop(s_motor, ANGLE_LOOP);
        DJIMotorSetPIDRef(s_motor,
                          s_zero + SignOf() * s_guide_deg * LAUNCH_YAW_RATIO);
        s_aim_i = 0.0f;
        break;
      case LAUNCH_YAW_MANUAL:
      default:
        DJIMotorOuterLoop(s_motor, ANGLE_LOOP);
        DJIMotorSetPIDRef(s_motor,
                          s_zero + SignOf() * s_manual_deg * LAUNCH_YAW_RATIO);
        s_aim_i = 0.0f;
        break;
    }
  }

  /* ---- 反馈 ---- */
  s_fb.cur_rpm = OutRpm();
  s_fb.aim_rpm = s_aim_rpm;
  s_fb.err = s_err;
  s_fb.angle_deg = (int16_t)OutDeg();
  s_fb.mode = s_mode;
  s_fb.vis_ok = s_vis_ok;
  s_fb.tick = now;
  PubPushMessage(s_pub_fb, &s_fb);

  /* 仅在"自瞄模式且视觉丢失"时才算故障; 手动/引导模式无视觉是正常的 */
  if (s_mode == LAUNCH_YAW_VISION && !s_vis_ok) err_bits |= LAUNCH_ERR_VISION_OFF;

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = s_fb.cur_rpm; /* 关键量: yaw 输出转速 */
  s_st.err = err_bits;
}

const Launcher_AppStatus_s *Yaw_GetStatus(void) { return &s_st; }
