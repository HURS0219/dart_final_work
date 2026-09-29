/*
 * imu.c — 姿态 app 实现
 * =============================================================================
 * 依赖: Modules/imu (ins_task) —— 由 INS_Init() 完成 BMI088 初始化与姿态解算,
 *       并在内部自行创建一个 1kHz 的 INS 任务, 因此本 app 不再调用 INS_Task()。
 * 行为: 每周期调用 INS_GetAttitude() 取欧拉角与角速度, 打包成 Dart_Attitude_s 发布。
 * =============================================================================
 */
#include "imu.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dart_final_cfg.h"
#include "ins_task.h"
#include "message_center.h"
#include "robot_def.h"

#define RAD2DEG (57.29577951f) /* rad/s -> deg/s */

static INS_t *s_ins = NULL;
static Publisher_t *s_pub = NULL;
static Dart_Attitude_s s_att;
static Dart_AppStatus_s s_st;

void Imu_Init(void) {
#if DART_IMU_ENABLE
  IMU_Init_Config_s cfg;

  /* IMU 安装/标定参数: 默认全 0, scale 置 1(1:1), 在线标定零偏(offset_flag=0)。
   * 现场如需修正安装偏角/刻度, 改这里或从 cfg 宏传入。 */
  memset(&cfg, 0, sizeof(cfg));
  cfg.scale[X] = 1.0f;
  cfg.scale[Y] = 1.0f;
  cfg.scale[Z] = 1.0f;
  cfg.offset_flag = 0; /* 0=上电在线标定陀螺零偏 */

  s_ins = INS_Init(&cfg); /* 阻塞约 1s, 内部创建 INS 任务 */
#endif

  memset(&s_att, 0, sizeof(s_att));
  memset(&s_st, 0, sizeof(s_st));
  s_pub = PubRegister(TOPIC_ATTITUDE, sizeof(Dart_Attitude_s));

#if DART_IMU_ENABLE
  LOGINFO("[imu] INS init %s", (s_ins != NULL) ? "OK" : "FAIL");
#else
  LOGINFO("[imu] DART_IMU_ENABLE=0: skip INS (board has no BMI088)");
#endif
}

void Imu_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();

#if DART_IMU_ENABLE
  {
    attitude_t a;
    if (INS_GetAttitude(&a)) {
      /* 角速度轴向映射与 dart_fc 一致: Gyro[0]-Pitch, Gyro[1]-Roll, Gyro[2]-Yaw (rad/s) */
      s_att.roll_deg = a.Roll;
      s_att.pitch_deg = a.Pitch;
      s_att.yaw_deg = a.Yaw;
      s_att.gx_dps = a.Gyro[1] * RAD2DEG; /* Roll 轴 */
      s_att.gy_dps = a.Gyro[0] * RAD2DEG; /* Pitch 轴 */
      s_att.gz_dps = a.Gyro[2] * RAD2DEG; /* Yaw 轴 */
      s_att.valid = 1;
      s_st.err = DART_ERR_NONE;
    } else {
      s_att.valid = 0;
      s_st.err = DART_ERR_IMU_OFF;
    }
  }
#else
  s_att.valid = 0; /* 无 IMU: 姿态恒无效 */
  s_st.err = DART_ERR_NONE;
#endif

  s_att.tick = (uint32_t)DWT_GetTimeline_ms();
  PubPushMessage(s_pub, &s_att);

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = s_att.roll_deg; /* 关键量: roll 角, 便于 RTT 观察 */
}

const Dart_AppStatus_s *Imu_GetStatus(void) { return &s_st; }

void Imu_GetAttitude(Dart_Attitude_s *out) {
  if (out != NULL) *out = s_att;
}
