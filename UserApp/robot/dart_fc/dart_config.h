/*
 * dart_config.h  —  从 dart_fc/Config/config.h 移植 (保持 HAL 无关, 纯 C)。
 * 全部可调/需实测参数集中于此。
 */
#ifndef DART_CONFIG_H
#define DART_CONFIG_H

/* ===================== 模式 / 灯光 ===================== */
#define CFG_MODE_DEFAULT        0       /* 上电默认: 0=比赛 1=解耦手动 2=全手动 */

/* ===================== 舵机 ===================== */
#define CFG_SERVO_COUNT         4
#define CFG_SERVO_FREQ_HZ       50u
#define CFG_SERVO_PERIOD_US     20000u
#define CFG_SERVO_PULSE_MIN_US  500u    /* 0 deg */
#define CFG_SERVO_PULSE_MAX_US  2500u   /* 180 deg */
#define CFG_SERVO_MAX_ANGLE     35.0f   /* 舵面机械限幅 (deg) */
#define CFG_SERVO_TRIM_DEG      0.0f    /* 中立微调 */
#define CFG_SERVO_RATE_LIMIT    600.0f  /* 舵面速率限幅 (deg/s) */

/* ===================== X 型四舵面解耦矩阵 ===================== */
#define CFG_MIX_PITCH_1   (+1.0f)
#define CFG_MIX_PITCH_2   (-1.0f)
#define CFG_MIX_PITCH_3   (-1.0f)
#define CFG_MIX_PITCH_4   (+1.0f)
#define CFG_MIX_YAW_1     (+1.0f)
#define CFG_MIX_YAW_2     (+1.0f)
#define CFG_MIX_YAW_3     (-1.0f)
#define CFG_MIX_YAW_4     (-1.0f)
#define CFG_MIX_ROLL_1    (+1.0f)
#define CFG_MIX_ROLL_2    (-1.0f)
#define CFG_MIX_ROLL_3    (+1.0f)
#define CFG_MIX_ROLL_4    (-1.0f)

/* ===================== 比例导引 (比赛模式) ===================== */
#define CFG_NAV_RATIO_N         4.0f
#define CFG_V_CLOSE             38.0f
#define CFG_PNG_MAX_G           3.0f
#define CFG_PNG_SIGN            1.0f

/* ===================== 视线角速率滤波 ===================== */
#define CFG_LOS_FILTER_ALPHA    0.30f
#define CFG_FOCAL_LENGTH_PX     120.0f
#define CFG_IMAGE_CX            160.0f
#define CFG_IMAGE_CY            120.0f

/* ===================== 图像解旋 (滚转补偿) ===================== */
#define CFG_DEROT_ENABLE        1
#define CFG_DEROT_SIGN         (-1.0f)

/* ===================== 姿态互补滤波 ===================== */
#define CFG_ATT_ALPHA           0.98f
#define CFG_GYRO_LPF_ALPHA      0.50f
#define CFG_ACC_LPF_ALPHA       0.50f

/* ===================== 解耦手动: 滚转/俯仰/偏航 PID ===================== */
#define CFG_ROLL_KP             2.5f
#define CFG_ROLL_KI             0.0f
#define CFG_ROLL_KD             0.35f
#define CFG_PITCH_KP            1.2f
#define CFG_PITCH_KI            0.0f
#define CFG_PITCH_KD            0.30f
#define CFG_YAW_KP              1.2f
#define CFG_YAW_KI              0.0f
#define CFG_YAW_KD              0.30f
#define CFG_PID_I_LIMIT         0.30f
#define CFG_STICK_DEADBAND      30
#define CFG_STICK_EXPO          0.30f

#define CFG_MANUAL_MAX_ROLL_DEG   45.0f
#define CFG_MANUAL_MAX_PITCH_DEG  30.0f
#define CFG_MANUAL_MAX_YAW_RATE   60.0f
#define CFG_MANUAL_CMD_LIMIT      1.0f

/* ===================== 全手动调试模式 ===================== */
#define CFG_FM_GAIN_DEG         30.0f

/* ===================== 失效保护 ===================== */
#define CFG_FAILSAFE_MS         500u
#define CFG_HEARTBEAT_MS        100u
#define CFG_VISION_TIMEOUT_MS   200u

/* ===================== 控制周期 ===================== */
#define CFG_CTRL_PERIOD_MS      10u     /* 100Hz 控制任务 */
#define CFG_IMU_PERIOD_MS       2u      /* 500Hz IMU 任务 */

/* ================= 串口 (ESP32 网页调参, USART6 115200) ================= */
#define DART_UART_BAUD          115200u
#define DART_RECV_SIZE          128
#define DART_CMD_TIMEOUT_MS     2000u
#define DART_FB_PERIOD_MS       150u

#endif /* DART_CONFIG_H */
