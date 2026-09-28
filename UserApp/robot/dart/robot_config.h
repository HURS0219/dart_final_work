//
// Created by PC on 2025/11/18.
//

#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

/* ================= 硬件 ID 配置(须修改） ================= */
#define YAW_MOTOR_ID 2  // 6020 Yaw轴
#define PUSH_VERT_ID 7  // 2006 垂直推杆
#define PUSH_HORI_ID 6  // 2006 水平推杆

// 摩擦轮 (3508 x4)
#define FRIC_MOTOR_LU 1  // 左上Left Upward
#define FRIC_MOTOR_LD 2  // 左下Left Downward
#define FRIC_MOTOR_RU 3  // 右上Right Upward
#define FRIC_MOTOR_RD 4  // 右下Right Downward

/* ================= 校准参数 ================= */
#define CALI_SPEED           8000.0f         // 校准时的归位速度 (RPM/电流单位)
#define CALI_BACK_OFFSET     10000.0f        // 竖直回退距离
#define HORI_BACK_OFFSET     200.0f          // 水平回退距离
#define CALI_POS_THRESHOLD   200.0f          // 回退到位判断阈值
#define CALI_SPEED_THRESHOLD 10.0f           // 速度判断阈值

/* ================= 摩擦轮参数 ================= */
#define FRIC_IDLE_SPEED      1000.0f         // 待机速度
#define FRIC_BASE_MIN_SPEED  9000.0f         // (auto)摩擦轮最低运行速度 (防止速度归零)
#define FRIC_BASE_MAX_SPEED  18000.0f        // (auto)摩擦轮最高运行速度
#define FRIC_INCREMENT_SENSITIVITY 500.0f    // 摇杆推到底，每秒增加 500 RPM 的灵敏度系数
#define DART_FRICTION_MAX_SPEED 16000.0f     // (debug))摩擦轮最大速度

/* ================= 推杆参数 ================= */
#define DART_PUSHROD_MAX_SPEED  30000.0f     // 推杆最大速度
#define VERT_FIRE_OFFSET_DEG    650000.0f    // 垂直发射偏移量（编码器值）
#define HORI_RELOAD_OFFSET_DEG  14700.0f     // 水平换弹偏移量（编码器值）

/* ================= 遥控器参数 ================= */
#define RC_DEADZONE           50             // 摇杆死区
#define YAW_SENSITIVITY       0.2f           // Yaw轴控制灵敏度

/**
 * @brief 2006 水平推杆电机配置 (带堵转检测)
 * @note  关键点：speed_PID.Improve 必须包含 PID_ErrorHandle
 */
#define PUSH_ROD_CONFIG(can_h, _id, _reverse)                                                    \
  {                                                                                              \
      .motor_type = M2006,                                                                       \
      .can_init_config =                                                                         \
          {                                                                                      \
              .can_handle = can_h,                                                               \
              .tx_id = _id,                                                                      \
          },                                                                                     \
      .controller_setting_init_config =                                                          \
          {                                                                                      \
              .angle_feedback_source = MOTOR_FEED,                                               \
              .speed_feedback_source = MOTOR_FEED,                                               \
              .outer_loop_type = SPEED_LOOP,                                                     \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                        \
              .motor_reverse_flag = _reverse,                                                    \
              .feedback_reverse_flag = _reverse,                                                 \
},                                                                                               \
      .controller_param_init_config =                                                            \
          {                                                                                      \
              .speed_PID =                                                                       \
                  {                                                                              \
                      .Kp = 2.0f,                                                                \
                      .Ki = 0.1f,                                                                \
                      .Kd = 0.0f,                                                                \
                      .MaxOut =10000.0f,                                                        \
                      .IntegralLimit = 3000.0f,                                                  \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle, \
                  },                                                                             \
              .angle_PID =                                                                       \
                  {                                                                              \
                      .Kp = 15.0f,                                                               \
                      .Ki = 0.0f,                                                                \
                      .Kd = 0.0f,                                                                \
                      .MaxOut = 6000.0f,                                                         \
                      .Improve = PID_Integral_Limit,                                             \
                  },                                                                             \
          },                                                                                     \
  }

/**
 * @brief 竖直推杆 (ID 7)
 * @note  关键点：speed_PID.Improve 必须包含 PID_ErrorHandle
 */
#define PUSH_VERT_CONFIG(can_h, _id, _reverse)                                                   \
  {                                                                                              \
      .motor_type = M2006,                                                                       \
      .can_init_config =                                                                         \
          {                                                                                      \
              .can_handle = can_h,                                                               \
              .tx_id = _id,                                                                      \
          },                                                                                     \
      .controller_setting_init_config =                                                          \
          {                                                                                      \
              .angle_feedback_source = MOTOR_FEED,                                               \
              .speed_feedback_source = MOTOR_FEED,                                               \
              .outer_loop_type = SPEED_LOOP,                                                     \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                        \
              .motor_reverse_flag = _reverse,                                                    \
              .feedback_reverse_flag = _reverse,                                                 \
          },                                                                                     \
      .controller_param_init_config =                                                            \
          {                                                                                      \
              .speed_PID =                                                                       \
                  {                                                                              \
                      .Kp = 2.0f,                                                                \
                      .Ki = 0.1f,                                                                \
                      .Kd = 0.0f,                                                                \
                      .MaxOut = 15000.0f,                                                        \
                      .IntegralLimit = 8000.0f,                                                  \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle, \
                  },                                                                             \
              .angle_PID =                                                                       \
                  {                                                                              \
                      .Kp = 30.0f,                                                               \
                      .Ki = 0.0f,                                                                \
                      .Kd = 0.3f,                                                                \
                      .MaxOut = 38000.0f,                                                        \
                      .Improve = PID_Integral_Limit,                                             \
                  },                                                                             \
          },                                                                                     \
  }

/**
 * @brief 6020 Yaw轴电机配置 (双环 PID)
 */
#define YAW_MOTOR_CONFIG(can_h, _id)                        \
  {                                                         \
      .motor_type = GM6020,                                 \
      .can_init_config =                                    \
          {                                                 \
              .can_handle = can_h,                          \
              .tx_id = _id,                                 \
          },                                                \
      .controller_setting_init_config =                     \
          {                                                 \
              .angle_feedback_source = MOTOR_FEED,          \
              .speed_feedback_source = MOTOR_FEED,          \
              .outer_loop_type = ANGLE_LOOP,                \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,   \
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, \
              .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL, \
          },                                                \
      .controller_param_init_config =                       \
          {                                                 \
              .speed_PID =                                  \
                  {                                         \
                      .Kp = 7.5f,                           \
                      .Ki = 0.0f,                           \
                      .Kd = 0.0f,                           \
                      .MaxOut = 30000.0f,                   \
                      .IntegralLimit = 3000.0f,             \
                      .Improve = PID_Integral_Limit,        \
                  },                                        \
              .angle_PID =                                  \
                  {                                         \
                      .Kp = 135.0f,                         \
                      .Ki = 0.0f,                           \
                      .Kd = 0.0f,                           \
                      .MaxOut = 3000.0f,                    \
                      .Improve = PID_Integral_Limit,        \
                  },                                        \
          },                                                \
  }

/**
 * @brief 3508 摩擦轮电机配置 (单环速度 PID)
 */
#define FRIC_MOTOR_CONFIG(can_h, _id, _reverse)      \
  {                                                  \
      .motor_type = M3508,                           \
      .can_init_config =                             \
          {                                          \
              .can_handle = can_h,                   \
              .tx_id = _id,                          \
          },                                         \
      .controller_setting_init_config =              \
          {                                          \
              .outer_loop_type = SPEED_LOOP,         \
              .close_loop_type = SPEED_LOOP,         \
              .motor_reverse_flag = _reverse,        \
              .feedback_reverse_flag = _reverse,     \
          },                                         \
      .controller_param_init_config =                \
          {                                          \
              .speed_PID =                           \
                  {                                  \
                      .Kp = 5.0f,                    \
                      .Ki = 0.0f,                    \
                      .Kd = 0.0f,                    \
                      .MaxOut = 16000.0f,            \
                      .IntegralLimit = 8000.0f,      \
                      .Improve = PID_Integral_Limit, \
                  },                                 \
          },                                         \
  }

#endif  // CONTROL_2026_ROBOT_CONFIG_H
