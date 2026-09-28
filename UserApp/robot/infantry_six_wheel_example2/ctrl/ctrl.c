#include "ctrl.h"

#include <math.h>

#include "general_def.h"
#include "robot_config.h"
#include "ui.h"

// Static variables for control state
#ifdef USE_RC_CTRL
static RC_ctrl_t* rc_data;
static RC_ctrl_t rc_data_last;
#elifdef USE_OCD_CTRL
static VT13_RC_t* rc_data;
static VT13_RC_t* rc_data_last;
#endif
static uint8_t is_first_update = 1;

// Intermediate variables
static float chassis_vx;
static float chassis_vy;
static float input_mag;
static float follow_err;
static float align_attenuation;

static uint8_t joystick_fire = 0;   // 遥控器要求开火
static uint8_t joystick_burst = 0;  // 遥控器要求连发
static uint8_t mouse_fire = 0;      // 鼠标要求开火
static uint8_t mouse_burst = 0;     // 鼠标要求连发

#define TURN_BOOST_DEADZONE 10
#define TURN_BOOST_GAIN 3.0f

#ifdef USE_RC_CTRL
void JoyStickCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  static float trigger_time = 0;

  if (is_first_update) {
    rc_data_last = rc_data[TEMP];
    is_first_update = 0;
  }
  // 右[中]，底盘使能 ROBOT_CHASSIS_FOLLOW
  if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_PROSTRATE;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift) {
      robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_FOLLOW;
    }
  }
  // 右[上]，自由底盘运动
  else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift) {
      robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_FREE;
    }
  }

  if (!switch_is_up(rc_data[TEMP].rc.switch_right)) {
    // 左[中],云台启动，摩擦轮启动，准备射击
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      // shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
    }
    // 左[上]，开火，发射，根据时间判断单发或者连发
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      if (switch_is_mid(rc_data_last.rc.switch_left)) {
        trigger_time = DWT_GetTimeline_s();
      }
      if (DWT_GetTimeline_s() - trigger_time > 1.0f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      } else {
        shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      }
    }
    // 云台使能,或视觉未识别到目标,纯遥控器拨杆控制
    if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
      if (abs(rc_data[TEMP].rc.rocker_r1) > 20.0f)
        gimbal_ctrl_cmd->pitch -= 0.00006f * (float)rc_data[TEMP].rc.rocker_r1;
    }
  } else {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
    gimbal_ctrl_cmd->yaw += -0.00035f * (float)rc_data[TEMP].rc.rocker_r_;
  }
  // 云台PITCH轴软件限位
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW: {
#if (!defined(ONE_BOARD))
      chassis_ctrl_cmd->is_rotate = 0;
      // 获取输入（左摇杆 → 云台坐标系下的 vx, vy）
      chassis_vx = (float)rc_data[TEMP].rc.rocker_l_;
      chassis_vy = (float)rc_data[TEMP].rc.rocker_l1;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 5.0f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      } else {
        // 静止回正：底盘对齐云台
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      // 直接传摇杆原始值，ChassisProstrateMode 里做映射
      chassis_ctrl_cmd->vx = input_mag;
#endif
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_ROTATE: {
      chassis_ctrl_cmd->is_rotate = 1;
      chassis_ctrl_cmd->vx = 0.0f;
      chassis_ctrl_cmd->wz = 800.0f;

      // chassis_vx = 0.0f;
      // chassis_vy = 0.0f;
      //
      // input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      // if (input_mag > 5.0f) {  // 加死区，摇杆归中时不算
      //   float target_angle_to_gimbal_p = atan2f(chassis_vy, chassis_vx);
      //   float target_angle_to_chassis_p = target_angle_to_gimbal_p + robot->offset_angle * DEGREE_2_RAD;
      //   chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis_p - 1.57);
      // } else
      //   chassis_ctrl_cmd->vx = 0.0f;
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_FREE: {
      chassis_ctrl_cmd->is_rotate = 0;
      // 左竖直 → 前进后退
      chassis_ctrl_cmd->vx = (float)rc_data[TEMP].rc.rocker_l1;
      chassis_ctrl_cmd->wz = (float)rc_data[TEMP].rc.rocker_l_;
      break;
    }
    default:
      break;
  }
  //  记录上一次数据
  // rc_data_last = rc_data[TEMP];
}
#elifdef USE_OCD_CTRL
void JoyStickCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  static float trigger_time = 0;

  // 使用VT13遥控器的新控制逻辑
  if (switch_middle(rc_data->rc.mode_switch)) {  // 中档
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_PROSTRATE;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data->mouse_key.keyboard.shift) {
      robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_FOLLOW;
    }
  }
  if (switch_right(rc_data->rc.mode_switch)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    static uint8_t trigger_last = 0;
    if (trigger_last == 0 && rc_data->rc.trigger == 1) {
      trigger_time = DWT_GetTimeline_s();
    }
    trigger_last = rc_data->rc.trigger;
    if (rc_data->rc.trigger == 1) {
      if (DWT_GetTimeline_s() - trigger_time > 1.0f) {
        joystick_fire = 1;
        joystick_burst = 1;
      } else {
        joystick_fire = 1;
        joystick_burst = 0;
      }
    } else {
      joystick_fire = 0;
      joystick_burst = 0;
    }
    // ← 删除这里对 load_mode 的直接写入
  }
  // 云台控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw += -0.00035f * (float)rc_data->rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.00006f * (float)rc_data->rc.rocker_r1;
  }

  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW: {
#if (!defined(ONE_BOARD))
      chassis_ctrl_cmd->is_rotate = 0;
      // 获取输入（左摇杆 → 云台坐标系下的 vx, vy）
      chassis_vx = (float)rc_data[TEMP].rc.rocker_l_;
      chassis_vy = (float)rc_data[TEMP].rc.rocker_l1;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 5.0f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      } else {
        // 静止回正：底盘对齐云台
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      // 直接传摇杆原始值，ChassisProstrateMode 里做映射
      chassis_ctrl_cmd->vx = input_mag;
#endif
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_ROTATE: {
      chassis_ctrl_cmd->is_rotate = 1;
      chassis_ctrl_cmd->vx = 0.0f;
      chassis_ctrl_cmd->wz = 800.0f;

      chassis_vx = 0.0f;
      chassis_vy = 0.0f;

      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 5.0f) {  // 加死区，摇杆归中时不算
        float target_angle_to_gimbal_p = atan2f(chassis_vy, chassis_vx);
        float target_angle_to_chassis_p = target_angle_to_gimbal_p + robot->offset_angle * DEGREE_2_RAD;
        chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis_p - 1.57);
      } else
        chassis_ctrl_cmd->vx = 0.0f;
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_FREE: {
      chassis_ctrl_cmd->is_rotate = 0;
      // 左竖直 → 前进后退
      chassis_ctrl_cmd->vx = (float)rc_data[TEMP].rc.rocker_l1;
      chassis_ctrl_cmd->wz = (float)rc_data[TEMP].rc.rocker_l_;
      break;
    }
    default:
      break;
  }
}
#endif

/*
 * ==================== 键鼠控制键位说明 ====================
 *
 * 【移动控制】
 *   W        - 前进
 *   S        - 后退
 *   A        - 左平移（底盘先转向再平移）
 *   D        - 右平移（底盘先转向再平移）
 *
 * 【云台控制】
 *   鼠标 X轴  - 云台 Yaw 轴旋转（左右）
 *   鼠标 Y轴  - 云台 Pitch 轴旋转（上下）
 *
 * 【射击控制】
 *   鼠标左键   - 短按单发，长按(>0.3s)连发
 *   鼠标右键   - 按住开启自瞄（视觉识别到目标时生效）
 *
 * 【模式切换】
 *   Shift     - 按住进入小陀螺模式（ROTATE），松开回到底盘跟随模式（FOLLOW）
 *
 * 【功能键】
 *   X         - 单次触发，云台 Yaw 转 180°（快速掉头/逃跑）
 *   C         - 单次触发，超级电容开关（预留）
 *   Ctrl + Z  - 循环切换速度档位：
 *                 档位0: ×1.0（默认）
 *                 档位1: ×1.5
 *                 档位2: ×2.5
 *
 * ===========================================================
 */
#ifdef USE_RC_CTRL
void MouseKeyCtrl(RobotInstance* robot) {
  // Helper pointers
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  Vision_Receive_s* vision_recv_data = robot->vision_recv_data;

  if (is_first_update) {
    rc_data_last = rc_data[TEMP];
    is_first_update = 0;
  }
  static float target_speed = 535.f;
  // 1. 基础初始化
  if (robot->chassis->super_cap->super_cap_ctrl_cmd == NORMAL) {
    target_speed = 535.f;  // 80+40W
  } else {
    target_speed = 630.f;  // 120w+40w
  }

  static float speed_coff = 1.0f;      // 速度系数
  static float rotate_coff = 1.0f;     // 小陀螺旋转频率系数
  static uint8_t x_key_last = 0;       // 记录上一次Space键状态
  static uint8_t f_key_last = 0;       // 记录上一次F键状态
  static uint8_t ctrl_g_key_last = 0;  // 记录上一次Ctrl+G键状态
  static uint8_t c_key_last = 0;       // 记录上一次C键状态
  static uint8_t is_rotate_mode = 0;   // 小陀螺模式标志位
  static float trigger_time = 0;       // 开火时间

  // float max_power = robot->referee_data->GameRobotState.chassis_power_limit;
  // // 注意：在你们的 referee_protocol.h 中，0x0202 的底盘功率对应的是 reserved_3 字段
  // float current_power = robot->referee_data->PowerHeatData.reserved_3;

  // // 设定目标功率，预留一点安全裕量 (比如留 5W，防止超限被底层强行砍电流导致跑偏)
  // float target_power = max_power - 5.0f;
  // float power_err = target_power - current_power;

  // // 非对称积分控制器（I控制）
  // if (power_err < 0) {
  //     // 超功率了！紧急降速，系数给大一点 (比如 2.0f，需根据实车微调)
  //     // 如果超了 10W，每跑一次这段代码 target_speed 就下降 20
  //     target_speed += 2.0f * power_err;
  // } else {
  //     // 功率有富裕，缓慢提速去试探最大速度，系数给小一点 (比如 0.1f)
  //     // 这样静止时 target_speed 会自动回升到最大允许速度，保证起步时的爆发力
  //     target_speed += 0.1f * power_err;
  // }

  // // 对 target_speed 进行安全限幅
  // if (target_speed > 1000.0f) target_speed = 1000.0f; // 起步或功率充裕时的最大期望速度上限
  // if (target_speed < 200.0f)  target_speed = 200.0f;  // 即使严重超功率，也保留一个最低速度保证底盘还能动
  // // ==============================================================

  // [Ctrl+Z]键设置速度，测试用
  switch (rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL][Key_Z] % 3) {
    case 0:
      speed_coff = 1.0f;
      rotate_coff = 1.0f;
      break;
    case 1:
      speed_coff = 1.2f;
      rotate_coff = 1.2f;
      break;
    case 2:
      speed_coff = 2.0f;
      rotate_coff = 2.0f;
      break;
    default:
      break;
  }
  // 2. 云台
  // 2.1 [右键]按住开启自瞄

  if (rc_data[TEMP].mouse.press_r) {
    if (has_non_zero_data(vision_recv_data)) {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;  // 右键自瞄开启
      gimbal_ctrl_cmd->yaw = vision_recv_data->gimbal_receive.yaw;
      gimbal_ctrl_cmd->pitch = vision_recv_data->gimbal_receive.pitch;
    } else {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;  // 没识别到目标，保持手动
    }
  } else {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;  // 松开右键，手动模式
  }

  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_VISION) {
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kp = 2.5f;
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kd = 0.05f;
    robot->gimbal->pitch_motor->motor_controller.angle_PID.Kp = 1.8f;
    robot->gimbal->pitch_motor->motor_controller.angle_PID.Kd = 0.03f;
  } else if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kp = 0.8f;
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kd = 0.02f;
    robot->gimbal->pitch_motor->motor_controller.angle_PID.Kp = 1.0f;
    robot->gimbal->pitch_motor->motor_controller.angle_PID.Kd = 0.02f;
  }

  // 2.2 射击控制逻辑,[左键]射击
  if (rc_data[TEMP].mouse.press_l) {
    // 校验：摩擦轮开启
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
      // 默认先设为单发
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      // 检查按下的持续时间 (当前时间 - 上次松开的时间/按下起始时间),超过1秒，覆盖为连发
      if (DWT_GetTimeline_s() - trigger_time > 0.3f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      }
    }
  } else {
    if (!switch_is_up(rc_data[TEMP].rc.switch_left)) {
      //  鼠标左键松开时
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      // 记录松开时间
      trigger_time = DWT_GetTimeline_s();
    }
  }

  // 2.3鼠标云台控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= (float)rc_data[TEMP].mouse.x * 0.002f;    // X轴灵敏度
    gimbal_ctrl_cmd->pitch -= (float)rc_data[TEMP].mouse.y * 0.002f;  // Y轴灵敏度
    // 云台Pitch限位
    if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
    else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 3. 功能键
  // [Shift] 键按住小陀螺
  if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift) {
    is_rotate_mode = 1;
  } else {
    is_rotate_mode = 0;
  }

  // [X] 单次触发：转 180° 逃跑
  if (rc_data[TEMP].key[KEY_PRESS].x != x_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].x) {
      gimbal_ctrl_cmd->yaw += 180.0f;
    }
    x_key_last = rc_data[TEMP].key[KEY_PRESS].x;
  }

  if (!rc_data_last.key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].ctrl) {
    if (robot->chassis_fetch_data) {
      robot->chassis_fetch_data->force_refresh_ui = 1;  // 告诉底盘板去刷新UI
    }
  }

  // [C] 单次切换：超级电容开关
  if (rc_data[TEMP].key[KEY_PRESS].c != c_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].c) {
      if (robot->chassis->super_cap->super_cap_ctrl_cmd == BOOST) {
        robot->chassis->super_cap->super_cap_ctrl_cmd = NORMAL;
      } else {
        robot->chassis->super_cap->super_cap_ctrl_cmd = BOOST;
      }
      // 超级电容开关
    }
    c_key_last = rc_data[TEMP].key[KEY_PRESS].c;
  }

  // 4. 核心运动算法
  //  确定 Robot Mode
  if (is_rotate_mode) {  // 小陀螺
    robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_ROTATE;
  } else {
    robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_FOLLOW;  // 默认是FOLLOW模式
  }
  // 处理对应模式
  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_PROSTRATE_ROTATE:
      chassis_ctrl_cmd->is_rotate = 1;
      chassis_ctrl_cmd->vx = 0.0f;
      chassis_ctrl_cmd->wz = 800.0f;

      // // 设置目标速度矢量 (vx, vy)
      // if (rc_data[TEMP].key[KEY_PRESS].w)
      //   chassis_vy += target_speed * speed_coff;
      // else if (rc_data[TEMP].key[KEY_PRESS].s)
      //   chassis_vy += -target_speed * speed_coff;
      // else
      //   chassis_vy += 0.0f;
      //
      // if (rc_data[TEMP].key[KEY_PRESS].d)
      //   chassis_vx += target_speed * speed_coff;
      // else if (rc_data[TEMP].key[KEY_PRESS].a)
      //   chassis_vx += -target_speed * speed_coff;
      // else
      //   chassis_vx += 0.0f;
      //
      // input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      // if (input_mag > 5.0f) {  // 加死区，摇杆归中时不算
      //   float phase_compensation;
      //   if (rc_data[TEMP].key[KEY_PRESS].w) {
      //     phase_compensation = -52.0f * DEGREE_2_RAD;
      //   } else if (rc_data[TEMP].key[KEY_PRESS].s) {
      //     phase_compensation = -52.0f * DEGREE_2_RAD;
      //   } else if (rc_data[TEMP].key[KEY_PRESS].a) {
      //     phase_compensation = 128.0f * DEGREE_2_RAD;
      //   } else if (rc_data[TEMP].key[KEY_PRESS].d) {
      //     phase_compensation = 128.0f * DEGREE_2_RAD;
      //   }
      //   float target_angle_to_gimbal_p = atan2f(chassis_vy, chassis_vx);
      //   float target_angle_to_chassis_p = target_angle_to_gimbal_p + robot->offset_angle * DEGREE_2_RAD;
      //   chassis_ctrl_cmd->vx += input_mag * sinf(target_angle_to_chassis_p + phase_compensation);
      // } else
      //   chassis_ctrl_cmd->vx += 0.0f;
      break;
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW:
#if (!defined(ONE_BOARD))
      chassis_ctrl_cmd->is_rotate = 0;
      // 设置目标速度矢量 (vx, vy),单位为m/s
      if (rc_data[TEMP].key[KEY_PRESS].w)
        chassis_vy += target_speed * speed_coff;
      else if (rc_data[TEMP].key[KEY_PRESS].s)
        chassis_vy += -target_speed * speed_coff;
      else
        chassis_vy += 0.0f;
      if (rc_data[TEMP].key[KEY_PRESS].d)
        chassis_vx += target_speed * speed_coff;
      else if (rc_data[TEMP].key[KEY_PRESS].a)
        chassis_vx += -target_speed * speed_coff;
      else
        chassis_vx += 0.0f;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      float max_speed = target_speed * speed_coff;
      if (input_mag > max_speed) {
        chassis_vx = chassis_vx / input_mag * max_speed;
        chassis_vy = chassis_vy / input_mag * max_speed;
        input_mag = max_speed;
      }
      // ===== 默认无前馈 =====
      chassis_ctrl_cmd->wz = 0.0f;
      if (input_mag > 5.0f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
        // ===== 前馈：误差大时额外给 wz 加速转向 =====
        float abs_err = fabsf(follow_err);
        if (abs_err > TURN_BOOST_DEADZONE) {
          chassis_ctrl_cmd->wz = follow_err * TURN_BOOST_GAIN;
        }
      } else {
        // 静止回正：底盘对齐云台
        follow_err = 0.0f;  // 静止时清零，防止衰减计算用到脏值
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      chassis_ctrl_cmd->vx += input_mag;
      break;
#endif
    case ROBOT_CHASSIS_PROSTRATE_FREE: {
      chassis_ctrl_cmd->is_rotate = 0;
      // 左竖直 → 前进后退
      chassis_ctrl_cmd->vx = (float)rc_data[TEMP].rc.rocker_l1;
      chassis_ctrl_cmd->wz = (float)rc_data[TEMP].rc.rocker_l_;
      break;
    }
    default:
      break;
  }
  // 6.更新历史数据(遥控器有的话这里就不用)
  rc_data_last = rc_data[TEMP];
}
#elifdef USE_OCD_CTRL
void MouseKeyCtrl(RobotInstance* robot) {
  // Helper pointers
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  Vision_Receive_s* vision_recv_data = robot->vision_recv_data;

  if (is_first_update) {
    rc_data_last = rc_data;
    is_first_update = 0;
  }

  // 1. 基础初始化
  static float target_speed = 800.f;
  static float speed_coff = 1.0f;      // 速度系数
  static float rotate_coff = 1.0f;     // 小陀螺旋转频率系数
  static uint8_t x_key_last = 0;       // 记录上一次Space键状态
  static uint8_t f_key_last = 0;       // 记录上一次F键状态
  static uint8_t ctrl_g_key_last = 0;  // 记录上一次Ctrl+G键状态
  static uint8_t c_key_last = 0;       // 记录上一次C键状态
  static uint8_t is_rotate_mode = 0;   // 小陀螺模式标志位
  static float trigger_time = 0;       // 开火时间

  // 2. 云台
  // 2.1 [右键]按住开启自瞄
  if (rc_data->mouse_key.mouse.press_r) {
    if (has_non_zero_data(vision_recv_data)) {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;  // 右键自瞄开启
      gimbal_ctrl_cmd->yaw = vision_recv_data->gimbal_receive.yaw;
      gimbal_ctrl_cmd->pitch = vision_recv_data->gimbal_receive.pitch;
    } else {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;  // 没识别到目标，保持手动
    }
  } else {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;  // 松开右键，手动模式
  }

  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_VISION) {
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kp = 2.0f;
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kd = 0.03f;
  } else if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kp = 0.8f;
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kd = 0.02f;
  }

  // 2.2 射击控制逻辑,[左键]射击
  static float mouse_trigger_time = 0;
  if (rc_data->mouse_key.mouse.press_l) {
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
      mouse_fire = 1;
      if (DWT_GetTimeline_s() - mouse_trigger_time > 0.3f) {
        mouse_burst = 1;
      } else {
        mouse_burst = 0;
      }
    }
  } else {
    mouse_fire = 0;
    mouse_burst = 0;
    mouse_trigger_time = DWT_GetTimeline_s();
  }

  // 2.3合并开火决策
  if (joystick_burst || mouse_burst) {
    shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
  } else if (joystick_fire || mouse_fire) {
    shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
  } else {
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }

  // 2.4鼠标云台控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= (float)rc_data->mouse_key.mouse.x * 0.002f;    // X轴灵敏度
    gimbal_ctrl_cmd->pitch -= (float)rc_data->mouse_key.mouse.y * 0.002f;  // Y轴灵敏度
    // 云台Pitch限位
    if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
    else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 3. 功能键
  // [Shift] 键按住小陀螺
  if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data->mouse_key.keyboard.shift) {
    is_rotate_mode = 1;
  } else {
    is_rotate_mode = 0;
  }

  // [X] 单次触发：转 180° 逃跑
  if (rc_data->mouse_key.keyboard.x != x_key_last) {
    if (rc_data->mouse_key.keyboard.x) {
      gimbal_ctrl_cmd->yaw += 180.0f;
    }
    x_key_last = rc_data->mouse_key.keyboard.x;
  }

  if (!rc_data->mouse_key.keyboard.ctrl && rc_data->mouse_key.keyboard.ctrl) {
    if (robot->chassis_fetch_data) {
      robot->chassis_fetch_data->force_refresh_ui = 1;  // 告诉底盘板去刷新UI
    }
  }

  // [C] 单次切换：超级电容开关
  if (rc_data->mouse_key.keyboard.c != c_key_last) {
    if (rc_data->mouse_key.keyboard.c) {
      if (robot->chassis->super_cap->super_cap_ctrl_cmd == BOOST) {
        robot->chassis->super_cap->super_cap_ctrl_cmd = NORMAL;
      } else {
        robot->chassis->super_cap->super_cap_ctrl_cmd = BOOST;
      }
      // 超级电容开关
    }
    c_key_last = rc_data->mouse_key.keyboard.c;
  }

  // 4. 核心运动算法
  //  确定 Robot Mode
  if (is_rotate_mode) {  // 小陀螺
    robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_ROTATE;
  } else {
    robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_FOLLOW;  // 默认是FOLLOW模式
  }
  // 处理对应模式
  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_PROSTRATE_ROTATE:
      chassis_ctrl_cmd->is_rotate = 1;
      chassis_ctrl_cmd->vx = 0.0f;
      chassis_ctrl_cmd->wz = 800.0f;

      // 设置目标速度矢量 (vx, vy)
      if (rc_data->mouse_key.keyboard.w)
        chassis_vy += target_speed * speed_coff;
      else if (rc_data->mouse_key.keyboard.s)
        chassis_vy += -target_speed * speed_coff;
      else
        chassis_vy += 0.0f;

      if (rc_data->mouse_key.keyboard.d)
        chassis_vx += target_speed * speed_coff;
      else if (rc_data->mouse_key.keyboard.a)
        chassis_vx += -target_speed * speed_coff;
      else
        chassis_vx += 0.0f;

      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 5.0f) {  // 加死区，摇杆归中时不算
        float phase_compensation;
        if (rc_data->mouse_key.keyboard.w) {
          phase_compensation = -52.0f * DEGREE_2_RAD;
        } else if (rc_data->mouse_key.keyboard.s) {
          phase_compensation = -52.0f * DEGREE_2_RAD;
        } else if (rc_data->mouse_key.keyboard.a) {
          phase_compensation = 128.0f * DEGREE_2_RAD;
        } else if (rc_data->mouse_key.keyboard.d) {
          phase_compensation = 128.0f * DEGREE_2_RAD;
        }
        float target_angle_to_gimbal_p = atan2f(chassis_vy, chassis_vx);
        float target_angle_to_chassis_p = target_angle_to_gimbal_p + robot->offset_angle * DEGREE_2_RAD;
        chassis_ctrl_cmd->vx += input_mag * sinf(target_angle_to_chassis_p + phase_compensation);
      } else
        chassis_ctrl_cmd->vx += 0.0f;
      break;
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW:
      chassis_ctrl_cmd->is_rotate = 0;
      // 设置目标速度矢量 (vx, vy),单位为m/s
      if (rc_data->mouse_key.keyboard.w)
        chassis_vy += target_speed * speed_coff;
      else if (rc_data->mouse_key.keyboard.s)
        chassis_vy += -target_speed * speed_coff;
      else
        chassis_vy += 0.0f;
      if (rc_data->mouse_key.keyboard.d)
        chassis_vx += target_speed * speed_coff;
      else if (rc_data->mouse_key.keyboard.a)
        chassis_vx += -target_speed * speed_coff;
      else
        chassis_vx += 0.0f;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      float max_speed = target_speed * speed_coff;
      if (input_mag > max_speed) {
        chassis_vx = chassis_vx / input_mag * max_speed;
        chassis_vy = chassis_vy / input_mag * max_speed;
        input_mag = max_speed;
      }
      // ===== 默认无前馈 =====
      chassis_ctrl_cmd->wz = 0.0f;
      if (input_mag > 5.0f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
        // ===== 前馈：误差大时额外给 wz 加速转向 =====
        float abs_err = fabsf(follow_err);
        if (abs_err > TURN_BOOST_DEADZONE) {
          chassis_ctrl_cmd->wz = follow_err * TURN_BOOST_GAIN;
        }
      } else {
        // 静止回正：底盘对齐云台
        follow_err = 0.0f;  // 静止时清零，防止衰减计算用到脏值
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      chassis_ctrl_cmd->vx += input_mag;
      break;
    case ROBOT_CHASSIS_PROSTRATE_FREE: {
      chassis_ctrl_cmd->is_rotate = 0;
      // 左竖直 → 前进后退
      chassis_ctrl_cmd->vx = (float)rc_data[TEMP].rc.rocker_l1;
      chassis_ctrl_cmd->wz = (float)rc_data[TEMP].rc.rocker_l_;
      break;
    }
    default:
      break;
  }
  // 6.更新历史数据
  rc_data_last = rc_data;
}

#endif

#ifdef USE_RC_CTRL
void EmergencyHandler(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  // 两switch都在下或者遥控器断连，断电
  if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) |
      switch_is_off(rc_data[TEMP].rc.switch_right)) {
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    LOGERROR("[CMD] emergency stop!");

  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }  // 底盘失能
  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }  // 发射失能
  if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
}
#elifdef USE_OCD_CTRL
void EmergencyHandler(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  // 新VT13遥控器紧急处理逻辑
  if (switch_left(rc_data->rc.mode_switch) || rc_data->button_status.pause_flag == 1 ||
      rc_data == NULL) {  // 拨杆在左或按下暂停键时断电
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    LOGERROR("[CMD] emergency stop!");
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }
}
#endif
