//
// Created by PC on 2025/11/18.
//
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
#include "dji_motor.h"
#include "main.h"

RobotInstance* robot = NULL;
static DartInstance* dart = NULL;
static RC_ctrl_t* rc_data = NULL;

bool IsInDeadzone(int16_t value) {
  return (abs(value) < RC_DEADZONE);
}    //死区检测

float MapStickToSpeed(int16_t stick_value, float max_speed) {
  if (IsInDeadzone(stick_value)) {
    return 0.0f;
  }
  return ((float)stick_value / 660.0f) * max_speed;
}     //映射摇杆值到电机速度

void DartInit(void) {
  // 内存分配与清零: zmalloc 在 user_lib.c 中定义，它会自动执行 memset 清零
  dart = (DartInstance*)zmalloc(sizeof(DartInstance));
  memset(dart, 0, sizeof(DartInstance));

  // 初始化 Yaw 轴电机 (GM6020)
  Motor_Init_Config_s yaw_conf = YAW_MOTOR_CONFIG(&hcan2, YAW_MOTOR_ID);
  dart->yaw_motor = DJIMotorInit(&yaw_conf);

  // 初始化 垂直推杆 (M2006)
  // 这里的PID配置包含了 PID_ErrorHandle，用于校准时的堵转检测
//  Motor_Init_Config_s vert_conf = PUSH_ROD_CONFIG(&hcan1, PUSH_VERT_ID, MOTOR_DIRECTION_NORMAL);
//  dart->vertical_pushrod = DJIMotorInit(&vert_conf);
  Motor_Init_Config_s vert_conf = PUSH_VERT_CONFIG(&hcan1, PUSH_VERT_ID, MOTOR_DIRECTION_NORMAL);
  dart->vertical_pushrod = DJIMotorInit(&vert_conf);
  // 初始化 水平推杆 (M2006)
  Motor_Init_Config_s hori_conf = PUSH_ROD_CONFIG(&hcan1, PUSH_HORI_ID, MOTOR_DIRECTION_NORMAL);
  dart->horizontal_pushrod = DJIMotorInit(&hori_conf);

  // 初始化 摩擦轮 (M3508 x4): 左边: LU(1), LD(2) | 右边: RU(3), RD(4)

  // 1. 左上 (Left Up) - ID 1
  Motor_Init_Config_s fric_lu_conf = FRIC_MOTOR_CONFIG(&hcan1, FRIC_MOTOR_LU, MOTOR_DIRECTION_REVERSE);
  dart->friction_motor[0] = DJIMotorInit(&fric_lu_conf);

  // 2. 左下 (Left Down) - ID 2
  Motor_Init_Config_s fric_ld_conf = FRIC_MOTOR_CONFIG(&hcan1, FRIC_MOTOR_LD, MOTOR_DIRECTION_REVERSE);
  dart->friction_motor[1] = DJIMotorInit(&fric_ld_conf);

  // 3. 右上 (Right Up) - ID 3 反转
  Motor_Init_Config_s fric_ru_conf = FRIC_MOTOR_CONFIG(&hcan1, FRIC_MOTOR_RU, MOTOR_DIRECTION_NORMAL);
  dart->friction_motor[2] = DJIMotorInit(&fric_ru_conf);

  // 4. 右下 (Right Down) - ID 4 反转
  Motor_Init_Config_s fric_rd_conf = FRIC_MOTOR_CONFIG(&hcan1, FRIC_MOTOR_RD, MOTOR_DIRECTION_NORMAL);
  dart->friction_motor[3] = DJIMotorInit(&fric_rd_conf);

  /* ================= 状态机初始值 ================= */
  dart->current_mode = DART_MODE_DEBUG;      // 上电默认调试，防误触
  dart->calibration_step = CALI_STEP_IDLE;   // 校准闲置
  dart->is_calibrated = false;               // 未校准

  // 初始命令清零
  dart->dart_ctrl_cmd.fire_command = false;
  dart->dart_ctrl_cmd.friction_speed = 0;
  dart->fric_base_speed = FRIC_BASE_MIN_SPEED;

  // 自动发射相关初始化
  dart->is_hori_blocked = false;
  dart->hori_block_position = 0.0f;
  dart->fric_base_speed = FRIC_BASE_MIN_SPEED;

}

void RobotInit(void) {
  // 1. 全局实例分配
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  // 2. 初始化遥控器
  rc_data = RemoteControlInit(&huart3);
  robot->rc_data = rc_data;

  // 3. 初始化飞镖系统
  DartInit();
  robot->dart = dart; // 关联指针
  robot->robot_mode = ROBOT_POWER_ON;
  DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
  DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);
  DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
  DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
}

/* 检查电机是否堵转 (用于校准) */
static bool IsMotorBlocked(DJIMotorInstance *motor) {
  if (motor == NULL) return false;
  // 读取底层PID错误状态
  if (motor->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
    // 发生堵转后清除错误标志，防止死锁
    motor->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;
    motor->motor_controller.speed_PID.ERRORHandler.ERRORCount = 0;
    return true;
  }
  return false;
}


/* 状态机逻辑 */
void DartStateMachineUpdate(void) {
  static DartMode_e last_mode = DART_MODE_DEBUG;

  if (robot->robot_mode == ROBOT_EMERGENCY_STOP) {
    dart->current_mode = DART_MODE_EMERGENCY_STOP;
    return;
  }

  // 模式切换时重置状态
  if (last_mode != dart->current_mode) {
    last_mode = dart->current_mode;
  }

  // 遥控器右侧拨杆 (switch_right) 控制主模式
  // 下档：调试模式 (DEBUG)
  if (switch_is_down(rc_data->rc.switch_right)) {
    dart->current_mode = DART_MODE_DEBUG;
    dart->calibration_step = CALI_STEP_IDLE;
    //    dart->is_calibrated = false;
  }
  // 中档：自动/就绪模式
  else if (switch_is_mid(rc_data->rc.switch_right)) {
    if (!dart->is_calibrated) {
      // 未校准则强制进入校准模式
      dart->current_mode = DART_MODE_CALIBRATING;
    } else {
      // 已校准则进入就绪状态
      dart->current_mode = DART_MODE_AUTO_READY;
    }
  }
  // 上档：自动打弹
  else if (switch_is_up(rc_data->rc.switch_right)) {
    if (dart->is_calibrated) {
      dart->current_mode = DART_MODE_AUTO_FIRE; // 只有校准过才允许自动打弹
    } else {
      dart->current_mode = DART_MODE_CALIBRATING; // 否则强制去校准
    }
  }
}

/* 调试模式处理(纯手动控制，用于测试电机好坏和方向) */
static float yaw_current_target = 0.0f;
/* 调试模式： */
void DartDebugModeHandler(void) {
  if (rc_data == NULL) return;// 检查指针有效性 (防止空指针)
  DJIMotorEnable(dart->yaw_motor); //因为在cali里写了yaw轴失能代码，回到debug里需要使能
  int16_t stick_val = rc_data->rc.rocker_l_;
//   --- 1. Yaw 轴 (左摇杆左右 rocker_l_) ---
//   使用“增量式”位置控制：推杆增加角度，回中保持角度
  float angle_inc = ((float)stick_val / 66.0f) * YAW_SENSITIVITY;
  yaw_current_target -= angle_inc;
  float current_angle = dart->yaw_motor->measure.total_angle;
  float max_lead_angle = 15.0f;
  if (yaw_current_target > current_angle + max_lead_angle) {
    yaw_current_target = current_angle + max_lead_angle;
  }
  else if (yaw_current_target < current_angle - max_lead_angle) {
    yaw_current_target = current_angle - max_lead_angle;
  }
  VAL_LIMIT(yaw_current_target, -5000.0f, 5000.0f);
  DJIMotorOuterLoop(dart->yaw_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(dart->yaw_motor, yaw_current_target);


  // --- 2. 垂直推杆 (左摇杆上下 rocker_l1) ---
 // 速度环控制
  float vert_speed = MapStickToSpeed(rc_data->rc.rocker_l1, DART_PUSHROD_MAX_SPEED);
  DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
  DJIMotorSetPIDRef(dart->vertical_pushrod, vert_speed);

  // 角度环测试（操作方法：先进校准模式，确定零点切debug）
//    float vert_fire_target_debug = dart->vert_zero_ecd + CALI_BACK_OFFSET + 676000;//+往上，-往下719000650000
//      DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
//      DJIMotorSetPIDRef(dart->vertical_pushrod, vert_fire_target_debug);

  // --- 3. 水平推杆 (右摇杆左右 rocker_r_) ---
  // 速度环控制
  float hori_speed = MapStickToSpeed(rc_data->rc.rocker_r_, DART_PUSHROD_MAX_SPEED);
  DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
  DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_speed);

  // 角度环测试（操作方法：先进校准模式，确定零点切debug）
  //  float hori_fire_target_debug = dart->hori_zero_ecd + 14300;//-往右，+往左(normal时应校准后靠右墙）
  //    DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
  //    DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_fire_target_debug);


  // --- 4. 摩擦轮 (右摇杆上下 rocker_r1) ---
  // 速度环控制，四个轮子同步
  float fric_speed = MapStickToSpeed(rc_data->rc.rocker_r1, DART_FRICTION_MAX_SPEED);

  for(int i=0; i<4; i++) {
    DJIMotorOuterLoop(dart->friction_motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(dart->friction_motor[i], fric_speed);
  }
}


/* 校准模式处理 */
static void DartCalibrationHandler(void) {
  if (dart->yaw_motor != NULL) {
    // 停止yaw轴电机，防止积分残留
    DJIMotorStop(dart->yaw_motor);
    // 重置Yaw目标值到当前位置
    yaw_current_target = dart->yaw_motor->measure.total_angle;

  }
  switch (dart->calibration_step) {
      // --- 【关键修复】IDLE 状态：打扫战场，清除一切残留 ---
    case CALI_STEP_IDLE:
      // 1. 清除垂直电机 PID 残留
      dart->vertical_pushrod->motor_controller.speed_PID.ITerm = 0;
      dart->vertical_pushrod->motor_controller.speed_PID.Output = 0;
      dart->vertical_pushrod->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;
      dart->vertical_pushrod->motor_controller.speed_PID.ERRORHandler.ERRORCount = 0;

      // 2. 清除水平电机 PID 残留
      dart->horizontal_pushrod->motor_controller.speed_PID.ITerm = 0;
      dart->horizontal_pushrod->motor_controller.speed_PID.Output = 0;
      dart->horizontal_pushrod->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;
      dart->horizontal_pushrod->motor_controller.speed_PID.ERRORHandler.ERRORCount = 0;

      // 3. 启动
      dart->calibration_step = CALI_STEP_VERT_PUSH;
      break;

    // --- 阶段1: 垂直推杆归零 ---
    case CALI_STEP_VERT_PUSH:
      dart->vertical_pushrod->motor_controller.speed_PID.MaxOut = 1000.0f;
      dart->vertical_pushrod->motor_controller.speed_PID.IntegralLimit = 500.0f;
      DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->vertical_pushrod, -CALI_SPEED);
      DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

      // 检测堵转
      if (IsMotorBlocked(dart->vertical_pushrod)) {
        // 记录当前编码器值作为零点参考
        dart->vert_zero_ecd = dart->vertical_pushrod->measure.total_angle;
        dart->vertical_pushrod->motor_controller.speed_PID.ITerm = 0;
        dart->vertical_pushrod->motor_controller.speed_PID.Output = 0;
        dart->vertical_pushrod->motor_controller.speed_PID.Iout = 0;
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);

        dart->vertical_pushrod->motor_controller.speed_PID.MaxOut = 16000.0f;
        dart->vertical_pushrod->motor_controller.speed_PID.IntegralLimit = 3000.0f;
        dart->calibration_step = CALI_STEP_VERT_BACK;
      }
      break;

    case CALI_STEP_VERT_BACK:
      static uint32_t vert_back_start_time = 0;
      static float saved_vert_angle_maxout = 0.0f;

      // 记录开始时间
      if (vert_back_start_time == 0) {
        vert_back_start_time = HAL_GetTick();
        // 保存原来的位置环MaxOut
        saved_vert_angle_maxout = dart->vertical_pushrod->motor_controller.angle_PID.MaxOut;
        // 临时限制位置环输出
        dart->vertical_pushrod->motor_controller.angle_PID.MaxOut = 3000.0f;  // 安全值
      }

      DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
      float vert_target = dart->vert_zero_ecd + CALI_BACK_OFFSET;
      DJIMotorSetPIDRef(dart->vertical_pushrod, vert_target);

      DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

      // 计算误差和速度
      float vert_error = abs(dart->vertical_pushrod->measure.total_angle - vert_target);
      float vert_speed_actual = abs(dart->vertical_pushrod->measure.speed_aps);

      // 必须至少运行500ms才允许判断完成
      uint32_t elapsed_time = HAL_GetTick() - vert_back_start_time;

      if (elapsed_time > 1500 &&
          (vert_error < CALI_POS_THRESHOLD ||
           vert_speed_actual < CALI_SPEED_THRESHOLD)) {
        // 完成回退
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        dart->vertical_pushrod->motor_controller.angle_PID.MaxOut = saved_vert_angle_maxout;
        dart->calibration_step = CALI_STEP_HORI_PUSH;
        vert_back_start_time = 0; // 重置计时器
      }
      break;

    // --- 阶段2: 水平推杆归零 ---
    case CALI_STEP_HORI_PUSH:
      dart->horizontal_pushrod->motor_controller.speed_PID.MaxOut = 1000.0f;
      dart->horizontal_pushrod->motor_controller.speed_PID.IntegralLimit = 500.0f;

      DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->horizontal_pushrod, -CALI_SPEED); // 确认方向是否正确

      DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->vertical_pushrod, 0);

      if (IsMotorBlocked(dart->horizontal_pushrod)) {
        // 记录零点
        dart->hori_zero_ecd = dart->horizontal_pushrod->measure.total_angle;

        // 清空 PID 积分，防止回退时暴冲
        dart->horizontal_pushrod->motor_controller.speed_PID.ITerm = 0;
        dart->horizontal_pushrod->motor_controller.speed_PID.Output = 0;
        dart->horizontal_pushrod->motor_controller.speed_PID.Iout = 0;

        dart->calibration_step = CALI_STEP_HORI_BACK;
        // 恢复PID状态
        dart->horizontal_pushrod->motor_controller.speed_PID.MaxOut = 16000.0f;
        dart->horizontal_pushrod->motor_controller.speed_PID.IntegralLimit = 3000.0f;


      }
      break;

    case CALI_STEP_HORI_BACK:
      // 1. 设置水平目标 (保持不变)
      float hori_target = dart->hori_zero_ecd + HORI_BACK_OFFSET;
      DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
      DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_target);

      // 2. 计算“水平推杆”的误差和速度
      float hori_error = abs(dart->horizontal_pushrod->measure.total_angle - hori_target);
      float hori_speed_actual = abs(dart->horizontal_pushrod->measure.speed_aps);

      // 3. 判断水平推杆是否到位
      if (hori_error < CALI_POS_THRESHOLD || hori_speed_actual < CALI_SPEED_THRESHOLD) {
        // 4. 水平校准完成，停转
        DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

        // 5. 垂直推杆也要确保是停的 (保险)
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        dart->hori_zero_ecd = dart->horizontal_pushrod->measure.total_angle;

        // 6. 完成所有校准
        dart->calibration_step = CALI_STEP_DONE;
      }
      break;

    case CALI_STEP_DONE:
      dart->is_calibrated = true;
      // 校准完成，自动切回 Ready 模式
      // 重置角度环的Ref，使其与当前位置一致
      dart->horizontal_pushrod->motor_controller.angle_PID.Ref =
          dart->horizontal_pushrod->measure.total_angle;
      dart->vertical_pushrod->motor_controller.angle_PID.Ref =
          dart->vertical_pushrod->measure.total_angle;
      dart->vertical_pushrod->motor_controller.speed_PID.MaxOut = 15000.0f;
      dart->current_mode = DART_MODE_AUTO_READY;
      break;
  }
}

/* 自动发射模式 - 遥控控制版 */
void DartAutoFireHandler(void) {
  if (rc_data == NULL) return;

  // ========== 【遥控器输入处理】 ==========
  int16_t stick_right_y = rc_data->rc.rocker_r1; // 右摇杆垂直 (控制运行/暂停)
  int16_t stick_left_x  = rc_data->rc.rocker_l_; // 左摇杆水平 (Yaw瞄准)
  int16_t stick_left_y  = rc_data->rc.rocker_l1; // 左摇杆垂直 (摩擦轮微调)

  // 安全检查：必须已校准
  if (!dart->is_calibrated || dart->vert_zero_ecd == 0.0f) {
    // 强制停车
    DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
    DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
    DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
    DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);
    return;
  }

  // 强制使能所有电机
  DJIMotorEnable(dart->vertical_pushrod);
  DJIMotorEnable(dart->horizontal_pushrod);
  if (dart->yaw_motor != NULL) {
    DJIMotorEnable(dart->yaw_motor);
  }

  // --- Yaw 轴瞄准 (始终有效) ---
  if (dart->yaw_motor != NULL) {
    float angle_inc = ((float)stick_left_x / 66.0f) * YAW_SENSITIVITY;
    yaw_current_target -= angle_inc;
    float current_angle = dart->yaw_motor->measure.total_angle;
    float max_lead_angle = 15.0f;
    if (yaw_current_target > current_angle + max_lead_angle) {
      yaw_current_target = current_angle + max_lead_angle;
    }
    else if (yaw_current_target < current_angle - max_lead_angle) {
      yaw_current_target = current_angle - max_lead_angle;
    }
    VAL_LIMIT(yaw_current_target, -5000.0f, 5000.0f);
    DJIMotorOuterLoop(dart->yaw_motor, ANGLE_LOOP);
    DJIMotorSetPIDRef(dart->yaw_motor, yaw_current_target);
  }


  // --- 摩擦轮速度微调 (始终有效) ---
  static uint32_t last_speed_adjust_time = 0;
  if (abs(stick_left_y) > RC_DEADZONE) {
    float speed_delta = ((float)stick_left_y / 660.0f) * FRIC_INCREMENT_SENSITIVITY;
    // 每5ms调整一次，避免变化过快
    if (HAL_GetTick() - last_speed_adjust_time > 5) {
      dart->fric_base_speed += speed_delta;
      VAL_LIMIT(dart->fric_base_speed, FRIC_BASE_MIN_SPEED, FRIC_BASE_MAX_SPEED);
      last_speed_adjust_time = HAL_GetTick();
    }
  }

  // ========== 【关键变量定义】 ==========
  static uint32_t step_start_time = 0;
  static uint32_t pause_timer = 0;
  static uint8_t last_mode = 255;
  static bool step_completed = false; // 作为完成标志以判断能否进入下一个状态
  static bool was_running = false;  // 用于判断是否仍在运行，确保每次运行的时候都开始重新计时
  static bool just_started_running = false; // 是否刚刚开始运行

  // 模式切换时重置状态
  if (last_mode != dart->current_mode) {
    dart->current_step = AUTO_FIRE_INIT;
    step_start_time = 0;
    pause_timer = 0;
    step_completed = false;
    was_running = false;
    just_started_running = false;
    last_mode = dart->current_mode;
  }

  // 如果当前不是自动发射模式，直接返回
  if (dart->current_mode != DART_MODE_AUTO_FIRE) {
    return;
  }

  // 计算目标位置
  float vert_top_target = dart->vert_zero_ecd + CALI_BACK_OFFSET + VERT_FIRE_OFFSET_DEG;
  float vert_bottom_target = dart->vert_zero_ecd + CALI_BACK_OFFSET;
  float hori_right_target = dart->hori_zero_ecd;
  float hori_left_target = dart->hori_zero_ecd + HORI_RELOAD_OFFSET_DEG;

  // --- 遥控器运行/暂停控制 ---
  // 向前推：运行
  // 向后拨：暂停
  // 回中：保持当前状态（不前进也不重置）
  bool should_run = (stick_right_y > RC_DEADZONE);    // 向前推
  bool should_pause = (stick_right_y < -RC_DEADZONE); // 向后拨

  just_started_running = (should_run && !was_running);
  was_running = should_run;

  if (should_run) {
    // ================= 【运行状态】 =================

    // 摩擦轮：全速旋转
    float fric_speed = FRIC_IDLE_SPEED;
    if (dart->current_step == PUSH_UP_RIGHT || dart->current_step == PUSH_UP_LEFT) {
      fric_speed = dart->fric_base_speed;  // 发射速度
    }
    for(int i=0; i<4; i++) {
      DJIMotorOuterLoop(dart->friction_motor[i], SPEED_LOOP);
      DJIMotorSetPIDRef(dart->friction_motor[i], fric_speed);
    }

    // 根据当前步骤执行动作
    switch (dart->current_step) {
      case AUTO_FIRE_INIT:
        step_start_time = HAL_GetTick();
        dart->current_step = PUSH_UP_RIGHT;
        step_completed = false;
        break;

      case PUSH_UP_RIGHT:  // 右上
        if (just_started_running) {
          step_start_time = HAL_GetTick();
          step_completed = false;
        }
        if (!step_completed) {
          DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->vertical_pushrod, vert_top_target);
          DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_right_target);

          bool vertical_in_position = (abs(dart->vertical_pushrod->measure.total_angle - vert_top_target) < 500.0f) &&
                                      (abs(dart->vertical_pushrod->measure.speed_aps) < 50.0f);
          bool horizontal_in_position = (abs(dart->horizontal_pushrod->measure.total_angle - hori_right_target) < 500.0f) &&
                                        (abs(dart->horizontal_pushrod->measure.speed_aps) < 50.0f);
          uint32_t elapsed_time = HAL_GetTick() - step_start_time;

          if (elapsed_time > 200 && vertical_in_position && horizontal_in_position) {
            step_completed = true;
            step_start_time = 0;
            pause_timer = HAL_GetTick();
            dart->current_step = PUSH_UP_RIGHT_PAUSE;
          }
        }
        break;

      case PUSH_UP_RIGHT_PAUSE:  // 右上停
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

        if (pause_timer == 0) pause_timer = HAL_GetTick();
        if (HAL_GetTick() - pause_timer > 1000) {
          step_completed = false;
          step_start_time = HAL_GetTick();
          pause_timer = 0;
          dart->current_step = PUSH_DOWN_RIGHT;
        }
        break;

      case PUSH_DOWN_RIGHT:  // 右下
        if (just_started_running) {
          step_start_time = HAL_GetTick();
          step_completed = false;
        }
        if (!step_completed) {
          DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->vertical_pushrod, vert_bottom_target);
          DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_right_target);

          bool vertical_in_position = (abs(dart->vertical_pushrod->measure.total_angle - vert_bottom_target) < 500.0f) &&
                                      (abs(dart->vertical_pushrod->measure.speed_aps) < 50.0f);
          bool horizontal_in_position = (abs(dart->horizontal_pushrod->measure.total_angle - hori_right_target) < 500.0f) &&
                                        (abs(dart->horizontal_pushrod->measure.speed_aps) < 50.0f);

          uint32_t elapsed_time = HAL_GetTick() - step_start_time;

          if (elapsed_time > 200 && vertical_in_position && horizontal_in_position) {
            step_completed = true;
            step_start_time = 0;
            pause_timer = HAL_GetTick();
            dart->current_step = PUSH_DOWN_RIGHT_PAUSE;
          }
        }
        break;

      case PUSH_DOWN_RIGHT_PAUSE:  // 右下停
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

        if (pause_timer == 0) pause_timer = HAL_GetTick();
        if (HAL_GetTick() - pause_timer > 1000) {
          step_start_time = HAL_GetTick();
          pause_timer = 0;
          step_completed = false;
          dart->current_step = PUSH_LEFT;
        }
        break;

      case PUSH_LEFT:  // 左移
        if (just_started_running) {
          step_start_time = HAL_GetTick();
          step_completed = false;
        }
        if (!step_completed) {
          DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->vertical_pushrod, vert_bottom_target);
          DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_left_target);

          bool vertical_in_position = (abs(dart->vertical_pushrod->measure.total_angle - vert_bottom_target) < 750.0f) &&
                                      (abs(dart->vertical_pushrod->measure.speed_aps) < 50.0f);
          bool horizontal_in_position = (abs(dart->horizontal_pushrod->measure.total_angle - hori_left_target) < 750.0f) &&
                                        (abs(dart->horizontal_pushrod->measure.speed_aps) < 50.0f);
          uint32_t elapsed_time = HAL_GetTick() - step_start_time;

          if (elapsed_time > 200 && vertical_in_position && horizontal_in_position) {
            step_completed = true;
            step_start_time = 0;
            pause_timer = HAL_GetTick();
            dart->current_step = PUSH_LEFT_PAUSE;
          }
        }
        break;

      case PUSH_LEFT_PAUSE:  // 左停
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

        if (pause_timer == 0) pause_timer = HAL_GetTick();
        if (HAL_GetTick() - pause_timer > 1000) {
          pause_timer = 0;
          step_completed = false;
          step_start_time = HAL_GetTick();
          dart->current_step = PUSH_UP_LEFT;
        }
        break;

      case PUSH_UP_LEFT:  // 左上
        if (just_started_running) {
          step_start_time = HAL_GetTick();
          step_completed = false;
        }
        if (!step_completed) {
          DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->vertical_pushrod, vert_top_target);
          DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_left_target);

          bool vertical_in_position = (abs(dart->vertical_pushrod->measure.total_angle - vert_top_target) < 500.0f) &&
                                      (abs(dart->vertical_pushrod->measure.speed_aps) < 50.0f);
          bool horizontal_in_position = (abs(dart->horizontal_pushrod->measure.total_angle - hori_left_target) < 500.0f) &&
                                        (abs(dart->horizontal_pushrod->measure.speed_aps) < 50.0f);

          uint32_t elapsed_time = HAL_GetTick() - step_start_time;

          if (elapsed_time > 200 && vertical_in_position && horizontal_in_position) {
            step_completed = true;
            step_start_time = 0;
            pause_timer = HAL_GetTick();
            dart->current_step = PUSH_UP_LEFT_PAUSE;
          }
        }
        break;

      case PUSH_UP_LEFT_PAUSE:  // 左上停
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

        if (pause_timer == 0) pause_timer = HAL_GetTick();
        if (HAL_GetTick() - pause_timer > 1000) {
          pause_timer = 0;
          step_completed = false;
          step_start_time = HAL_GetTick();
          dart->current_step = PUSH_DOWN_LEFT;
        }
        break;

      case PUSH_DOWN_LEFT:  // 左下
        if (just_started_running) {
          step_start_time = HAL_GetTick();
          step_completed = false;
        }
        if (!step_completed) {
          DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->vertical_pushrod, vert_bottom_target);
          DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_left_target);

          bool vertical_in_position = (abs(dart->vertical_pushrod->measure.total_angle - vert_bottom_target) < 500.0f) &&
                                      (abs(dart->vertical_pushrod->measure.speed_aps) < 50.0f);
          bool horizontal_in_position = (abs(dart->horizontal_pushrod->measure.total_angle - hori_left_target) < 500.0f) &&
                                        (abs(dart->horizontal_pushrod->measure.speed_aps) < 50.0f);

          uint32_t elapsed_time = HAL_GetTick() - step_start_time;

          if (elapsed_time > 200 && vertical_in_position && horizontal_in_position) {
            step_completed = true;
            step_start_time = 0;
            pause_timer = HAL_GetTick();
            dart->current_step = PUSH_DOWN_LEFT_PAUSE;
          }
        }
        break;

      case PUSH_DOWN_LEFT_PAUSE:  // 左下停
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

        if (pause_timer == 0) pause_timer = HAL_GetTick();
        if (HAL_GetTick() - pause_timer > 1000) {
          pause_timer = 0;
          step_completed = false;
          step_start_time = HAL_GetTick();
          dart->current_step = PUSH_RIGHT;
        }
        break;

      case PUSH_RIGHT:  // 右移
        if (just_started_running) {
          step_start_time = HAL_GetTick();
          step_completed = false;
        }
        if (!step_completed) {
          DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->vertical_pushrod, vert_bottom_target);
          DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
          DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_right_target);

          bool vertical_in_position = (abs(dart->vertical_pushrod->measure.total_angle - vert_bottom_target) < 750.0f) &&
                                      (abs(dart->vertical_pushrod->measure.speed_aps) < 50.0f);
          bool horizontal_in_position = (abs(dart->horizontal_pushrod->measure.total_angle - hori_right_target) < 750.0f) &&
                                        (abs(dart->horizontal_pushrod->measure.speed_aps) < 50.0f);

          uint32_t elapsed_time = HAL_GetTick() - step_start_time;

          if (elapsed_time > 200 && vertical_in_position && horizontal_in_position) {
            step_completed = true;
            step_start_time = 0;
            pause_timer = HAL_GetTick();
            dart->current_step = PUSH_RIGHT_PAUSE;
          }
        }
        break;

      case PUSH_RIGHT_PAUSE:  // 右移停
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

        if (pause_timer == 0) pause_timer = HAL_GetTick();
        if (HAL_GetTick() - pause_timer > 1000) {
          dart->current_step = AUTO_FIRE_INIT;
          step_completed = false;
          step_start_time = HAL_GetTick();
          pause_timer = 0;
        }
        break;

      default:
        // 未知状态，重置到初始化
        dart->current_step = AUTO_FIRE_INIT;
        step_start_time = 0;
        pause_timer = 0;
        step_completed = false;
        break;
    }

  } else if (should_pause) {
    // ================= 【暂停状态】 =================
    // 向后拨摇杆时暂停

    // 摩擦轮：怠速
    for(int i=0; i<4; i++) {
      DJIMotorOuterLoop(dart->friction_motor[i], SPEED_LOOP);
      DJIMotorSetPIDRef(dart->friction_motor[i], 1000.0f); // 怠速
    }

    // 推杆：给0速度，停在当前位置
    DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
    DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
    DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
    DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

    // 注意：不重置current_step，这样下次向前推时会从暂停的地方继续

  } else {
    // ================= 【摇杆回中状态】 =================
    // 既不向前也不向后，保持当前状态

    // 摩擦轮：保持当前速度（如果是运行状态）或怠速（如果是暂停状态）
    // 这里我们简单处理为怠速
    for(int i=0; i<4; i++) {
      DJIMotorOuterLoop(dart->friction_motor[i], SPEED_LOOP);
      DJIMotorSetPIDRef(dart->friction_motor[i], 1000.0f); // 怠速
    }

    // 推杆：保持当前位置（给0速度）
    DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
    DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
    DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
    DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);

    // 同样不重置current_step
  }
}

/* 机器人主任务 */
void RobotTask(void) {
  // 1. 刷新状态机
  DartStateMachineUpdate();

  // 2. 执行对应模式的逻辑
  switch (dart->current_mode) {
    case DART_MODE_DEBUG:
      DartDebugModeHandler();
      break;

    case DART_MODE_CALIBRATING:
      DartCalibrationHandler();
      break;

    case DART_MODE_AUTO_READY:
      // 就绪状态：锁死位置，等待发射指令
      // 这里简单处理：给 0 速度保持静止 (或者写位置环锁住当前位置)
      DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
      DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);
      if (dart->yaw_motor != NULL) {
        DJIMotorEnable(dart->yaw_motor);
        // 就绪状态：Yaw轴保持当前位置
        DJIMotorOuterLoop(dart->yaw_motor, ANGLE_LOOP);
        DJIMotorSetPIDRef(dart->yaw_motor, yaw_current_target);
      }
      // 摩擦轮保持怠速或停止
      for(int i=0; i<4; i++) DJIMotorSetPIDRef(dart->friction_motor[i], 0);
      break;

    case DART_MODE_AUTO_FIRE:
      DartAutoFireHandler();
      break;

    case DART_MODE_EMERGENCY_STOP:
      // 急停：所有电机断电 (发送 0 电流)
      DJIMotorStop(dart->yaw_motor);
      DJIMotorStop(dart->vertical_pushrod);
      DJIMotorStop(dart->horizontal_pushrod);
      for(int i=0; i<4; i++) DJIMotorStop(dart->friction_motor[i]);
      break;

    default:
      break;
  }

  // 3. 底层发送：将计算好的 PID 输出发送给电调
  DJIMotorTask();
}