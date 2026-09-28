/**
******************************************************************************
* @file    robot.c
* @author  Shiyu Li
* @date    2026/3/28
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief  wheel_leg->prostrate
******************************************************************************
* @attention
* None
*
******************************************************************************
*/

#include "robot.h"

#include "robot_config.h"

// bsp
#include "bsp_gpio.h"
// modules
#include "can_comm.h"
#include "general_def.h"
#include "master_process.h"
#include "user_lib.h"
#include "vofa.h"
// robot components
#include "VT13.h"
#include "ctrl.h"
#include "ui.h"

static RobotInstance* robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd;
static Vision_Receive_s* vision_recv_data;

// vofa数据
float visualized_data[20];

void VOFATask() {
#if defined(GIMBAL_BOARD)
  // visualized_data[0] = robot->gimbal->gimbal_IMU_data->Yaw;
  // visualized_data[1] = robot->gimbal->gimbal_IMU_data->Pitch;
  // visualized_data[2] = robot->shoot->friction_motor[0]->motor_controller.pid_ref;
  // visualized_data[3] = robot->shoot->friction_motor[0]->measure.speed_aps;
  // visualized_data[4] = robot->shoot->friction_motor[1]->motor_controller.pid_ref;
  // visualized_data[5] = robot->shoot->friction_motor[1]->measure.speed_aps;
  // visualized_data[6] = robot->shoot->shoot_ctrl_cmd.initial_speed;
  // visualized_data[7] = robot->shoot->loader_motor->measure.total_angle;
  // visualized_data[8] = robot->gimbal->pitch_motor->motor_controller.pid_ref;
  // visualized_data[9] = robot->gimbal->yaw_motor->motor_controller.pid_ref;
  // visualized_data[10] = robot->gimbal->pitch_motor->motor_controller.final_output;
  // visualized_data[11] = shoot_init_config.shoot_param.shooter_barrel_cooling_value;
  // visualized_data[0] = shoot_ctrl_cmd->shooter_barrel_heat;
  // visualized_data[1] = robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
  // visualized_data[0] = robot->chassis->chassis_ctrl_cmd.target_yaw;
  // visualized_data[1] = robot->chassis->yaw_prostrate_PID.Measure;
  // visualized_data[2] = robot->chassis->yaw_prostrate_PID.Ref;
  // visualized_data[3] = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
  // visualized_data[4] = robot->offset_angle;
  // visualized_data[5] = YAW_ALIGN_ANGLE;
  // visualized_data[6] = robot->gimbal->yaw_motor->measure.angle_single_round;
  visualized_data[0] = robot->shoot->loader_motor->motor_controller.pid_ref;
  visualized_data[1] = robot->shoot->loader_motor->measure.total_angle;
  visualized_data[2] = robot->shoot->loader_motor->motor_controller.angle_PID.Output;
  visualized_data[3] = robot->shoot->loader_motor->motor_controller.angle_PID.MaxOut;
  visualized_data[4] = robot->shoot->loader_motor->motor_controller.speed_PID.Output;
  visualized_data[5] = robot->shoot->loader_motor->motor_controller.speed_PID.MaxOut;
#elif defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  visualized_data[0] = robot->chassis->super_cap->cap_msg.cap_v;
  visualized_data[1] = robot->chassis->super_cap->cap_msg.in_p;
  visualized_data[2] = robot->chassis->super_cap->cap_msg.out_p;
  visualized_data[3] = robot->chassis->chassis_ctrl_cmd.max_power;
  visualized_data[4] = robot->chassis->super_cap->super_cap_ctrl_cmd;

#endif
  VOFAJustFloatSend(visualized_data, 20);
}

// 双板通信
#if !defined(ONE_BOARD)
static Chassis_Upload_Data_s* chassis_upload_data;
static Chassis_Fetch_Data_s* chassis_fetch_data;

static void DoubleBoardCommsInit() {
#if defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  shoot_ctrl_cmd->heat_mode = REFEREE_CONTROL;

  // 视觉接收初始化
  vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);
  robot->vision_recv_data = vision_recv_data;  // Assign to robot instance for access in ctrl module

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;

  robot->chassis = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));
  robot->chassis->imu = (INS_t*)zmalloc(sizeof(INS_t));
  robot->chassis->super_cap = (SuperCapInstance*)zmalloc(sizeof(SuperCapInstance));
  robot->can_comm = CANCommInit(&gimbal_comm_conf);
  // VOFAInit(&huart1);
#endif
#if defined(CHASSIS_BOARD)
  chassis_ctrl_cmd->max_power = robot->referee_data->GameRobotState.chassis_power_limit;

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;
  robot->can_comm = CANCommInit(&chassis_comm_conf);  // can comm初始化
  VOFAInit(&huart1);
#endif
}

static void DoubleBoardComms() {
#if defined(GIMBAL_BOARD)
  static SuperCap_Ctrl_Cmd_e last_local_cmd = NORMAL;
  static float last_change_time = 0;

  // 检测云台板本地（如按键）是否修改了超电状态
  if (robot->chassis->super_cap->super_cap_ctrl_cmd != last_local_cmd) {
    last_change_time = DWT_GetTimeline_ms();
    last_local_cmd = robot->chassis->super_cap->super_cap_ctrl_cmd;
  }

  // 发送底盘控制指令
  chassis_fetch_data->chassis_ctrl_cmd = *chassis_ctrl_cmd;
  chassis_fetch_data->super_cap_ctrl_cmd = robot->chassis->super_cap->super_cap_ctrl_cmd;
  
  // 接收底盘回传数据
  *chassis_upload_data = *(Chassis_Upload_Data_s*)CANCommGet(robot->can_comm);
  robot->chassis->imu->Pitch = chassis_upload_data->Pitch;
  robot->chassis->imu->YawTotalAngle = chassis_upload_data->YawTotalAngle;
  robot->chassis->imu->Gyro[2] = chassis_upload_data->yaw_speed;
  shoot_ctrl_cmd->initial_speed = chassis_upload_data->bullet_speed;
  shoot_ctrl_cmd->shooter_barrel_heat = chassis_upload_data->shooter_17mm_barrel_heat;
  shoot_ctrl_cmd->shooter_barrel_heat_limit = chassis_upload_data->shoot_heat_limit;
  VisionSetRefereeData(chassis_upload_data->bullet_speed, chassis_upload_data->robot_id);
  
  // 延迟 500ms 接收底盘的覆盖（防止云台板刚按下的指令被底盘延迟发来的旧状态吃掉）
  if (DWT_GetTimeline_ms() - last_change_time > 500.0f) {
    robot->chassis->super_cap->super_cap_ctrl_cmd = chassis_upload_data->super_cap_ctrl_cmd;
    last_local_cmd = chassis_upload_data->super_cap_ctrl_cmd;
  }

  CANCommSend(robot->can_comm, (void*)chassis_fetch_data);
  // 重置标志位，避免重复发送
  chassis_fetch_data->force_refresh_ui = 0;
#elif defined(CHASSIS_BOARD)
  // 发送底盘回传数据
  chassis_upload_data->Pitch = robot->chassis->imu->Pitch;
  chassis_upload_data->YawTotalAngle = robot->chassis->imu->YawTotalAngle;
  chassis_upload_data->yaw_speed = robot->chassis->imu->Gyro[2];
  chassis_upload_data->bullet_speed = robot->referee_data->ShootData.initial_speed;
  chassis_upload_data->robot_id = robot->referee_data->GameRobotState.robot_id;
  chassis_upload_data->shooter_17mm_barrel_heat = robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
  chassis_upload_data->shoot_heat_limit = robot->referee_data->GameRobotState.shooter_barrel_heat_limit;
  chassis_upload_data->super_cap_ctrl_cmd = robot->chassis->super_cap->super_cap_ctrl_cmd;
  // 接收底盘控制指令
  *chassis_fetch_data = *(Chassis_Fetch_Data_s*)CANCommGet(robot->can_comm);
  robot->chassis->chassis_ctrl_cmd = chassis_fetch_data->chassis_ctrl_cmd;
  robot->chassis->super_cap->super_cap_ctrl_cmd = chassis_fetch_data->super_cap_ctrl_cmd;
  
  // 处理云台板传来的UI刷新标志
  if (chassis_fetch_data->force_refresh_ui) {
    Referee_Interactive_info_t* ui_data = getUI();
    if (ui_data != NULL) {
      ui_data->force_refresh_ui = 1;
    }
  }

  CANCommSend(robot->can_comm, (void*)chassis_upload_data);
#endif
}
#endif

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 * @note 单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle() {
  float angle = robot->gimbal->yaw_motor->measure.angle_single_round;
#if YAW_CHASSIS_ALIGN_ECD > 4096  // 如果大于180度
  if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
  else
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
#else  // 小于180度
  if (angle > YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else
    robot->offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 云台对齐底盘正方向
 * @note 每次进入 recovery 时，覆盖 gimbal yaw 指令将云台转向底盘正前方。
 * 对齐完成前持续强制 CHASSIS_RECOVERY；对齐后放行，恢复正常控制。
 * 必须在 CalcOffsetAngle() 之后调用。
 */
static void GimbalAlignToChassisForward(void) {}

/**
 * @brief 机器人核心控制任务，200 Hz 频率运行（必须高于视觉发送频率）
 * @note  该函数在 RobotTask 中被循环调用，负责解析输入、生成控制指令并协调各模块
 */
void RobotCMDTask() {
// 控制板指令处理与控制逻辑 (只有底盘板不用处理)
#if !defined(CHASSIS_BOARD)
  JoyStickCtrl(robot);
  MouseKeyCtrl(robot);
#if defined(GIMBAL_BOARD)
  CalcOffsetAngle();
  // GimbalAlignToChassisForward();
  gimbal_ctrl_cmd->chassis_rotate_wz = -0.11f * robot->chassis->imu->Gyro[2];
#endif
  EmergencyHandler(robot);  // 急停必须在 CAN 发送之前,确保 POWER_OFF 优先级最高
#endif

// 双板通信
#if !defined(ONE_BOARD)
  static float last_comm_time = 0.0f;
  if (DWT_GetTimeline_ms() - last_comm_time >= 10.f) {
    last_comm_time = DWT_GetTimeline_ms();
    DoubleBoardComms();
  }
#endif
}

void RobotInit() {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
#if !defined(CHASSIS_BOARD)

  // 遥控器初始化
#if defined(STM32F4)
#ifdef USE_RC_CTRL
  robot->rc_data = RemoteControlInit(&huart3);
#elifdef USE_OCD_CTRL
  robot->rc_data = VT13RemoteInit(&huart1);
#endif

#elif defined(STM32H7)
  robot->rc_data = RemoteControlInit(&huart5);
#endif
#endif
#if !defined(GIMBAL_BOARD)                     // PC15 引脚初始化并注册为 GPIO 输出
  robot->referee_data = RefereeInit(&huart7);  // 裁判系统初始化
  robot->chassis = ChassisInit(&chassis_init_config);
#endif
#if !defined(ONE_BOARD)
  DoubleBoardCommsInit();
#endif
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  DWT_GetDeltaT(&robot->DWT_CNT);
#if !defined(GIMBAL_BOARD)
  // UI初始化
  // MyUIInit(robot);
#endif
  chassis_ctrl_cmd->max_power = 80.0f;
}

void RobotTask() {
  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  RobotCMDTask();
  VOFATask();
#if !defined(CHASSIS_BOARD)
  GimbalTask();
  ShootTask();
  VisionSend();
#endif

#if !defined(GIMBAL_BOARD)
  ChassisTask();
  // UITask(robot);
#endif
}
