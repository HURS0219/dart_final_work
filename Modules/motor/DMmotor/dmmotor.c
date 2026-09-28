#include "dmmotor.h"

#include "bsp_log.h"
#include "cmsis_os.h"
#include "daemon.h"
#include "general_def.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"

static uint8_t idx;
static DMMotorInstance* dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance* motor) {
  memset(motor->motor_can_instance->tx_buff, 0xff, 7);   // 发送电机指令的时候前面7bytes都是0xff
  motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd;  // 最后一位是命令id
  CANTransmit(motor->motor_can_instance, 1);
}

static void DMMotorDecode(CANInstance* motor_can) {
  uint16_t tmp;  // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
  uint8_t* rxbuff = motor_can->rx_buff;
  DMMotorInstance* motor = (DMMotorInstance*)motor_can->id;
  DM_Motor_Measure_s* measure = &(motor->measure);  // 将can实例中保存的id转换成电机实例的指针
  Motor_Control_Setting_s* motor_setting = &(motor->motor_settings);
  motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

  // 先保存当前位置作为上一次位置
  measure->last_position = measure->position;
  switch (motor->motor_type) {
    case J4310:
      // 然后更新当前位置
      tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
      measure->position = uint_to_float(tmp, DM_P_MIN_J4310, DM_P_MAX_J4310, 16);

      tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
      measure->velocity = uint_to_float(tmp, DM_V_MIN_J4310, DM_V_MAX_J4310, 12);

      tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
      measure->torque = uint_to_float(tmp, DM_T_MIN_J4310, DM_T_MAX_J4310, 12);
      break;
    case H6215:
      // 然后更新当前位置
      tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
      measure->position = uint_to_float(tmp, DM_P_MIN_H6215, DM_P_MAX_H6215, 16);

      tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
      measure->velocity = uint_to_float(tmp, DM_V_MIN_H6215, DM_V_MAX_H6215, 12);

      tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
      measure->torque = uint_to_float(tmp, DM_T_MIN_H6215, DM_T_MAX_H6215, 12);
      break;
    case J8009P:
      // 然后更新当前位置
      tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
      measure->position = uint_to_float(tmp, DM_P_MIN_J8009P, DM_P_MAX_J8009P, 16);

      tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
      measure->velocity = uint_to_float(tmp, DM_V_MIN_J8009P, DM_V_MAX_J8009P, 12);

      tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
      measure->torque = uint_to_float(tmp, DM_T_MIN_J8009P, DM_T_MAX_J8009P, 12);
      break;
    default:
      break;
  }
  measure->id = rxbuff[0] & 0x0F;            // 低4位是电机ID
  measure->state = (rxbuff[0] >> 4) & 0x0F;  // 高4位是错误码

  measure->T_Mos = (float)rxbuff[6];
  measure->T_Rotor = (float)rxbuff[7];

  if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
    measure->position = -measure->position;
    measure->velocity = -measure->velocity;
    measure->torque = -measure->torque;
    measure->total_angle = -measure->total_angle;
  }
  // 多圈角度计算,前提是假设两次采样间电机转过的角度小于12.5弧度
  // DM电机的position范围是-12.5到12.5弧度，跳变点在12.5和-12.5之间
  if (measure->position - measure->last_position > 12.5f)  // 从负值(-12.5)变成正值(12.5)，电机逆向旋转过边界
    measure->total_round--;
  else if (measure->position - measure->last_position < -12.5f)  // 从正值(12.5)变成负值(-12.5)，电机正向旋转过边界
    measure->total_round++;
  measure->total_angle = measure->total_round * 2.0f * 12.5f + measure->position;

  if (measure->state != 0) {
    DaemonReload(motor->daemon);
  }
}

// todo: 会跟控制抢，有概率控不了电机
static void DMMotorLostCallback(void* motor_ptr) {
  DMMotorSetMode(DM_CMD_MOTOR_MODE, motor_ptr);
  osDelay(100);
  // DWT_Delay(0.0001f);
}

void DMMotorCaliEncoder(DMMotorInstance* motor) {
  DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
  DWT_Delay(0.3);
}

DMMotorInstance* DMMotorInit(Motor_Init_Config_s* config) {
  DMMotorInstance* motor = (DMMotorInstance*)malloc(sizeof(DMMotorInstance));
  memset(motor, 0, sizeof(DMMotorInstance));
  motor->motor_type = config->motor_type;
  motor->motor_settings = config->controller_setting_init_config;

  PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
  PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
  PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
  motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
  motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

  config->can_init_config.can_module_callback = DMMotorDecode;
  config->can_init_config.id = motor;
  motor->motor_can_instance = CANRegister(&config->can_init_config);

  // todo: 有shit，开了之后有时候电机控不了
  Daemon_Init_Config_s conf = {
      .callback = DMMotorLostCallback,
      .owner_id = motor,
      .reload_count = 10,
  };
  motor->daemon = DaemonRegister(&conf);

  DMMotorEnable(motor);
  DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
  DWT_Delay(0.1);
  dm_motor_instance[idx++] = motor;
  return motor;
}

void DMMotorSetRef(DMMotorInstance* motor, float ref) { motor->motor_controller.final_output = ref; }

void DMMotorEnable(DMMotorInstance* motor) { motor->stop_flag = MOTOR_ENALBED; }

void DMMotorStop(DMMotorInstance* motor)  // 不使用使能模式是因为需要收到反馈
{
  motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance* motor, Closeloop_Type_e type) { motor->motor_settings.outer_loop_type = type; }

void DMMotorSetPIDRef(DMMotorInstance* motor, float ref) {
  // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
  motor->motor_controller.pid_ref = ref;
  Motor_Control_Setting_s* motor_setting;  // 电机控制参数
  Motor_Controller_s* motor_controller;    // 电机控制器
  DM_Motor_Measure_s* measure;             // 电机测量值
  float pid_measure, pid_ref;              // 电机PID测量值和设定值

  motor_setting = &motor->motor_settings;
  motor_controller = &motor->motor_controller;
  measure = &motor->measure;
  pid_ref = motor_controller->pid_ref;  // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改

  // pid_ref会顺次通过被启用的闭环充当数据的载体
  // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
  if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP) {
    if (motor_setting->angle_feedback_source == OTHER_FEED)
      pid_measure = *motor_controller->other_angle_feedback_ptr;
    else
      pid_measure = measure->total_angle;  // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
    // 更新pid_ref进入下一个环
    pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
  }

  // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
  if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
    if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD) pid_ref += *motor_controller->speed_feedforward_ptr;

    if (motor_setting->speed_feedback_source == OTHER_FEED)
      pid_measure = *motor_controller->other_speed_feedback_ptr;
    else  // MOTOR_FEED
      pid_measure = measure->velocity;
    // 更新pid_ref进入下一个环
    pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
  }

  // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
  if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD) pid_ref += *motor_controller->current_feedforward_ptr;
  if (motor_setting->close_loop_type & CURRENT_LOOP) {
    pid_ref = PIDCalculate(&motor_controller->current_PID, measure->torque, pid_ref);
  }

  // 获取最终输出
  motor->motor_controller.final_output = pid_ref;
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
__attribute__((noreturn)) void DMMotorTask(void const* argument) {
  float set;
  DMMotorInstance* motor = (DMMotorInstance*)argument;
  Motor_Control_Setting_s* setting = &motor->motor_settings;
  DM_Motor_Send_s motor_send_mailbox;

  while (1) {
    set = motor->motor_controller.final_output;

    if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) set *= -1;
    switch (motor->motor_type) {
      case J4310:
        LIMIT_MIN_MAX(set, DM_T_MIN_J4310, DM_T_MAX_J4310);
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN_J4310, DM_P_MAX_J4310, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN_J4310, DM_V_MAX_J4310, 12);
        motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN_J4310, DM_T_MAX_J4310, 12);
        if (motor->stop_flag == MOTOR_STOP)
          motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN_J4310, DM_T_MAX_J4310, 12);
        break;
      case H6215:
        LIMIT_MIN_MAX(set, DM_T_MIN_H6215, DM_T_MAX_H6215);
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN_H6215, DM_P_MAX_H6215, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN_H6215, DM_V_MAX_H6215, 12);
        motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN_H6215, DM_T_MAX_H6215, 12);
        if (motor->stop_flag == MOTOR_STOP)
          motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN_H6215, DM_T_MAX_H6215, 12);
        break;
      case J8009P:
        LIMIT_MIN_MAX(set, DM_T_MIN_J8009P, DM_T_MAX_J8009P);
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN_J8009P, DM_P_MAX_J8009P, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN_J8009P, DM_V_MAX_J8009P, 12);
        motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN_J8009P, DM_T_MAX_J8009P, 12);
        if (motor->stop_flag == MOTOR_STOP)
          motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN_J8009P, DM_T_MAX_J8009P, 12);
        break;
      default:
        break;
    }
    motor_send_mailbox.Kp = 0;
    motor_send_mailbox.Kd = 0;

    motor->motor_can_instance->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
    motor->motor_can_instance->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
    motor->motor_can_instance->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
    motor->motor_can_instance->tx_buff[3] =
        (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
    motor->motor_can_instance->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
    motor->motor_can_instance->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
    motor->motor_can_instance->tx_buff[6] =
        (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
    motor->motor_can_instance->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
    CANTransmit(motor->motor_can_instance, 1);
    osDelay(1);
  }
}

void DMMotorTaskInit() {
  char dm_task_name[5] = "dm";
  // 遍历所有电机实例,创建任务
  if (!idx) return;
  for (size_t i = 0; i < idx; i++) {
    char dm_id_buff[2] = {0};
    __itoa(i, dm_id_buff, 10);
    strcat(dm_task_name, dm_id_buff);
    osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 64);
    dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
  }
}
