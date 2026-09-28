/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2025-12-10
 */
#include "super_cap.h"

#include "bsp_log.h"

#include "memory.h"
#include "stdlib.h"
#include "referee.h"

static SuperCapInstance* super_cap_instance = NULL;  // 可以由app保存此指针
DaemonInstance *supercap_daemon_instance; // daemon实例，用于监测通信状态

/**
 * @brief 超级电容离线回调函数
 *
 * @param instance 超级电容实例指针
 */
static void SuperCapOfflineCallback()
{
  LOGWARNING("[supercap] supercap offline, restart communication.");
  // 可以在这里添加离线处理逻辑，例如禁用超电输出等
}

static void DecodeSuperCap(CANInstance* _instance) {
  uint8_t* rxbuff;
  SuperCap_Measure_s* Msg;
  rxbuff = _instance->rx_buff;
  Msg = &super_cap_instance->cap_msg;

  int16_t cap_v = rxbuff[1] << 8 | rxbuff[0];  // 单位：mV
  Msg->error_detect = rxbuff[2];
  int16_t out_p = rxbuff[4] << 8 | rxbuff[3];  // 除以100以后单位是W
  int16_t in_p = rxbuff[6] << 8 | rxbuff[5];   // 除以100以后单位是W

  Msg->cap_v = (float)cap_v / 1000.0f;
  Msg->out_p = (float)out_p / 100.0f;
  Msg->in_p = (float)in_p / 100.0f;
  // if (Msg->out_p < 0) {
  //   Msg->out_p = 0;
  // }

  // 重载daemon，表示正常接收到数据
  DaemonReload(supercap_daemon_instance);
}

SuperCapInstance* SuperCapInit(SuperCap_Init_Config_s* supercap_config) {
  super_cap_instance = (SuperCapInstance*)malloc(sizeof(SuperCapInstance));
  memset(super_cap_instance, 0, sizeof(SuperCapInstance));

  supercap_config->can_config.can_module_callback = DecodeSuperCap;
  super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
  super_cap_instance->super_cap_mode = SAFETY_MODE;

  Daemon_Init_Config_s daemon_config= {
    .callback = SuperCapOfflineCallback, // 离线时调用的回调函数,会重启串口接收
    .owner_id = NULL,
    .reload_count = 5, // 50ms
  };; // daemon配置
  supercap_daemon_instance = DaemonRegister(&daemon_config);

  return super_cap_instance;
}

void SuperCapSendMessage(SuperCapInstance* instance, int16_t power, uint16_t buffer, uint8_t state) {
  uint8_t tx_data[8] = {0};  // 初始化发送数据

  // 按照原函数的格式填充数据
  memcpy(
      tx_data, &power,
      sizeof(
          power));  // 主控板设定的功率值，就是裁判系统ID_game_robot_state（0x0201）接收到的GameRobotState的chassis_power_limit。
  memcpy(
      tx_data + 4, &buffer,
      sizeof(
          buffer));  // 缓冲能量状态，是裁判系统ID_power_heat_data（0x0202）接收到的ext_power_heat_data_t的buffer_energy
  tx_data[6] =
      state;  // 输出状态（开启/关闭），是裁判系统ID_game_robot_state（0x0201）接收到的GameRobotState的power_management_chassis_output

  // 使用现有的CAN发送机制发送数据
  memcpy(instance->can_ins->tx_buff, tx_data, 8);
  CANTransmit(instance->can_ins, 1);
}
// 去年的超电控制逻辑
//  要开超电的时候底盘的功率就在这个power limit上加需要的功率多出来的50由超电提供。
//  当超电电压低于12时，底盘功率限制不应该超过裁判系统读到的powerlimit值。
//  被动使用模式：去年的逻辑是12V以下就必须等待超电充电，等到充电到18V以上再能打开超电。
//  主动使用模式：power直接给200。
//  超电最大给200W，但正常来说用不到那么大。

uint16_t SuperCapModeControl(SuperCapInstance* super_cap, uint16_t power_limit) {
  SuperCap_Ctrl_Cmd_e ctrl_cmd = super_cap->super_cap_ctrl_cmd;
  if (super_cap == NULL) {
    return power_limit;
  }

  if (super_cap->cap_msg.error_detect != 0) {
    super_cap->super_cap_mode = SAFETY_MODE;
    return power_limit;
  }

  // 状态机，根据命令和电压更新状态，并同时计算对应状态的最大功率
  uint16_t max_power = power_limit;
  switch (super_cap->super_cap_mode) {
    case SAFETY_MODE:
      if (ctrl_cmd == BOOST) {
        super_cap->super_cap_mode = ACTIVE_MODE;
        max_power = 200;
      } else if (super_cap->cap_msg.cap_v > 18.0f) {
        super_cap->super_cap_mode = PASSIVE_MODE;
        max_power = power_limit + 40;
      } else {
        max_power = power_limit;
      }
      break;

    case CHARGING_MODE:
      super_cap->super_cap_ctrl_cmd = NORMAL;
      if (super_cap->cap_msg.cap_v < 10.0f) {
        max_power = (uint16_t)(power_limit * 0.8f);
      } else if (super_cap->cap_msg.cap_v > 18.0f) {
        super_cap->super_cap_mode = PASSIVE_MODE;
        max_power = power_limit + 40;
      } else {
        max_power = power_limit * 0.9f;
      }
      break;

    case PASSIVE_MODE:
      if (ctrl_cmd == BOOST) {
        super_cap->super_cap_mode = ACTIVE_MODE;
        max_power = 180;
      } else if (super_cap->cap_msg.cap_v < 15.0f) {
        super_cap->super_cap_mode = CHARGING_MODE;
        max_power = power_limit - (uint16_t)(power_limit * power_limit * 0.0025f);
      } else if (super_cap->cap_msg.cap_v >= 15.0f && super_cap->cap_msg.cap_v <= 18.0f) {
        super_cap->super_cap_mode = SAFETY_MODE;
        max_power = power_limit;
      } else {
        // 由于上面涵盖了 <12.0f 和 12.0f~18.0f 的情况，此处必然 > 18.0f
        max_power = power_limit + 40;
      }
      break;

    case ACTIVE_MODE:
      if (super_cap->cap_msg.cap_v < 15.0f) {
        super_cap->super_cap_mode = CHARGING_MODE;
        max_power = power_limit * 0.9f;
      } else if (ctrl_cmd == NORMAL) {  // 修复了原代码中 cmd_mode != ACTIVE_MODE 的枚举类型不匹配 Bug
        super_cap->super_cap_mode = PASSIVE_MODE;
        max_power = (super_cap->cap_msg.cap_v > 18.0f) ? (power_limit + 40) : power_limit;
      } else {
        max_power = 200;  // 主动模式放宽到180W
      }
      break;

    default:
      super_cap->super_cap_mode = SAFETY_MODE;
      max_power = power_limit;
      break;
  }
  return max_power;
}