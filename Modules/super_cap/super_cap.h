/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"
#include "daemon.h"

// 超级电容策略结构体
typedef enum {
  SAFETY_MODE = 0,       // 安全模式，超电电压低于8伏时进入，大于18伏退出，底盘限制30W
  PASSIVE_MODE,          // 被动模式，超电电压正常时的工作模式
  ACTIVE_MODE,           // ，主动模式，主动使用超电能量
  CHARGING_MODE,         // 充电模式，衰减底盘功率，保障电容电压健康
} SuperCap_Mode_e;

typedef enum {
  NORMAL = 0,  // 正常模式，不使用超电能量
  BOOST = 1,       // 超电模式，主动使用超电能量
} SuperCap_Ctrl_Cmd_e;

typedef struct {
  float cap_v;
  uint8_t error_detect;
  float out_p;  // 除以100以后单位是W
  float in_p;   // 除以100以后单位是W
} SuperCap_Measure_s;

/* 超级电容实例 */
typedef struct {
  CANInstance* can_ins;            // CAN实例
  SuperCap_Ctrl_Cmd_e super_cap_ctrl_cmd;    // 期望的超电模式
  SuperCap_Mode_e super_cap_mode;  // 超级电容模式
  SuperCap_Measure_s cap_msg;      // 超级电容信息
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct {
  CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 *
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance* SuperCapInit(SuperCap_Init_Config_s* supercap_config);

/**
 * @brief 超级电容模式控制与功率限制计算
 * @param super_cap 超级电容实例
 * @param cmd_mode 期望的超电模式
 * @param power_limit 当前裁判系统给出的功率限制
 * @return 计算得到的底盘最大功率
 */
uint16_t SuperCapModeControl(SuperCapInstance* super_cap, uint16_t power_limit);

/**
 * @brief 发送超级电容控制消息
 *
 * @param instance 超级电容实例
 * @param power 功率值
 * @param buffer 缓冲区值
 * @param state 状态值
 */
void SuperCapSendMessage(SuperCapInstance* instance, int16_t power, uint16_t buffer, uint8_t state);
#endif  // !SUPER_CAP_H