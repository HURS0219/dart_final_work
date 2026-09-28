//
// Created by ASUS on 2025/11/23.
//

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "crc_func.h"
#include "string.h"
#include "bsp_usart.h"
#include "cmsis_os.h"

// 协议帧头定义
#define PROTOCOL_SOF              0xA5
#define PROTOCOL_HEADER_LEN       5
#define PROTOCOL_CRC8_INIT        0xFF
#define PROTOCOL_CRC16_INIT       0xFFFF
#define BUFFER_MAX_SIZE           256
#define NAVIGATOR_RECV_SIZE       64

// 数据包ID定义 (根据文档中的Header字段)
// RoboMaster C型开发板发送的数据包
#define PKT_ID_DEBUG              0x01
#define PKT_ID_IMU                0x02
#define PKT_ID_ROBOT_STATE_INFO   0x03
#define PKT_ID_EVENT              0x04
#define PKT_ID_PID_DEBUG          0x05
#define PKT_ID_ALL_ROBOT_HP       0x06
#define PKT_ID_GAME_STATUS        0x07
#define PKT_ID_ROBOT_MOTION       0x08
#define PKT_ID_GROUND_ROBOT_POS   0x09
#define PKT_ID_RFID_STATUS        0x0A
#define PKT_ID_ROBOT_STATUS       0x0B
#define PKT_ID_JOINT_STATE        0x0C

// MiniPC发送的数据包
#define PKT_ID_ROBOT_CMD          0x01


#pragma pack(push, 1)

typedef struct {
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint16_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} __attribute__((packed)) HeaderFrame;
// ========== RoboMaster C型开发板发送的数据包 ==========

// Debug数据包结构体
typedef struct {
    char name[32];      // 调试数据名称
    uint8_t type;       // 数据类型
    float data;         // 调试数据值
}__attribute__((__packed__)) debug_data_t;


// 机器人状态信息
typedef struct {
    struct {
        uint16_t chassis : 3;
        uint16_t gimbal : 3;
        uint16_t shoot : 3;
        uint16_t arm : 3;
        uint16_t custom_controller : 3;
        uint16_t reserve : 1;
    } type;

    struct {
        uint8_t chassis : 1;
        uint8_t gimbal : 1;
        uint8_t shoot : 1;
        uint8_t arm : 1;
        uint8_t custom_controller : 1;
        uint8_t reserve : 3;
    } state;
} __attribute__((__packed__)) robot_state_info_t;

// 事件数据包 (裁判系统区域占领信息)
typedef  struct {
    uint32_t time_stamp;                    // 时间戳
    uint8_t supply_station_front;           // 己方补给站前补血点占领状态
    uint8_t supply_station_internal;        // 己方补给站内部补血点占领状态
    uint8_t supply_zone;                    // 己方补给区占领状态
    uint8_t center_gain_zone;               // 中心增益点占领情况
    uint8_t small_energy;                   // 己方小能量机关激活状态
    uint8_t big_energy;                     // 己方大能量机关激活状态
    uint8_t circular_highland;              // 己方环形高地占领状态
    uint8_t trapezoidal_highland_3;         // 己方3号梯形高地占领状态
    uint8_t trapezoidal_highland_4;         // 己方4号梯形高地占领状态
    uint8_t base_virtual_shield_remaining;  // 己方基地虚拟护盾剩余值百分比
} __attribute__((__packed__)) event_data_t;

// 所有机器人血量数据包
typedef struct {
  uint16_t ally_1_robot_hp;
  uint16_t ally_2_robot_hp;
  uint16_t ally_3_robot_hp;
  uint16_t ally_4_robot_hp;
  uint16_t reserved;
  uint16_t ally_7_robot_hp;
  uint16_t ally_outpost_hp;
  uint16_t ally_base_hp;
} __attribute__((__packed__)) all_robot_hp_t;

// 游戏状态数据包
typedef struct {
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t sync_time_stamp;
}__attribute__((__packed__)) game_status_t;

// 机器人运动数据
typedef struct {
    float vx;   // x方向速度
    float vy;   // y方向速度
    float wz;   // 转轴角速度
} __attribute__((__packed__)) robot_motion_t;

// 地面机器人位置数据包
typedef struct {
    float hero_x, hero_y;               // 英雄机器人位置
    float engineer_x, engineer_y;       // 工程机器人位置
    float standard_3_x, standard_3_y;   // 3号步兵位置
    float standard_4_x, standard_4_y;   // 4号步兵位置
    float standard_5_x, standard_5_y;   // 5号步兵位置
} __attribute__((__packed__)) ground_robot_position_t;

// RFID状态数据包
typedef struct {
    uint8_t base_gain_point : 1;                    // 己方基地增益点
    uint8_t circular_highland_gain_point : 1;       // 己方环形高地增益点
    uint8_t enemy_circular_highland_gain_point : 1; // 对方环形高地增益点
    uint8_t friendly_r3_b3_gain_point : 1;          // 己方R3/B3梯形高地增益点
    uint8_t enemy_r3_b3_gain_point : 1;             // 对方R3/B3梯形高地增益点
    uint8_t friendly_r4_b4_gain_point : 1;          // 己方R4/B4梯形高地增益点
    uint8_t enemy_r4_b4_gain_point : 1;             // 对方R4/B4梯形高地增益点
    uint8_t energy_mechanism_gain_point : 1;        // 己方能量机关激活点
    uint8_t friendly_fly_ramp_front_gain_point : 1; // 己方飞坡增益点(前)
    uint8_t friendly_fly_ramp_back_gain_point : 1;  // 己方飞坡增益点(后)
    uint8_t enemy_fly_ramp_front_gain_point : 1;    // 对方飞坡增益点(前)
    uint8_t enemy_fly_ramp_back_gain_point : 1;     // 对方飞坡增益点(后)
    uint8_t friendly_outpost_gain_point : 1;        // 己方前哨站增益点
    uint8_t friendly_healing_point : 1;             // 己方补血点
    uint8_t friendly_sentry_patrol_area : 1;        // 己方哨兵巡逻区
    uint8_t enemy_sentry_patrol_area : 1;           // 对方哨兵巡逻区
    uint8_t friendly_big_resource_island : 1;       // 己方大资源岛增益点
    uint8_t enemy_big_resource_island : 1;          // 对方大资源岛增益点
    uint8_t friendly_exchange_area : 1;             // 己方兑换区
    uint8_t center_gain_point : 1;                  // 中心增益点(RMUL适用)
} __attribute__((__packed__)) rfid_status_t;

// 机器人状态数据包 (融合多个数据包)
typedef struct {
    uint8_t robot_id;                           // 本机器人ID
    uint8_t robot_level;                        // 机器人等级
    uint16_t current_hp;                        // 机器人当前血量
    uint16_t maximum_hp;                        // 机器人血量上限
    uint16_t shooter_barrel_cooling_value;      // 枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;         // 枪口热量上限
    uint16_t shooter_17mm_1_barrel_heat;        // 第1个17mm发射机构枪口热量
    float robot_pos_x;                          // 机器人位置x坐标(m)
    float robot_pos_y;                          // 机器人位置y坐标(m)
    float robot_pos_angle;                      // 机器人朝向(度，正北为0)
    uint8_t armor_id;                           // 装甲模块ID
    uint8_t hp_deduction_reason;                // 血量变化类型
    uint16_t projectile_allowance_17mm_1;       // 17mm弹丸剩余发射次数
    uint16_t remaining_gold_coin;               // 剩余金币数量
} __attribute__((__packed__)) robot_status_t;


// 云台状态
typedef struct {
    float pitch;    // 俯仰角
    float yaw;      // 偏航角
} __attribute__((__packed__)) joint_state_t;

typedef struct {
  debug_data_t debug_data;
  robot_state_info_t state_info;
  event_data_t event_data;
  all_robot_hp_t all_robot_hp;
  game_status_t game_status;
  robot_motion_t robot_motion;
  ground_robot_position_t ground_robot_position;
  rfid_status_t rfid_status;
  robot_status_t robot_status;
  joint_state_t joint_state;
} __attribute__((__packed__)) navigator_send_t;

// ========== MiniPC发送的数据包 ==========
// 接收状态枚举
typedef enum {
  RECV_STATE_SOF = 0,
  RECV_STATE_LEN,
  RECV_STATE_ID,
  RECV_STATE_CRC8,
  RECV_STATE_TIMESTAMP,
  RECV_STATE_DATA,
  RECV_STATE_CRC16
} recv_state_t;

// 接收数据结构体
typedef struct {
  // 速度向量
  struct {
    float vx;
    float vy;
    float wz;
  } __attribute__((__packed__)) speed_vector;

//   // 底盘控制
//   struct {
//     float roll;
//     float pitch;
//     float yaw;
//     float leg_length;
//   } __attribute__((__packed__)) chassis;
//
//   // 云台控制
//   struct {
//     float pitch;
//     float yaw;
//   } __attribute__((__packed__)) gimbal;
//
//   // 射击控制
//   struct {
//     uint8_t fire;
//     uint8_t fric_on;
//   } shoot;
} __attribute__((__packed__)) robot_cmd_t;

// 接收数据结构体
typedef struct {
  robot_cmd_t robot_cmd;
  // 接收状态信息
  uint32_t last_update_time;
  uint8_t data_valid;
  uint8_t crc_errors;
} __attribute__((__packed__)) navigator_recv_t;

// ========== 新增需求数据包 (待完成) ==========
// 根据实际需求添加新的数据包结构体
void navigator_send(UART_HandleTypeDef *instance);
navigator_recv_t* navigator_init(UART_HandleTypeDef *usart_handle);

#pragma pack(pop)

#endif