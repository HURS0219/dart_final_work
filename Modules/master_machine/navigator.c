//
// Created by ASUS on 2025/11/23.
//
#include "navigator.h"
#include "bsp_usart.h"
#include "crc_func.h"
#include "bsp_dwt.h"

static navigator_send_t send_data;
static USARTInstance *navigator_usart_instance ;
static navigator_recv_t recv_data;
// 接收状态变量
static recv_state_t recv_state = RECV_STATE_SOF;
static uint8_t recv_buffer[NAVIGATOR_RECV_SIZE];
static uint16_t recv_index = 0;
static uint16_t expected_data_len = 0;
static uint8_t current_packet_id = 0;
static uint32_t last_packet_time = 0;
static uint32_t test_flag=0;
// 缓冲区最大尺寸
#define BUFFER_MAX_SIZE    256
static uint8_t internal_tx_buffer[BUFFER_MAX_SIZE];

static uint8_t* protocol_packed(const uint8_t* data_ptr, uint16_t pack_id, uint16_t data_len, uint8_t data_id, uint8_t* tx_buff, uint16_t* tx_buff_len)
{
  // 1. 参数有效性检查
  if (tx_buff == NULL || tx_buff_len == NULL) {
    return NULL;
  }

  // 2. 计算总帧长并检查缓冲区是否足够
  uint16_t total_frame_len = PROTOCOL_HEADER_LEN + 4 + data_len + 2;
  if (total_frame_len > BUFFER_MAX_SIZE) {
    *tx_buff_len = 0;
    return NULL;
  }

  // 3. 填充帧头
  uint16_t current_index = 0;
  tx_buff[current_index++] = PROTOCOL_SOF;                  // sof
  tx_buff[current_index++] = data_len&0xFF;
  tx_buff[current_index++]=(data_len>>8) &0xFF;
  tx_buff[current_index++] = data_id;                       // 包序号
  tx_buff[current_index++] = get_CRC8_check_sum(&tx_buff[0], 4, PROTOCOL_CRC8_INIT); // crc

  // 4.填充包序号
  tx_buff[current_index++] = (pack_id >> 0)  & 0xFF;
  tx_buff[current_index++] = (pack_id >> 8)  & 0xFF;
  // tx_buff[current_index++] = (time_stamp >> 16) & 0xFF;
  // tx_buff[current_index++] = (time_stamp >> 24) & 0xFF;

  // 5. 填充数据段
  if (data_ptr != NULL && data_len > 0) {
    memcpy(&tx_buff[current_index], data_ptr, data_len);
    current_index += data_len;
  }

  // 6. 计算并填充帧尾CRC16
  uint16_t checksum_len = PROTOCOL_HEADER_LEN + 2 + data_len;
  uint16_t frame_crc16 = get_CRC16_check_sum(&tx_buff[0], checksum_len, PROTOCOL_CRC16_INIT);
  tx_buff[current_index++] = frame_crc16 & 0xFF;
  tx_buff[current_index++] = (frame_crc16 >> 8) & 0xFF;

  // 7. 设置最终帧长并返回
  *tx_buff_len = total_frame_len;
  return tx_buff;
}

uint8_t *protocol_pack(uint16_t pack_id, const uint8_t *data, uint8_t data_len, uint8_t data_id, uint16_t *packed_length) {
  // 调用核心打包函数，使用静态的 internal_tx_buffer
  return protocol_packed(data, pack_id, data_len, data_id, internal_tx_buffer, packed_length);
}

uint8_t protocol_send(UART_HandleTypeDef* huart, uint16_t pack_id, const uint8_t* data_ptr, uint8_t data_len, uint8_t data_id, uint32_t timeout) {
  if (huart == NULL) {
    return 0;
  }

  uint16_t packed_length = 0;
  uint8_t local_buffer[BUFFER_MAX_SIZE];  // 使用局部缓冲区

  // 1. 打包到局部缓冲区
  uint8_t *packed_data = protocol_packed(data_ptr, pack_id, data_len, data_id, local_buffer, &packed_length);

  // 2. 检查打包是否成功
  if (packed_data == NULL || packed_length == 0) {
    return 0;
  }

  // 3. 使用DMA传输局部缓冲区
  HAL_StatusTypeDef hal_status = HAL_UART_Transmit_DMA(huart, packed_data, packed_length);
  if (hal_status != HAL_OK) {
    return 0;
  }

  // 4. 等待DMA传输完成
  uint32_t start_tick = HAL_GetTick();
  while (huart->gState == HAL_UART_STATE_BUSY_TX) {
    if ((HAL_GetTick() - start_tick) > timeout) {
      HAL_UART_DMAStop(huart);
      return 0;
    }
    osDelay(1);
  }

  return 1;
}

// ========== RoboMaster C型开发板发送的数据包发送函数 ==========

/**
 * @brief 发送Debug数据
 * @param huart UART句柄
 * @param name 调试数据名称
 * @param type 数据类型
 * @param data 数据值
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_debug_data(UART_HandleTypeDef* huart, const char* name, uint8_t type, float data)
{
    if (huart == NULL || name == NULL) return 0;

    debug_data_t debug_data;
    memset(&debug_data, 0, sizeof(debug_data));

    // 安全拷贝名称
    strncpy(debug_data.name, name, sizeof(debug_data.name) - 1);
    debug_data.type = type;
    debug_data.data = data;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)&debug_data,
                        sizeof(debug_data),
                        PKT_ID_DEBUG, 10);
}

/**
 * @brief 发送机器人状态信息
 * @param huart UART句柄
 * @param state_info 状态信息结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_robot_state_info(UART_HandleTypeDef* huart, const robot_state_info_t* state_info)
{
    if (huart == NULL || state_info == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)state_info,
                        sizeof(robot_state_info_t),
                        PKT_ID_ROBOT_STATE_INFO, 10);
}

/**
 * @brief 发送事件数据
 * @param huart UART句柄
 * @param event_data 事件数据结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_event_data(UART_HandleTypeDef* huart, const event_data_t* event_data)
{
    if (huart == NULL || event_data == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)event_data,
                        sizeof(event_data_t),
                        PKT_ID_EVENT, 10);
}

/**
 * @brief 发送所有机器人血量数据
 * @param huart UART句柄
 * @param robot_hp 血量数据结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_all_robot_hp(UART_HandleTypeDef* huart, const all_robot_hp_t* robot_hp)
{
    if (huart == NULL || robot_hp == NULL) return 0;

    return protocol_send(huart, 0x0003,
                        (uint8_t*)robot_hp,
                        sizeof(all_robot_hp_t),
                        PKT_ID_ALL_ROBOT_HP, 10);
}

/**
 * @brief 发送游戏状态数据
 * @param huart UART句柄
 * @param game_status 游戏状态结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_game_status(UART_HandleTypeDef* huart, const game_status_t* game_status)
{
    if (huart == NULL||game_status==NULL) return 0;
    return protocol_send(huart, 0x0001,
                        (uint8_t*)game_status,
                        sizeof(game_status_t),
                        PKT_ID_GAME_STATUS, 10);
}

/**
 * @brief 发送机器人运动数据
 * @param huart UART句柄
 * @param vx x方向速度
 * @param vy y方向速度
 * @param wz z轴角速度
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_robot_motion(UART_HandleTypeDef* huart, const robot_motion_t* motion)
{
    if (huart == NULL||motion==NULL) return 0;


    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)motion,
                        sizeof(robot_motion_t),
                        PKT_ID_ROBOT_MOTION, 10);
}

/**
 * @brief 发送地面机器人位置数据
 * @param huart UART句柄
 * @param position 位置数据结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_ground_robot_position(UART_HandleTypeDef* huart, const ground_robot_position_t* position)
{
    if (huart == NULL || position == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)position,
                        sizeof(ground_robot_position_t),
                        PKT_ID_GROUND_ROBOT_POS, 10);
}

/**
 * @brief 发送RFID状态数据
 * @param huart UART句柄
 * @param rfid_status RFID状态结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_rfid_status(UART_HandleTypeDef* huart, const rfid_status_t* rfid_status)
{
    if (huart == NULL || rfid_status == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)rfid_status,
                        sizeof(rfid_status_t),
                        PKT_ID_RFID_STATUS, 10);
}

/**
 * @brief 发送机器人状态数据
 * @param huart UART句柄
 * @param robot_status 机器人状态结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_robot_status(UART_HandleTypeDef* huart, const robot_status_t* robot_status)
{
    if (huart == NULL || robot_status == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)robot_status,
                        sizeof(robot_status_t),
                        PKT_ID_ROBOT_STATUS, 10);
}

/**
 * @brief 发送云台关节状态
 * @param huart UART句柄
 * @param joint_state 关节状态结构体指针
 */
static  uint8_t send_joint_state(UART_HandleTypeDef* huart, const joint_state_t* joint_state)
{
    if (huart == NULL||joint_state==NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)joint_state,
                        sizeof(joint_state_t),
                        PKT_ID_JOINT_STATE, 10);
}



void updata_senddata(void) {
  send_data.game_status.game_type=0x0A;
  send_data.game_status.game_progress=0x0B;
  // send_data.game_status.stage_remain_time=0xAABB;
  send_data.game_status.sync_time_stamp=0xEFEFEFEFEFEFEFEF;
  if (test_flag<40) {
    test_flag++;
    send_data.game_status.stage_remain_time=0x102C;
  }else if (test_flag>=40&&test_flag<=80) {
    send_data.game_status.stage_remain_time=0x011C;
    test_flag++;
  }else if (test_flag>80) {
    test_flag=0;
  }
}

void navigator_send(UART_HandleTypeDef *instance) {
  updata_senddata();
  send_all_robot_hp(instance,&send_data.all_robot_hp);
  // send_event_data(instance,&send_data.event_data);
  //send_game_status(instance,&send_data.game_status);
  // send_ground_robot_position(instance,&send_data.ground_robot_position);
  // send_joint_state(instance,&send_data.joint_state);
  // send_rfid_status(instance,&send_data.rfid_status);
  // send_robot_motion(instance,&send_data.robot_motion);
  // send_robot_state_info(instance,&send_data.state_info);
  // send_robot_status(instance,&send_data.robot_status);

}

static void DecodeNavigator() {
    if (navigator_usart_instance == NULL) {
        return;
    }

    uint8_t* buffer = navigator_usart_instance->recv_buff;
    uint8_t buffer_size = navigator_usart_instance->recv_buff_size;

    // 检查最小长度
    if (buffer_size < PROTOCOL_HEADER_LEN + 4 + 2) { // 帧头 + 时间戳 + CRC16
        return;
    }

    // 查找帧头
    uint16_t index = 0;
    uint16_t processed_len = 0;

    while (index < buffer_size) {
        // 查找SOF
        if (buffer[index] == PROTOCOL_SOF) {
            // 检查剩余长度是否足够
            if (index + PROTOCOL_HEADER_LEN > buffer_size) {
                break;
            }

            // 解析帧头
            HeaderFrame* header = (HeaderFrame*)&buffer[index];

            // 检查数据长度是否合理
            uint16_t total_frame_len = PROTOCOL_HEADER_LEN + 4 + header->len + 2;
            if (index + total_frame_len > buffer_size) {
                index++;
                continue; // 数据不完整，继续查找
            }

            // 校验帧头CRC8
            uint8_t crc8_calc = get_CRC8_check_sum(&buffer[index], 4, PROTOCOL_CRC8_INIT);
            if (crc8_calc != header->crc) {
                index++;
                continue; // CRC校验失败，继续查找
            }

            // 计算数据段CRC16 - 只计算有效数据部分
            uint16_t data_len_for_crc = PROTOCOL_HEADER_LEN + header->len;
            uint16_t crc16_calc = get_CRC16_check_sum(&buffer[index], data_len_for_crc, PROTOCOL_CRC16_INIT);

            // 正确获取接收到的CRC16（使用固定索引，不依赖total_frame_len）
            uint16_t crc16_recv = (buffer[index + data_len_for_crc + 1] << 8) | buffer[index + data_len_for_crc];

            if (crc16_calc != crc16_recv) {
                index++;
                continue; // CRC16校验失败，继续查找
            }

            // 校验通过，处理数据包
            uint16_t data_index = index + PROTOCOL_HEADER_LEN; // 跳过帧头和时间戳
            uint8_t check_len=sizeof(navigator_recv_t)-6;
            // 根据数据包ID进行处理
            switch (header->id) {
                case PKT_ID_ROBOT_CMD: {
                    if (header->len == (check_len)) {
                        memcpy(&recv_data, &buffer[data_index], sizeof(navigator_recv_t));
                    }
                    break;
                }
                default:
                    break;
            }

            // 移动索引到下一帧
            index += total_frame_len;
            processed_len = index;
        } else {
            index++;
        }
    }

    // 清除已处理的数据
    if (processed_len > 0) {
        if (processed_len < buffer_size) {
            uint16_t remaining_len = buffer_size - processed_len;
            memmove(buffer, &buffer[processed_len], remaining_len);
            memset(&buffer[remaining_len], 0, buffer_size - remaining_len);
        } else {
            memset(buffer, 0, buffer_size);
        }
    }
}

navigator_recv_t* navigator_init(UART_HandleTypeDef *usart_handle) {
  USART_Init_Config_s conf;
  conf.module_callback = DecodeNavigator;
  conf.recv_buff_size = NAVIGATOR_RECV_SIZE;
  conf.usart_handle =usart_handle;
  navigator_usart_instance = USARTRegister(&conf);


  return &recv_data;
}