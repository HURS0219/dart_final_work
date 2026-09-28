#ifndef CONTROL_2026_F4_SRM_PROTOCOL_H
#define CONTROL_2026_F4_SRM_PROTOCOL_H
#define PROTOCOL_CMD_ID 0XA5
#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

#include <stdint.h>
#include <stdio.h>

/// @brief 发弹云台接收数据结构体


typedef struct
{
  void *ptr_list[32];  ///< 数据包指针
  short size_list[32]; ///< 数据包大小
} Message;

/*接收数据处理*/
uint16_t get_srm_protocol_info(uint8_t *rx_buf, Message *receive);


// 发送打包函数声明
uint16_t srm_protocol_pack_send_data(Message *send, uint8_t *tx_buffer, uint16_t *tx_len);

#endif  // CONTROL_2026_F4_SRM_PROTOCOL_H
