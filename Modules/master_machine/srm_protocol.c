#include "srm_protocol.h"

#include "../../Hardware/stm32-h7/USB_DEVICE/App/usbd_cdc_if.h"
#include "memory.h"

int vision_data_ready_to_send;
short receive_size; //从包首读到的发送过来的数据包长度
short send_size;
short send_id_list[32];
short send_id_num;
char buffer[256];
short buffer_size; //buffer还剩余的字节数量


uint16_t get_srm_protocol_info(uint8_t *rx_buf, Message *receive)
{
  short buf_pos = 0, id = 0;
  char *ptr;

  if (receive == NULL || rx_buf == NULL) {
    return USBD_OK;
  }
  // Step 1: 读取总数据长度
  if (receive_size == 0) {
    memcpy(&receive_size, rx_buf + buf_pos, sizeof(short));
    buf_pos += sizeof(short);
    buffer_size = 0;

    // 增加边界检查，防止 receive_size 过大导致缓冲区溢出
    if (receive_size < 0 || receive_size > (short)sizeof(buffer)) {
      receive_size = 0; // 无效长度，重置状态
      return USBD_OK;   // 丢弃此包以尝试重新同步
    }
  }

  // Step 2: 累积数据到buffer
  short remain_size = receive_size - buffer_size; //若是一个数据被拆分开发，还剩多少字节没有接收
  if (remain_size > 0) {
    if (remain_size > 64 - buf_pos) {
      memcpy(buffer + buffer_size, rx_buf + buf_pos, 64 - buf_pos);
      buffer_size += 64 - buf_pos;
    } else {
      memcpy(buffer + buffer_size, rx_buf + buf_pos, remain_size);
      buffer_size += remain_size;
    }
  }

  // Step 3: 数据未收完则等待
  if (receive_size != buffer_size) {
    return USBD_OK;
  }

  // Step 4: 解析数据
  ptr = buffer; //一个在buffer里往后滑动的指针，来表示读到哪一位了。
  while (ptr < buffer + buffer_size) {
    // 检查剩余长度是否足够读取 id
    if (buffer + buffer_size < ptr + sizeof(short)) {
      break;
    }
    memcpy(&id, ptr, sizeof(short));
    ptr += sizeof(short);

    if (id == -1) {
      receive_size = 0;
      return USBD_OK;
    } else if (id >= 0 && id < 32) {
      if(receive->ptr_list[id] != NULL) {
        // 检查剩余长度是否足够读取数据
        if (buffer + buffer_size - ptr < receive->size_list[id]) {
          break; // 数据不完整，退出解析
        }
        memcpy(receive->ptr_list[id], ptr, receive->size_list[id]);
        ptr += receive->size_list[id];
      }
    }
  }

  receive_size = 0;
  return USBD_OK;
}

uint16_t srm_protocol_pack_send_data(Message *send, uint8_t *tx_buffer, uint16_t *tx_len)
{
  short send_size = 0;
  short id = 0;
  char *ptr = (char*)tx_buffer;

  // 计算总发送数据大小
  for(int i = 0; i < 32; i++) {
    if(send->ptr_list[i] != NULL) {
      send_size += send->size_list[i] + sizeof(short);
    }
  }

  // 打包数据
  memcpy(ptr, &send_size, sizeof(short));
  ptr += sizeof(short);

  for(int i = 0; i < 32; i++) {
    if(send->ptr_list[i] != NULL) {
      id = i;
      memcpy(ptr, &id, sizeof(short));
      ptr += sizeof(short);
      memcpy(ptr, send->ptr_list[i], send->size_list[i]);
      ptr += send->size_list[i];
    }
  }

  *tx_len = sizeof(short) + send_size;
  return USBD_OK;
}