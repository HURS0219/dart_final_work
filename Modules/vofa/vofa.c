#include "vofa.h"

#include "string.h"

static USARTInstance *vofa_usart_instance = NULL;

/*
 * 发送缓冲区
 * 大小 = (最大通道数 * 4字节) + 4字节帧尾
 * 必须定义为 static 或全局变量，保证 DMA 发送期间内存有效
 */
static uint8_t vofa_tx_buffer[VOFA_MAX_CHANNELS * 4 + 4];
/**
 * @brief VOFA 初始化
 *        利用 bsp_usart 的 USARTRegister 接口进行注册
 */

void VOFAInit(UART_HandleTypeDef *huart) {
  USART_Init_Config_s config;

  // 1. 配置串口句柄
  config.usart_handle = huart;

  // 2. 配置接收相关
  config.module_callback = NULL;  // 不需要回调

  // 这里的 size 只要小于结构体里定义的 256 即可
  // bsp_usart 会使用结构体自带的 recv_buff[256] 中的前 10 个字节作为缓冲区
  config.recv_buff_size = 25;

  // 3. 注册实例
  vofa_usart_instance = USARTRegister(&config);
}

/**
 * @brief 发送 JustFloat 协议数据
 */
void VOFAJustFloatSend(float *data, uint16_t count) {
  // 1. 检查是否初始化且未超限
  if (vofa_usart_instance == NULL) return;
  if (count > VOFA_MAX_CHANNELS) count = VOFA_MAX_CHANNELS;

  // JustFloat 协议帧尾: 0x00, 0x00, 0x80, 0x7f
  static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

  // 2. 填充数据 (使用 memcpy 直接复制内存，最高效)
  // 复制浮点数据
  memcpy(vofa_tx_buffer, data, count * sizeof(float));
  // 复制帧尾
  memcpy(vofa_tx_buffer + (count * sizeof(float)), tail, 4);

  // 3. 计算总长度
  uint16_t total_len = (count * sizeof(float)) + 4;

  // 4. 调用 BSP 发送接口
  // 使用 DMA 模式发送，不阻塞 CPU
  USARTSend(vofa_usart_instance, vofa_tx_buffer, total_len, USART_TRANSFER_DMA);
}
