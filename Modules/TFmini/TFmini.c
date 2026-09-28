/**
******************************************************************************
* @file    TFmini.c
* @author  Enhao Zhang
* @date    2026/03/24
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief TFmini lidar module
******************************************************************************
* @attention
* None
*
******************************************************************************
*/
#include "TFmini.h"

#include <stdlib.h>
#include <string.h>

/**
 * @brief 读取TFmini传感器数据 (被bsp_usart的回调触发)
 */
static void TFminiRxCallback(TFminiInstance* _instance) {
  if (_instance == NULL || _instance->usart_ins == NULL) {
    return;
  }

  uint8_t* data = _instance->usart_ins->recv_buff;

  // 1. 校验帧头 0x59 0x59
  if (data[0] != 0x59 || data[1] != 0x59) {
    return;
  }

  // 2. 校验和 (Sum Check): 0~7字节相加求和后的最低字节
  uint8_t sum_check = 0;
  for (int i = 0; i < 8; i++) {
    sum_check += data[i];
  }

  if (sum_check != data[8]) {
    return;  // 校验失败
  }

  // 3. 解析数据 (小端模式: 低字节在前，高字节在后)
  _instance->data.dist = data[2] | (data[3] << 8);
  _instance->data.strength = data[4] | (data[5] << 8);
  
  // 官方温度换算公式: Temp = (raw_temp / 8) - 256
  int16_t raw_temp = data[6] | (data[7] << 8);
  _instance->data.temp = (raw_temp / 8) - 256;
}

/**
 * @brief 初始化TFmini
 *
 * @param huart 绑定的串口句柄
 * @return TFminiInstance* TFmini实例指针
 */
TFminiInstance* TFminiInit(UART_HandleTypeDef* huart) {
  TFminiInstance* instance = (TFminiInstance*)malloc(sizeof(TFminiInstance));
  memset(instance, 0, sizeof(TFminiInstance));
  USART_Init_Config_s config;

  config.recv_buff_size = BUFF_SIZE;
  config.usart_handle = huart;
  config.module_callback = TFminiRxCallback;

  instance->usart_ins = USARTRegister(&config);

  return instance;
}
