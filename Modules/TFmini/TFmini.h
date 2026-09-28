/**
******************************************************************************
* @file    TFmini.h
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

#pragma once
#include "bsp_usart.h"

#define BUFF_SIZE 9u

#pragma pack(1)
typedef struct {
  uint16_t dist;      // 距离
  uint16_t strength;  // 强度
  int16_t temp;       // 温度 (可能为负，改为有符号)
} TFmini_Data_s;
#pragma pack()

/* TFmini实例 */
typedef struct {
  USARTInstance* usart_ins;  // USART实例
  TFmini_Data_s data;        // TFmini信息
} TFminiInstance;

/**
 * @brief 初始化TFmini
 *
 * @param tfmini_config TFmini初始化配置
 * @return TFminiInstance* TFmini实例指针
 */
TFminiInstance* TFminiInit(UART_HandleTypeDef* huart);