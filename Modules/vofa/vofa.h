#ifndef __VOFA_H
#define __VOFA_H

#include "bsp_usart.h"
#include <stdint.h>

/* 定义最大支持的通道数，防止缓冲区溢出 */
#define VOFA_MAX_CHANNELS 25

/**
 * @brief VOFA初始化函数
 * @param huart 传入要使用的串口句柄（例如 &huart1 或 &huart6）
 *              注意：不要和printf使用的串口冲突！
 */
void VOFAInit(UART_HandleTypeDef *huart);

/**
 * @brief 发送浮点数组到VOFA+上位机 (JustFloat协议)
 * @param data  浮点数组指针
 * @param count 数组中的浮点数个数
 */
void VOFAJustFloatSend(float *data, uint16_t count);

#endif
