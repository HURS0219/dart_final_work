//
// Created by ASUS on 2025/11/23.
//

#ifndef CONTROL_2026_CRC_FUNC_H
#define CONTROL_2026_CRC_FUNC_H
#include "main.h"

#define TRUE 1
#define FALSE 0

/**
 * @brief          计算CRC8
 * @param[in]      pch_message: 数据
 * @param[in]      dw_length: 数据和校验的长度
 * @param[in]      ucCRC8:初始CRC8
 * @retval         计算完的CRC8
 */
uint8_t get_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8);


/**
 * @brief          CRC8校验函数
 * @param[in]      pch_message: 数据
 * @param[in]      dw_length: 数据和校验的长度
 * @retval         真或者假
 */
uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
 * @brief          添加CRC8到数据的结尾
 * @param[in]      pch_message: 数据
 * @param[in]      dw_length: 数据和校验的长度
 * @retval         none
 */
void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
 * @brief          计算CRC16
 * @param[in]      pch_message: 数据
 * @param[in]      dw_length: 数据和校验的长度
 * @param[in]      wCRC:初始CRC16
 * @retval         计算完的CRC16
 */
uint16_t get_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

/**
 * @brief          CRC16校验函数
 * @param[in]      pch_message: 数据
 * @param[in]      dw_length: 数据和校验的长度
 * @retval         真或者假
 */
uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief          添加CRC16到数据的结尾
 * @param[in]      pch_message: 数据
 * @param[in]      dw_length: 数据和校验的长度
 * @retval         none
 */
void append_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
#endif  // CONTROL_2026_CRC_FUNC_H
