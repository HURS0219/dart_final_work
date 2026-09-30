/*
 * ICM42688driver.h — ICM-42688-P (dart 实板 IMU, SPI2) 驱动
 * =============================================================================
 * 与 BMI088driver 接口对齐, 复用 IMU_Data_t; 由 ins_task (DART_F405_BOARD 分支) 调用。
 * 硬件: SPI2 (PB13 SCK / PB14 MISO / PB15 MOSI), CS = PB12 (IMU_CS, 低有效)。
 * 输出: Accel[m/s^2] / Gyro[rad/s] / Temperature[degC]。
 * =============================================================================
 */
#ifndef ICM42688DRIVER_H
#define ICM42688DRIVER_H

#include "BMI088driver.h" /* 复用 IMU_Data_t */
#include "main.h"

#define ICM42688_NO_ERROR 0x00

extern IMU_Data_t ICM42688;

/* 初始化; 成功返回 ICM42688_NO_ERROR。calibrate 参数保持与 BMI088Init 一致(此处忽略) */
uint8_t ICM42688_Init(SPI_HandleTypeDef *spi, uint8_t calibrate);

/* 读取一次 accel/gyro/temp 到结构体(会扣除 GyroOffset) */
void ICM42688_Read(IMU_Data_t *icm);

/* 读 WHO_AM_I (应为 0x47) */
uint8_t ICM42688_WhoAmI(void);

#endif /* ICM42688DRIVER_H */
