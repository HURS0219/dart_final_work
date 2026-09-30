/*
 * ICM42688driver.c — ICM-42688-P SPI 驱动 (dart 实板, SPI2 / CS=PB12)
 * =============================================================================
 * 仅在 DART_F405_BOARD 下编译; 其它板级为空 TU, 不影响原 BMI088 路径。
 * 量程: Accel ±16g(2048 LSB/g), Gyro ±2000dps(16.4 LSB/dps), ODR 1kHz。
 * =============================================================================
 */
#include "ICM42688driver.h"

#if defined(DART_F405_BOARD)

#include <string.h>

#include "bsp_dwt.h"
#include "spi.h"

IMU_Data_t ICM42688 = {0};

static SPI_HandleTypeDef *s_spi = NULL;
static uint8_t txb[16];
static uint8_t rxb[16];

#define ICM_CS_LOW()  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH() HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)

/* Bank0 寄存器 */
#define ICM_REG_DEVICE_CONFIG 0x11
#define ICM_REG_TEMP_DATA1 0x1D
#define ICM_REG_ACCEL_DATA_X1 0x1F
#define ICM_REG_PWR_MGMT0 0x4E
#define ICM_REG_GYRO_CONFIG0 0x4F
#define ICM_REG_ACCEL_CONFIG0 0x50
#define ICM_REG_GYRO_CONFIG1 0x51
#define ICM_REG_ACCEL_CONFIG1 0x53
#define ICM_REG_WHO_AM_I 0x75

static void ICM_WriteReg(uint8_t reg, uint8_t val) {
  txb[0] = (uint8_t)(reg & 0x7F);
  txb[1] = val;
  ICM_CS_LOW();
  HAL_SPI_Transmit(s_spi, txb, 2, 100);
  ICM_CS_HIGH();
}

static void ICM_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len) {
  memset(txb, 0, len + 1);
  txb[0] = (uint8_t)(reg | 0x80);
  ICM_CS_LOW();
  HAL_SPI_TransmitReceive(s_spi, txb, rxb, len + 1, 100);
  ICM_CS_HIGH();
  memcpy(buf, rxb + 1, len);
}

static uint8_t ICM_ReadReg(uint8_t reg) {
  uint8_t v = 0;
  ICM_ReadRegs(reg, &v, 1);
  return v;
}

uint8_t ICM42688_WhoAmI(void) { return ICM_ReadReg(ICM_REG_WHO_AM_I); }

uint8_t ICM42688_Init(SPI_HandleTypeDef *spi, uint8_t calibrate) {
  (void)calibrate;
  s_spi = spi;

  ICM_CS_HIGH();
  DWT_Delay(0.01f);
  ICM_WriteReg(ICM_REG_DEVICE_CONFIG, 0x01); /* soft reset */
  DWT_Delay(0.05f);

  if (ICM_ReadReg(ICM_REG_WHO_AM_I) != 0x47) return 1;

  ICM_WriteReg(ICM_REG_PWR_MGMT0, 0x0F); /* accel + gyro, Low Noise */
  DWT_Delay(0.05f);
  ICM_WriteReg(ICM_REG_GYRO_CONFIG0, 0x06);  /* FS ±2000dps, ODR 1kHz */
  ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, 0x06); /* FS ±16g,     ODR 1kHz */
  ICM_WriteReg(ICM_REG_GYRO_CONFIG1, 0x16);  /* 滤波器带宽 */
  ICM_WriteReg(ICM_REG_ACCEL_CONFIG1, 0x16);
  DWT_Delay(0.05f);

  ICM42688.gNorm = 9.80665f;
  ICM42688.AccelScale = 1.0f;
  for (int i = 0; i < 3; i++) ICM42688.GyroOffset[i] = 0.0f;
  return ICM42688_NO_ERROR;
}

void ICM42688_Read(IMU_Data_t *d) {
  uint8_t b[12];
  uint8_t t[2];

  ICM_ReadRegs(ICM_REG_ACCEL_DATA_X1, b, 12); /* accel(6) + gyro(6) 连续 */
  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  int16_t gx = (int16_t)((b[6] << 8) | b[7]);
  int16_t gy = (int16_t)((b[8] << 8) | b[9]);
  int16_t gz = (int16_t)((b[10] << 8) | b[11]);

  const float acc_s = 9.80665f / 2048.0f;                   /* ±16g */
  const float gyr_s = (3.14159265358979f / 180.0f) / 16.4f; /* ±2000dps -> rad/s */

  d->Accel[0] = ax * acc_s;
  d->Accel[1] = ay * acc_s;
  d->Accel[2] = az * acc_s;
  d->Gyro[0] = gx * gyr_s - d->GyroOffset[0];
  d->Gyro[1] = gy * gyr_s - d->GyroOffset[1];
  d->Gyro[2] = gz * gyr_s - d->GyroOffset[2];

  ICM_ReadRegs(ICM_REG_TEMP_DATA1, t, 2);
  int16_t tr = (int16_t)((t[0] << 8) | t[1]);
  d->Temperature = tr / 132.48f + 25.0f;
}

#endif /* DART_F405_BOARD */
