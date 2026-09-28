#ifndef ST7735_PORT_H
#define ST7735_PORT_H

/*
 * ST7735 硬件抽象层 (RoboMaster C 型开发板, 8-pin 牛角座扩展口)
 *
 * C 板 8-pin 牛角座线序 (用户手册):
 *   1 SPI2_CS(PB12)  2 GND       3 SPI2_CLK(PB13) 4 3.3V
 *   5 SPI2_MOSI(PB15) 6 I2C2_SCL(PF1) 7 SPI2_MISO(PB14) 8 I2C2_SDA(PF0)
 *
 * 该口没有独立 DC/RST/BLK, 因此借用空闲脚:
 *   MISO  -> DC   (ST7735 只写不读, MISO 用不到)
 *   I2C2  -> RST / BLK (工程里 I2C2/OLED 未使用)
 *
 * 接屏 (按丝印名):
 *   CS->1  GND->2  SCL/SCK->3  VCC->4  SDA/MOSI->5
 *   RST/RES->6  DC/A0->7  BLK/LED->8
 *
 * 如实际接线不同, 只改这里的宏即可 (并保证对应端口时钟在
 * ST7735_GpioInit() 中使能)。
 */

#include "spi.h"
#include "main.h"

/* 使用的 SPI 外设 */
#define ST7735_SPI_HANDLE     hspi2

/* 控制脚 (牛角座: 1=CS, 6=RST, 7=DC, 8=BLK) */
#define ST7735_CS_PORT        GPIOB
#define ST7735_CS_PIN         GPIO_PIN_12
#define ST7735_DC_PORT        GPIOB
#define ST7735_DC_PIN         GPIO_PIN_14   /* 借 SPI2_MISO */
#define ST7735_RST_PORT       GPIOF
#define ST7735_RST_PIN        GPIO_PIN_1    /* 借 I2C2_SCL */
#define ST7735_BLK_PORT       GPIOF
#define ST7735_BLK_PIN        GPIO_PIN_0    /* 借 I2C2_SDA */

/* SPI2 时钟: PCLK1=42MHz, /4 = 10.5MHz (ST7735 安全上限内) */
#define ST7735_SPI_PRESCALER  SPI_BAUDRATEPRESCALER_4

#endif /* ST7735_PORT_H */
