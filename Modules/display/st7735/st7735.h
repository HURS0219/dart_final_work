#ifndef ST7735_H
#define ST7735_H

#include <stdint.h>

/* 面板参数: 1.8" 128x160 RGB565 */
#define ST7735_WIDTH   128
#define ST7735_HEIGHT  160

/* 显存偏移, 不同 tab 的 128x160 面板一般用 0/0 */
#define ST7735_X_OFFSET 0
#define ST7735_Y_OFFSET 0

/* 扫描方向: 竖屏, BGR, 从左到右/从上到下 */
#define ST7735_MADCTL 0xC8

/* RGB565 常用颜色 */
#define ST7735_BLACK   0x0000
#define ST7735_WHITE   0xFFFF
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_BLUE    0x001F
#define ST7735_YELLOW  0xFFE0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_ORANGE  0xFD20
#define ST7735_GRAY    0x8410

/* 字符尺寸: 复用 oledfont 的 6x12 ASCII 字库 */
#define ST7735_CHAR_W  6
#define ST7735_CHAR_H  12

/* 初始化 (配置 GPIO / 提速 SPI2 / 复位 / 上电序列 / 清屏) */
void ST7735_Init(void);

/* 背光开关 */
void ST7735_SetBacklight(uint8_t on);

/* 设置绘制窗口 (含偏移, 内部使用) */
void ST7735_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/* 基础绘制 */
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* 把一块 w*h 的 RGB565 像素流到 (x,y) (行优先) */
void ST7735_Blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *colors);

/* 6x12 ASCII 文本 */
void ST7735_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg);
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg);

#endif /* ST7735_H */
