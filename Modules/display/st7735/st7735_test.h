#ifndef ST7735_TEST_H
#define ST7735_TEST_H

/* ST7735 bring-up 测试 (临时)
 *
 * 上电后在 1.8" 128x160 屏上循环显示: 纯色块 -> 彩条 -> 文字,
 * 用于确认 SPI2 通路、接线和供电是否正常。
 *
 * 正式使用时: 删除 st7735_test.c/.h, 并去掉 motor_task.c 中的调用。
 * 仅在 STM32F407 (GIMBAL_BOARD) 生效, 其它板子为空实现。 */

void ST7735TestInit(void);
void ST7735TestTask(void);

#endif /* ST7735_TEST_H */
