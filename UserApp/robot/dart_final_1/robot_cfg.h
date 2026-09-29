#pragma once

/*
 * robot_cfg.h — dart_final_1 (dart_final 的沙盒测试版) 专用开关
 * =============================================================================
 * 仅 dart_final_1 使用; 生产版 dart_final 不受影响。
 * =============================================================================
 */

/* 1=初始化真实 BMI088; 0=跳过(测 SPI/时钟时避免 IMU 卡死干扰) */
#define DF1_IMU_ENABLE 0

/* 1=启用 vision app; 0=不启用(用 TGT 注入目标) */
#define DF1_VISION_ENABLE 1

/* 1=视觉走 SPI2(从机; OpenMV 做主机); 0=走 USART(3线串口) */
#define DF1_VISION_SPI 0

/* SPI 诊断: 1=把 PB13 配成 EXTI 统计 SCK 时钟数(临时, 不启用 SPI); 0=正常 SPI 从机 */
#define DF1_SPI_SCKDIAG 0

/* 诊断: 1=在 USART1/3/6 同时统计收到字节数(定位板子 3pin 到底接哪路); 0=正常视觉 */
#define DF1_UART_SCAN 0

/* 视觉串口选择(非扫描模式): 1=USART6(PG14/PG9, 板子3pin); 0=USART3(PC10/PC11) */
#define DF1_VISION_UART6 1
