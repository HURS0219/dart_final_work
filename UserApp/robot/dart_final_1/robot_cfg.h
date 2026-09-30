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
/* 1=视觉走 SPI2(从机; OpenMV 做主机); 0=走 USART(3线串口) */
/* H743 板无 USART3/6(仅 USART1/5/7), 故 H743 上视觉走 SPI 分支并关闭,
 * 使 vision.c 不引用 huart3/huart6; F407 板沿用原串口方案。 */
#if defined(STM32H743xx)
#define DF1_VISION_ENABLE 0
#define DF1_VISION_SPI 1
#else
#define DF1_VISION_ENABLE 1
#define DF1_VISION_SPI 0
#endif

/* SPI 诊断: 1=把 PB13 配成 EXTI 统计 SCK 时钟数(临时, 不启用 SPI); 0=正常 SPI 从机 */
#define DF1_SPI_SCKDIAG 0

/* 诊断: 1=在 USART1/3/6 同时统计收到字节数(定位板子 3pin 到底接哪路); 0=正常视觉 */
#define DF1_UART_SCAN 0

/* 视觉串口选择(非扫描模式): 1=USART6(PG14/PG9, 板子3pin); 0=USART3(PC10/PC11) */
#define DF1_VISION_UART6 1

/* ===================== ESP32 链路 app (测试用) =====================
 * 检测 743 板(USART1) 与 ESP32(UART0) 的 UART 通信是否成功。协议同 dart_fc:
 * MCU 周期发 "PING", ESP 回 "PONG"; ESP 周期发心跳 "H"。
 * ESP=UART0 -> STM32 USART1(PA9 TX/PA10 RX), 115200 8N1。 */
#define DF1_ESP_ENABLE 1
#define DF1_ESP_UART (&huart1)
#define DF1_ESP_RX_SIZE 64
#define DF1_ESP_PERIOD_MS 500u
#define DF1_ESP_TIMEOUT_MS 1500u
