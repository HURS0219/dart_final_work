#pragma once

/*
 * robot_cfg.h — dart_final_1 (dart_final 的沙盒测试版) 专用开关
 * =============================================================================
 * 仅 dart_final_1 使用; 生产版 dart_final 不受影响。
 * =============================================================================
 */

/* 1=初始化真实 BMI088(C 板板载, hspi1); 0=跳过(用 ATT 注入姿态) */
#define DF1_IMU_ENABLE 1

/* 1=启用 vision app; 0=不启用(用 TGT 注入目标) */
#define DF1_VISION_ENABLE 1

/* 1=视觉走 SPI2(从机; OpenMV 做主机); 0=走 USART3(3线串口; OpenMV 做发送方) */
#define DF1_VISION_SPI 0
