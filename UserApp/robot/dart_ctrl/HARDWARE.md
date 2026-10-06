# 制导飞镖 控制板 (Board2) 硬件接口 — dart_ctrl

> 来源: EasyEDA Pro 工程 `ProPrj_飞镖配套硬件0926` 的 PCB 网表(`.epro` 内 `PCB/*.epcb` 的 `PAD_NET`)。
> 这些引脚是**画板人用铜箔连死的**, 固件必须按其配置; 不是软件自选。
> 本文件只记录本飞控用得到的 MCU 接口。
>
> ⚠ **现状**: 该实板(F405) **尚未投板**; 本 app 的端口/定时器在 `robot_config.h` 里
>   先按 C 板(F407) **占位**。上板后按本表改 `robot_config.h`。

## Board2 控制板器件(位号)
| 位号 | 器件 | 说明 |
|---|---|---|
| U4 | **STM32F405RGT6** (LQFP-64) | 主飞控 (dart_ctrl 运行 MCU) |
| U2 | STM32H743VIT6 | 板载视觉 MCU(OpenMV 式): DCMI 摄像头(FPC2) + SD(TF1) + USB + RGB |
| U5 | **ICM-42688-P** | 6 轴 IMU |
| U6 | W25Q128JVSIQ | SPI Flash (当前不用) |
| U1 | TLV62569 | 降压 |
| U3 / U12 | ZX-PM1.27-1-14PU-Z | 14P 排针 → ESP 板(Board1_1) |
| U60 | CH334P | USB 2.0 HUB |
| USB1 | USB_TYPE-C-16P | Type-C |

> 视觉"OpenMV"就是板上 **U2(H743)**, 不是独立模块; 摄像头走 U2 的 DCMI。

## UART — STM32F405RGT6 (U4)
| 外设 | TX | RX | 引脚(TX/RX) | 连到 |
|---|---|---|---|---|
| USART1 | PA9 | PA10 | 42 / 43 | 排针 U12 → ESP(Board1_1) |
| USART2 | PA2 | PA3 | 16 / 17 | H743 视觉 U2 (H7 USART2, PD5/PD6) |
| USART6 | PC6 | PC7 | 37 / 38 | 排针 U3 → ESP(Board1_1) |
| USART3 | PB10 | PB11 | 29 / 30 | 留空(未再引出) |

## SPI — STM32F405RGT6 (U4)
| 外设 | NSS/CS | SCK | MISO | MOSI | 引脚 | 器件 |
|---|---|---|---|---|---|---|
| SPI1 | PA4 | PA5 | PA6 | PA7 | 20/21/22/23 | → H743 视觉 U2 的 HSPI3(经 33Ω 串阻) |
| SPI2 | PB12 | PB13 | PB14 | PB15 | 33/34/35/36 | ICM-42688-P (U5) |
| SPI3 | PA15 | PC10 | PC11 | PC12 | 50/51/52/53 | W25Q128 (U6) — 不用 |

## 定时器 / 其它
| 功能 | 引脚 | 备注 |
|---|---|---|
| 舵机 PWM ×4 | **PB6/PB7/PB8/PB9** | = TIM4_CH1/2/3/4 (LD/RD/RU/LUservo) |
| IMU 中断 | PB0(INT1) / PB1(INT2) | EXTI |
| USB OTG_FS | PA11(DM) / PA12(DP) | → U60 CH334P HUB → Type-C |
| SWD | PA13(SWDIO) / PA14(SWCLK) | 调试 |
| CAN | — | 本板不使用 |

## 链路拓扑(固件视角)
- **视觉(主链路, 两路都用)**
  - SPI1: F405(从) ↔ H743 HSPI3(主) —— 视觉坐标帧
  - USART2: F405 ↔ H743 —— 辅助/配置/备份
- **ESP**: USART1(PA9/PA10) 主; USART6(PC6/PC7) 亦引到 ESP 板(备用)
- **IMU**: SPI2 (ICM-42688-P)
- **调试**: RTT(J-Link) + SWD(PA13/PA14)

## 时钟
- HSE = X1 (X50328MSB2GI, 8MHz) → 目标 SYSCLK 168MHz。

## 上板后需在 `robot_config.h` 改的项
1. `DART_USART_OPENMV`：C 板 `huart3` → 实板应按 **USART2(`huart2`)** 或 SPI1(从) 双链路（见 `F405_TODO.md`）。
2. `DART_USART_ESP32`：→ **USART1(`huart1`)**。
3. `DART_SERVO_TIM`：C 板 `htim1`(PE9/11/13/14) → 实板 **TIM4(`htim4`, PB6/7/8/9)**。
4. IMU：实板为 **ICM-42688-P@SPI2**，与本仓库默认 BMI088 不同（需对应驱动/后端，见 `F405_TODO.md`）。
