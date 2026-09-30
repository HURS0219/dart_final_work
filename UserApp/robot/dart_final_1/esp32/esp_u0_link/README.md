# esp_u0_link — ESP32-S3 <-> STM32H743 UART0 链路测试 (ESP-IDF)

用 ESP32-S3-DevKitC-1 的 **UART0（J3: TX=GPIO43 / RX=GPIO44）** 与 STM32H743 的
**USART1（PA9=TX / PA10=RX）** 做串口互通/自测。

## 协议（同 dart_fc 行协议）
- STM32 每 500ms 发 `PING\n` → ESP 回 `PONG\n`
- ESP 每 500ms 发心跳 `H\n`
- STM32 遥测帧 `F,...`（ESP 打印日志，忽略内容）

## 构建 / 烧录（在 ESP-IDF 环境）
```
idf.py set-target esp32s3
idf.py -p COMx flash monitor
```
- **控制台**：`sdkconfig.defaults` 已把 console 设为 **USB Serial/JTAG**，请用板子的
  **native USB 口**看日志；这样 **UART0(GPIO43/44) 才空闲**。
- **测试时务必拔掉 USB-to-UART 数据线**（板载桥接芯片也挂在 GPIO43/44 上，会争线）。

## 接线（ESP32-S3 <-> H743）
| ESP32-S3 | H743 |
|---|---|
| GPIO43 (U0TXD) | PA10 (USART1_RX) |
| GPIO44 (U0RXD) | PA9  (USART1_TX) |
| GND | GND |

## 独立回环自测
1. **ESP 自环**：用跳线短接 **GPIO43 ↔ GPIO44**，把 `main/esp_u0_link.c` 里
   `#define ESP_LINK_LOOPBACK 1` 重新编译烧录，看日志 `LOOPBACK PASS`。
2. **743 自环**：用跳线短接 H743 的 **PA9 ↔ PA10**，抓 RTT 应看到 `rxB` 增长、`last='PING'`。
3. 两项都 PASS 后，按上表交叉接线，正常模式固件，抓 RTT 期望 `esp ok=1 pong>=1`。
