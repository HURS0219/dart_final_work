# dart_final on F405 实板 — 未完成 / 待定细节 (以后再调)

> ⚠ **ESP32 相关已移除**：`dart_final_1/esp32/`（含 `esp_u0_link` C 工程）及其它 app 的
> `esp32/` 已删除。网页/上位机统一用 **`Tools/dart_launcher_web_pc/`**（PC + J-Link RTT）。

> **状态**: 板子尚未投板/打印。以下均为**代码级已完成、但未上电验证**, 或**参数/协议未定**,
> 待实板到手或联调时处理。引脚网表见同目录 `HARDWARE.md`。
> 本次改动**尚未 git commit**。

---

## 1. 上电/联调验证清单 (板子到手后按序做)
1. **烧录**: `Tools\scripts\oneclick_flash.ps1 -Robot dart_final -Board DART_F405_BOARD` (J-Link 器件 `STM32F405RG`)。
2. **RTT 启动检查**: 无 HardFault; 打印 `[imu] INS init OK`; 静止时 `roll/pitch ≈ 0`; `[vision] ... armed`。
3. **回环自检**(分别验证, 拆线后再交叉接):
   - USART2(PA2/PA3) 短接 → 视觉帧回读计数增长;
   - SPI1(PA4-7) 由 H7 做主机发帧 → F405 从机能收。
4. **舵机**: TIM4(PB6/PB7/PB8/PB9) 输出 50Hz, 量中位脉宽/方向。

## 2. 未标定 / 未确认 (需实板)
### IMU — ICM-42688-P (SPI2: PB13/PB14/PB15, CS=PB12)
- **轴/符号/安装方向未知**: 现按默认 X/Y/Z 直出。需按丝印/机体系标定 → 在 `UserApp/robot/dart_final/app/imu.c`
  的 `IMU_Init_Config` 设 `scale/Yaw/Pitch/Roll`, 或在 `ICM42688_Read` 做轴映射。
- `gNorm=9.80665`(→`AccelScale≈1`)、量程 ±16g / ±2000dps、ODR 1kHz、SPI mode3(CPOL=1,CPHA=2edge) —— 均为默认值, 需与实器件核对。
- 陀螺零偏: 上电静态标定 5000 次(≈5s); 可改 `ins_task.c` 的 `INS_CalibrateGyroForDebug(5000)`。
- 中断 `PB0(INT1)/PB1(INT2)` 已在 CubeMX 配 EXTI, 但**驱动目前用轮询, 未用中断**。

### 视觉 — 双链路 (SPI1 从机 + USART2)
- **帧格式未定**: 现两条链路都按仓库 7 字节 `AA 55 X_hi X_lo Y_hi Y_lo CRC8` 解析。
  **必须与 H7(视觉 MCU)端固件约定**; 若不同 → 改 `app/vision.c` 的解码。
- **SPI1**: F405 从机 / H7 主; 软件 NSS(SSI=0)、CPOL=0、CPHA=1edge(SPI mode0)、位序 MSB、速率待定。
  需与 H7 的 SPI 模式/CS 极性/时钟完全一致。
- **USART2**: 115200 8N1(可改), 帧格式同上。
- **双链路策略**: 谁先解出整帧谁更新(无优先级/仲裁)。

### ESP 链路 — USART1 (备用 USART6)
- `dart_final` **目前没有 ESP 通信 app**; 仅在 `dart_final_1` 里有 `esp_link` 测试(含 `esp32/esp_u0_link` C 工程)。
- 波特 115200 占位; 协议未定(参考 `dart_fc` 的 `PING/PONG/F` 行协议)。

### 舵机 — TIM4 CH1-4 = PB6/PB7/PB8/PB9
- **中位/方向/行程/限幅需台架标定** → `Modules/algorithm/servo_mix_ai/servo_mix_ai_cfg.h`(SCALE/TRIM/REVERSE)
  与 `Modules/motor/servo_motor/servo_motor_cfg.h`(脉宽规格)。
- PWM: 50Hz, Prescaler 83, Period 19999(1µs 分辨); 脉宽范围按 PTK7350 规格填。

### USB — PA11/PA12 → CH334P HUB → Type-C
- 已生成 USB‑FS + CDC(虚拟串口); **是否启用/用途待定**。

### 其它
- USART3(PB10/PB11): **留空**。
- SPI3 / W25Q128 (PA15 + PC10-12): **不接, 未用**。

## 3. 代码级已知项 / 风险
- **CubeMX 生成代码 + 手工补丁混合**: USART1 及 `USART1_RX`/`USART2_RX` 的 DMA/NVIC 是我在**生成代码里补的**
  (`usart.c/.h`、`dma.c`、`stm32f4xx_it.c`、`main.c`)。`.ioc` 也已补回 USART1, 但**若在 CubeMX 重新 Generate, 需再核对/重打补丁**。
- **编译裁剪**: 根 `CMakeLists.txt` 新增可选 `BOARD_EXCLUDE`; `Hardware/stm32-f405-dart/stm32-f405-dart.cmake`
  排除 `Bsp/can,Bsp/iic` 与多个未用 Modules(alarm/can_comm/display/oled/referee/remote/super_cap/master_machine/TFmini/vofa/VT13/motor 除 servo_motor 外)。
  **本板新增模块时注意此裁剪**。
- **共享层 F405 分支**(改了他处, 均按 `DART_F405_BOARD` 隔离):
  - `Bsp/bsp_init.h`: F405 → `DWT_Init(168)`;
  - `UserApp/os_task.c`: 跳过 buzzer/motor_task/DMmotor 调用;
  - `Modules/algorithm/servo_mix_ai/servo_mix_ai_cfg.h`: F405 → `SERVO_MIX_TIM=&htim4`。
- **`Modules/imu/ins_task.c`**: 新增 IMU 后端抽象 `IMU_DEV/IMU_Read()/IMU_Init()/IMU_NO_ERROR`;
  F405→ICM(SPI2), 其它板→BMI088(原样)。并**跳过实板的恒温等待**(否则死等)。
- **`Bsp/usb`**: 用仓库自定义 `usbd_cdc_if.{c,h}` 覆盖了 CubeMX 生成版。
- **ICM 驱动**: `Modules/imu/ICM42688driver.{c,h}`, 仅 `DART_F405_BOARD` 编译(其它板为空 TU)。
- **启动耗时**: IMU 初始化约 1s + 零偏标定 ≈5s → **约 6s**, 需缩短请调标定次数。
- **`dart_final_1` 沙盒**: 仍在 main(含 `esp_link` / `TX/RX/scan` 测试); 之前计划“转分支并从 main 移除”**未做**。
- **ESP‑IDF 未安装**: ESP 端 `esp32/esp_u0_link/` C 工程已写好但未编译/烧录(esp_link 实测搁置)。

## 4. 计划中但尚未做
- `bsp_flash` 掉电参数保存(本板 SPI3 W25Q128 未接)。
- 视觉 **H7 端发送固件**的约定与适配。
- `SMC/ADRC/roll_dec` 实验分支的台架整定(与实板无关, 同样未验证)。
- `dart_final_1` → 实验分支。
- ESP 无线烧录。

## 5. 复现 / 烧录
```powershell
# 编译
powershell -File make_one\build.ps1 -Robot dart_final -Board DART_F405_BOARD
# 烧录 (J-Link, 器件 STM32F405RG)
powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final -Board DART_F405_BOARD
```

## 6. 本板相关文件
- 板级: `Hardware/stm32-f405-dart/`(`dart_f405.ioc`、`stm32-f405-dart.cmake`、`Src/`、`Inc/` — 含生成代码+手补)
- 应用: `UserApp/robot/dart_final/{HARDWARE.md, F405_TODO.md, dart_final_cfg.h, robot.cmake, app/imu.c, app/vision.c}`
- 模块: `Modules/imu/ICM42688driver.{c,h}`、`Modules/imu/ins_task.c`
- 构建/共享: `CMakeLists.txt`、`make_one/build.ps1`、`Tools/scripts/oneclick_flash.ps1`、
  `Bsp/bsp_init.h`、`UserApp/os_task.c`、`Modules/algorithm/servo_mix_ai/servo_mix_ai_cfg.h`
