# dart_ctrl on F405 实板 — 未完成 / 待定细节 (以后再调)

> **状态**: 板子尚未投板/打印。以下均为**代码级已完成、但未上电验证**, 或**参数/协议未定**,
> 待实板到手或联调时处理。引脚网表见同目录 `HARDWARE.md`。
>
> **当前分支**: `final`（干净库基线）。

---

## 1. 上电/联调验证清单 (板子到手后按序做)
1. **烧录**: 按实板板级目标烧录（F405 板级尚未加入本仓库，需先移植，见 §3）。
2. **RTT 启动检查**: 无 HardFault; 打印 `[dart_ctrl] INS init OK`; 静止时 `roll/pitch ≈ 0`。
3. **回环自检**(分别验证, 拆线后再交叉接):
   - USART2(PA2/PA3) 短接 → 视觉帧回读计数增长;
   - SPI1(PA4-7) 由 H7 做主机发帧 → F405 从机能收。
4. **舵机**: TIM4(PB6/PB7/PB8/PB9) 输出 50Hz, 量中位脉宽/方向。

## 2. 未标定 / 未确认 (需实板)
### IMU — ICM-42688-P (SPI2: PB13/PB14/PB15, CS=PB12)
- 本仓库默认用 **BMI088**；实板为 **ICM-42688-P**，需移植/接入对应驱动后端（源 `dart_final` 仓库有
  `Modules/imu/ICM42688driver.*` 与 `ins_task.c` 的 IMU 后端抽象 `IMU_DEV/IMU_Read/IMU_Init`）。
- **轴/符号/安装方向未知**: 现按默认 X/Y/Z 直出，需按丝印/机体系标定（`IMU_Init_Config_s` 或驱动内轴映射）。
- 陀螺零偏、量程、ODR、SPI 模式需与实器件核对。

### 视觉 — 双链路 (SPI1 从机 + USART2)
- **帧格式**: 现按仓库 7 字节 `AA 55 X_hi X_lo Y_hi Y_lo CRC8` 解析；**必须与 H7(视觉 MCU)端固件约定**，
  不同则改 `robot.c` 的 `Vision_RxCallback`。
- **SPI1**: F405 从机 / H7 主；软件 NSS(SSI=0)、CPOL/CPHA、位序、速率需与 H7 一致。
- **双链路策略**: 目前仅实现 USART2(串口) 一路；SPI1 备用链路为**待办**。

### ESP 链路 — USART1 (备用 USART6)
- 本 app **无 ESP 通信/控制**；仅 `robot_config.h` 占位端口。
- 波特 115200 占位；协议未定。

### 舵机 — TIM4 CH1-4 = PB6/PB7/PB8/PB9
- **中位/方向/行程/限幅需台架标定** → 改 `robot_config.h` 的 `DART_SERVO_SCALE/TRIM/REVERSE` 与
  `SERVO_CENTER_US/HALF_US/HALF_DEG/PULSE_MIN/MAX_US`。
- PWM: 50Hz, 1µs 分辨; 脉宽范围按 PTK7350 规格（center=1500us, half=1000us, half_deg=139.5°）。

### USB — PA11/PA12 → CH334P HUB → Type-C
- 与飞行控制无关；用途待定。

### 其它
- USART3(PB10/PB11): 留空。
- SPI3 / W25Q128 (PA15 + PC10-12): 不接, 未用。

## 3. 计划中但尚未做
- **F405 板级移植**：本仓库 `Hardware/` 目前只有 `stm32-f4`(F407 C 板) 与 `stm32-h7`(H723)；
  实板 F405RGT6 需新增板级目录 + CubeMX 工程 + 链接/启动 + MCU 宏（`STM32F405xx`）。
- **ICM-42688-P 驱动后端**接入 `ins_task`。
- **视觉 SPI1 从机**备用链路。
- **ESP 无线烧录**（三方案待试）。
- **roll 控制方案**最终确定；**速率环**（`GUID_CTRL_MODE`）预留未实现。
- **黑匣子飞行日志**（内存环形 + 赛后导出）。
- 视觉与 IMU **时间对齐**。

## 4. 复现 / 烧录（模板，待板级确定）
```powershell
cmake -G Ninja -S . -B build -DROBOT_TYPE=dart_ctrl -DMCU_TYPE=stm32-f4 -DbuildType=Debug
cmake --build build
# 烧录: 用 J-Link(Ozone / JLink) 或 CLion 的 OpenOCD 配置，指向 build/control-2026.elf
```

## 5. 本 app 相关文件
- 应用: `UserApp/robot/dart_ctrl/{robot.c, robot.h, robot_config.h, robot.cmake, README.md, HARDWARE.md, LOG.md, F405_TODO.md}`
- 依赖模块(本仓库): `Modules/imu/ins_task.*`、`Modules/motor/servo_motor/`、`Modules/algorithm/controller/`、
  `Modules/algorithm/crc8/`、`Bsp/usart/`、`Bsp/pwm/`、`Bsp/log/`、`Bsp/dwt/`
