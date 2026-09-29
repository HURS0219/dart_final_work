# CONTEXT — 压缩记忆（工程规范 + 飞镖 userapp 调试）

> 本文件是项目“压缩上下文”，供 AI/队员快速上手。详细规范见 `Tools/AGENTS.md`、`Tools/agent_profile.md`。

## 工程规范（务必遵守）
- **铁律**：禁改 `Bsp/`/`Modules/`/`Hardware/`（除非用户明确授权，如已授权 `message_center`、`servo_motor`、`openmv/uart_ai`）；app 平行 pub-sub、无相互 include、无全局；git 提交做版本管理、禁整文件夹复制；源码统一 UTF-8。
- **允许**：自建算法模块放 `Modules/algorithm/<算法名>_ai/`（如 `servo_mix_ai`/`png_ai`/`smc_ai`/`adrc_ai`）。
- **Module 风格**（仿 `dji_motor`）：doxygen 文件头；聚合 `*_Init_Config_s`；`static` 实例数组 + `idx`；复用 `bsp_*/controller/daemon`。
- **App 解耦**：`robot.c` 只编排；业务拆平行 app；经 `message_center` 话题通信。
- **命名**：模块函数 `PascalCase`；宏全大写；类型 `_s/_e`。头文件用 `#pragma once`（勿再写 `#endif`）。
- **注释**：每函数 doxygen + 关键步骤行内；cfg 宏逐项注明“单位/含义/取值/怎么调/影响”。
- **单位**：deg、deg/s、s（ms 显式）、us。
- **构建**：`powershell -File make_one\build.ps1 -Robot <robot> -Board GIMBAL_BOARD`（产物 `make_one\build_<robot>\control-2026.hex`）；每次改动必须编译 0 error 再提交。
- **git**：`type(scope): 中文描述(细节)`；tag 语义化 `1.0.x`；22 端口被拒时走 443（`Tools/scripts/git_push_443.ps1`）。
- **调参**：只走 cfg、不用 Flash。对照表：整机过偏→`SERVO_MIX_MAX_DEG`；某一路→`SERVO_MIX_SCALE[ch]`；制导震荡→`N`/`k`；滚转→roll 控制器增益；零点→`trim`；方向→`reverse`。
- **常见坑**：Flash 擦除前必须清错误标志（否则静默失败）；老文件可能 GBK（用 `write` 统一 UTF-8）；`x=(x++)%N` UB；`INS_Init` 无 BMI088 会 `while` 死等；`nano.specs` 禁 `%f`（用整数定标）。
- 详见 `Tools/AGENTS.md`（总纲）+ `Tools/agent_profile.md`（方法论/教训）。

## 飞镖整机 userapp（`UserApp/robot/dart_final`）
- **模块栈**：`motor/servo_motor`(单路 PWM 舵机: 标定/限速/调零) + `algorithm/servo_mix_ai`(四舵面 MIX/MANUAL + 上电回中) + `algorithm/png_ai`(视线比例导引 PPN/APN) + `imu/ins_task` + `message_center`。
- **结构**：`app/{imu,vision,guidance,fin}` 四平行 app + `robot.c`(编排 + 状态机 `IDLE/ARMED/GUIDING/FAULT` + Monitor + 两个待配置接口 `Dart_IsEnabled`/`Dart_ShouldGuide`)；话题 `attitude/target/mix/servo_fb`；数据结构/错误位在 `robot_def.h`。
- **控制**：只控 yaw（制导）+ 稳 roll（PID），pitch 恒 0；调参走 cfg、不用 Flash。详见 `dart_final/README.md`。
- **平台**：STM32F407（GIMBAL_BOARD）；4×PTK7350 = **TIM1 CH1–4 = PE9/PE11/PE13/PE14**；BMI088(hspi1)；USART6↔ESP32、huart3↔OpenMV、huart1(VOFA 预留)、hspi2(OpenMV SPI 预留)。最终拟移植 H7。

## 飞镖 userapp 调试（`UserApp/robot/dart_final_test_app`）
- **用途**：逐个黑盒测 `dart_final` 各 app（自底向上 fin→guidance→vision/imu），无需 IMU/视觉硬件；`test.c` 代替 `robot.c`，复用 `dart_final/app/*`（robot.cmake include+glob，不含 `dart_final/robot.c`）。
- **命令（RTT）**：`PING` / `ATT,<r>,<p>,<y>`(注入姿态) / `TGT,<x>,<y>` / `TGTN`(丢目标) / `FMIX,<p>,<y>,<r>`(注入混控) / `FIN,<ch>,<deg>`(逐路手动) / `RESET[,<APP>]`(软复位/单关某app) / `STAT`。
- **状态表**（方案①，ros2 风格）：每话题 `prod=<生产者app>:1/0 sub=<消费者app>:1/0 data=1/0 | 值`；**默认静默**（`LOG_PERIOD_MS=0` 不自动输出），只在你发 `STAT` 时打印一次（>0 则周期存档用）；`fs=1` 附 `fsreason`。浮点用整数定标（`roll10/p100/d10`，÷10/÷100）。注入类命令无回执。
- **cfg**（`test_cfg.h`）：`TEST_IMU_ENABLE=0`/`TEST_VISION_ENABLE=0`（默认注入，避免死等/串口）、`MON_PERIOD_MS=100`、`LOG_PERIOD_MS=1000`、话题↔生产者/消费者映射。
- 详见 `dart_final_test_app/README.md`。
- **协议**：OpenMV 帧 **7 字节** `AA 55 X_hi X_lo Y_hi Y_lo CRC8`（SHT75 poly0x31，CRC 覆盖 0..5），去掉了 W/H；STM32 侧 `vision.c` 解析。

## 调试工作流（脚本在 `Tools/scripts/`）
- **一键烧录**：VS Code `Ctrl+Shift+B`（弹框选 robot → `oneclick_flash.ps1` 编译+烧录；**不再默认烧 dart_final**）；或 `Tools/scripts/oneclick_flash.ps1 -Robot <r> [-NoBuild|-NoFlash|-Clean]`。
- **可视化上位机**：`Tools/gui`（PyQt5 + pyserial，走板载 **USB-CDC** 串口；输入=滑条 target-x/4×fin/mix-PYR，输出=文本+`STAT` 按钮）；打包 `Tools/gui/build_exe.ps1` → `dist\dart_test_gui.exe`。
- 烧录 `flash.ps1 -Hex <hex>`；复位 `reset.ps1`。
- RTT 注入命令：`rtt_send.ps1 -Cmd "<...>" -Elf <elf>`（nm 动态解析 `_acDownBuffer`/`_SEGGER_RTT`，免硬编码）；或 RTT Viewer。
- 读 RAM/Flash：`mem_read.py --elf --sym/--addr`（nm + JLink）。
- RTT 存盘：`rtt_log.ps1 -Out Debug\...`（JLinkRTTLogger）。
- 一键全部编译：`build_all.ps1`。

## 分支 / 版本
- `main` = `d4ea016`（tag `1.0.6`）；实验分支 `smc`(`f6df8fb`)/`adrc`(`9adad72`)/`roll_dec`(`13343a0`) 已 merge main 同步（保留各自实验、**不反向合并**）；`esp`=`d4ea016`（与 main 同步，待开展）。
- 版本线 `1.0.0 → 1.0.6`（1.0.3 servo/mix/dart_final；1.0.4 Tools；1.0.5 根 README；1.0.6 test app + 7字节帧 + servo_mix_ai 简化）。
- 远程 `origin` 各分支已同步；工作区干净。

## Next Move
- 待用户指令：接 `esp`（ESP32 无线/网页/烧录）、H743 移植、或各实验分支的台架验证/调参。
