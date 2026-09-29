# dart_final_1 — dart_final 的沙盒测试版（状态机在环测试）

由 `dart_final` **拷贝**而来，在**不改生产版**的前提下加了一个 **RTT 测试控制台 + 话题注入**，
用于在真实硬件上把状态机（IDLE/ARMED/GUIDING/FAULT）**全部跑通**。

> 生产版仍是 `UserApp/robot/dart_final`（本次**未改动**）。

## 与 dart_final 的差异
- `robot.c`：加 RTT 控制台（命令见下）+ `attitude`/`target` 注入 + 使能/故障注入；
  **关掉的 app 不参与卡死判定**；遥测同时打印 **mix + 舵面回读**；`LOG,<ms>` 调遥测周期(0=关)。
- `app/imu.c`：`offset_flag=1`（跳过在线零偏标定 → 避开 40℃ 温度等待；**仅 SWD 供电时加热不达标会卡死**）。
- `robot_cfg.h`：`DF1_IMU_ENABLE` / `DF1_VISION_ENABLE`。
- `syscalls.c`：补 newlib 桩（本 app 用了 `snprintf/strtof/strtoul`）。

## 命令（RTT 下行通道0, ASCII 行, 大写不敏感）
| 命令 | 作用 |
|---|---|
| `PING` | -> PONG |
| `EN,<0\|1>` | 使能（0=IDLE, 1=ARMED） |
| `IMUEN,<0\|1>` | IMU app 开关（0=不跑真实IMU, 改用 ATT 注入） |
| `VISEN,<0\|1>` | 视觉 app 开关（0=不注册OpenMV, 改用 TGT 注入） |
| `ATT,<r>,<p>,<y>` | 注入姿态（valid=1） |
| `TGT,<x>,<y>` | 注入目标（found=1） |
| `TGTN` | 目标丢失（found=0） |
| `FAULT,<hex>` | 注入故障位（如 `FAULT,1`=IMU_OFF） |
| `CLRFAULT` | 清除注入故障 |
| `LOG,<ms>` | 遥测周期；0=关闭（测试时用） |
| `STAT` | 立即打印一次状态 |

## 实测结果（硬件：C 板, 仅 J-Link SWD）
- 上电：`state=1 ARMED`, `fault=0x0000`, 舵面回中 `p=1500`。
- `IMUEN,0`+`ATT,0,-10,0`+`TGT,240,120` → `state=2 GUIDING`, `fs=0`；
  再注入 `ATT,20,-10,0` → `mix r100=-100`, `d10=-350,350,-350,350`（满舵 ±35°）。
- `TGTN` → 仍 `state=2`（GUIDING），但 `fs=1` 回中（**设计如此**：丢目标不加退，靠 guidance failsafe）。
- `FAULT,1` → `state=3 FAULT`, `fault=0x0001`, 回中。
- `CLRFAULT` → `state=1 ARMED`（故障消除恢复）。
- `EN,0` → `state=0 IDLE`；`EN,1` → `state=1`。
- `IMUEN,1` → 真实 IMU 恢复（hb 继续涨）。

**结论：4 个状态与全部分支转移均顺畅。**

## 发现（影响生产版 `dart_final`）
1. **仅 SWD 供电时 IMU 初始化会卡死**：`Modules/imu/ins_task.c` 的 `INS_CalibrateGyroForDebug()`
   有 `while(Temp<=39||Temp>=41)` 的 40℃ 温度等待，靠 IMU 加热；SWD 供电不足 → 永远卡在 `DWT_Delay`。
   接 USB/VIN 供电时正常。（本沙盒版用 `offset_flag=1` 跳过。）
2. **生产 `dart_final` 的使能接口是桩**：`Dart_IsEnabled()` 恒 `true`（`DART_ENABLE_ON_BOOT` 被无视）
   → `IDLE` / “撤销使能”分支不可达，需赛事规则确定后实现。
3. 视觉链路：OpenMV 脚本(`115200 8N1`+握手) 与 STM32 `USART3`(`100000 9E`,且不发握手) **不匹配**，需对齐。
