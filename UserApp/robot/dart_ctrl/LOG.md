# dart_ctrl 开发日志 / 问题记录

> 记录每次改动、踩坑与决策，便于回溯。格式：日期 + 内容。

## 由 dart_final 改写为 dart_ctrl（单文件 · 组合根直连）
- **占用并改名**：`UserApp/robot/infantry_mecanum` → `git mv` 为 **`UserApp/robot/dart_ctrl`**（步兵 app 被替换，git 历史保留）。
- **架构**：取消 `message_center` 发布订阅；改为**一个 `robot.c` + `RobotInstance` 直连**（本仓库 userapp 统一风格，如 `hero_mecanum`/`infantry_mecanum`）。
- **文件**：`robot.c / robot.h / robot_config.h / robot.cmake`（无 `app/`、无 `robot_def.h`；结构体并进 `robot.h`）。
- **算法内联**（本仓库无对应 Module）：`png_ai`(比例导引) → `PngCalc()`；`servo_mix_ai`(四舵面混控+逐路标定+速率限幅) → `FinStep()`。
- **IMU 适配干净库 `ins_task`**：`INS_Init()` 返回 `INS_t*` 且**内部自建 1kHz 任务**；直接读 `INS_t->Roll/Pitch/Yaw/Gyro[]`；`Gyro` 已是 **°/s**（**去掉源版的 ×57.3/RAD2DEG**）。
- **舵机适配干净库 `servo_motor`**：其 PWM 分支 `ServoSetAngle(servo, angle)` **直接把参数当占空比 0..1**；故在 app 内做 **逻辑角→脉宽(500–2500us)→占空比** 的映射（`AngleToDuty()`），并保留逐路 scale/trim/reverse/rate 标定。
- **端口**：**板子留空**，`robot_config.h` 先按 C 板(F407) 占位（OpenMV=`huart3`、舵机=`htim1` CH1-4）；实板 F405 以后再改（见 `HARDWARE.md`/`F405_TODO.md`）。

## 与 dart_final 的关键差异
| 项 | dart_final | dart_ctrl |
|---|---|---|
| 文件 | robot.c + robot_def.h + app/{imu,vision,guidance,fin} + 多 cfg | **单 robot.c** + robot.h + robot_config.h |
| 通信 | message_center 伪 pub-sub 话题 | **RobotInstance 直连** |
| png_ai / servo_mix_ai | Modules 模块 | **内联进 app** |
| servo_motor | 源仓库版(center/half/trim) | 干净库版(PWM 吃占空比) + app 内映射 |
| IMU | INS_GetAttitude()+rad/s | 干净库 ins_task(INS_t*, °/s) |
| 致命故障 | 含舵机/制导卡死 | **姿态无效 或 舵机注册失败**（视觉掉线非致命） |

## 待办
- 见 `F405_TODO.md`（实板端口/IMU/视觉/舵机/ESP）。
- roll 控制方案最终确定；速率环预留（`GUID_CTRL_MODE`，本版未实现）；OpenMV SPI 冗余；黑匣子日志。

## 踩坑记录
- 干净库 `servo_motor` 的 PWM `ServoSetAngle` 参数是**占空比**不是角度 → 必须在 app 侧换算。
- 干净库 `ins_task` 的 `Gyro` 单位是 **°/s**（源仓库版为 rad/s，勿照搬 ×57.3）。
- 干净库 `ins_task` 无 `INS_GetAttitude()`，改为持 `INS_t*` 直读。
- F4 可用串口：`huart1/3/6`；本 app 用 `huart3`(OpenMV 占位)。
- LQFP/F4 无 newlib `syscalls`：本 app 未用 `printf/atof`，遥测走 `bsp_log`(RTT)。
