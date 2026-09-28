# dart_final 开发日志 / 问题记录

> 记录每次改动、踩坑与决策，便于回溯。格式：日期 + 内容。

## 2026-09-28　初始化
- 新建 `UserApp/robot/dart_final`：严格解耦整机飞控（imu/vision/guidance/fin 四 app + robot.c 编排）。
- 话题：`attitude` / `target` / `mix` / `servo_fb`（`message_center` 伪 pub-sub）。
- 控制：只控 yaw（制导）+ 稳 roll（PID），pitch 恒 0；详见 `README.md` §7。
- 状态机 IDLE/ARMED/GUIDING/FAULT，FAULT 回中；Monitor 汇总各 app 心跳/错误位。
- 两个待配置接口（使能 / 制导时机）集中在 `robot.c`，赛事规则出来再配。
- 参数用 cfg（`dart_final_cfg.h` + 各 module cfg，`dart_all_cfg.h` 汇总），**不使用 Flash**。
- 修复 `Modules/message_center`（**唯一获授权修改的 Modules**）：
  1. `SubGetMessage` 的 `front_idx = (front_idx++) % N` 写法错误 → `(front_idx + 1) % N`；
  2. `PubPushMessage` 的 `static` 迭代器 → 局部变量（可重入）；
  3. `PubPushMessage` 返回实际推送的订阅者数（原恒返回 1）；并加空指针防御。

## 待办（详见 README §15）
- roll 控制方案最终确定；速率环预留；OpenMV SPI 冗余；ESP32 无线烧录三方案；引脚落实；时间对齐；黑匣子日志；H743 移植。

## 踩坑记录
- `app/vision.c` 使用 `DART_USART_OPENMV(&huart3)` 需 `#include "usart.h"`（否则 `huart3` 未声明）。
- 头文件统一 `#pragma once`，勿再写 `#endif`（曾误加导致预处理错误）。
- F4 可用串口：`huart1/3/6`；SPI：`hspi1`(BMI088) / `hspi2`(留给 OpenMV)。

## 2026-09-28　分支 roll_dec（旋转解耦·路线二）
- 定义与方案见本分支 `roll_dec.md`（路线一/二对比；本分支选路线二：允许自旋, 用 γ 旋转空间指令到体轴）。
- `app/guidance.c`：新增 `#if GUID_ROLL_DEC_ENABLE` 分支——用 `attitude.roll_deg(γ)` 做 `R(γ)` 变换，
  体轴 `yaw=ay·cosγ`, `pitch=-ay·sinγ`, 不再稳滚；关闭则退回传统“仅控 yaw + roll PID”。
- `dart_final_cfg.h`：新增 `GUID_ROLL_DEC_ENABLE / GUID_ROLL_SIGN / GUID_ROLL_OFFSET_DEG`。
- 分支已推 origin（实验用，不合并）。
