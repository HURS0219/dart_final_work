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

## 2026-09-28　分支 smc（滑模稳滚）
- 新增 `Modules/algorithm/smc_ai/`（纯算法）: 一阶滑模面 `s=c·e+ė` + 边界层趋近律 `u=-k·sat(s/phi)`。
- `guidance.c`: 用 `ROLL_CTRL_MODE` 在 PID / SMC 间切换（本分支默认 SMC）；roll 通道接入。
- `dart_final_cfg.h`: 新增 `ROLL_CTRL_MODE`（1=SMC）。
- **坑**: 滑模符号约定——`u=-k·sat(s)` 要求 `e=被测-期望`（output-desired），
  最初误用 `e=期望-被测` 导致发散；已改为 `e=roll_deg, e_dot=gx_dps`。
- 主机测试 `Debug/test_smc.c`（gcc，不入库）验证收敛。分支实验用，不合并 main。
