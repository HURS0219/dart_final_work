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

## 2026-09-28　分支 adrc（自抗扰稳滚）
- 新增 `Modules/algorithm/adrc_ai/`（纯算法）: 线性二阶 ADRC（ESO + PD），
  对象 `y''=f+b0·u`；`u=(wc²(ref-z1)-2ζwc·z2 - z3)/b0`，ESO 估计 `z3≈总扰动`。
- `guidance.c`: `ROLL_CTRL_MODE`=2 时 roll 走 ADRC（measure=roll, ref=0）。
- `dart_final_cfg.h`: `ROLL_CTRL_MODE`（0=PID,1=SMC,2=ADRC; 本分支=2）。
- 主机测试 `Debug/test_adrc.c`（gcc，不入库）验证收敛。
- 参数整定法: 先 `wc`（闭环快慢）→ 再 `wo≈3~5·wc`; `b0` 符号/大小最关键。
- 分支实验用，不合并 main。
