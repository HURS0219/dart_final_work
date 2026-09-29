# 制导飞镖 · dart_final_work

<p align='right'>RoboMaster 制导飞镖 · 机载飞控 / 舵机 / 制导算法 · SHU SRM</p>

面向 RoboMaster **制导飞镖**的嵌入式工程：视觉制导 + X 型四舵面 + 自研飞控。基于 RM 通用框架
（Bsp / Modules / UserApp / Hardware），在 STM32F407 上实现，目标可移植到 H7。

> 本仓库采用**分支研发**：`main` 为稳定主线；`smc` / `adrc` / `roll_dec` 为三条**姿态控制实验分支**；
> `esp` 为无线（网页 + 无线烧录）分支。

---

## 仓库结构

```
Bsp/        板级/外设驱动(usart/pwm/can/flash/dwt/log/gpio/iic/spi/usb ...)
Modules/    可复用模块(imu / algorithm / motor / message_center / vofa ...)
Hardware/   CubeMX 工程(stm32-f4 / stm32-h7)
UserApp/    应用层
  components/  通用 app 组件(gimbal/chassis/shoot ...)
  robot/       各机器人 app(见下)
  os_task.c    统一任务框架(调用所选 app 的 RobotInit/RobotTask)
Tools/      工具、调试脚本、AI Agent 档案(见 Tools/AGENTS.md)
make_one/   一键编译脚本 build.ps1
```

---

## `main`（主线）实现了什么

### 1. 通用飞控框架
- `Bsp/*`、`Modules/*`、`Hardware/*`（F4/H7 两套）、`UserApp/os_task.c` 任务框架、`make_one/build.ps1` 一键编译。

### 2. 关键模块（`Modules/`）
| 模块 | 说明 |
|---|---|
| `motor/servo_motor` | **单路 PWM 舵机驱动**：逻辑角↔脉宽标定、速率限幅、调零（`zero_enable` + 安全窗口）、封装标定 API。参数见 `servo_motor_cfg.h`。 |
| `algorithm/servo_mix_ai` | **四舵面 X 型混控**：仅 `MIX`/`MANUAL` 两模式 + 上电回中；直接驱动 4 路 `servo_motor`。参数见 `servo_mix_ai_cfg.h`。 |
| `algorithm/png_ai` | **视线比例导引（PNG）**：PPN / APN，`a_cmd = N·v·dλ`（+APN 前馈），死区 + 限幅。参数见 `png_cfg.h`。 |
| `message_center` | 伪 pub-sub（app 间解耦通信）；修复了 3 处 bug（索引自增写法 / 静态迭代器 / 返回值）。 |
| `Bsp/flash` | 内部 Flash 读写 + **通用掉电保存**（双 Bank + magic + seq + FNV-1a CRC，回读校验）。 |

### 3. 整机飞控 app：`UserApp/robot/dart_final`
严格解耦的制导飞镖整机飞控：`imu / vision / guidance / fin` 四个**平行 app** 经话题通信，`robot.c` 只做编排 + 状态机 + 监控。
- 话题：`attitude` / `target` / `mix` / `servo_fb`；
- 数据结构、错误位：`robot_def.h`；状态机：`IDLE/ARMED/GUIDING/FAULT`（失效回中）；
- 控制：**只控 yaw（制导）+ 稳 roll（PID）**，pitch 恒 0；
- 两个“待配置接口”（使能 / 制导时机）集中在 `robot.c`；
- 调参全走 cfg、**不使用 Flash**。详见 `UserApp/robot/dart_final/README.md`。

### 4. 工具与档案（`Tools/`）
- `AGENTS.md`：面向后续 AI/人的项目总纲（规范/流程/坑/可改矩阵）。
- `agent_profile.md`：AI Agent 工作档案（方法论/质量标准/失败案例）。
- `scripts/`：可复用调试脚本（`rtt_send` / `mem_read` / `build_all` / `flash` / `reset` / `git_push_443`），见 `scripts/README.md`。

### 5. 版本
`1.0.0 → 1.0.1 → 1.0.2 → 1.0.3 → 1.0.4`（最新主线 = `1.0.4`）。

---

## 分支各自实现了什么

| 分支 | 主题 | 内容 | 是否合并 |
|---|---|---|---|
| **`smc`** | 滑模稳滚 | `Modules/algorithm/smc_ai`：一阶滑模面 `s=c·e+ė` + 边界层趋近律 `u=-k·sat(s/φ)`；`guidance.c` 经 `ROLL_CTRL_MODE` 接入 roll 通道 | 实验，不合并 |
| **`adrc`** | 自抗扰稳滚 | `Modules/algorithm/adrc_ai`：线性二阶 ADRC（ESO + PD），`u=(wc²(ref−z1)−2ζwc·z2−z3)/b0`，`z3` 估计总扰动 | 实验，不合并 |
| **`roll_dec`** | 旋转解耦（路线二） | 允许弹体自旋：用滚转角 γ 做 `R(γ)` 把空间指令旋到弹体坐标系（含**输入解旋** + **相位超前**补偿舵机滞后），舵面随弹体滚也产生正确空间方向 | 实验，不合并 |
| **`esp`** | 无线 | ESP32 侧：网页 + **无线烧录**（替代 SWD）；仅 ESP32，不动 C 板 | 规划中 |

> 三条姿态控制分支（`smc`/`adrc`/`roll_dec`）是**独立实验版**，用于对比不同“不滚转/解耦”方案；
> `main` 保持稳定，不受影响。

### 分支常用命令
```powershell
# 编译
powershell -ExecutionPolicy Bypass -File make_one\build.ps1 -Robot dart_final -Board GIMBAL_BOARD
# 切换分支
git switch smc      # 或 adrc / roll_dec / esp / main
```

---

## 如何开始
1. **看懂主线**：读 `UserApp/robot/dart_final/README.md`（整机飞控详解）与各模块 `*.md`。
2. **构建**：`make_one\build.ps1 -Robot <app> -Board GIMBAL_BOARD`（产物 `make_one\build_<app>\control-2026.hex`）。
3. **面向 AI/新成员**：先读 `Tools/AGENTS.md`（规范与流程）与 `Tools/agent_profile.md`。
4. **调试**：`Tools/scripts/`（RTT 注入、读内存、烧录…），或 J-Link RTT Viewer。

## 硬件
- 主控：STM32F407（RoboMaster C 板）；未来移植 H7（见 `dart_final/README.md` 的 H743 checklist）。
- 执行：4× PTK7350 PWM 舵机（TIM1 CH1–4 = PE9/PE11/PE13/PE14）X 型布置。
- 传感：板载 BMI088 IMU；OpenMV 视觉（绿光目标）。

---

<p align='right'>© SHU SRM · 详见各目录 README / *.md</p>
