# AGENTS.md — 面向 AI Agent 的项目总纲

<p align='right'>制导飞镖 · 由 opencode/deepseek-flash 整理，供后续 agent 复用</p>

> 本文件是本仓库对 **AI 编码 agent** 的“总纲”。接手前请先读完本文件与各模块的 `*.md`。
> 目标：让后来的 agent 也能达到“**严谨、细节、可回溯、少踩坑**”的效果。

---

## 0. 语言与编码

- **注释与文档用中文**；**所有源文件统一 UTF-8**（严禁 GBK/混合编码，否则乱码）。
- 提交信息用中文，格式见 §5。

---

## 1. 目录结构

```
dart_final_work/
  Bsp/            # 板级/外设驱动(usart/pwm/can/flash/dwt/log/gpio/iic/spi/usb...)
  Modules/        # 可复用模块(imu/algorithm/motor/...); 一般**禁改**, 见 §2
  Hardware/       # CubeMX 生成(F4 / H7 两套); MCU 相关, 用宏条件编译
  UserApp/        # 应用层
    components/   # 可复用 app 组件(gimbal/chassis/shoot...)
    robot/<name>/ # 各机器人 app(robot.c/.h + robot_config.h + robot.cmake + 子目录)
    application.md / APP层应用编写指引.md / os_task.c
  Tools/          # 工具/脚本/资料/agent 档案(本文件所在)
    scripts/      # 可复用调试脚本(见 scripts/README.md)
  make_one/build.ps1  # 一键编译某个 robot
```

---

## 2. 铁律（务必遵守）

1. **禁止修改 `Bsp/`、`Modules/`、`Hardware/`**（除非用户**明确授权**某文件）。
   - 新增算法模块允许：放 `Modules/algorithm/<算法名>_ai/`（如 `png_ai`、`servo_mix_ai`）。
   - 追加过的一处授权例外：`Modules/message_center`（修 bug）、`Modules/motor/servo_motor`。
2. **App 之间禁止相互 include / 禁止全局变量** → 用 `message_center` 的 pub-sub。
3. **版本管理用 git commit**；**禁止把整个文件夹复制**来“备份”。
4. **不要**在未获指示时 `push` / `tag` / 改动 git 配置。
5. 新增/修改后**必须编译通过**（见 §4）再提交。

> 详见根目录 `agent_must_read.txt` / `agent_must_read_dart.txt` / `dart_servo.txt`。

---

## 3. 代码规范

- **Module 风格**：仿 `Modules/motor/DJImotor/dji_motor`：文件头 doxygen、聚合 `*_Init_Config_s`、`static` 实例数组 + `idx`、公共子设施复用（`bsp_*`/`controller`/`daemon`）。
- **App 解耦**：`robot.c` 只做编排；业务拆成平行 app（如 `dart_final/app/{imu,vision,guidance,fin}`），经话题通信。
- **命名**：模块函数 `PascalCase`（`ServoInit`）；宏全大写；类型 `_s`/`_e` 后缀。
- **注释**：每个函数 doxygen（`@brief/@param/@return/@note`）；关键步骤逐行中文注释；**cfg 宏逐项**注明“单位/含义/取值/怎么调/改了会怎样”。
- **头文件**只用 `#pragma once`（**不要再写 `#endif`**）。
- 单位统一：角度 deg、角速度 deg/s、时间 s(ms 显式标注)、脉宽 us。

---

## 4. 构建 / 烧录 / 调试（可复现流程）

```powershell
# 编译(产物 make_one/build_<robot>/control-2026.hex)
powershell -ExecutionPolicy Bypass -File make_one\build.ps1 -Robot <robot> -Board GIMBAL_BOARD
# 一键编译全部 robot(见 Tools/scripts/build_all.ps1)
```

调试通道：
- **RTT（J-Link）**：文本日志(`bsp_log`)，不占串口。看：J-Link RTT Viewer；**注入命令**：`Tools/scripts/rtt_send.ps1`（当无法在 Viewer 打字时）。
- **VOFA+**：串口发浮点波形(`Modules/vofa`)。
- **读 RAM/Flash**：`Tools/scripts/mem_read.py`（`nm` 解析符号 + J-Link `mem32`）。
- **烧录**：`Tools/scripts/flash.ps1`、`reset.ps1`。

> 关键技巧：`arm-none-eabi-nm -S <elf>` 取符号地址；每次**重编地址会变**，脚本已动态解析，勿硬编码。

---

## 5. Git 规范

- 提交信息：`type(scope): 中文描述(细节); 分号分点`
  - type：`feat/fix/refactor/docs/chore`
  - 例：`refactor(servo_motor,bsp_flash): ...`
- tag：语义化 `1.0.x`。发布=commit + `git tag -a` + push（分支与 tag）。
- **GitHub 22 端口常被拒** → 走 443：`Tools/scripts/git_push_443.ps1`（或 `core.sshCommand` 指定 `ssh -p 443 ... ssh.github.com`）。

---

## 6. 调参方法论

- 参数**只放 cfg**（`Modules/**/x_cfg.h` + app `*_cfg.h`），不用 Flash（本项目 Flash 保存不稳定）。
- **各旋钮分工（避免两头调糊）**：
  - 过偏/欠偏(整机) → 执行器行程增益（如 `SERVO_MIX_MAX_DEG`）；
  - 某一路偏 → 该路比例（`SERVO_MIX_SCALE[ch]`）；
  - 震荡/收敛慢(指令) → 制导律增益（`N`/`k`）；
  - 姿态不稳 → 姿态环 PID 增益。
- 标定模型 `servo_motor`：`applied = 逻辑角*scale + trim`（reverse 取负）→ `pulse = center + applied*(half_us/half_deg)`。
  比例不对→`scale`；零点偏→`trim`；方向反→`reverse`。

---

## 7. 工程化 checklist（逐步补齐）

硬件看门狗 IWDG、上电时序自检、失效分级、统一错误码；单元测试(算法可主机自带)、SIL/HIL 回放、回归脚本；黑匣子飞行日志 + 参数快照 + 时间对齐；运行期参数表 + cfg 版本号 + 范围校验；CI(全 robot×board)、cppcheck/clang-format、内存/栈审查；协议契约/pinout/ADR/CONTRIBUTING；任务周期/长任务/中断延迟核算。

---

## 8. 常见坑（血泪）

- **Flash 擦除前必须清错误标志**，否则**静默失败**（曾把 `0xD2A70002 & 0x5352564F` 当成写入结果）。
- **编码**：老文件可能是 GBK；用 `edit` 可能沿袭 GBK → 之后用 `write` 统一 UTF-8。
- **UB**：`x = (x++) % N` 写法错误（`message_center` 曾中招）。
- `INS_Init()` 会**自建 1kHz 任务**且 `while(BMI088Init...)` **无 IMU 会死等**；app 不要重复调 `INS_Task()`。
- 不要给 J-Link RTT Viewer 手动打字靠“点”，用脚本注入更可靠。

---

## 9. 模块清单 / 可改矩阵（简表）

| 模块 | 说明 | 可改? |
|---|---|---|
| `Modules/motor/servo_motor` | 单路 PWM 舵机驱动(标定/限速/调零) | 授权可改 |
| `Modules/algorithm/servo_mix_ai` | 四舵面混控(MIX/MANUAL) | 可改(自建) |
| `Modules/algorithm/png_ai` | 视线比例导引(PPN/APN) | 可改(自建) |
| `Modules/message_center` | 伪 pub-sub | 已授权(修 bug) |
| `Modules/imu` / `Bsp/*` / `Hardware/*` | IMU/板级/CubeMX | **禁改** |
| `UserApp/robot/dart_final` | 整机飞控(app) | 可改 |

---

## 10. 参考

- `UserApp/application.md`、`UserApp/APP层应用编写指引.md`（app 规范）。
- `Bsp/flash/README.md`、`Modules/motor/servo_motor/README.md`、`Modules/algorithm/servo_mix_ai/servo_mix_ai.md`、`Modules/algorithm/png_ai/README.md`。
- `UserApp/robot/dart_final/README.md`（整机飞控详解）。
- `Tools/agent_profile.md`（本 agent 的方法论与教训）。
