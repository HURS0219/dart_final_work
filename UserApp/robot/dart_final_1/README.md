# dart_final · 制导飞镖整机飞控

<p align='right'>ai @ dart_final_work</p>

`dart_final` 是制导飞镖的**整机机电飞控**：上电回中 → 识别绿光且俯冲后进入制导 → 比例导引解算 → 四舵面 X 型混控。**严格解耦**，各 app 平行、经 `message_center`(伪 pub-sub) 通信，参数用 cfg 调参、不用 Flash。

> 与 `dart_fc`(pre 原型版) 的区别：`dart_fc` 未严格解耦、自带 `dart_ptk7350`+`dart_surface`；`dart_final` 使用统一的 module 栈（`ins_task`/`png_ai`/`servo_mix_ai`/`servo_motor`），app 之间只通过话题通信。

---

## 1. 架构与数据流

```
                      ┌──────────── message_center (伪 pub-sub 话题) ────────────┐
                      │                                                         │
  app/imu  ──pub "attitude"─┐                                                    │
  app/vision ─pub "target" ─┼──> app/guidance ──pub "mix"──> app/fin ──pub "servo_fb"
                            │        (png_ai + PID)             │
                            │                                   │
  robot.c: 建话题 + 周期调用各 app + 状态机(IDLE/ARMED/GUIDING/FAULT) + Monitor
                                                                 │
                                            app/fin ─> servo_mix_ai ─> servo_motor ×4 ─> PWM
```

分层：
- **Module**：`ins_task`(IMU/姿态)、`png_ai`(比例导引)、`servo_mix_ai`(四舵面混控)、`servo_motor`(单路舵机)、`bsp_usart`(视觉串口)、`bsp_log`(RTT 日志)、`controller`(PID)。
- **App**：`imu / vision / guidance / fin`（平行，互不 include）。
- **编排**：`robot.c` 唯一入口，只做初始化 + 周期调度 + 状态机 + 监控。

---

## 2. 文件清单

| 文件 | 作用 |
|---|---|
| `robot.cmake` | 板→MCU 映射，收集本 app 源码 |
| `robot.h/.c` | 入口：`RobotInit/RobotTask` + 状态机 + Monitor + 两个待配置接口 |
| `ui.h` | `os_task.c` 需要的占位 |
| `robot_def.h` | 话题名 / 健康位 / 跨 app 数据结构（`#pragma pack(1)`） |
| `dart_final_cfg.h` | **本 app 全部可调参数**（制导 k、roll PID、俯冲阈值、超时、端口占位） |
| `dart_all_cfg.h` | **包含式超级 cfg**：汇总查看各 cfg（不覆盖） |
| `app/imu.{c,h}` | 封装 `ins_task`，发布 `attitude` |
| `app/vision.{c,h}` | 解析 OpenMV 9 字节帧，发布 `target` |
| `app/guidance.{c,h}` | 订阅 `attitude`+`target` → png_ai + 控制 → 发布 `mix` |
| `app/fin.{c,h}` | 订阅 `mix` → `servo_mix_ai` 驱动 4 舵机 → 发布 `servo_fb` |
| `README.md` / `LOG.md` | 本文档 / 开发日志 |

---

## 3. 话题（`robot_def.h`）

| 话题名 | 发布者 | 订阅者 | 负载结构 |
|---|---|---|---|
| `"attitude"` | imu | guidance | `Dart_Attitude_s` |
| `"target"` | vision | guidance | `Dart_Target_s` |
| `"mix"` | guidance | fin | `Dart_Mix_s` |
| `"servo_fb"` | fin | (调试/遥测) | `Dart_ServoFb_s` |

接口（`message_center.h`）：
```c
Publisher_t  *p = PubRegister("mix", sizeof(Dart_Mix_s));   // 注册发布者
Subscriber_t *s = SubRegister("mix", sizeof(Dart_Mix_s));   // 注册订阅者
PubPushMessage(p, &data);      // 发布(拷贝给每个订阅者; 返回订阅者数)
SubGetMessage(s, &data);       // 取最新(有则返回1)
```
> `QUEUE_SIZE=1`：只保留最新一帧，正好适合“最新姿态/目标/指令”。

---

## 4. 各 app 接口

```c
/* imu */      void Imu_Init(void); void Imu_Task(void);
               const Dart_AppStatus_s *Imu_GetStatus(void); void Imu_GetAttitude(Dart_Attitude_s*);
/* vision */   void Vision_Init(void); void Vision_Task(void);
               const Dart_AppStatus_s *Vision_GetStatus(void); void Vision_GetTarget(Dart_Target_s*);
/* guidance */ void Guidance_Init(void); void Guidance_Task(float dt, uint8_t guide_enable);
               const Dart_AppStatus_s *Guidance_GetStatus(void);
/* fin */      void Fin_Init(void); void Fin_Task(void);
               const Dart_AppStatus_s *Fin_GetStatus(void);
```
所有 app 的 `*_Task()` 由 `robot.c` 的 `RobotTask()` 周期调用，**不要**在别处重复调用。

---

## 5. 状态机（`robot.c`）

```
IDLE ──使能──> ARMED ──ShouldGuide()──> GUIDING
  ^              ^  |                        |
  |              |  └────── 致命故障 ──> FAULT ──故障消除──> ARMED
  └──── 撤销使能 ┴─────────── 撤销使能 ─────────┘
```
- **IDLE**：未使能，舵面回中。
- **ARMED**：已使能、待触发，舵面回中。
- **GUIDING**：`guidance` 输出制导指令。
- **FAULT**：致命故障（app 卡死 / IMU 无效 / 舵机失败）→ **回中**；故障消除回 ARMED。
- 非 GUIDING 时 `guidance` 收到 `guide_enable=0` → `mix.failsafe=1` → `fin` 回中。

---

## 6. 【待配置接口】（**赛事规则出来后只改这里！**）

`robot.c` 顶部两个函数，改动集中、不要散落：

```c
/* 【待配置接口 1】使能: 现在=按 cfg DART_ENABLE_ON_BOOT(默认上电即 armed) */
static bool Dart_IsEnabled(void);

/* 【待配置接口 2】制导时机: 现在=识别到绿光(target.found) && 姿态俯冲(Pitch<DIVE_PITCH_DEG) */
static bool Dart_ShouldGuide(void);
```
> 备选触发（未采用）：**IMU 检测发射过载**——飞镖被弹射瞬间加速度尖峰，用阈值判定“已发射”。需要时改 `Dart_ShouldGuide`。

---

## 7. 控制律 & 调参对照表（精品）

只控 **yaw 制导 + roll 稳定**，pitch 恒 0（目标水平移动）：

```
guidance:  λ_yaw = (x-CX)/focal ; dλ = LPF(Δλ/Δt) ; a_cmd = png_ai(dλ, v_c)
           mix.yaw   = k * a_cmd          // 制导侧
           mix.roll  = roll_PID(roll,0)   // 姿态稳定(现成 controller PID)
           mix.pitch = 0
fin:       servo_mix_ai:  target[i] = clamp(Σ matrix[i][j]*mix[j] * MAX_DEG, ±MAX_DEG)
```

**调参分工（务必遵守，避免“两头调糊”）**

| 现象 | 改哪个 | 说明 |
|---|---|---|
| **整机过偏/欠偏**（舵面摆太多/太少） | **`SERVO_MIX_MAX_DEG`**(`servo_mix_ai_cfg.h`) | 全局舵面行程增益 |
| **某一路**偏多/偏少 | `SERVO_MIX_SCALE[ch]` | 该路比例（安装误差） |
| **制导震荡/收敛慢**（指令本身） | `N`(`png_cfg.h`) 或 `GUID_GAIN_K` | 制导律强度 |
| **滚转压不住/来回摆** | `ROLL_KP`/`ROLL_KD` | roll PID |
| 零点/中位偏 | `SERVO_MIX_TRIM_DEG[ch]` | 逐路机械零点 |
| 方向反 | `SERVO_MIX_REVERSE[ch]` | 逐路（改后需重新对零） |

> **k 固定为物理值 `1/PNG_MAX_OUT`**（= 1/`PNG_DEFAULT_MAX_OUT`）；“过偏”统一调 `SERVO_MIX_MAX_DEG`。
> 原理：舵偏 δ → 升力 ≈ `q·S·CL_α·δ`(q=½ρV²)，小 δ 时 **a ≈ K(V)·δ 近似线性**，故线性 k 是一阶近似；K 随速度变化，短时/低速可当常数。若全程跟随差，改速率环（见 §16）。

---

## 8. 配置文件

- 本 app：`dart_final_cfg.h`（k / v_c / LOS α / focal / 俯冲阈值 / 超时 / roll PID / 端口占位）。
- 制导算法：`png_cfg.h`（N / MaxOut / DeadBand / a_o_gain）。
- 单路舵机：`servo_motor_cfg.h`（信号层 center/half/pulse + 缺省标定）。
- 本机 4 路 + 混控：`servo_mix_ai_cfg.h`（PWM 通道 + 逐路 scale/trim/reverse/limit/rate + 矩阵 + MAX_DEG）。
- 汇总查看：`dart_all_cfg.h`（`#include` 以上，**只读汇总，不覆盖**）。

---

## 9. 构建 / 烧录

```powershell
powershell -ExecutionPolicy Bypass -File make_one\build.ps1 -Robot dart_final -Board GIMBAL_BOARD
# 产物: make_one\build_dart_final\control-2026.hex
```
默认板 `GIMBAL_BOARD`（STM32F407，4 舵机 = TIM1 CH1–4 = PE9/PE11/PE13/PE14）。

---

## 10. 调参 / 标定 SOP（现场照做）

1. **单路舵机**：用 `servo_test` app 先确认每路中位、方向、行程（见其 `robot.c` 命令）。
2. **四舵面混控**：改 `servo_mix_ai_cfg.h` 逐路 `TRIM/SCALE/REVERSE/LIMIT`；上电确认 4 舵面回中。
3. **滚转稳定**：先调 `ROLL_KP`（角度），再 `ROLL_KD`（阻尼），直到无持续滚转、无明显摆振。
4. **制导强度**：`N`、`v_c`、`GUID_GAIN_K`；观察 `png` 输出 `a_cmd` 是否平滑。
5. **过偏**：最后用 `SERVO_MIX_MAX_DEG` 统一收紧/放大舵面行程。

---

## 11. 协议契约

### 11.1 OpenMV → STM32（9 字节定长帧，大端）
```
[0]0xAA [1]0x55 [2]X_hi [3]X_lo [4]Y_hi [5]Y_lo [6]W [7]H [8]CRC8
CRC8: SHT75(poly=0x31, init=0), 对 [0..7] 校验
丢失目标: X=Y=W=H=0
```
STM32 端解析见 `app/vision.c`（`USARTRegister` 回调 + `crc_8`）。**另一路 SPI 冗余传输列为待办(§16)**。

### 11.2 STM32 ↔ ESP32（USART6）
ESP32 本次**不用于运行时控制**，而是作**无线烧录**（替代 SWD）。飞行 app 里先仅作为普通串口占位。烧录协议见 §16。

---

## 12. 引脚 / 端口（**待定，先在 cfg 占位**）

| 用途 | 宏（`dart_final_cfg.h`） | 当前占位 |
|---|---|---|
| ESP32 | `DART_USART_ESP32` | `&huart6` |
| OpenMV UART | `DART_USART_OPENMV` | `&huart3` |
| OpenMV SPI | `DART_SPI_OPENMV` | `&hspi2`（BMI088 占 hspi1） |
| VOFA（预留） | `DART_USART_VOFA` | `&huart1` |
| 4 舵机 | `servo_mix_ai_cfg.h` | TIM1 CH1–4 = PE9/11/13/14 |

---

## 13. 监控 / 日志

- **Monitor**（`robot.c`）：逐 app 心跳 `hb` 检测卡死（>阈值判死），汇总错误位 → 致命则 FAULT。
- **RTT**（本次启用）：`bsp_log` 的 `LOGINFO/LOGERROR`，经 J-Link 读，不占串口；每秒打印状态机+故障位+各 app 心跳。
- **VOFA**（**预留**）：`Dart_Vofa_Push()` 空实现，将来填 `VOFAInit(DART_USART_VOFA)` + `VOFAJustFloatSend(...)` 看波形。

---

## 14. 健壮性 / 已修

- `Modules/message_center` 的 3 处 bug 已修：`SubGetMessage` 的 `front_idx` 自增写法、`PubPushMessage` 的 `static` 迭代器、返回订阅者数。
- 跨 app 结构体统一 `#pragma pack(1)`；错误位统一定义；输入均带 `valid/found/failsafe`。

---

## 15. 未完工 / 待办（TODO）

- [ ] **roll 控制方案**：当前为 roll 角 PD；最终控制策略待讨论（可能加角速率环/更完整姿态稳定）。
- [ ] **速率环控制**（`GUID_CTRL_MODE=1`）：仅预留接口，未实现（飞镖 <20m/s，大概率不用）。
- [ ] **OpenMV SPI 并行传输**（防丢帧）：接口已留 `DART_SPI_OPENMV`，未实现。
- [ ] **ESP32 无线烧录**：三种方案（① STM32 UART ROM bootloader + BOOT0/复位；② 自写 IAP + ESP32 转发；③ SWD-over-WiFi）**待逐一试验**，取最高效者。
- [ ] **引脚/端口**最终确定（现为占位）。
- [ ] **视觉与 IMU 时间对齐**（当前各自 tick，未做严格对齐）。
- [ ] **黑匣子飞行日志**（内存环形 + 赛后导出）——见 §17。
- [ ] **H743 移植**——见 §18。

---

## 16. 工程化清单（以后做，本次不做）

> 用户：本清单先记录，以后再搞。

**安全 / 失效**
- 硬件看门狗 **IWDG**（当前**无**，需 CubeMX 配 + 主任务喂狗）；**上电时序与自检**（回中→使能）；**失效分级**（可恢复 vs 致命）；统一**错误码/健康位**（已初步定义 `DART_ERR_*`）。

**测试 / 验证**
- 主机端**单元测试**（`png_ai`/`servo_mix_ai` 不依赖 HAL，可 PC 编 gcc）；**SIL/HIL 回放**（注入姿态/视觉序列跑闭环）；**标定 SOP**（见 §10）；**回归脚本**（一键跑 `servo_test`）。

**数据 / 复盘**
- **黑匣子飞行日志**（内存环形缓冲，赛后可导出）；**参数快照随日志**（否则复盘对不上）；**时间对齐**。

**参数治理**
- **运行期参数表**（`{id, value}`，可被上位机读写）+ **cfg 版本号** + **范围校验**（越界拒绝）；超级 cfg 升级为“覆盖式/运行期”（见 `dart_all_cfg.h` 说明）。

**工程质量**
- **CI**：一键编译所有 `robot × board`；**静态分析/格式**（cppcheck/clang-format，避免如 `bsp_flash.c` 的编码混用）；**内存/栈审查**（FreeRTOS 栈、malloc/heap）；**构建大小报告**。

**文档**
- **协议契约**（§11 完善，含 ESP32 烧录协议逐字节）；**接线/pinout 表**（§12 落实）；**架构决策记录 ADR**（为什么弃用 `servo_mixer` 等）；**CONTRIBUTING / 模块可改矩阵**。

**运行期确定性**
- 任务优先级/周期核算表；长任务（Flash 擦写、录像）不得阻塞控制回路；中断延迟审查。

---

## 17. H743 / H7 移植 checklist

> 现状：仓库 H7 = **STM32H723VG**（宏 `STM32H723xx`，cortex-m7）。**H743 是另一颗芯片**，需单独移植。

- [ ] 新增 CubeMX 工程 + HAL/时钟配置（cortex-m7，`-mcpu=cortex-m7 -mfpu=fpv5-sp-d16`）。
- [ ] 链接脚本（H743 Flash/RAM 布局）+ 启动文件 + DSP 库。
- [ ] `MCU_TYPE`/宏：新增板卡映射（现仅 `stm32-f4`/`stm32-h7`），app/module 用 `STM32H7xx` 家族宏条件编译。
- [ ] `bsp_*` 的 H7 分支核对（`bsp_flash` 已有 H7 分支，**未真机验证**；其它 bsp 已按 MCU 分支）。
- [ ] 外设句柄/引脚：改 `dart_final_cfg.h` 与 CubeMX（**不改逻辑**）。
- [ ] 可选：IWDG、Flash 分区（若上 ESP32 烧录）。

**软件适配原则（本次已遵守）**：不硬编码 F4/H7 专有内容；外设/引脚走 cfg；用 board/MCU 宏条件编译；新增代码沿用 `bsp_*` 分支模式。

---

## 18. 相关文件

- `Modules/imu/ins_task.*`、`Modules/algorithm/png_ai/`、`Modules/algorithm/servo_mix_ai/`、`Modules/motor/servo_motor/`、`Modules/message_center/`、`Modules/algorithm/controller/`、`Modules/vofa/`、`Bsp/usart/`、`Bsp/log/`。
- `UserApp/os_task.c`（任务框架）、`UserApp/robot/dart_fc/`（pre 原型参考）。
