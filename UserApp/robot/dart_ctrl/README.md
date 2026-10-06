# dart_ctrl · 制导飞镖整机飞控 (单文件 · 组合根直连)

<p align='right'>由 dart_final 改写 · SHU SRM</p>

`dart_ctrl` 是制导飞镖的**整机机电飞控**：上电回中 → 识别绿光且俯冲后进入制导 → 比例导引(PNG)解算 → 四舵面 X 型混控。

> **架构说明（重要）**：本 app 采用本仓库 userapp 的统一风格 —— **一个 `robot.c` 组合根 + `RobotInstance` 直连数据**，**不使用 `message_center` 发布订阅**，也不再拆分 `app/imu|vision|guidance|fin`。
> 由 `dart_final`（多 app + 伪 pub-sub）改写而来，功能等价，耦合更简单。

---

## 1. 架构与数据流

```
                 robot.c (唯一文件)  ── RobotTask() 顺序调用 ──
   ImuStep ──> robot->attitude ┐
                               ├─> GuidStep ──> robot->mix ──> FinStep ──> 4 舵机 PWM
   VisionStep ─> robot->target ┘    (PNG + roll PID)          (混控+标定)   └─> robot->servo_fb

   StateStep: 状态机 IDLE/ARMED/GUIDING/FAULT ; MonitorStep: 健康位
```

- **Module（复用本仓库现成的，不改）**：`ins_task`(IMU/姿态)、`servo_motor`(PWM 舵机)、`controller`(PID)、`bsp_usart`(视觉串口)、`crc8`(帧校验)、`bsp_dwt`/`bsp_log`、`user_lib`。
- **算法内联**：`png_ai`(比例导引) 与 `servo_mix_ai`(四舵面混控) 无对应 Module，**已内联进 `robot.c`**（`PngCalc()` / `FinStep()` + `robot_config.h` 参数）。

---

## 2. 文件清单

| 文件 | 作用 |
|---|---|
| `robot.cmake` | 收集本 app 源码（极简；板→MCU 映射由顶层 CMake 负责） |
| `robot.h` | `RobotInstance` + 数据结构 + 健康位 + `RobotInit/RobotTask` 声明 |
| `robot_config.h` | **本 app 全部可调参数**（端口占位 / 舵机 / png / PID / 阈值） |
| `robot.c` | **全部逻辑**：RobotInit + RobotTask(IMU/视觉/监控/状态机/制导/舵面/遥测) |
| `README.md` / `HARDWARE.md` / `LOG.md` / `F405_TODO.md` | 文档 |

---

## 3. 数据（`robot.h` 的 `RobotInstance`）

| 字段 | 生产者 | 消费者 | 类型 |
|---|---|---|---|
| `attitude` | `ImuStep` | `GuidStep` / 状态机 | `Dart_Attitude_s` |
| `target` | `VisionStep` | `GuidStep` / 状态机 | `Dart_Target_s` |
| `mix` | `GuidStep` | `FinStep` | `Dart_Mix_s` |
| `servo_fb` | `FinStep` | 调试/遥测 | `Dart_ServoFb_s` |

> 无话题、无 `PubRegister/SubRegister`；各步骤直接读写 `robot->xxx`（app 内直连）。

---

## 4. 状态机

```
IDLE ──使能──> ARMED ──ShouldGuide()──> GUIDING
  ^              ^  |                        |
  |              |  └────── 致命故障 ──> FAULT ──故障消除──> ARMED
  └──── 撤销使能 ┴─────────── 撤销使能 ─────────┘
```
- **IDLE**：未使能，舵面回中。
- **ARMED**：已使能、待触发，舵面回中。
- **GUIDING**：制导输出。
- **FAULT**：致命故障（**姿态无效 或 舵机注册失败**）→ 回中；消除后回 ARMED。
- 非 GUIDING 时 `GuidStep(guide_enable=0)` → `mix.failsafe=1` → `FinStep` 回中。（视觉掉线**非致命**，同样回中。）

---

## 5. 【待配置接口】（**赛事规则出来后只改这里！**）

`robot.c` 顶部两个函数：
```c
/* 【接口1】使能: 现在=按 cfg DART_ENABLE_ON_BOOT(默认上电即 ARMED) */
static bool Dart_IsEnabled(void);
/* 【接口2】制导时机: 现在=识别到绿光(target.found) && 姿态俯冲(Pitch<DIVE_PITCH_DEG) */
static bool Dart_ShouldGuide(void);
```
> 备选触发（未采用）：**IMU 检测发射过载**（弹射瞬间加速度尖峰阈值）。需要时改 `Dart_ShouldGuide`。

---

## 6. 控制律 & 调参对照表

只控 **yaw 制导 + roll 稳定**，pitch 恒 0（目标水平移动）：
```
guidance:  λ_yaw = (x-CX)/focal ; dλ = LPF(Δλ/Δt) ; a_cmd = N·v_c·dλ (PPN)
           mix.yaw   = k * a_cmd          // 制导侧
           mix.roll  = roll_PID(roll,0)   // 姿态稳定(controller PID)
           mix.pitch = 0
fin:       target[i] = clamp(Σ matrix[i][j]*mix[j] * DART_MIX_MAX_DEG, ±MAX_DEG)
```

| 现象 | 改哪个（`robot_config.h`） | 说明 |
|---|---|---|
| **整机过偏/欠偏** | `DART_MIX_MAX_DEG` | 全局舵面行程增益 |
| **某一路**偏多/偏少 | `DART_SERVO_SCALE[ch]` | 该路比例（安装误差） |
| **制导震荡/收敛慢** | `PNG_N` 或 `GUID_GAIN_K` | 制导律强度 |
| **滚转压不住/来回摆** | `ROLL_KP` / `ROLL_KD` | roll PID |
| 零点/中位偏 | `DART_SERVO_TRIM[ch]` | 逐路机械零点 |
| 方向反 | `DART_SERVO_REVERSE[ch]` | 逐路（改后重新对零） |

> `k` 固定为物理值 `1/PNG_MAX_OUT`；“过偏”统一调 `DART_MIX_MAX_DEG`（舵偏→升力小 δ 近似线性，故线性 k 是一阶近似）。

---

## 7. 协议契约

### 7.1 OpenMV → STM32（7 字节定长帧，大端）
```
[0]0xAA [1]0x55 [2]X_hi [3]X_lo [4]Y_hi [5]Y_lo [6]CRC8
CRC8: SHT75(poly=0x31, init=0), 对 [0..5] 校验
丢失目标: X=Y=0
```
STM32 端解析见 `robot.c` 的 `Vision_RxCallback`（`USARTRegister` 回调 + `crc_8`）。

### 7.2 STM32 ↔ ESP32（预留）
当前仅占位串口；无线烧录方案见 `F405_TODO.md`。

---

## 8. 引脚 / 端口（**板子留空，先占位**）

| 用途 | 宏（`robot_config.h`） | 当前占位(C 板) |
|---|---|---|
| OpenMV UART | `DART_USART_OPENMV` | `&huart3` |
| ESP32 | `DART_USART_ESP32` | `&huart6` |
| VOFA（预留） | `DART_USART_VOFA` | `&huart1` |
| 4 舵机 | `DART_SERVO_TIM` | `&htim1` CH1–4 |

> 实板(F405)网表见 `HARDWARE.md`；上板后**只改本文件端口/定时器**。

---

## 9. 构建 / 烧录

本 app 用顶层 CMake 的 `ROBOT_TYPE` 选择（无需改框架）：
```powershell
# 例(以后确定板子后):
cmake -G Ninja -S . -B build -DROBOT_TYPE=dart_ctrl -DMCU_TYPE=stm32-f4 -DbuildType=Debug
cmake --build build
```
默认板 `MCU_TYPE=stm32-f4`（C 板，4 舵机 = TIM1 CH1–4 = PE9/PE11/PE13/PE14）。F405 实板待定。

---

## 10. 调参 / 标定 SOP

1. **单路舵机**：先确认每路中位、方向、行程（可临时把 `DART_SERVO_*` 逐路单独给值）。
2. **四舵面混控**：改逐路 `SCALE/TRIM/REVERSE`；上电确认 4 舵面回中。
3. **滚转稳定**：先 `ROLL_KP`（角度）再 `ROLL_KD`（阻尼），直到无持续滚转、无明显摆振。
4. **制导强度**：`PNG_N`、`GUID_DLC_V_C`、`GUID_GAIN_K`；观察 `mix.yaw` 是否平滑。
5. **过偏**：最后用 `DART_MIX_MAX_DEG` 统一收紧/放大。

---

## 11. 与 dart_final 的差异（本次改写）

| 项 | dart_final | dart_ctrl(本 app) |
|---|---|---|
| 文件 | `robot.c` + `robot_def.h` + `app/{imu,vision,guidance,fin}` + 多个 cfg | **单文件 `robot.c`** + `robot.h` + `robot_config.h` |
| 通信 | `message_center` 伪 pub-sub 话题 | **`RobotInstance` 直连** |
| png_ai / servo_mix_ai | Modules 模块 | **内联进 app**（本仓库无此 Module） |
| servo_motor | 源仓库版(center/half/trim) | **干净库版(PWM, 直接吃占空比)** + app 内做角度→占空比映射 |
| IMU | `INS_GetAttitude()` + rad/s | **干净库 `ins_task`**：`INS_Init`→`INS_t*`，Gyro 已是 °/s |

---

## 12. 相关文件

- Modules：`Modules/imu/ins_task.*`、`Modules/motor/servo_motor/`、`Modules/algorithm/controller/`、`Modules/algorithm/crc8/`、`Bsp/usart/`、`Bsp/log/`、`Bsp/dwt/`。
- 框架：`UserApp/os_task.c`（调用 `RobotInit/RobotTask`）。
- 源参考：`dart_final`（另一仓库，含 `app/` + pub-sub 版）。
