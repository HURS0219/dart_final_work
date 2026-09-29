# dart_final_test_app · 制导飞镖分-app 调试工具

<p align='right'>ai @ dart_final_work</p>

用于**逐个黑盒测试** `dart_final` 的各 app 服务：姿态可用**真实 IMU**（C 板板载 BMI088）或注入，视觉/ESP32 未就绪也能在台架上验证：
自底向上 **fin（舵机）→ guidance → vision / imu**。`test.c` 代替 `robot.c`，复用 `dart_final/app/*`。

> 每个 app 当**黑盒**：只“注入它的输入、看它的输出”。例：测 guidance 就注入 `attitude`+`target`，只看它输出的 `mix`。

---

## 1. 为什么用它
- 硬件通信（OpenMV 串口 / ESP32 / 甚至 IMU）还没就绪时，也能验证每个 app 是否**独立正常运行**；
- 提供 **ros2 风格**的状态表（每个 pub/sub 的状态 + 值 + 时间戳），发 `STAT` 打印一次；也可开周期 log（供 MATLAB）。

## 2. 编译 / 烧录
```powershell
# 编译
powershell -ExecutionPolicy Bypass -File make_one\build.ps1 -Robot dart_final_test_app -Board GIMBAL_BOARD
# 产物: make_one\build_dart_final_test_app\control-2026.hex

# 烧录 / 复位
powershell -File Tools\scripts\flash.ps1 -Hex make_one\build_dart_final_test_app\control-2026.hex
powershell -File Tools\scripts\reset.ps1
```

## 3. 命令通道（USB-CDC 串口 / RTT）
同时支持两路，命令与输出完全一致：
- **板载 USB-CDC 虚拟串口**（板子 USB 插电脑 → 出现 COM 口，波特率随意，如 115200）：给 `Tools/gui` 可视化上位机用；
- **J-Link RTT**（SWD）：RTT Viewer 底部输入框，或 `Tools\scripts\rtt_send.ps1 -Cmd "<命令>" -Elf make_one\build_dart_final_test_app\control-2026.elf`。

### 命令表
| 命令 | 作用 |
|---|---|
| `PING` | 回 `PONG`（测通道） |
| `ATT,<roll>,<pitch>,<yaw>` | 注入姿态（发布 `attitude`） |
| `TGT,<x>,<y>` | 注入目标（发布 `target`，found=1） |
| `TGTN` | 注入“目标丢失”（found=0） |
| `FMIX,<p>,<y>,<r>` | 注入混控指令（发布 `mix`，−1..1） |
| `FIN,<ch>,<deg>` | fin 切 MANUAL，给第 ch 路(0..3)逻辑角 |
| `RESET` | 软复位测试系统（全使能 + 清零监视） |
| `RESET,<APP>` | 单独关掉某 app；APP ∈ `IMU / VISION / GUIDANCE / FIN` |
| `STAT` | 立即打印一次状态表（**唯一的按需输出**） |

> 注入类命令（`ATT`/`TGT`/`TGTN`/`FMIX`/`FIN`/`RESET`）**无回执** → 发完后用 `STAT` 查看结果；`PING` 例外，会回 `PONG`。

## 4. 状态表（ros2 风格）
**静默运行**：默认**不自动输出**，只在你发 **`STAT`** 时打印一次。格式：
```
[t=0012s] <话题>  prod=<生产者app>:1/0  sub=<消费者app>:1/0  data=1/0 | 值...
```
- **prod=1**：该话题的**生产者 app 正在运行**（其任务心跳在涨，SWD/RTT 可读）；
- **sub=1**：该话题的**消费者 app 正在运行**；
- **data=1**：该话题**近期（≤500ms）有推送**。

> 生产者/消费者映射见 `test_cfg.h`：`attitude`=IMU→GUIDANCE、`target`=VISION→GUIDANCE、`mix`=GUIDANCE→FIN、`servo_fb`=FIN→(监视)。
> `RESET,IMU` 后你会看到 `attitude` 的 `prod=0`，一眼定位“IMU 没在发”。

**数值单位（`nano.specs` 禁 `%f`，故用整数定标）**：
| 字段 | 含义 | 换算 |
|---|---|---|
| `roll10/pitch10/yaw10` | 姿态角 | ÷10 → deg |
| `gx10/gy10/gz10` | 角速率 | ÷10 → deg/s |
| `x,y` | 目标中心像素 | 原值（QVGA 0..319 / 0..239） |
| `p100/y100/r100` | 混控指令 | ÷100 → −1..1 |
| `d10` | 4 路逻辑角 | ÷10 → deg（±35°） |
| `p` | 4 路脉宽 | 原值 us（500~2500） |
| `fs` | mix 失效标志 | 1 时下附 `fsreason=NO_ENABLE/NO_VISION/NO_ATTITUDE` |

## 5. 日志 / 存档（可选）
- **默认静默**（`LOG_PERIOD_MS=0`）：RTT 不自动输出，只在你发 `STAT` 时打印一次；
- 若想**周期存档**到 `Debug\`：把 `test_cfg.h` 的 `LOG_PERIOD_MS` 设为 `1000`（每秒一条），再跑
  `powershell -File Tools\scripts\rtt_log.ps1 -Out Debug\dart_test_log.txt`（Ctrl+C 结束）；
- MATLAB 里按上面表格 ÷10 / ÷100 还原物理量。

## 6. 标准测试流程（自底向上）
1. **fin / 舵机**（最底层）
   - `STAT` → 确认 `fin prod=1`、`servo_fb data=1 p=1500`（中位）；
   - `FIN,0,20` → `servo_fb d10=200 p≈1643`（ch0 +20°，脉宽 1500+20×7.17）；`FIN,0,-20` 反向；逐路 0..3；
   - `FMIX,50,0,0` / `FMIX,0,50,0` / `FMIX,0,0,50`（注意是 −1..1，故 0.5）→ 看混控解耦；再用 `RESET` 或发 `FMIX,0,0,0` 回中。
2. **guidance**（黑盒 → 只看 `mix`）
   - `ATT,0,0,0` + `TGT,160,120` → `mix fs` 应由 1 变 **0**；
   - 改 `TGT` 到偏心（如 `TGT,240,120`）→ 观察 `mix` 的 `y100` 变化（LOS 速率→指令）。
3. **vision**：接真实 OpenMV 时，`TEST_VISION_ENABLE=1` 编，看 `target` 由真实帧驱动；否则用 `TGT` 注入对照。
4. **imu**：接真实 BMI088 时，`TEST_IMU_ENABLE=1` 编，看 `attitude valid=1` 与角速率；否则用 `ATT` 注入对照。

## 7. 配置（`test_cfg.h`）
| 宏 | 默认 | 说明 |
|---|---|---|
| `TEST_IMU_ENABLE` | 0 | 0=用 `ATT` 注入姿态；1=真实 BMI088（**本板实测会卡在 `SPI_WaitFlagStateUntilTimeout`，暂用 0**） |
| `TEST_VISION_ENABLE` | 0 | 0=不注册 OpenMV 串口，用 `TGT` 注入 |
| `LOG_PERIOD_MS` | 0 | 自动日志周期（**0=静默，仅 `STAT`**；>0 每秒一条，存档用） |
| `DATA_FRESH_MS` | 500 | 数据“新鲜”判定窗口 |
| `TOPIC_PRODUCER/CONSUMER/NAME` | — | 话题↔生产者/消费者映射 |

## 8. 注意
- 本 app **无状态机**：`guidance` 恒 `guide_enable=1`，故 `fs` 只随 `target.found` / `attitude.valid` 变化。
- 注入走**话题发布**，不破坏 pub-sub 结构；测真实硬件前先 `RESET`。
- 复用 `dart_final/app/*`（`robot.cmake` include+glob），**不含** `dart_final/robot.c`。
- 相关：`dart_final/README.md`（整机飞控）、`Modules/algorithm/openmv`（7 字节帧）、`Tools/scripts`（脚本）。
