# ESP32-S3-CAM 上位机 — 接线与使用

<p align='right'>制导飞镖发射架 · dart_launcher_final</p>

本目录是发射架的**网页控制台 + 串口桥**，运行在 **ESP32-S3-CAM**（MicroPython）上，
通过 UART 与 C 板（STM32F407）通信，并通过 WiFi 提供网页界面。

---

## 1. 接线

### 1.1 ESP32-S3-CAM ↔ C 板（UART1 ↔ USART6）

| ESP32-S3-CAM | 方向 | C 板 (STM32F407) | 说明 |
|---|---|---|---|
| **GPIO4** | TX → | **PG9** | C 板 `USART6_RX` |
| **GPIO5** | RX ← | **PG14** | C 板 `USART6_TX` |
| **GND** | — | **GND** | **必须共地** |

- 波特率 **115200 8N1**（两端一致；C 板侧 `LAUNCH_LINK_UART = &huart6`）
- 交叉接线：**ESP32 的 TX 接 C 板的 RX**，不要接反

### 1.2 ESP32 ↔ PC（USB）

| 板上的 Type-C 口 | 芯片 | 用途 | 本工程 |
|---|---|---|---|
| **OTG**（左边） | 原生 USB (GPIO19/20) | 对应 MicroPython 的 `sys.stdin` | ✅ **必须插这个** |
| UART（右边） | CH343 转串口 (GPIO43/44) | REPL / 调试日志 | 可选，用于看日志 |

> ⚠ **`main.py` 用 `sys.stdin` 做 PC↔C板 双向桥，只有 OTG 口才有数据。**
> 插 UART 口时桥接会静默失效（无报错，但 PC 发的指令到不了 C 板）。

双向桥的数据流：

```
PC --OTG--> ESP32 --UART1(GPIO4/5)--> C 板        (发指令: N/M/P/G/C ...)
C 板 --UART1--> ESP32 --OTG--> PC                 (回遥测: F,... 帧)
```

---

## 2. 引脚占用（ESP32-S3-CAM）

### 本工程使用

| 引脚 | 用途 |
|---|---|
| GPIO4 | UART1 TX → C 板 |
| GPIO5 | UART1 RX ← C 板 |
| GPIO19 / GPIO20 | 原生 USB（PC 桥，由 MicroPython 占用） |

### ⚠ 绝不能占用

| 引脚 | 占用者 |
|---|---|
| GPIO26 ~ GPIO32 | 内部 SPI Flash |
| **GPIO35 / GPIO36 / GPIO37** | **PSRAM**（板上明确标注） |
| GPIO19 / GPIO20 | 原生 USB（PC 桥靠它） |
| GPIO0 / GPIO45 / GPIO46 | Strapping（启动模式） |
| GPIO43 / GPIO44 | U0TXD/U0RXD（调试串口；若从 UART 口取日志则勿占用） |

### 说明：为什么不用 GPIO17/18

板上 GPIO17/18 的标注是 **`CAM_*`（摄像头/SD 预留）**。虽然排针引出、不接摄像头时
可以借用，但：

1. 换一块**非 CAM 版**的 S3，这两个脚未必引出；
2. 一旦以后要加摄像头就直接冲突。

故改用通用 IO **GPIO4/5**，可移植性更好。旧版 `dart_launcher_web_v5_HIK` 用的是 17/18，
本版已改。

---

## 3. 使用步骤

1. **接线**：按 §1.1 接好 UART1 与 GND；USB 线插 **OTG** 口
2. **烧录 C 板固件**：
   ```powershell
   powershell -File make_one\build.ps1 -Robot dart_launcher_final -Board GIMBAL_BOARD
   powershell -File Tools\scripts\flash.ps1 -Hex make_one\build_dart_launcher_final\control-2026.hex
   ```
3. **拷贝本目录到 ESP32**：`proto.py` `motor.py` `servo.py` `task.py` `web.py` `main.py`
   以及 `www/` 整个目录（保持相对结构）
4. **运行 `main.py`**，串口会打印：
   ```
   AP ready: 192.168.4.1
   HTTP server listening on :80
   ```
5. **连 WiFi**：SSID `DART_CTRL`，密码 `12345678`
6. **打开浏览器**：`http://192.168.4.1`

---

## 4. 协议速查（与 C 板固件对应）

### 下行指令

| 命令 | 含义 |
|---|---|
| `PING` | 回 `PONG` |
| `H` | 心跳（桥接脚本每 500ms 自动发） |
| `S` | 扫描在线电机 → `S,<n>,<id...>` |
| `Z[,slot]` | 取零；省略 slot = 全部 |
| `M,slot,mode,value` | 电机模式+目标（`mode`: 0=STOP 1=SPEED 2=ANGLE；value ×10） |
| **`N,slot,deg`** | **设目标角度（整数 deg，输出侧）** ← 取代旧的 `W,turns100` |
| `P,slot,id,value` | PID 参数（×100） |
| `R,slot` | 恢复该路 cfg 默认参数 |
| `D,slot` | 反转该路方向 |
| `Y,mode` | yaw 模式（0=手动 1=自瞄 2=引导） |
| `A,rpm100` | 自瞄最大转速 |
| `C,x,center` | 注入视觉坐标（PC/Jetson 用） |
| `V,a[,b]` | 舵机操作 |
| `G,cmd` | 时序 / 急停（10=启动 11=停止 12=急停 13=解除） |
| ~~`W,...`~~ | **已废弃** → 回 `ERR W_DEPRECATED use_N,slot,deg` |
| `SAVE` | 回 `SAVED` 但**不持久化**（本工程纯 cfg，无 Flash） |

### 上行遥测（单行 `F,...`）

```
F,<n>,
  每电机 12 项 × n : slot,type,id,online,dir,mode,target100,rpm,angle100,deg100,temp,cur
  舵机 4 项        : cur10,state,std10,prep10
  任务 5 项        : spring_a_deg,step,yaw,estop,aim100
  视觉 4 项        : x,ok,center,err
  每电机 11 参数 × n
```

> **`n = 3`**（拉簧A / 拉簧B / 丝杆）。**yaw 已独立成 app，不在其中**，
> 其状态见 `task.yaw`（模式）与 `yaw_fb`。
>
> 字段**个数与顺序**与旧版一致，仅语义变化（圈 → 角度），故解析不会错位。

---

## 5. 文件说明

| 文件 | 作用 |
|---|---|
| `main.py` | 入口：拉起 AP + HTTP 服务 + PC↔C板 串口桥 |
| `proto.py` | 总线驱动：UART 收发、指令生成、遥测解析 |
| `motor.py` | 电机对象（M3508/M2006/GM6020） |
| `servo.py` | 舵机对象（PTK7350/PTK7465） |
| `task.py` | 自动发射时序对象 |
| `web.py` | HTTP 服务 + 结构化 API |
| `www/` | 网页静态文件（`index.html` + `js/` + `style.css`） |

---

## 6. 常见问题

**Q: 网页连不上 / 一直显示离线**
- 确认 USB 插的是 **OTG** 口
- 确认 GPIO4/5 与 C 板 PG9/PG14 **交叉**接对，且 **GND 共地**
- 确认 C 板已烧 `dart_launcher_final` 固件并运行
- 用 `PING` 测试：网页"识别连接"页点测试，应回 `PONG`

**Q: 电机显示离线（LED 不亮）**
- C 板侧看 RTT 日志是否刷 `Motor lost` → 说明 CAN 总线无应答
- 检查电机供电、CAN_H/CAN_L 接线、120Ω 终端电阻、电机 ID

**Q: 点"上膛"没反应**
- 确认用的是 v1.0.7+ 固件与配套前端（`N` 命令）。若固件是旧版（只认 `W`），
  前端会收到 `ERR`。两边必须配套。

**Q: 参数改了重启就没了**
- 设计如此：本工程**纯 cfg、不用 Flash**。网页改的是运行时值，满意后需写回
  `UserApp/robot/dart_launcher_final/launcher_cfg.h` 再烧录。
