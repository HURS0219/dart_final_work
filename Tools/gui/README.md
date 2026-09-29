# Tools/gui — 制导飞镖可视化上位机

<p align='right'>制导飞镖 · dart_final_test_app 配套</p>

基于 `UserApp/robot/dart_final_test_app` 的命令/输出接口做的可视化上位机：

- **输入**：用**滑条**产生命令（模拟绿光点 x / 4 路舵面逻辑角 / 混控 PYR）；
- **输出**：把设备回传的文本**打印**出来（点 `STAT` 按钮打印一次状态表）。

## 传输通道（两种，二选一）

| 传输 | 需要 | 说明 |
|---|---|---|
| **RTT (J-Link SWD)** **默认** | 仅 J-Link + SWD | 只用 SWD 就能用；命令走 RTT down ch0，输出走 up ch0 |
| Serial (COM) | 板载 USB-CDC 线 | 板子 USB 插电脑 → 虚拟串口；命令/输出同 RTT |

> 用 RTT 时 **J-Link 被上位机独占**：期间不能同时开 RTT Viewer / `rtt_send.ps1` / 烧录（要烧录先点「断开」）。

## 架构（输入 / 输出 隔离）

```
                 core/link.py  (隔离边界: send(cmd) / on_line(cb))
   input/panel ────────────────┤├──────────────── output/panel
   (滑条 -> 命令)               │                    (收到的文本)
                    core/link_rtt.py  |  core/link_serial.py
                     (J-Link RTT/SWD) |  (pyserial COM)
```

- `input/` 只调 `link.send(cmd)`；`output/` 只订阅 `link.on_line`；**二者互不 import**。
- 传输实现放 `core/`，面板不关心是 RTT 还是串口。

## 目录

```
Tools/gui/
├─ main.py                 入口 (python main.py)
├─ requirements.txt        PyQt5 / pyserial / pylink-square / pyinstaller
├─ build_exe.ps1           打包 -> dist\dart_test_gui.exe
├─ core/
│   ├─ link.py             隔离边界(抽象)
│   ├─ link_rtt.py         J-Link RTT/SWD 实现 (pylink-square)
│   └─ link_serial.py      pyserial 实现 + 串口枚举
├─ input/
│   ├─ slider.py           ★滑条接口对象 SliderControl(极值/输入值 -> 命令串)
│   └─ panel.py            输入面板(target / 4×fin / mix PYR)
└─ output/
    └─ panel.py            输出面板(文本 + STAT 按钮)
```

## 滑条接口对象 `SliderControl`

对象输入：两端极值 `lo/hi` + 实际用户输入值 `value`；
对象输出：由 `value` 生成命令串（`formatter` 注入）。
拖动时**限频 ~20Hz** 发 `commandReady`，松手立即再发一次。

## 命令映射

| 面板 | 控件 | 命令 |
|---|---|---|
| target | x: 0..319 | `TGT,<x>,120`（y 固定 120；只调 x） |
| target | 按钮「目标丢失」 | `TGTN` |
| 舵面 | fin0..3: -35..35° | `FIN,<ch>,<deg>`（逻辑角，标定在 servo_motor cfg 内） |
| 混控 | p/y/r: -1..1 | `FMIX,<p>,<y>,<r>` |
| 输出 | 按钮「STAT」 | `STAT`（打印一次状态表） |

> 命令/输出格式与 test app 完全一致，详见 `UserApp/robot/dart_final_test_app/README.md`。
> 注入类命令**无回执** → 发完后点「STAT」看结果。

## 用法

```powershell
cd Tools\gui
python -m pip install -r requirements.txt
python main.py
```
1. 传输选 **RTT (J-Link SWD)**（默认，device `STM32F407IG`, speed `4000`）→「连接」；
2. 拖滑条 → 自动发命令；点「STAT」→ 看状态表。

## 打包 exe

```powershell
powershell -File Tools\gui\build_exe.ps1        # 产物: Tools\gui\dist\dart_test_gui.exe
powershell -File Tools\gui\build_exe.ps1 -Clean # 先清理
```

## 注意

- 固件需 **`TEST_IMU_ENABLE=0`**（用 `ATT` 注入姿态；本板真实 IMU 初始化会卡在 `SPI_WaitFlagStateUntilTimeout`）。
  若要测 guidance，需先用命令注入姿态（RTT/串口发 `ATT,<r>,<p>,<y>`）。
- 测「混控/舵面」时 `guidance` 每周期也在发 `mix` 会盖掉注入的 `FMIX`；想纯手测先 `RESET,GUIDANCE`。
