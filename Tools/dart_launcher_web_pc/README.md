# dart_launcher_web_pc — 用 J-Link 跑发射架网页(不需要 ESP)

ESP 坏了/不想用 ESP 时, 用**只用 J-Link(SWD)** 的方案:
PC 运行一个本地网页服务器, 经 **J-Link RTT** 直接和 C 板(STM32F407)通信——
网页 UI 与协议跟 ESP 版完全一样(复用同一套 `www/` 与 `proto.py`)。

```
浏览器  http://127.0.0.1:8000
   │  HTTP
PC: web.py (复用 ESP 的网页与 API)
   │  proto.Bus.send/poll
rtt_uart.py  ──J-Link RTT 通道1──>  C 板 dart_link.c (RTT)
```

## 前置
- C 板已烧 **`dart_launcher_web_v5_HIK`**(含 RTT 支持, 见 `dart_link.c` 的 `DART_RTT_CH`)。
- J-Link 用 **SWD** 接 C 板(和烧录同一根线, 无需额外 UART/ESP)。
- `pip install pylink-square`

## 运行
```powershell
cd Tools\dart_launcher_web_pc
python main.py
# 然后浏览器打开 http://127.0.0.1:8000
```

## 文件
| 文件 | 作用 |
|---|---|
| `main.py` | 入口: 开 J-Link RTT + 起 HTTP |
| `rtt_uart.py` | 用 RTT 模拟 UART(通道1), 供 `proto.Bus` 直接用 |
| `proto.py` | 总线驱动(与 ESP 版仅 `UART` 来源不同) |
| `motor.py`/`servo.py`/`task.py` | 与 ESP 版**完全相同** |
| `web.py` | HTTP + API(监听 `127.0.0.1:8000`) |
| `www/` | 网页静态文件(与 ESP 版相同) |

## 说明 / 限制
- 同一根 J-Link 同一时刻只能被一个程序占用(RTT Viewer / 烧录 / 本工具 不能同时开)。
- 命令走 RTT 下行通道1(**缓冲区 512B**), 单条命令短, 足够。
- 遥测走 RTT 上行通道1(缓冲区 4KB)。
- 与 ESP 版协议一致, 因此两端(固件 + 本工具)必须同为 v5_HIK 协议(含旧的 `W` 圈数命令)。
