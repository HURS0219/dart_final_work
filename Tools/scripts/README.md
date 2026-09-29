# Tools/scripts — 复用调试脚本

<p align='right'>制导飞镖 · opencode/deepseek-flash</p>

一组与具体项目无关、可**跨 robot 复用**的调试脚本。用于：编译、烧录、复位、RTT 注入、读内存、git 推送。
所有脚本都**参数化**、尽量**动态解析符号**（重编后无需改地址）。

> 依赖：`arm-none-eabi-nm.exe`（工具链）、`JLink.exe`（SEGGER）、Git、Python3。默认路径见各脚本参数，可覆盖。

---

## 脚本清单

### `build_all.ps1` — 一键编译所有 robot
```powershell
powershell -File Tools\scripts\build_all.ps1 -Board GIMBAL_BOARD
```
遍历 `UserApp\robot\*\robot.cmake`，逐个调 `make_one\build.ps1`，最后打印 OK/FAIL 汇总（有失败则退出码 1，便于 CI）。改动公共代码后做全量回归很有用。

### `flash.ps1` — 烧录 hex
```powershell
powershell -File Tools\scripts\flash.ps1 -Hex make_one\build_dart_final\control-2026.hex [-Device STM32F407IG]
```

### `oneclick_flash.ps1` — 一键编译 + 烧录（VS Code 任务的后端）
一步完成「编译指定 robot → 用 J-Link 烧录」。封装 `build.ps1` + `flash.ps1`，被 `.vscode/tasks.json` 调用，也可命令行用。
```powershell
powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final                 # 编译 + 烧录
powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final_test_app        # 换 robot
powershell -File Tools\scripts\oneclick_flash.ps1 -Robot servo_test -Clean          # 清洁重编 + 烧录
powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final -NoBuild        # 只烧录已有 hex
powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final -NoFlash        # 只编译不烧录
powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final -Device STM32H723ZG
```
`-Device` 缺省按板子推断（`GIMBAL_BOARD`/`ONE_BOARD`→`STM32F407IG`，`CHASSIS_BOARD`→`STM32H723ZG`），可覆盖。

### VS Code 一键烧录（`.vscode/`）
仓库已随附 `.vscode/tasks.json`（+`settings.json`/`extensions.json`），装好推荐插件后即可用：
- **`Ctrl+Shift+B`** → 默认任务 **「Flash: 一键烧录 (选择 robot)」**：**先弹框选 robot**（默认高亮 `dart_final`，也可选 `dart_final_test_app` 等），再选 board，然后编译+烧录 —— **杜绝误烧**（不再默认烧 `dart_final`）。
  脚本开头会大字打印 `#### ROBOT = xxx  BOARD = xxx ####`，烧前再打印所用 hex 及其时间戳，便于确认烧的是谁。
- 命令面板 `Tasks: Run Task` → 可选：
  - `Flash: 一键烧录 dart_final_test_app`；
  - `Flash: 一键烧录 (选择 robot)` / `仅烧录, 不编译` / `清洁重编 + 烧录`（会弹出 robot、board 选择）；
  - `Build: 仅编译 (选择 robot)`、`Build: 编译全部 robot (回归)`；
  - `Board: 复位并运行 (Reset)`。
- 依赖外部命令：`powershell`（系统自带）、`cmake`+`ninja`、`arm-none-eabi` 工具链、`JLink.exe`（路径见 `flash.ps1` 的 `-JLink`，默认 `C:\Program Files\SEGGER\JLink_V966\JLink.exe`）。
- `.gitignore` 只放行上述 3 个共享配置文件，其余 `.vscode/*` 仍被忽略（个人设置不入库）。

### `reset.ps1` — 复位并运行
```powershell
powershell -File Tools\scripts\reset.ps1 [-Device STM32F407IG]
```

### `rtt_send.ps1` — 向 RTT 注入命令（**核心工具**）
无法在 J-Link RTT Viewer 里手动打字时，用它把一行命令喂给固件。
- 动态解析 ELF 的 `_acDownBuffer` / `_SEGGER_RTT`，计算通道 0 的 `WrOff/RdOff`（偏移由 `SEGGER_RTT_MAX_NUM_UP_BUFFERS` 决定，默认 3 → `+0x6C/+0x70`）。
- 写缓冲 → 清 `RdOff` → 置 `WrOff` 触发固件读取。
```powershell
powershell -File Tools\scripts\rtt_send.ps1 -Cmd "PING" -Elf make_one\build_servo_test\control-2026.elf
```
> 命令最长 16B（RTT down buffer 默认 16B）。若改了 `SEGGER_RTT_Conf.h` 的 `MAX_NUM_UP_BUFFERS`，用 `-MaxNumUpBuffers N` 同步。

### `mem_read.py` — nm 解析符号 + J-Link 读内存解码
```powershell
# 读符号 g_servo 处 21 个 word，下标 7..15,17..19 按 float 打印
python Tools\scripts\mem_read.py --elf make_one\build_servo_test\control-2026.elf --sym g_servo --words 21 --floats 7 8 9 10 11 12 13 14 15 17 18 19
# g_servo 存的是指针 -> 加 --deref
python Tools\scripts\mem_read.py --elf <elf> --sym g_servo --deref --words 21
# 直接读地址(如 Flash bank)
python Tools\scripts\mem_read.py --elf <elf> --addr 0x080C0000 --words 3
```

### `git_push_443.ps1` — 走 443 推送
GitHub 的 SSH 22 端口常被网络拒绝，本脚本用 `core.sshCommand` 指定经 `ssh.github.com:443` 推送。
```powershell
powershell -File Tools\scripts\git_push_443.ps1                # 默认 main + --tags
powershell -File Tools\scripts\git_push_443.ps1 -Refs main,smc # 指定分支
```

---

## 备注

- **务必**用 `-Device` 与实际芯片一致（F4 板 `STM32F407IG`；H7 板另填）。
- 读 RTT/内存前，目标需在运行（J-Link 会短暂 halt 再 go）。
- 脚本只为**调试**方便，不参与固件编译；放仓库里便于团队与后续 agent 复用。
- 更多工作流见上级目录 `Tools/AGENTS.md`、`Tools/agent_profile.md`。
