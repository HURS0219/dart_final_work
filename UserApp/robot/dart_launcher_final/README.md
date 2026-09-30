# dart_launcher_final · 制导飞镖发射架整机控制

<p align='right'>ai @ dart_final_work</p>

`dart_launcher_final` 是制导飞镖**发射架**的整机控制 app：**两根拉簧上膛 + 自锁丝杆 + 扳机舵机 + yaw 自瞄轴**。
按 `UserApp/application.md` 与 `APP层应用编写指引.md` 的架构规范，做成**7 个平行 app**，经 `message_center`(伪 pub-sub) 通信，参数全部走 cfg、**不使用 Flash**。

> 与旧版 `dart_launcher_web_v5_HIK` 的区别：旧版是"6 个模块互相 include + 12 个可写全局变量"的平面结构（`dart_link` 直接调 `MotorSet`/`ServoGo`/`FsmAutoStart` 并读 `Axis[]`）；
> 本版严格解耦——**app 之间零 include、零全局变量**，只收发话题。旧版保留不动作为对照。

---

## 1. 架构与数据流

```
                        message_center (伪 pub-sub 话题)
   app/link ──launch_cmd──> app/cmd ──fsm_cmd────> app/fsm
      ^                       │                       │
      │                       │        ┌──────────────┴─────────────┐
      │                       │        v                            v
      │                       │   motor_cmd                  servo_cmd
      │                       │        │                            │
      │                       │        v                            v
      │                       │  app/motor ──> 4x DJImotor    app/trigger ──> 舵机
      │                       │  (CAN1)                       (servo_motor)
      │                       v
      │                 launch_state ──> app/yaw
      │                                      ^
      │                                      │
      │                                 aim_cmd
      │                                      │
      │              app/vision  <──vision_cmd── app/link
      │
      └<── motor_fb / servo_fb / yaw_fb / launch_state ── (各 app 反馈)

   robot.c: 只建话题 + 周期调用各 app + Monitor + 顶层状态机
```

**分层**：
- **Module**：`DJImotor`(CAN 电机) / `servo_motor`(PWM 舵机) / `message_center` / `bsp_usart` / `bsp_log`。
- **App**：`link` / `cmd` / `fsm` / `motor` / `trigger` / `vision` / `yaw`（平行，互不 include）。
- **编排**：`robot.c` 唯一入口，只做初始化 + 周期调度 + Monitor + 顶层阶段状态机。

**关键点**：`app/link` 是全工程**唯一**感知"上位机/网页存在"的地方。换掉上位机（ESP32 / PC 直连 / Jetson）只需改 `link.c`，其余 6 个 app 一行不动。

---

## 2. 文件清单

| 文件 | 作用 |
|---|---|
| `robot.cmake` | 板→MCU 映射，收集本 app 源码 |
| `robot.h/.c` | 入口：`RobotInit/RobotTask` + 顶层阶段机 + Monitor |
| `ui.h` | `os_task.c` 需要的占位 |
| `robot_def.h` | 话题名 / 错误位 / 跨 app 数据结构（`#pragma pack(1)`）+ **尺寸静态断言** |
| `launcher_cfg.h` | **本 app 全部可调参数**（逐路电机 / 舵机 / 状态机角度 / 协议码） |
| `launcher_all_cfg.h` | **包含式汇总 cfg**（只读汇总，不覆盖） |
| `app/link.{c,h}` | 上位机串口协议（解析指令 + 组遥测帧） |
| `app/cmd.{c,h}` | 大脑：操作员指令 → 各机构目标 |
| `app/fsm.{c,h}` | 时序：自动发射流程状态机 |
| `app/motor.{c,h}` | 4 路 CAN 电机（含保持/归零/自锁卸力） |
| `app/trigger.{c,h}` | 扳机舵机（复用 `servo_motor`） |
| `app/vision.{c,h}` | 视觉坐标归一化（PC/Jetson → aim） |
| `app/yaw.{c,h}` | 自瞄 yaw 轴（视觉 PI + 速度环） |
| `README.md` / `LOG.md` | 本文档 / 开发日志 |

---

## 3. 话题（`robot_def.h`）

| 话题名 | 发布者 | 订阅者 | 负载 |
|---|---|---|---|
| `launch_cmd` | link | cmd | `Launcher_Cmd_s` |
| `launch_state` | cmd | link/yaw | `Launcher_State_s` |
| `fsm_cmd` | cmd | fsm | `Launcher_FsmCmd_s` |
| `fsm_fb` | fsm | cmd | `Launcher_FsmFb_s` |
| `motor_cmd` | cmd/fsm | motor | `Launcher_MotorCmd_s` |
| `motor_fb` | motor | cmd/link | `Launcher_MotorFb_s` |
| `motor_param` | motor | link | `Launcher_MotorParam_s` |
| `servo_cmd` | cmd/fsm | trigger | `Launcher_ServoCmd_s` |
| `servo_fb` | trigger | cmd/link | `Launcher_ServoFb_s` |
| `aim_cmd` | vision | yaw/link | `Launcher_Aim_s` |
| `vision_cmd` | link | vision | `Launcher_VisCmd_s` |
| `yaw_cmd` | cmd | yaw | `Launcher_YawCmd_s` |
| `yaw_fb` | yaw | cmd/link | `Launcher_YawFb_s` |

> **⚠ 255 字节上限**：`message_center` 的 `data_len` 是 `uint8_t`，所有话题负载**必须 ≤ 255 字节**，否则会被**静默截断**导致数据损坏。
> `robot_def.h` 末尾有编译期静态断言（`LAUNCH_ASSERT_FITS`）拦住这类错误。
> 这也是为什么"电机参数回显"单独拆成 `motor_param` 话题（180 B），而不是塞进 `motor_fb`（否则 284 B 超限）。

---

## 4. 机构与逐路独立 cfg

四个电机**逐路完全独立**（ID / 方向 / 减速比 / 行程角度 / PID / 急停策略各自可配），因为两根拉簧是不同的弹簧。

| 机构 | 型号 | CAN ID | 自锁? | 急停策略 |
|---|---|---|---|---|
| 拉簧 A | M3508 | 2 | ❌ 非自锁 | **锁位保持 → 20s 缓慢归零** |
| 拉簧 B | M3508 | 3 | ❌ 非自锁 | **锁位保持 → 20s 缓慢归零** |
| 丝杆 | M3508 | 4 | ✅ 自锁 | 减速到零 → 卸力 |
| yaw | M2006 | 1 | — | 直接卸力 |

### 角度制（不用"圈"）

全工程统一**输出侧 deg（整数）**，不再出现"圈"：
- 少一次 `×360`/`÷360` 换算，就少一个出错点；
- 到位容差、行程、状态机目标全部直接用角度，更直观。

内部换算（在 `app/motor.c`）：
```
转子目标 = zero + sign * 输出角 * ratio
输出角   = sign * (total_angle - zero) / ratio
```

### 丝杆（自锁，不参与状态机）

丝杆转到设定角度后由 `app/motor` 自行卸力，自锁保证位置不变，**不需要耗电保持**，也**不参与时序协调**。
比赛时可把 `LAUNCH_SC_DEG_PREP` 设为 `0` = 全程不动。

> **⚠ 丝杆参数须按实机填写**：`LAUNCH_SC_RATIO`（传动比）、`LAUNCH_SC_DEG_PREP`（行程角）、`LAUNCH_SC_REVERSE`（方向）目前是**占位值**，上电前必须实测确认，否则可能撞机械限位。

---

## 5. 【安全红线】急停语义

**拉簧 A/B 是非自锁的**：一旦输出电流为 0，弹簧会立刻释放。

### 绝不允许 `DJIMotorStop()` 拉簧

```c
DJIMotorStop(m);   /* ❌ 电流置 0 → 弹簧瞬间释放 → 危险 */
```

`DJIMotorStop()` 的真实行为是把 CAN 报文的电流值写 0（`dji_motor.c` 里 `memset(..., 0, 2)`），
即"失力矩"，**不是"刹停"**。

### 正确做法：锁位保持 → 缓慢归零

```c
/* 急停时拉簧走 HOLD_AND_HOME(见 motor.c 的 EstopHoldAndHome) */
a->hold_angle = OutDeg(i);        /* 锁存当前角(不清 PID) */
ApplyAngle(i, a->hold_angle);     /* 目标=当前角, 继续跑角度环 */
/* 之后每周期把目标以 cfg 速率朝零位推进 */
```

**为什么这样电流不会断**：目标从"预备位"改成"当前位置"时误差瞬间归零 → `Pout`/`Dout` 自然为 0，
而**积分项 `Iout` 保留** → `final_output` 从上周期连续过渡，电流一帧都不断。
之后由 `Iout` 持续提供对抗弹簧的力矩，实现"受力平衡"。

### 保持模式的三个必要配置

| 参数 | 值 | 原因 |
|---|---|---|
| `LAUNCH_S?_ANGLE_DEADBAND` | **0** | 非 0 时 PID 在死区内会把输出清零（`controller.c` 死区分支），导致"松开-拉回"锯齿振荡 |
| `LAUNCH_S?_ANGLE_KI` | **> 0** | 纯 PD 面对恒定弹簧拉力**必然有稳态误差**，顶不到位；积分是"记住需要多大力气"的存储器 |
| `LAUNCH_S?_ANGLE_ILIMIT` | 按实测 | `Ki>0` 必须配积分限幅，否则堵转时积分吹爆，松开瞬间猛冲 |

`LAUNCH_S?_ANGLE_MAXOUT` 是"顶得住"与"烧电机"的界限，须按实测保持电流留 1.5~2 倍余量。

### 急救归零：用"时长"语义

```c
#define LAUNCH_SPRING_HOMING_SEC 20
#define LAUNCH_SA_HOMING_RATE_DPS ((float)(LAUNCH_SA_DEG_PREP) / (float)(LAUNCH_SPRING_HOMING_SEC))
```

真正要控制的安全量是"卸能需要多久"，速率只是推导结果，这样改行程时速率自动跟随。
**`(float)` 强制转换是必需的**：角度是整数，`1800 / 20` 会做整数除法，换个数（如 `1810/20=90`）就静默截断。

### 掉线保护

**拉簧掉线时绝不卸力**：`app/motor` 会继续盲发最后一次的保持电流（尽力而为）。

> **⚠ 固有风险（软件无法消除）**：机械上**无棘轮、无自锁**，因此
> ① 若电机实际顶不住（拉力 > 电机极限），保持物理上不可能；
> ② 若 STM32→电机 的 CAN 指令丢失，DJI 电调约 100 ms 后自动停止输出，软件无法阻止。
> **建议评估加装机械缓冲/阻尼以降低释放速度。**

---

## 6. 顶层状态机 vs 时序状态机

两层，职责不同、不重复：

| 层 | 位置 | 管什么 |
|---|---|---|
| **顶层阶段** | `robot.c` | 整机处于哪个阶段：`IDLE / READY / RUN / FAULT`（是否允许动作） |
| **发射时序** | `app/fsm.c` | 自动流程步骤：`SERVO_STD1 → SPRING_STD1 → SPRING_PREP → SERVO_PREP → SPRING_BACK → SERVO_STD2 → DONE` |

**每一步各机构该到的角度全部在 `launcher_cfg.h` 第 10 节**，改完烧录即可，**不动任何逻辑代码**：

```c
#define LAUNCH_STEP_SPRING_PREP_A_DEG LAUNCH_SA_DEG_PREP
#define LAUNCH_STEP_SPRING_PREP_B_DEG LAUNCH_SB_DEG_PREP
#define LAUNCH_STEP_SERVO_PREP_DEG    LAUNCH_SV_DEG_PREP
...
```

**丝杆不出现在时序表中**（自锁，转到就完事）。

---

## 7. 上位机协议（与旧版兼容）

**下行**（ASCII 行，`\n` 分隔，大小写敏感）：

| 命令 | 含义 | 备注 |
|---|---|---|
| `PING` | → `PONG` | |
| `H` | 心跳，仅刷新链路活性 | ESP32 每 500 ms 发一次 |
| `S` | 扫描在线电机 → `S,<n>,<id...>` | |
| `Z[,slot]` | 取零；省略 slot = 全部 | |
| `M,slot,mode,value` | 电机模式 + 目标 | `mode`: 0=STOP 1=SPEED 2=ANGLE；value ×10 |
| `P,slot,id,value` | PID 参数（×100） | id 见表 `Launcher_ParamId_e` |
| `R,slot` | 恢复该路 cfg 默认参数 | |
| `D,slot` | 反转该路方向 | |
| **`N,slot,deg`** | **设该路目标角度（整数 deg）** | **取代旧的 `W,turns100`** |
| `Y,mode` | yaw 模式 | 0=手动 1=自瞄 2=引导 |
| `A,rpm100` | 自瞄最大转速 | |
| `C,x,center` | 注入视觉坐标 | PC/Jetson 用 |
| `V,a[,b]` | 舵机操作 | 见 `LAUNCH_SERVO_OP_*` |
| `G,cmd` | 时序 / 急停 | 见 `LAUNCH_FSM_OP_*` |
| ~~`W,...`~~ | **已废弃** → 回 `ERR W_DEPRECATED use_N,slot,deg` | 防旧上位机静默发错角度 |

**上行遥测**（单行 `F,...`，字段顺序与 `esp32/proto.py` 严格对齐）：

```
F,<n>,
  每电机 12 项 × n : slot,type,id,online,dir,mode,target100,rpm,angle100,deg100,temp,cur
  舵机 4 项        : cur10,state,std10,prep10
  任务 5 项        : spring_a_deg,step,yaw,estop,aim100
  视觉 4 项        : x,ok,center,err
  每电机 11 参数 × n
```

> **字段个数与顺序未变**，故 `esp32/proto.py` 的 `_parse_state()` 无需改动；
> 但**语义**有两处变化（原"圈"改"角度"），前端显示单位需同步改 `°`：
> - 电机第 10 项：`turns100`（圈×100）→ `deg100`（角度×100）
> - 任务第 1 项：`spring100`（圈×100）→ `spring_a_deg`（A 拉簧预备位角度）

### 前端需要同步修改的地方

| 文件 | 改动 |
|---|---|
| `esp32/task.py` | `set_turns()` → 发 `N,<slot>,<deg>` |
| `esp32/www/js/motor.js` | "圈"控件改"°"；`turns100` → `deg100` |
| `esp32/www/js/app.js` | `turns100`/`spring100` 显示单位改 `°` |
| `esp32/www/js/task.js` | 上膛圈数 → 角度 |

> 在改完前端前，网页的"上膛"按钮会失效（旧 `W` 被拒绝），此时可用串口手动发 `N,0,1800` / `N,1,1980`。

---

## 8. 构建 / 烧录

```powershell
powershell -ExecutionPolicy Bypass -File make_one\build.ps1 -Robot dart_launcher_final -Board GIMBAL_BOARD
# 产物: make_one\build_dart_launcher_final\control-2026.hex
```

目标板 `GIMBAL_BOARD`（STM32F407，C 板）。
当前占用：**FLASH 73 612 B (7.0%)** / **RAM 91 008 B (69.4%)**。

---

## 9. 调参对照表

| 现象 | 改哪个 |
|---|---|
| 整机行程不对（上膛不到位/过偏） | `LAUNCH_S?_DEG_PREP` |
| 某路到位慢 / 顶不住弹簧 | `LAUNCH_S?_ANGLE_KI`、`LAUNCH_S?_ANGLE_ILIMIT` |
| 某路抖动 / 冲击 | `LAUNCH_S?_ANGLE_KP`、`LAUNCH_S?_ANGLE_KD` |
| 保持时电机发热严重 | `LAUNCH_S?_ANGLE_MAXOUT`（降）|
| 急停归零太快/太慢 | `LAUNCH_SPRING_HOMING_SEC` |
| 方向反 | `LAUNCH_S?_REVERSE`（改后需重新取零）|
| 舵机零点偏 / 方向反 / 行程不对 | `LAUNCH_SV_TRIM_DEG` / `LAUNCH_SV_REVERSE` / `LAUNCH_SV_SCALE` |
| 状态机某步目标角度 | `launcher_cfg.h` 第 10 节 `LAUNCH_STEP_*` |
| 自瞄追不上 / 震荡 | `LAUNCH_YAW_AIM_KP/KI`（应用层）；`LAUNCH_YAW_SPEED_KP/KI`（速度环）|

> **参数只放 cfg**（规范要求），网页 `P` 命令改的是**运行时**值，**重启恢复 cfg 值**。满意后请把值抄回 `launcher_cfg.h`。

---

## 10. 待办 / 已知问题

- [ ] **丝杆实测参数**：`LAUNCH_SC_RATIO` / `LAUNCH_SC_DEG_PREP` / `LAUNCH_SC_REVERSE` 须按实机填写（当前为占位值）。
- [ ] **前端同步**：`esp32/` 的 `task.py` 与 `www/js/*.js` 需改 `W`→`N`、圈→角度。
- [ ] **"顶不住"判据**：当前检测不出"保持时缓慢失守"（`PID_ErrorHandle` 用相对误差 >95% 判定堵转，保持场景误差≈0 永不触发）。建议补"电流饱和 + 位置持续后退"的判据。
- [ ] **掉线盲发保持电流**：目前只是"尽力而为"，且无超时保护。
- [ ] **机械兜底**：无棘轮/自锁，建议评估加装缓冲或阻尼。
- [ ] 丝杆是否需要在时序中做"退壳/复位"动作（当前完全不参与）。
- [ ] `app/cmd` 与 `app/fsm` 同时写 `motor_cmd`：目前靠"后者覆盖"，建议明确仲裁（如加 `from_fsm` 优先级）。

---

## 11. 相关文件

- `Modules/motor/DJImotor/`、`Modules/motor/servo_motor/`、`Modules/message_center/`、`Modules/algorithm/controller/`、`Bsp/usart/`、`Bsp/log/`。
- `UserApp/application.md`、`UserApp/APP层应用编写指引.md`（架构规范）。
- `Tools/AGENTS.md`（工程总纲）、`Tools/CONTEXT.md`（压缩记忆）。
- `UserApp/robot/dart_launcher_web_v5_HIK/`（旧版，对照与回退）。
- `UserApp/robot/dart_final/`（制导飞镖整机飞控，本 app 的架构参照）。
