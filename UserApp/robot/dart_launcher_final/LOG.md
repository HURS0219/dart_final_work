# dart_launcher_final 开发日志 / 问题记录

> 记录每次改动、踩坑与决策，便于回溯。格式：日期 + 内容。

## 2026-09-30　新建：由 dart_launcher_web_v5_HIK 解耦重写

- 新建 `UserApp/robot/dart_launcher_final`，**严格按 `application.md` / `APP层应用编写指引.md` 重写**。
  旧版 `dart_launcher_web_v5_HIK` **保留不动**，作为对照与回退。
- **消除的耦合**（旧版 6 个模块互相 include + 12 个可写全局变量）：
  1. `dart_link.c` 直接调 `MotorSet`/`MotorSetParam`/`MotorZero`/`DartServoGo`/`DartFsmAutoStart`
     并读 `Axis[]`/`g_servo_cur_deg`/`g_task_step`/`g_vis_x` → 现改为**只收发话题**；
  2. `dart_motor.c` 的 `extern int g_estop`（反向伸手进 FSM）→ 改为订阅 `motor_cmd.estop`；
  3. `dart_store.c` include fsm+motor+servo → **整块删除**（改纯 cfg）。
- **架构**：7 个平行 app（`link`/`cmd`/`fsm`/`motor`/`trigger`/`vision`/`yaw`）+ `robot.c` 编排。
  每个机构一个 app，互不 include，只见话题。
- **不建 module 层**：电机直接调 `DJIMotorInit/OuterLoop/SetPIDRef/Enable/Stop`
  （`DJIMotorTask()` 由 `os_task.c` 的 `MotorControlTask()` 统一调用，本 app 不重复调）。
- **舵机复用 `Modules/motor/servo_motor`**：旧版 `dart_servo.c` 自己裸调 `bsp_pwm` 手写脉宽换算；
  现直接用 module 的标定链（scale/trim/reverse/limit/rate_limit）+ 带安全窗口的调零。
- **全程角度制**（不用"圈"）：少一次 ×360/÷360 换算，就少一个出错点。

## 关键设计决策

### 1. 急停：拉簧绝不允许 `DJIMotorStop()`

`DJIMotorStop()` 的真实行为是把 CAN 报文的电流值写 0（`dji_motor.c` 的 `memset(..., 0, 2)`），
即**失力矩**而非刹停。拉簧非自锁，电流一归零弹簧立刻释放 —— **绝不能用**。

改为**锁位保持 → 缓慢归零**：
- 目标从"预备位"改成"当前的角"，误差瞬间归零 → `Pout`/`Dout` 为 0，
  但 **`Iout` 保留** → `final_output` 连续过渡，**电流一帧都不断**；
- 之后每周期把目标以 cfg 速率朝零位推进（20 s 走完全程），主动卸掉弹簧储能；
- 全程**不清 PID 积分**（清了反而会让电流从 0 重建，正是要避免的"闪断"）。

### 2. 逐路独立急停策略（`Launcher_EstopMode_e`）

| 机构 | 策略 | 原因 |
|---|---|---|
| 拉簧 A/B | `HOLD_AND_HOME` | 非自锁，卸力即弹回 |
| 丝杆 | `RAMP_STOP` | 自锁，减速后卸力（省电不发热），且避免高速中被自锁硬抓 |
| yaw | `COAST` | 无储能负载 |

### 3. 保持模式的三个必要配置

- `DEADBAND` **必须 0**：非 0 时 PID 进死区会把输出清零（`controller.c` 死区分支），
  造成"松开-拉回"锯齿振荡，持续冲击机械。
- `KI` **必须 > 0**：纯 PD 面对恒定弹簧拉力**必然有稳态误差**（理论必然），顶不到位。
- `ILIMIT` 必须显式给值且按实测保持电流的 1.5~2 倍：旧配置 `angle_PID` **没写 `IntegralLimit`**，
  结构体 memset 后为 0，一旦给 `Ki > 0` 就会把积分钳到 0（因为 `Improve` 含 `PID_Integral_Limit`），
  **静默废掉积分**。本工程显式给出。

### 4. `Improve` 用"最小必要集"

```c
Improve = PID_Integral_Limit | PID_Derivative_On_Measurement
```
- `PID_Integral_Limit`：有 `Ki` 就必须有；
- `PID_Derivative_On_Measurement`：急停时目标跳到当前角，**误差会突变**，
  若对误差微分会瞬间产生大冲量（`Kd × ΔErr / dt`）；对测量值微分则免疫（电机位置不会突变）。
- 其余 6 个环节（梯形积分/变速积分/输出滤波/微分滤波/比例先行/堵转检测）**先都不加**，
  实测有问题再逐个添加 —— 每层非线性都会让定位变难。

### 5. 归零速率用"时长"语义自动推导

```c
#define LAUNCH_SPRING_HOMING_SEC 20
#define LAUNCH_SA_HOMING_RATE_DPS ((float)(LAUNCH_SA_DEG_PREP) / (float)(LAUNCH_SPRING_HOMING_SEC))
```
真正想控制的安全量是"卸能需要多久"，速率只是推导结果；这样改行程时速率自动跟随。
**`(float)` 强制转换必需**：角度是整数，`1800/20` 是整数除法，换成 `1810/20=90` 会静默截断。

### 6. 协议：`W` 废弃，改 `N,slot,deg`

旧 `W,turns100` 有两个问题：
1. **没有 slot**，而 `dart_fsm.c` 把它同时设给 A/B —— 与"两根弹簧圈数不同"直接冲突；
2. 改角度制后 `W,500` 从"5 圈"变成"500 度"，**旧前端发出来的值会危险地错误**。

故 `W` **明确回 `ERR`**（而非静默执行错误角度），新命令 `N,<slot>,<deg>`。

---

## 踩坑记录

### ① `message_center` 的 `data_len` 是 `uint8_t`（255 上限）——**静默截断**

初版把"11 项参数 × 4 路"放进 `Launcher_MotorFb_s`，结构体算出 **284 字节**，
编译只报了个 `-Woverflow`（284 → 28），**运行时会把每条 `motor_fb` 消息截断损坏**。

修复：
- 参数回显拆成独立的 `motor_param` 话题（180 B），`motor_fb` 降到 108 B；
- 在 `robot_def.h` 加**编译期静态断言** `LAUNCH_ASSERT_FITS(...)`，把这类错误挡在编译期。

> 教训：`message_center` 用 `sizeof()` 传长度时，**任何超 255 字节的负载都是隐性炸弹**。
> 同时这也解释了旧版把 `MotorAxis` 拆得很散、遥测手拼字符串的原因。

### ② 注释里的 `*/` 会提前结束注释块

`link.c` 里写 `...一律置 req_*/标志位...`，其中的 `*/` **终止了 doxygen 注释**，
导致后面的函数声明被当成代码 → 一连串语法错误。已改为 `req_ 系列标志位`。

### ③ 头文件引用链：`motor_def.h` 需要 `bsp_can.h`

`motor_def.h` 里用到 `CAN_Init_Config_s`（定义在 `bsp_can.h`），但它自己**没有 include**，
依赖包含者先引入。`launcher_cfg.h` 里显式 `#include "bsp_can.h"` 解决。

### ④ 急停锁存变量作用域

`s_estop_latch` 最初声明在 `for` 循环体内，但反馈分支（另一个循环）也要读它 → 未定义。
提到文件作用域。

### ⑤ `Launcher_TaskStep_e` 只写在注释里没定义

`robot_def.h` / `fsm.c` / `robot.c` 都引用了 `LAUNCH_TASK_*`，但枚举只在注释里提过。
已补上定义（数值与旧版 `dart_fsm.h` 一致，便于协议对照）。

---

## 当前状态

- ✅ **编译通过**：`dart_launcher_final` @ `GIMBAL_BOARD`
  FLASH 73 612 B (7.0%) / RAM 91 008 B (69.4%)，**本 app 代码零警告**。
- ✅ 4 个电机 + 舵机 + 状态机 + 协议全部就位。
- ⚠️ **丝杆参数为占位值**（`LAUNCH_SC_RATIO` / `_DEG_PREP` / `_REVERSE`），**上电前必须实测确认**。
- ⚠️ **前端未同步**：`esp32/` 仍发旧 `W` 命令，网页"上膛"按钮会失效（回 `ERR`）；
  改完 `task.py` + `www/js/*.js` 后恢复正常。
- ⚠️ **机械无自锁/棘轮**：软件保持是唯一防线，且"掉线"与"顶不住"两种情况下软件无能为力，
  详见 README §5 的固有风险说明。
- 未验证（需台架）：保持电流实测值 → 据此校准 `MAXOUT`/`ILIMIT`；舵机标定；丝杆行程。

## 待办

见 README §10。优先：丝杆实测参数、前端同步、"顶不住"判据。
