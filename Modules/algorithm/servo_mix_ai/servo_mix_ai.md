# servo_mix_ai · 四舵面混控

<p align='right'>ai @ dart_final_work</p>

X 型四舵面的**混控**模块。**只做两件事**：把 `(pitch, yaw, roll)` 用解耦矩阵分到 4 路舵面（MIX），或对 4 路直接给逻辑角（MANUAL）；**上电默认 MIX + 零指令 = 回中**。

> 单路舵机的“标定 / 速率限幅 / 脉宽 / 调零”由 [`servo_motor`](../../motor/servo_motor/servo_motor.md) 负责，本模块**不重复实现**。

---

## 1. 职责与分层

```
App              : ServoMixInit / SetMode / SetCmd / SetManual / 周期 ServoMixTask
servo_mix_ai     : 两种模式 + 上电回中 + 解耦矩阵(本模块)
servo_motor      : 单路标定 / 限速 / 调零 / 脉宽(每路一个 ServoInstance)
```

- 本模块**当前直接持有并驱动 4 路 `ServoInstance`**（`ServoMixInit` 内部 `ServoInit` ×4）。
- **@note 后续可优化层级**：若需进一步解耦，可把“驱动”下沉——本模块只输出 4 路目标角，由 App 调 `ServoSetAngle`；本模块已按“只依赖 servo_motor 公共接口”编写，改造只需去掉 `ServoInit/ServoTask` 两处。

---

## 2. 两种模式

| 模式 | 说明 | 输入 |
|---|---|---|
| `SERVO_MIX_MODE_MIX` | 解耦混控 | `ServoMixSetCmd(pitch, yaw, roll)`，各 −1..1 |
| `SERVO_MIX_MODE_MANUAL` | 四路纯手动 | `ServoMixSetManual(ch, deg)` / `ServoMixSetManualAll(deg[4])` |

计算（`ServoMixTask` 每周期）：

```
MANUAL: target[ch] = manual[ch]
MIX   : u[ch] = Σ matrix[ch][j]*cmd[j];  target[ch] = clamp(u * MAX_DEG, ±MAX_DEG)
之后：target[ch] -> ServoSetAngle(servo[ch])   // ±limit 限位、限速由 servo_motor 处理
末尾：ServoTask()                              // 推进速率限幅
```

**上电回中**：默认模式为 MIX 且 `cmd = (0,0,0)`，故 `ServoMixInit()` 后各舵面停在逻辑 0°（中位）。

---

## 3. 解耦矩阵

`matrix[4][3]`：行 = 舵面编号 `0..3`，列 = `(pitch, yaw, roll)`，取值 `+1/-1`（可小数微调）。X 型 4 舵面的经典符号（与 `dart_fc` / `dart_servo_v0.2` 一致）：

```
        pitch yaw roll
  0:     +1    +1   +1
  1:     -1    +1   -1
  2:     -1    -1   +1
  3:     +1    -1   -1
```

> 某一路方向不符时，**优先改该路 `SERVO_MIX_REVERSE[ch]`**，再考虑改矩阵对应符号。

---

## 4. 对外接口

```c
void           ServoMixInit(void);                         // 注册 4 路 + 上电回中(幂等)
void           ServoMixSetMode(ServoMixMode_e mode);       // MIX / MANUAL
ServoMixMode_e ServoMixGetMode(void);

void ServoMixSetCmd(float pitch, float yaw, float roll);   // -1..1 (MIX)
void ServoMixSetManual(uint8_t ch, float deg);             // 逐路逻辑角 (MANUAL)
void ServoMixSetManualAll(const float deg[SERVO_MIX_N]);

void    ServoMixTask(void);                                // 周期调用(建议 100Hz~1kHz)
uint8_t ServoMixZero(uint8_t ch);                          // 转 servo_motor 调零(窗口/开关限制)
void    ServoMixEnable(void);  void ServoMixDisable(void);

void ServoMixGetDeflDeg(float out[SERVO_MIX_N]);           // 目标逻辑角
void ServoMixGetPulseUs(float out[SERVO_MIX_N]);           // 当前脉宽
```

---

## 5. 使用示例（App）

```c
#include "servo_mix_ai.h"

void RobotInit(void) {
    ServoMixInit();                        // 注册 4 路 + 回中
    ServoMixSetMode(SERVO_MIX_MODE_MIX);   // 默认即 MIX
}

void RobotTask(void) {
    /* MIX: 由制导/遥控给出 pitch/yaw/roll(-1..1) */
    ServoMixSetCmd(cmd_pitch, cmd_yaw, cmd_roll);
    ServoMixTask();                        // 计算 + 输出 + 推进限速

    /* 需要逐路调试时:
     *   ServoMixSetMode(SERVO_MIX_MODE_MANUAL);
     *   ServoMixSetManual(2, -20.0f);
     */
}
```

---

## 6. 调参（现场改哪个）

现场调参集中在 **`servo_mix_ai_cfg.h`**（改源码 → 烧录）：

| 宏 | 作用 |
|---|---|
| `SERVO_MIX_SCALE[4]` | 逐路 比例/行程 |
| `SERVO_MIX_TRIM_DEG[4]` | 逐路 零点 |
| `SERVO_MIX_REVERSE[4]` | 逐路 方向 |
| `SERVO_MIX_RATE_DPS[4]` | 逐路 速率限幅 |
| `SERVO_MIX_MAX_DEG` | 混控满偏对应的舵面角（**同时作每路逻辑角限位**；逐路限位宏已移除） |
| `SERVO_MIX_MATRIX` | 解耦矩阵 |
| `SERVO_MIX_DEFAULT_MODE` | 上电模式(0=MIX, 1=MANUAL) |

信号层（`center_us / half_us / half_deg / pulse_min/max`）统一在 `servo_motor_cfg.h`。
**调参方法论（比例→`scale`、零点→`trim`、方向→`reverse`）见 [`servo_motor.md` 的《调参指南》](../../motor/servo_motor/servo_motor.md)。**

---

## 7. 文件

| 文件 | 说明 |
|---|---|
| `servo_mix_ai.h` | 接口与模式枚举 |
| `servo_mix_ai.c` | 实现（2 模式 + 回中 + 矩阵，直接驱动 4 路） |
| `servo_mix_ai_cfg.h` | 本机 4 路配置 + 混控参数（现场调参入口） |
| `servo_mix_ai.md` | 本文档 |

## 8. 注意

1. 本模块**不含**掉电保存；参数改动请改 `servo_mix_ai_cfg.h` 后重新烧录。
2. `ServoMixTask()` 已内含 `ServoTask()`，App 周期只需调前者（**不要**再单独调 `ServoTask()`，否则限速会推进两次）。
3. 手动模式(`MANUAL`)的逐路角度最终仍受该路 `limit_deg` 限位。
