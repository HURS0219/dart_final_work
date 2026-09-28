# servo_motor

<p align='right'>ai @ dart_final_work</p>

PWM 舵机（PTK7350 等）底层驱动。**只负责单路舵机的“逻辑角 ↔ 脉宽”标定、速率限幅与调零**；四舵面混控由 `Modules/algorithm/servo_mix_ai` 负责。可调默认参数集中在 `servo_motor_cfg.h`（改源码 + 烧录即可现场调参；掉电保存由 `bsp_flash` 提供，本阶段暂缓）。

## 总览和封装说明

> 如果不需要理解内部原理，只看这一节即可。

- 一路舵机 = 一个 `ServoInstance`。调用 `ServoInit()` 注册并保存返回的指针，之后用 `ServoSetAngle()` 设定逻辑角。
- 50Hz 舵机 PWM 由 `Bsp/pwm/bsp_pwm` 输出；本模块只把“逻辑角”换算成脉宽 `PWMSetDutyRatio()`。
- 标定公式（`reverse` 在最后取负）：

  ```
  applied = 逻辑角 * scale + trim            (机械偏角, 单位 deg)
  pulse   = center_us + applied * (half_us / half_deg)
  ```

- 未启用速率限幅时 `ServoSetAngle()` 立即到位；启用后由周期调用 `ServoTask()` 平滑推进。
- `ServoZero()` 把“当前位置”记为逻辑 0°（调零），需 `zero_enable = 1` 且当前机械偏角在 `±SERVO_CFG_ZERO_WINDOW_DEG` 之内，否则拒绝。

**单位**：逻辑角/机械角 = deg；脉宽 = us；速率 = deg/s。

## 类型定义

> 可调默认值已集中到 `servo_motor_cfg.h`（`SERVO_CFG_*` 宏），本头文件只保留结构与接口。

```c
#define SERVO_MOTOR_CNT 4              // 最大实例数

typedef struct {
  PWM_Init_Config_s pwm;   // PWM 通道(period 建议 0.02s)
  float center_us;         // 机械 0° 对应脉宽(1500)
  float half_us;           // 中位到行程端对应脉宽(1000)
  float half_deg;          // 行程一半(139.5 = 279/2)
  float pulse_min_us;      // 脉宽硬下限(500)
  float pulse_max_us;      // 脉宽硬上限(2500)
  float scale;             // 逻辑角->机械角 增益(1)
  float trim_deg;          // 零点/中立微调
  float limit_deg;         // 逻辑角对称限位 ±limit_deg; <=0 默认 half_deg
  float rate_limit_dps;    // 速率限幅 deg/s; <=0 不限速
  uint8_t reverse;         // 0 正常 / 1 反向
  uint8_t zero_enable;     // 调零功能开关: 0 禁用 / 1 启用
} Servo_Init_Config_s;

typedef struct {
  PWMInstance *pwm;
  Servo_Init_Config_s cfg;    // 运行时配置, 标定字段可在线改
  float target_deg;           // 目标逻辑角
  float angle_deg;            // 当前逻辑角(限速后)
  float pulse_us;             // 当前脉宽
  float last_time_s;
} ServoInstance;
```

## 初始化示例

```c
#include "servo_motor.h"
#include "tim.h"

ServoInstance *g_servo;

void ServoTest_Init(void) {
  Servo_Init_Config_s cfg = {
      .pwm = {.htim = &htim1, .channel = TIM_CHANNEL_1,
              .period = 0.02f, .dutyratio = 0.075f},
      .center_us = 1500.0f, .half_us = 1000.0f, .half_deg = 139.5f,
      .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f,
      .scale = 1.0f, .trim_deg = 0.0f, .limit_deg = 90.0f,
      .rate_limit_dps = 0.0f, .reverse = 0, .zero_enable = 1,
  };
  g_servo = ServoInit(&cfg);
  ServoSetAngle(g_servo, 0.0f);   // 上电回中位
}
```

## 外部接口

```c
ServoInstance *ServoInit(Servo_Init_Config_s *config);   // 注册一路舵机(返回实例指针)
void  ServoSetAngle(ServoInstance*, float angle);        // 设逻辑角
void  ServoSetPulseUs(ServoInstance*, float pulse_us);   // 直给脉宽(标定/测试)
void  ServoTask(void);                                   // 速率限幅推进(周期调用)
uint8_t ServoZero(ServoInstance*);                       // 调零, 1 成功 0 未启用/超窗口
void  ServoEnable(ServoInstance*);                       // 启动 PWM
void  ServoDisable(ServoInstance*);                      // 停止 PWM(失去保持力)
float ServoGetAngle(ServoInstance*);                     // 当前逻辑角
float ServoGetTarget(ServoInstance*);                    // 目标逻辑角
float ServoGetPulseUs(ServoInstance*);                   // 当前脉宽
```

### 标定(在线调参, 封装入口)

调用方只需依赖 `Servo_Calib_s`(无 padding), **不需要了解实例内部结构**：

```c
void  ServoSetCalib(ServoInstance*, const Servo_Calib_s*);   // 批量写标定并立即生效
void  ServoGetCalib(ServoInstance*, Servo_Calib_s*);         // 取快照(可直接存 Flash)
void  ServoSetLimit(ServoInstance*, float limit_deg);        // 逻辑角对称限位
void  ServoSetScale(ServoInstance*, float scale);            // 增益(>0)
void  ServoSetTrim(ServoInstance*, float trim_deg);          // 零点微调
void  ServoSetReverse(ServoInstance*, uint8_t reverse);      // 方向(0/1)
void  ServoSetRateLimit(ServoInstance*, float dps);          // 速率限幅(<=0 不限速)
```

```c
typedef struct {                 // 全部 4 字节字段 -> 无 padding, 可直接持久化
  float center_us, half_us, half_deg, pulse_min_us, pulse_max_us;
  float scale, trim_deg, limit_deg, rate_limit_dps;
  int32_t reverse, zero_enable;
} Servo_Calib_s;
```

## 私有函数和变量

`.c` 内 static：`Clamp / LimitDeg / PeriodUs / AppliedOf / ApplyLogical`，以及实例数组 `servo_motor_instance[]` 与索引 `servo_idx`。

## 掉电保存

模块**不直接读写 Flash**。用 `Servo_Calib_s` 快照 + `bsp_flash` 的通用双 Bank 存取即可：

```c
#include "bsp_flash.h"

#define BANK_A ADDR_FLASH_SECTOR_10
#define BANK_B ADDR_FLASH_SECTOR_11

/* 保存 */
Servo_Calib_s calib;
ServoGetCalib(servo, &calib);
flash_store_save(BANK_A, BANK_B, &calib, sizeof(calib));

/* 读回 */
Servo_Calib_s calib;
if (flash_store_load(BANK_A, BANK_B, &calib, sizeof(calib)) == 1) {
    ServoSetCalib(servo, &calib);
}
```

`Servo_Calib_s` 全为 4 字节字段(无 padding)，可直接交给 `flash_store_save/load`，无需再复制一份结构体。

## 调参指南

### 1. 标定模型

```
applied = 逻辑角 * scale + trim_deg        // 机械偏角(deg); reverse 时整体取负
pulse   = center_us + applied * (half_us / half_deg)
```

### 2. 三个“信号层”参数（舵机规格，通常不动）

```
脉宽(us):   500 ─────── 1500 ─────── 2500
             |            |            |
机械角(°): -139.5          0         +139.5
             |<--half_us(1000)-->|<-half_us(1000)->|
             |<--half_deg(139.5)->|<-half_deg(139.5)->|
```

- `center_us`：机械 0°(中位) 对应的脉宽（1500）。**调零点不要改它，改 `trim_deg`。**
- `half_us` / `half_deg`：中位到“行程一端”的脉宽 / 角度跨度；二者之比 `half_us / half_deg ≈ 7.17 us/°` 就是“每度多少微秒”。
- `pulse_min_us` / `pulse_max_us`：脉宽硬限幅（正常 = `center` ∓ `half_us`）。

### 3. 现场调参对照（照这个改就行）

| 现象 | 改哪个 | 说明 |
|---|---|---|
| 指令角度 : 实际偏角 **比例**不对 | `scale` | 斜率；`angle=0` 时不受它影响 |
| 指令 0° 时舵面**零点**停偏 | `trim_deg` | 平移；等于“逻辑 0° 的机械偏角” |
| **方向**反了 | `reverse` | 改后零点会变号，需**重新对零** |
| 整段**刻度**不对（换型号） | `half_us` / `half_deg` | 信号层，平时不碰 |

- `scale` 与 `trim` **正交**：改 `scale` 不动零点，改 `trim` 不动比例。两者都偏时：**先对零(`trim`) → 再校比例(`scale`)**。
- 零点也可用 `ServoZero()` 一键把“当前位置”记为逻辑 0°（内部自动算 `trim`），前提 `zero_enable = 1` 且 `|当前偏角| ≤ SERVO_CFG_ZERO_WINDOW_DEG`。

### 4. 两层区分

- **信号层**（舵机规格）：`center_us / half_us / half_deg / pulse_min_us / pulse_max_us`。
- **逻辑层**（装机标定）：`scale / trim_deg / reverse / limit_deg / rate_limit_dps / zero_enable`。

### 5. 配置文件在哪

- 本模块默认值：`Modules/motor/servo_motor/servo_motor_cfg.h`（`SERVO_CFG_*`）。
- 本飞镖 4 路的逐路标定：`Modules/algorithm/servo_mix_ai/servo_mix_ai_cfg.h`（`SERVO_MIX_SCALE/TRIM_DEG/REVERSE/LIMIT_DEG/RATE_DPS/...`）。

## 注意事项

1. `limit_deg` 是**逻辑角**对称限位（相对逻辑 0°，即调零后的零点）。
2. `pulse_min/max_us` 是硬限幅，超出会被截断（PTK7350 越程可能翻转）。
3. 调零是“把当前位置记为逻辑 0°”，物理上不动；禁用 `zero_enable` 时 `ServoZero()` 直接返回 0。
4. 保存的结构体应无隐式 padding（用固定宽度整型），否则跨编译器 CRC 可能不一致。
