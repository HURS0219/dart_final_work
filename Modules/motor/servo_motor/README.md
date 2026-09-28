# servo_motor · 单路 PWM 舵机驱动

<p align='right'>ai @ dart_final_work</p>

面向**后续调用者**的使用说明。详细设计见同目录 [`servo_motor.md`](servo_motor.md)。

**职责边界**
- ✅ 只做：单路舵机的「逻辑角 ↔ 脉宽」标定、速率限幅、调零。
- ❌ 不做：四舵面混控 / 状态机（→ `Modules/servo_mixer` + app）；掉电保存（→ `Bsp/flash` 的 `flash_store_save/load`）。

---

## 1. 信号与标定公式

50Hz（周期 20ms），脉宽 `pulse_min_us ~ pulse_max_us` 线性对应机械偏角 `-half_deg ~ +half_deg`；中位 `center_us` 对应机械 0°。

```
applied = 逻辑角 * scale + trim_deg          (机械偏角, deg; reverse 时取负)
pulse   = center_us + applied * (half_us / half_deg)
```

例：`center=1500, half_us=1000, half_deg=139.5, scale=1, trim=0, reverse=0` → 逻辑角 30° ⇒ pulse ≈ 1715us。

---

## 2. 类型

```c
/* 初始化配置（含 PWM 通道，含指针，不可持久化） */
typedef struct {
    PWM_Init_Config_s pwm;                 // htim / channel / period(=0.02s)
    float center_us, half_us, half_deg;    // 1500 / 1000 / 139.5
    float pulse_min_us, pulse_max_us;      // 500 / 2500
    float scale, trim_deg;                 // 1 / 0
    float limit_deg;                       // 逻辑角对称限位; <=0 默认 half_deg
    float rate_limit_dps;                  // 速率限幅 deg/s; <=0 不限速
    uint8_t reverse;                       // 0 正常 / 1 反向
    uint8_t zero_enable;                   // 调零开关: 0 禁用 / 1 启用
} Servo_Init_Config_s;

/* 标定快照（全 4 字节字段 -> 无 padding，可直接存 Flash） */
typedef struct {
    float center_us, half_us, half_deg, pulse_min_us, pulse_max_us;
    float scale, trim_deg, limit_deg, rate_limit_dps;
    int32_t reverse, zero_enable;
} Servo_Calib_s;
```

`ServoInstance` 为运行时实例，内部字段请勿直接读写，统一走下面的接口。

---

## 3. 初始化

```c
#include "servo_motor.h"
#include "tim.h"

ServoInstance *g_servo;

void Servo_Init(void) {
    Servo_Init_Config_s cfg = {
        .pwm = { .htim = &htim1, .channel = TIM_CHANNEL_1, .period = 0.02f, .dutyratio = 0.075f },
        .center_us = 1500.0f, .half_us = 1000.0f, .half_deg = 139.5f,
        .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f,
        .scale = 1.0f, .trim_deg = 0.0f, .limit_deg = 90.0f, .rate_limit_dps = 0.0f,
        .reverse = 0, .zero_enable = 1,
    };
    g_servo = ServoInit(&cfg);        // 返回 NULL 表示失败(实例已满/参数非法)
    ServoSetAngle(g_servo, 0.0f);
}
```

---

## 4. 驱动接口

```c
void  ServoSetAngle(ServoInstance*, float angle);      // 设逻辑角(受 ±limit_deg 限位; 限速时由 ServoTask 推进)
void  ServoSetPulseUs(ServoInstance*, float pulse_us); // 直给脉宽(标定/开环, 自动硬限幅)
void  ServoTask(void);                                 // 周期调用推进限速(建议 100Hz~1kHz)
void  ServoEnable(ServoInstance*);                     // 启动 PWM
void  ServoDisable(ServoInstance*);                    // 停止 PWM(失去保持力)
float ServoGetAngle(ServoInstance*);                   // 当前逻辑角
float ServoGetTarget(ServoInstance*);                  // 目标逻辑角
float ServoGetPulseUs(ServoInstance*);                 // 当前脉宽
```

---

## 5. 标定接口（在线调参，已封装）

调用方**只需依赖 `Servo_Calib_s`，无需访问实例内部字段**：

```c
void ServoSetCalib(ServoInstance*, const Servo_Calib_s*);  // 批量写并立即生效
void ServoGetCalib(ServoInstance*, Servo_Calib_s*);        // 取快照(可直接存 Flash)
void ServoSetLimit(ServoInstance*, float limit_deg);       // 逻辑角对称限位
void ServoSetScale(ServoInstance*, float scale);           // 逻辑角->机械角 增益(>0)
void ServoSetTrim(ServoInstance*, float trim_deg);         // 零点微调
void ServoSetReverse(ServoInstance*, uint8_t reverse);     // 方向(0/1)
void ServoSetRateLimit(ServoInstance*, float dps);         // 速率限幅(<=0 不限速)
```

> `ServoSetCalib` 会逐字段做合法性检查（如 `half_deg<=0`、`scale<=0`、脉宽上下界颠倒会被忽略）。

---

## 6. 调零（可选功能）

```c
uint8_t ServoZero(ServoInstance*);   // 1 成功 / 0 拒绝
```

- 仅当 `zero_enable = 1` 时可用（**可用可不用**，由 config 决定）；
- 且当前机械偏角必须在 **中位 ±`SERVO_ZERO_WINDOW_DEG`（默认 30°）** 之内，否则拒绝（防止在极端/饱和位置误调零）；
- 成功后把「当前位置」记为逻辑 0°（物理不动）。

---

## 7. 速率限幅

`rate_limit_dps > 0` 时，`ServoSetAngle()` 只更新目标角，由周期调用 `ServoTask()` 以最多 `rate_limit_dps * dt` 度/周期逼近，实现平滑转动；`<= 0` 时立即到位。

---

## 8. 掉电保存（配合 bsp_flash）

模块**不碰 Flash**。用 `Servo_Calib_s` 快照 + `bsp_flash` 的通用双 Bank 存取：

```c
#include "servo_motor.h"
#include "bsp_flash.h"

#define SERVO_BANK_A ADDR_FLASH_SECTOR_10
#define SERVO_BANK_B ADDR_FLASH_SECTOR_11

/* 上电读回 */
void Servo_LoadCalib(ServoInstance *s) {
    Servo_Calib_s cal;
    if (flash_store_load(SERVO_BANK_A, SERVO_BANK_B, &cal, sizeof(cal)) == 1) {
        ServoSetCalib(s, &cal);
    }
}

/* 调参后保存 */
void Servo_SaveCalib(ServoInstance *s) {
    Servo_Calib_s cal;
    ServoGetCalib(s, &cal);
    flash_store_save(SERVO_BANK_A, SERVO_BANK_B, &cal, sizeof(cal));
}
```

> `Servo_Calib_s` 全字段 4 字节、无 padding，可直接抛给 `flash_store_save/load`。

---

## 9. 注意事项

1. 多路舵机各自一个 `ServoInstance`；本 dart 使用 `PWM1~4 = TIM1 CH1~4 = PE9/PE11/PE13/PE14`。
2. 混控 / 状态机请在 app 层组合 `servo_mixer`，不要塞进驱动。
3. 调零后若需持久化，请自行调用 `Servo_SaveCalib()`（模块不会自动写 Flash）。
4. 掉电保存的实现与参数见 `Bsp/flash/README.md`。
