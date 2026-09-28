# servo_motor

<p align='right'>ai @ dart_final_work</p>

PWM 舵机（PTK7350 等）底层驱动。**只负责单路舵机的“逻辑角 ↔ 脉宽”标定、速率限幅与调零**；四舵面混控/状态机由应用层或 `servo_mixer` 负责，参数掉电保存由 `bsp_flash` 的 `flash_store_save/load` 负责。

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
- `ServoZero()` 把“当前位置”记为逻辑 0°（调零），需 `zero_enable = 1` 且当前机械偏角在 `±SERVO_ZERO_WINDOW_DEG` 之内，否则拒绝。

**单位**：逻辑角/机械角 = deg；脉宽 = us；速率 = deg/s。

## 类型定义

```c
#define SERVO_MOTOR_CNT       4        // 最大实例数
#define SERVO_ZERO_WINDOW_DEG 30.0f    // 调零窗口(相对中位)

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
ServoInstance *ServoInit(Servo_Init_Config_s *config);
void  ServoSetAngle(ServoInstance*, float angle);      // 设逻辑角
void  ServoSetPulseUs(ServoInstance*, float pulse_us); // 直给脉宽(标定/测试)
void  ServoTask(void);                                 // 速率限幅推进(周期调用)
uint8_t ServoZero(ServoInstance*);                     // 调零, 1 成功 0 未启用/超窗口
void  ServoSetLimit(ServoInstance*, float limit_deg);  // 逻辑角限位
void  ServoEnable(ServoInstance*);                     // 启动 PWM
void  ServoDisable(ServoInstance*);                    // 停止 PWM(失去保持力)
float ServoGetAngle(ServoInstance*);                   // 当前逻辑角
float ServoGetTarget(ServoInstance*);                  // 目标逻辑角
float ServoGetPulseUs(ServoInstance*);                 // 当前脉宽
```

## 私有函数和变量

`.c` 内 static：`Clamp / LimitDeg / PeriodUs / AppliedOf / ApplyLogical`，以及实例数组 `servo_motor_instance[]` 与索引 `servo_idx`。

## 掉电保存

模块**不直接读写 Flash**。参数持久化请用 `bsp_flash`：

```c
#include "bsp_flash.h"
flash_store_save(ADDR_FLASH_SECTOR_10, ADDR_FLASH_SECTOR_11, &calib, sizeof(calib));
flash_store_load(ADDR_FLASH_SECTOR_10, ADDR_FLASH_SECTOR_11, &calib, sizeof(calib));
```

## 注意事项

1. `limit_deg` 是**逻辑角**对称限位（相对逻辑 0°，即调零后的零点）。
2. `pulse_min/max_us` 是硬限幅，超出会被截断（PTK7350 越程可能翻转）。
3. 调零是“把当前位置记为逻辑 0°”，物理上不动；禁用 `zero_enable` 时 `ServoZero()` 直接返回 0。
4. 保存的结构体应无隐式 padding（用固定宽度整型），否则跨编译器 CRC 可能不一致。
