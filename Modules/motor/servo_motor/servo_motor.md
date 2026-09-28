# servo_motor

<p align='right'>panrui@hnu.edu.cn / merged by ai from dart_servo_v0.2</p>

> version: 1.0（由 `UserApp/robot/dart_servo_v0.2` 的舵机驱动抽提、合并重写为通用 module）
>
> todo:
> 1. 目前只实现“角度控制（开环位置）”与总线舵机角度回读，暂无负载/温度/到位反馈（PWM 舵机本身无反馈）。
> 2. 总线舵机的命令帧仍沿用旧实现的定长缓冲，后续可改为按 `servo_id` 动态组帧。

## 1. 简介

`servo_motor` 是舵机（Servo）的通用驱动模块，位于 **module 层**，对上层 app 屏蔽底层差异：

- **PWM 舵机**：通过 `bsp_pwm` 输出 50Hz PWM，脉宽线性对应机械角；内置**标定**（中性/微调/比例/方向/量程）与**速率限幅**。
- **串口总线舵机**：通过 `bsp_usart` 发送角度命令帧，并解析位置回读帧。

> 本模块只负责“**驱动 + 标定 + 速率限幅**”。状态机（待机/手动/混控/自检）、混控矩阵等**控制逻辑属于 app 层**，以保持 module 层通用、可复用。

## 2. 舵机基础知识

以最常见的 PWM 舵机为例：工作在 **50Hz（周期 20ms）**，高电平脉宽落在 `pulse_min ~ pulse_max`（通常 0.5ms~2.5ms）之间，线性对应舵机机械角：

```
0.5ms ---   0°      1.0ms ---  45°      1.5ms ---  90°
2.0ms --- 135°      2.5ms --- 180°
```

- 对 **180° 型**（如常见 SG90）：500~2500us ↔ 0~180°。
- 对 **270° 型**（如 PTK7350 270°/实测约 279°）：500~2500us ↔ 0~270°(279°)。
  即同样的脉宽范围对应更大的行程，**每度对应的脉宽更小**，标定时务必设对 `range_deg`。

PWM 由定时器产生：`Tout = (PSC+1)*(ARR+1)/Tclk`。C 板 TIM1 挂 APB2，168MHz，50Hz 时常用 `PSC=168-1, ARR=20000-1`。本模块不直接写 ARR/CCR，而是调用 `bsp_pwm` 的 `PWMSetDutyRatio()`，占空比 = `脉宽 / 周期`。

## 3. 逻辑角 / 机械角 / 标定模型

- **机械角** `raw_deg`：舵机真实转动角，范围 `0 ~ range_deg`。
- **逻辑角** `angle`：软件层的“偏角”，中立项为 0，正负表示两个方向。
- 换算：`raw = center_deg + trim_deg + reverse * scale * angle`（再限幅到 `0~range_deg`）；
  脉宽：`pulse = pulse_min + raw/range * (pulse_max - pulse_min)`。

各标定项：

| 项 | 含义 |
|---|---|
| `center_deg` | 逻辑 0° 对应的机械角（默认 `range/2`，即电气中位 1500us） |
| `trim_deg` | 中性微调（在 center 基础上平移） |
| `scale` | 比例（逻辑角→机械角的增益） |
| `reverse` | 方向（+1/-1） |
| `rate_limit_dps` | 速率限幅（°/s），>0 时由 `ServoTask()` 平滑推进 |

## 4. 代码结构

- `servo_motor.h`：类型定义与对外接口。
- `servo_motor.c`：实例注册、角度控制、速率限幅任务、标定、总线舵机解析。

## 5. 类型定义

```c
#define SERVO_MOTOR_CNT 7

typedef enum { Servo_None_Type = 0, Bus_Servo = 1, PWM_Servo = 2 } ServoType_e;

typedef struct {
  float pulse_min_us;    // 机械 0° 对应脉宽 (如 500)
  float pulse_max_us;    // 机械 range_deg 对应脉宽 (如 2500)
  float range_deg;       // 机械量程 (180 / 270 / 279 ...)
  float center_deg;      // 逻辑 0° 对应的机械角 (默认 range_deg/2)
  float trim_deg;        // 中性微调
  float scale;           // 比例 (默认 1)
  int8_t reverse;        // 方向: +1 / -1 (默认 +1)
  float rate_limit_dps;  // 速率限幅 deg/s (<=0 不限速)
} Servo_Calib_Config_s;

typedef struct {
  ServoType_e servo_type;
  uint8_t servo_id;
  UART_HandleTypeDef *_handle;        // Bus_Servo
  PWM_Init_Config_s pwm_init_config;  // PWM_Servo (周期建议 0.02s)
  Servo_Calib_Config_s calib;         // PWM_Servo
} Servo_Init_Config_s;

typedef struct {
  uint8_t servo_id;
  ServoType_e servo_type;
  PWMInstance *pwm_instance;
  USARTInstance *usart_instance;
  Servo_Calib_Config_s calib;
  float target_deg, angle, raw_deg, pulse_us, last_time_s;
  uint16_t recv_angle;
} ServoInstance;
```

## 6. 外部接口

```c
ServoInstance *ServoInit(Servo_Init_Config_s *config);   // 注册舵机
void ServoSetAngle(ServoInstance *servo, float angle);   // 设逻辑角(deg)
void ServoSetPulseUs(ServoInstance *servo, float pulse_us);// 直接给脉宽(us)
void ServoTask(void);                                    // 周期任务(速率限幅)

void ServoSetNeutral(ServoInstance*, float center_deg);  // 逻辑0°对应机械角
void ServoSetTrim(ServoInstance*, float trim_deg);
void ServoSetScale(ServoInstance*, float scale);
void ServoSetReverse(ServoInstance*, int8_t reverse);
void ServoSetRateLimit(ServoInstance*, float dps);
void ServoZero(ServoInstance*);                          // 当前位置记为逻辑0°
void ServoResetCal(ServoInstance*);                      // 清标定

void  ServoEnable(ServoInstance*);                       // 启动输出
void  ServoDisable(ServoInstance*);                      // 停止输出
void  ServoUnload(ServoInstance*);                       // 总线: 失力
void  ServoRequestAngle(ServoInstance*);                 // 总线: 请求位置回读
float ServoGetAngle(ServoInstance*);                     // 逻辑角
float ServoGetRawDeg(ServoInstance*);                    // 机械角
float ServoGetPulseUs(ServoInstance*);                   // 脉宽
uint16_t ServoGetRecvAngle(ServoInstance*);              // 总线回读角
```

## 7. 使用示例

### 7.1 PWM 舵机（PTK7350，四路 X 型舵面）

```c
#include "servo_motor.h"
#include "tim.h"

static ServoInstance *surfaces[4];

void ServoExampleInit(void) {
  const uint32_t ch[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
  for (uint8_t i = 0; i < 4; i++) {
    Servo_Init_Config_s cfg = {
        .servo_type = PWM_Servo,
        .servo_id = i,
        .pwm_init_config = {
            .htim = &htim1,
            .channel = ch[i],
            .period = 0.02f,     // 50Hz
            .dutyratio = 0.075f, // 1.5ms 中立
        },
        .calib = {
            .pulse_min_us = 500.0f,
            .pulse_max_us = 2500.0f,
            .range_deg = 270.0f,   // PTK7350 270°型;180°型改 180
            .center_deg = 135.0f,  // 逻辑0°=机械中位(270/2)
            .trim_deg = 0.0f,
            .scale = 1.0f,
            .reverse = 1,
            .rate_limit_dps = 600.0f,  // 启用速率限幅
        },
    };
    surfaces[i] = ServoInit(&cfg);
  }
}

/* 在 app 周期任务(建议 100Hz~1kHz)中调用 */
void ServoExampleTask(void) {
  ServoTask();          // 速率限幅推进
  ServoSetAngle(surfaces[0], 20.0f);  // 第 1 路逻辑角 +20°
}
```

### 7.2 串口总线舵机

```c
Servo_Init_Config_s cfg = {
    .servo_type = Bus_Servo,
    .servo_id = 1,
    ._handle = &huart6,
};
ServoInstance *s = ServoInit(&cfg);
ServoSetAngle(s, 1200.0f);          // 下发角度(按总线协议单位)
ServoRequestAngle(s);               // 请求位置回读
uint16_t a = ServoGetRecvAngle(s);  // 回读角(需周期收到应答帧)
```

## 8. 标定步骤（两步法）

1. **调零**：手动/上位机把舵面拖到物理中立（或期望的逻辑 0° 位置），调用 `ServoZero()` —— 把当前位置记为逻辑 0°（改 `center_deg`，脉宽不变，舵机不跳变）。
2. **定比例**：把舵面拖到某个已知机械角标记（如物理 90°），按 `scale = 期望机械角差 / 当前逻辑角差` 调整 `ServoSetScale()`；对 180° 型/270° 型一定要把 `range_deg` 设对。

> 标定结果建议由 app 层做掉电保存（module 层不做持久化）。

## 9. 注意事项

- **方向**：`reverse` 必须与实际安装方向一致，否则混控会“顶牛”。
- **量程**：`range_deg` 必须区分 180° / 270° 型；用错会导致行程比例错误、易撞机械止挡。
- **限速**：启用 `rate_limit_dps` 后**必须在周期任务中调用 `ServoTask()`**，否则舵机不会动。
- **无反馈**：PWM 舵机无位置/负载反馈，`ServoGetAngle()` 返回的是指令角而非实测角；防堵转需靠软限位 + 外部电流检测。
- **总线舵机**：`DecodeServo` 在中断上下文执行，只做轻量解析；角度单位为协议单位，`servo_id` 与帧内 ID 需一致。
