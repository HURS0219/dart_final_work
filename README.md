# 掉电保存 (bsp_flash) 与 舵机 (servo_motor) 使用说明

<p align='right'>制导飞镖 · ai</p>

本文档面向**后续接手/调用的队员**，介绍本仓库里两块底层设施的用法：

| 模块 | 路径 | 一句话 |
|---|---|---|
| **bsp_flash** | `Bsp/flash/` | 内部 Flash 读写 + **通用掉电保存**（双 Bank + 校验） |
| **servo_motor** | `Modules/motor/servo_motor/` | **单路 PWM 舵机**驱动：标定 / 速率限幅 / 调零 |

两者配合，即可实现「网页/串口调参 → 掉电不丢」。**它们各自独立**：`servo_motor` 不碰 Flash，`bsp_flash` 不认识舵机。

---

## 一、bsp_flash —— 掉电保存

### 1.1 分层

- **底层原语**：`flash_erase_address / flash_write_single_address / flash_read / get_next_flash_address`（裸扇区操作）。
- **上层通用存储**：`flash_store_save / flash_store_load`（双 Bank + `magic/seq/CRC`，推荐直接用这个）。

### 1.2 底层原语

```c
void   flash_erase_address(uint32_t address, uint16_t len);                              // len = 扇区数
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);  // len = 32bit 字数
void   flash_read(uint32_t address, uint32_t *buf, uint32_t len);                        // len = 32bit 字数
uint32_t get_next_flash_address(uint32_t address);                                       // 下一扇区起始地址
```

> 注意：`flash_write_single_address` / `flash_read` 的 `len` 单位是 **word(4 字节)**，不是字节。

### 1.3 通用掉电保存（推荐）

```c
int8_t flash_store_save(uint32_t bank_a, uint32_t bank_b, const void *data, uint32_t size); // 0 成功 / -1 失败
int8_t flash_store_load(uint32_t bank_a, uint32_t bank_b, void       *data, uint32_t size); // 1 有效 / 0 无数据
```

**记录布局**（每个 bank 从扇区起始地址开始）：

```
+0x00  magic (0xB1A5D00D)   记录标记
+0x04  seq                   递增序号
+0x08  crc                   data 区 FNV-1a 校验
+0x0C  data[]                调用方数据，长度 size（必须 4 字节对齐）
```

**策略**：保存时总写「非当前」那个 bank 并 `seq+1`，写完**回读校验**、通过才切换；目标 bank 写失败会**回退**写另一个。读取时两份都校验，取 `seq` 更大的那份。→ 写一半掉电也不会丢配置。

**约束**：
- `size` 必须 **4 字节对齐**，且单份数据要**小于一个扇区**。
- 传入的结构体应**无隐式 padding**（成员用固定宽度整型），否则跨编译器 CRC 可能不一致。
- 两个 bank 请选**未被固件占用**的整扇区起始地址（如 `ADDR_FLASH_SECTOR_10` / `ADDR_FLASH_SECTOR_11`）。

### 1.4 快速上手

```c
#include "bsp_flash.h"

#define MY_BANK_A ADDR_FLASH_SECTOR_10   // 0x080C0000
#define MY_BANK_B ADDR_FLASH_SECTOR_11   // 0x080E0000

typedef struct {                          // 4 字节字段，无 padding
    float kp, ki, kd;
    int32_t enable;
} MyCfg_t;

MyCfg_t cfg;

void Load(void) {
    if (flash_store_load(MY_BANK_A, MY_BANK_B, &cfg, sizeof(cfg)) == 0) {
        MyCfg_Defaults(&cfg);             // 无有效数据 -> 用默认值
    }
}

void Save(void) {
    flash_store_save(MY_BANK_A, MY_BANK_B, &cfg, sizeof(cfg));
}
```

> **务必在任务上下文调用**，不要在中断里调用：擦除会打断执行（128KB 扇区约 1s）。

---

## 二、servo_motor —— 单路 PWM 舵机

### 2.1 职责边界

**只做**：单路舵机的「逻辑角 ↔ 脉宽」标定、速率限幅、调零。
**不做**（交给别人）：四舵面混控 / 状态机（→ `Modules/servo_mixer` + app）；掉电保存（→ `bsp_flash`）。

### 2.2 信号与标定公式

50Hz（周期 20ms），脉宽 `pulse_min_us ~ pulse_max_us` 线性对应机械偏角 `-half_deg ~ +half_deg`；中位 `center_us` 对应机械 0°。

```
applied = 逻辑角 * scale + trim_deg        (机械偏角, deg; reverse 时取负)
pulse   = center_us + applied * (half_us / half_deg)
```

例：`center=1500, half_us=1000, half_deg=139.5, scale=1, trim=0, reverse=0` → 逻辑角 30° ⇒ pulse ≈ 1715us。

### 2.3 类型

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

### 2.4 初始化

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
    g_servo = ServoInit(&cfg);   // 返回 NULL 表示失败(实例满/参数非法)
    ServoSetAngle(g_servo, 0.0f);
}
```

### 2.5 驱动接口

```c
void  ServoSetAngle(ServoInstance*, float angle);      // 设逻辑角(受限位/限速)
void  ServoSetPulseUs(ServoInstance*, float pulse_us); // 直给脉宽(标定/开环)
void  ServoTask(void);                                 // 周期调用推进限速(100Hz~1kHz)
void  ServoEnable / ServoDisable(ServoInstance*);      // 启动/停止 PWM
float ServoGetAngle / ServoGetTarget / ServoGetPulseUs(ServoInstance*);
```

### 2.6 标定接口（在线调参，已封装）

调用方**只需依赖 `Servo_Calib_s`，无需访问实例内部字段**：

```c
void ServoSetCalib(ServoInstance*, const Servo_Calib_s*);  // 批量写并立即生效
void ServoGetCalib(ServoInstance*, Servo_Calib_s*);        // 取快照(可直接存 Flash)
void ServoSetLimit(ServoInstance*, float limit_deg);
void ServoSetScale(ServoInstance*, float scale);           // >0
void ServoSetTrim(ServoInstance*, float trim_deg);
void ServoSetReverse(ServoInstance*, uint8_t reverse);     // 0/1
void ServoSetRateLimit(ServoInstance*, float dps);         // <=0 不限速
```

### 2.7 调零（可选功能）

```c
uint8_t ServoZero(ServoInstance*);   // 1 成功 / 0 拒绝
```

- 仅当 `zero_enable = 1` 时可用（可用可不用）；
- 且当前机械偏角必须在 **中位 ±`SERVO_ZERO_WINDOW_DEG`（默认 30°）** 之内，否则拒绝；
- 成功后把「当前位置」记为逻辑 0°（物理不动，等价于原地重设 `trim`）。

### 2.8 速率限幅

`rate_limit_dps > 0` 时，`ServoSetAngle()` 只更新目标角，由周期调用 `ServoTask()` 以最多 `rate_limit_dps * dt` 度/周期逼近，实现平滑转动；`<= 0` 时立即到位。

---

## 三、两者配合：舵机标定掉电保存（完整示例）

```c
#include "servo_motor.h"
#include "bsp_flash.h"

#define SERVO_BANK_A ADDR_FLASH_SECTOR_10
#define SERVO_BANK_B ADDR_FLASH_SECTOR_11

static ServoInstance *servo;

void Servo_Setup(void) {
    /* 1. 注册舵机（默认标定） */
    Servo_Init_Config_s cfg = { /* ... 见 2.4 ... */ };
    servo = ServoInit(&cfg);

    /* 2. 上电读回标定（无有效数据则用默认） */
    Servo_Calib_s cal; 
    if (flash_store_load(SERVO_BANK_A, SERVO_BANK_B, &cal, sizeof(cal)) == 1) {
        ServoSetCalib(servo, &cal);
    }
}

/* 网页/串口调参后调用：把当前标定存起来 */
void Servo_SaveCalib(void) {
    Servo_Calib_s cal;
    ServoGetCalib(servo, &cal);
    flash_store_save(SERVO_BANK_A, SERVO_BANK_B, &cal, sizeof(cal));
}
```

> `Servo_Calib_s` 全字段 4 字节、无 padding，**可直接抛给 `flash_store_save/load`**，不必再定义持久化结构。

---

## 四、注意事项

1. **编码**：本仓库源码统一 **UTF-8**，请勿混入 GBK，否则注释会乱码。
2. **不要在中断里擦写 Flash**；`flash_store_save` 会阻塞约 1s。
3. **持久化结构体务必无 padding**（用 `float` / `int32_t`，勿用 `int8_t` 混排）。
4. **`FLASH_STORE_MAGIC`** 改动会使旧数据失效，改动记录布局时记得一并修改。
5. 多路舵机各自一个 `ServoInstance`（`PWM1~4 = TIM1 CH1~4 = PE9/PE11/PE13/PE14`），混控/状态机请在 app 层用 `servo_mixer` 组合，不要把逻辑塞进驱动。

---

## 五、相关文件

| 文件 | 说明 |
|---|---|
| `Bsp/flash/bsp_flash.h` | Flash 原语 + `flash_store_save/load` 声明 |
| `Bsp/flash/bsp_flash.c` | 实现（擦除清错误标志、H7/Bank、双 Bank 存储） |
| `Bsp/flash/bsp_flash.md` | bsp_flash 详细文档 |
| `Modules/motor/servo_motor/servo_motor.h` | 舵机接口与类型 |
| `Modules/motor/servo_motor/servo_motor.c` | 舵机实现 |
| `Modules/motor/servo_motor/servo_motor.md` | servo_motor 详细文档 |
| `UserApp/robot/servo_test/` | 单路舵机 + 掉电保存的测试 app（命令见其 `robot.c` 头注释） |
