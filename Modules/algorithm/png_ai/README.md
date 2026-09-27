# PNG · 视线比例引导（Proportional Navigation Guidance）

> 模块路径：`Modules/algorithm/png_ai/`
> 作者：ai（png_ai）　版本：V1.0.0
> 适用：RoboMaster 制导飞镖的横向过载指令解算

本模块实现**视线比例引导法**，把视线角速率 `d_lambda` 解算成飞镖需要的**横向加速度指令 `a_cmd`**。模块只负责“制导律”，`d_lambda` 由视觉/卡尔曼滤波直接给出（**不接收角度序列、内部不做微分与滤波**）。

---

## 1. 控制律

需求给出的核心公式为 `a_cmd = n * v * d_lambda`，本模块在此基础上提供两种模式：

| 模式 | 公式 | 适用场景 |
|---|---|---|
| **PPN**（纯比例导引） | `a_cmd = N * v * d_lambda` | 末端**静态靶**，实现最简单 |
| **APN**（增广比例导引） | `a_cmd = N * v * d_lambda + a_o_gain * N * a_o` | 末端**横向随机移动靶**，用目标加速度做前馈补偿 |

- `N`：导航常数（默认 `4.0`，工程经验 3~5）；
- `v`：比例项速度，由 `vel_src` 选择：
  - `PNG_VEL_USE_VC`：用接近速度 `v_c`（纯比例导引经典形式，**默认**）；
  - `PNG_VEL_USE_VM`：用导弹惯性系速度 `v_m`；
- `a_o`：目标加速度，APN 使用；可来自**卡尔曼滤波推算**，也可**人工输入固定值**；
- `a_o_gain`：目标加速度项系数，经典增广比例导引取 `0.5`（即 `(1/2)·N·a_o`）。

输出前会经过**死区**与**限幅**（过载保护）。

---

## 2. 单位约定（全 SI）

| 量 | 符号 | 单位 |
|---|---|---|
| 视线角速率 | `d_lambda` | rad/s |
| 弹目相对距离 | `r` | m |
| 导弹惯性系速度 | `v_m` | m/s |
| 导弹接近速度 | `v_c` | m/s |
| 目标速度（视线上分量） | `v_t` | m/s |
| 目标加速度 | `a_o` | m/s² |
| 导航常数 | `N` | 无量纲 |
| 输出横向加速度 | `a_cmd` | m/s² |

> 角度相关量若来自视觉（像素/度），请调用方先换算成 rad 与 rad/s 再传入。

---

## 3. 文件说明

| 文件 | 作用 |
|---|---|
| `png.h` | 对外接口：枚举、结构体、函数声明 |
| `png.c` | 算法实现（每段均带注释） |
| `png_cfg.h` | **默认可调参数**（N / 限幅 / 死区 / a_o_gain），集中人工调参 |
| `README.md` | 本文档 |

---

## 4. 对外接口

```c
void  PNGInit(PNGInstance *png, PNG_Init_Config_s *config); // 初始化(NULL 用默认值)
void  PNGClear(PNGInstance *png);                           // 清运行时状态(保留配置)
float PNGCalculate(PNGInstance *png, const PNG_Input_s *in);// 单周期解算, 返回 a_cmd
void  PNGSetNavConstant(PNGInstance *png, float N);         // 在线改导航常数
void  PNGSetMode(PNGInstance *png, PNG_Mode_e mode);        // 在线切换 PPN/APN
float PNGGetCommand(PNGInstance *png);                      // 读最近一次 a_cmd
```

### 输入结构体 `PNG_Input_s`

```c
float d_lambda; // 视线角速率 [rad/s] —— 核心输入
float r;        // 弹目相对距离 [m]   —— 预留, 当前不用
float v_m;      // 导弹惯性系速度     —— vel_src=VM 时用
float v_c;      // 导弹接近速度       —— vel_src=VC 时用
float v_t;      // 目标速度视线上分量 —— 预留, 当前不用
float a_o;      // 目标加速度         —— APN 用
```

> 未用到的输入可随意填 `0`（对应需求“输入不一定全都要用”）。

---

## 5. 使用示例

### 5.1 末端静态靶 —— PPN

```c
#include "png.h"

static PNGInstance g_png;
static PNG_Input_s g_in;

void Guidance_Init(void) {
    PNG_Init_Config_s cfg = {
        .mode      = PNG_MODE_PPN,      // 纯比例导引
        .vel_src   = PNG_VEL_USE_VC,    // 用接近速度
        .N         = 4.0f,
        .MaxOut    = 5.0f,
        .DeadBand  = 0.02f,
        .a_o_gain  = 0.5f,              // PPN 下不生效
    };
    PNGInit(&g_png, &cfg);
}

/* 每个制导周期调用(例如 200Hz ~ 1kHz) */
float Guidance_Step(float d_lambda, float v_c) {
    g_in.d_lambda = d_lambda;  // 由视觉/卡尔曼给出
    g_in.v_c      = v_c;
    g_in.r = 0; g_in.v_m = 0; g_in.v_t = 0; g_in.a_o = 0;
    return PNGCalculate(&g_png, &g_in);   // 返回横向加速度指令 a_cmd
}
```

### 5.2 末端移动靶 —— APN（a_o 来自卡尔曼 / 人工固定值）

```c
void Guidance_SwitchToAPN(void) {
    PNGSetMode(&g_png, PNG_MODE_APN);   // 切换到增广比例导引
}

float Guidance_Step_Moving(float d_lambda, float v_c, float a_o_est) {
    g_in.d_lambda = d_lambda;
    g_in.v_c      = v_c;
    g_in.a_o      = a_o_est;   // ← 卡尔曼推算值; 或直接填人工标定常数
    g_in.r = 0; g_in.v_m = 0; g_in.v_t = 0;
    return PNGCalculate(&g_png, &g_in);
}
```

---

## 6. 调参建议

| 参数 | 位置 | 说明 |
|---|---|---|
| `N` | `png_cfg.h` / `PNGSetNavConstant` | 3~5。偏小收敛慢，偏大会震荡、放大噪声 |
| `MaxOut` | `png_cfg.h` | 由飞镖可用过载决定，务必设上限 |
| `DeadBand` | `png_cfg.h` | 抑制零附近抖动；设 0 关闭 |
| `a_o_gain` | `png_cfg.h` | 默认 0.5（经典 APN）；`a_o` 噪声大时调小 |
| `vel_src` | 配置 | 用 `v_c`（默认）或 `v_m`，按实际定义选 |

> 现场调参请通过**网页/上位机**下发到 `PNGSetNavConstant` / `PNGSetMode`，本模块不直接读写 Flash。

---

## 7. 注意事项

1. 本模块**只解算制导律**：`d_lambda` 的来源（工业相机/OpenMV/卡尔曼）由调用方负责；
2. 坐标系与符号：`a_cmd` 的正方向应与飞镖舵面/执行机构的横向正方向一致，若相反请在执行层取负；
3. APN 的 `a_o` 需与视线垂直方向的加速度对应（若估计的是机体/地面系加速度，请先投影到视线法向）；
4. 换靶/重新制导时调用 `PNGClear` 清状态；
5. 模块不依赖 HAL/BSP，可用主机端 `gcc` 直接编译自测。
