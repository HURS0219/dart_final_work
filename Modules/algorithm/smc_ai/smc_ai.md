# smc_ai · 滑模控制(SMC) 纯算法模块

<p align='right'>ai (smc_ai) · V1.0.0 · 2026/09/28</p>

一阶滑模控制 + 边界层趋近律，用于**姿态轴（本项目 roll）稳控**。纯算法、不依赖 HAL/BSP，可主机端 gcc 自测。

## 1. 控制律

```
滑模面:  s = c·e + e_dot
趋近律:  u = clamp( -k·sat(s/phi), ±max_out )
sat(x) = x (|x|<1) / sign(x) (否则)      // 用饱和代替符号, 抑制抖振(舵机友好)
```
- `e`：误差；`e_dot`：误差导数。**符号约定 `e = 被测 - 期望`（output - desired）**，配合 `u=-k·sat(s)`（教科书形式）。
  本项目 roll 轴：`e = roll_deg - 0`，`e_dot = gx_dps - 0`。
- 输出 `u` 归一化（−1..1），直接作为 `mix.roll`。

## 2. 参数（`smc_cfg.h`）

| 参数 | 含义 | 影响 |
|---|---|---|
| `c` | 滑模面斜率(1/s) | 大→重角度、响应快易抖；小→重角速度、收敛慢 |
| `k` | 切换增益 | 大→趋近快/抗扰强，过大抖振 |
| `phi` | 边界层厚度 | 大→平滑但软；小→接近 sign、抖 |
| `max_out` | 输出限幅 | 防饱和；本项目 1.0 |

> 默认值仅为占位，**必须现场整定**。

## 3. 接口

```c
void  SMCInit(SMCInstance*, const SMC_Init_Config_s*);   // NULL→默认
void  SMCClear(SMCInstance*);
float SMCCalculate(SMCInstance*, float err, float err_dot, float dt);
```
- `dt` 当前不参与基础律（预留：等效控制 / DWT）。

## 4. 与 PID 对比

| | PID(controller) | SMC |
|---|---|---|
| 模型依赖 | 无（但需整定） | 弱模型 |
| 抗扰/鲁棒 | 一般 | **强**（滑模不变性） |
| 抖振 | 无 | 有（靠边界层压制） |
| 整定 | Kp/Ki/Kd | c/k/phi |

## 5. 用法（dart_final，smc 分支）

```c
static SMCInstance s_smc;
SMC_Init_Config_s sc = { .c=SMC_DEFAULT_C, .k=SMC_DEFAULT_K,
                         .phi=SMC_DEFAULT_PHI, .max_out=SMC_DEFAULT_MAXOUT };
SMCInit(&s_smc, &sc);
/* 每周期 */
roll_cmd = SMCCalculate(&s_smc, 0.0f - att.roll_deg, 0.0f - att.gx_dps, dt);
```
`guidance.c` 用 `ROLL_CTRL_MODE` 在 PID / SMC 之间切换（默认 SMC）。

## 6. 注意

- 输出需再经 `servo_mix_ai` 的 roll 列分配到 4 舵面。
- 抖振若仍大：增大 `phi` 或对 `u` 加低通；但会增加滞后。
- 本模块为实验分支（不合并 main）。主机测试见 `Debug/test_smc.c`。
