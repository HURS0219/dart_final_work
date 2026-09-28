# adrc_ai · 自抗扰控制(ADRC) 纯算法模块

<p align='right'>ai (adrc_ai) · V1.0.0 · 2026/09/28</p>

线性二阶 ADRC（LADRC：ESO + PD），用于**姿态轴（本项目 roll）稳控**。纯算法、不依赖 HAL/BSP，可主机端 gcc 自测。

## 1. 原理

对象模型 `y'' = f + b0·u`（`f` = 总扰动，含模型误差/耦合/外部扰动；`b0` = 控制增益）。

**扩张状态观测器 ESO（带宽 `wo`）**：估计 `z1≈y, z2≈y', z3≈f`
```
e  = z1 - y
z1 += dt*( z2 - 3·wo·e )
z2 += dt*( z3 - 3·wo²·e + b0·u )
z3 += dt*( -wo³·e )
```
**控制律（带宽 `wc`，阻尼 `ζ`）**：
```
u0 = wc²·(ref - z1) - 2·ζ·wc·z2
u  = clamp( (u0 - z3)/b0, ±max_out )
```

直观：ESO 把“一切没建模的东西”当成总扰动 `z3` 实时估计，控制律再把它**抵消**，因此不依赖精确模型、鲁棒性强。

## 2. 参数（`adrc_cfg.h`）— 带宽整定法

| 参数 | 含义 | 整定 |
|---|---|---|
| `wc` | 控制器带宽(rad/s) | 先定闭环快慢；越快越猛，受执行器带宽限制 |
| `wo` | 观测器带宽(rad/s) | 一般 `wo ≈ 3~5·wc`；大→估计快但噪声敏感 |
| `b0` | 控制增益 | 现场估计；**符号决定 u 极性**（错→越调越偏） |
| `zeta` | 阻尼比 | 默认 1.0（临界阻尼） |
| `max_out` | 输出限幅 | 本项目归一化 → 1.0 |

> 默认值仅为占位，**必须现场整定**。

## 3. 接口

```c
void  ADRCInit(ADRCInstance*, const ADRC_Init_Config_s*);   // NULL→默认
void  ADRCClear(ADRCInstance*);
float ADRCCalculate(ADRCInstance*, float measure, float ref, float dt);
```
- `dt` 由 app 传入（纯算法不依赖 DWT）。

## 4. 与 PID / SMC 对比

| | PID | SMC | **ADRC** |
|---|---|---|---|
| 模型依赖 | 无 | 弱 | 无(把误差当扰动) |
| 抗扰 | 一般 | 强 | **强** |
| 抖振 | 无 | 有(边界层压) | **无** |
| 需要可测状态 | 角度/速率 | 角度/速率 | 角度(速率由 ESO 估) |

## 5. 用法（dart_final，adrc 分支）

```c
static ADRCInstance s_adrc;
ADRC_Init_Config_s ac = { .wo=ADRC_DEFAULT_WO, .wc=ADRC_DEFAULT_WC,
                          .b0=ADRC_DEFAULT_B0, .max_out=ADRC_DEFAULT_MAXOUT,
                          .zeta=ADRC_DEFAULT_ZETA };
ADRCInit(&s_adrc, &ac);
/* 每周期 */
roll_cmd = ADRCCalculate(&s_adrc, att.roll_deg, 0.0f, dt);
```
`guidance.c` 用 `ROLL_CTRL_MODE` 在 PID / SMC / ADRC 之间切换（本分支默认 ADRC）。

## 6. 注意

- `b0` 的**符号与大小**最关键：先用地面试验确认“u 正 → 对应 roll 方向”。
- 输出需再经 `servo_mix_ai` 的 roll 列分配到 4 舵面。
- 本模块为实验分支（不合并 main）。主机测试见 `Debug/test_adrc.c`。
