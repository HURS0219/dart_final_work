# chassis_wheel_legged_sjtu 功率控制逻辑说明

本文档对应当前 `chassis_wheel_legged_sjtu` 的功率控制实现，重点文件：

- `UserApp/components/chassis/chassis_wheel_legged_sjtu/chassis.c`
- `UserApp/components/chassis/chassis_wheel_legged_sjtu/chassis.h`
- `UserApp/components/chassis/chassis_wheel_legged_sjtu/power_control/power_control.c`
- `UserApp/components/chassis/chassis_wheel_legged_sjtu/power_control/power_control.h`
- `Modules/super_cap/super_cap.c`

当前代码中有三层和功率相关的逻辑：

1. `SuperCapModeControl()`：根据裁判系统功率限制、超电电压、超电指令计算 `chassis_ctrl_cmd.max_power`。
2. `PowerControl()`：平衡/站立路径下，在 LQR 输出前改写运动指令，间接压低运动扭矩。
3. `PowerControl_Prostrate()`：卧倒路径下，在速度 PID 算完后直接改写轮电机 `final_output`，对本帧输出限流。

## 主循环调用顺序

`ChassisTask()` 中每帧大致顺序为：

```text
根据 chassis_mode 调用对应控制路径
  CHASSIS_ON / CHASSIS_STAIR      -> ChassisCtrlUpdate()
  CHASSIS_PROSTRATE               -> ChassisProstrate()
  CHASSIS_JUMP_READY / START      -> ChassisJump()
  其他模式                         -> 对应恢复/停机逻辑

chassis_ctrl_cmd->max_power = SuperCapModeControl(...)
每 10ms SuperCapSendMessage(...)
LimitChassisOutput()
```

需要注意：`PowerControl()` 在 `ChassisCtrlUpdate()` 内部执行，发生在本帧
`SuperCapModeControl()` 更新 `max_power` 之前。不过当前 `PowerControl()` 的
`P_total_ref` 是硬编码 `200.f`，没有使用 `chassis_ctrl_cmd.max_power`。

卧倒路径不同：`PowerControl_Prostrate()` 在 `LimitChassisOutput()` 内执行，此时
`chassis_ctrl_cmd.max_power` 已经由本帧 `SuperCapModeControl()` 更新。

## 平衡路径链路

`ChassisCtrlUpdate()` 的核心顺序为：

```text
LegModelUpdate()
StateVarUpdate()
LQR_K_Calc()
StateErrCalc()
PowerControl()
ChassisPlannerUpdate()
StateErrCalc()
LocomotionController()
LegController()
```

`PowerControl()` 并不直接限 `leg[i]->real_model.T`，而是改写
`chassis_ctrl_cmd.vx / target_yaw / wz`。随后 `ChassisPlannerUpdate()` 对这些
指令继续做规划/斜坡，第二次 `StateErrCalc()` 再生成真正进入 LQR 的
`state_err`。

因此平衡路径功控是“改运动需求”的间接限功率，不是“本帧强行截断轮端力矩”的
直接限幅。

## PowerControl() 当前算法

`PowerControl()` 先把轮电机 LQR 输出按状态维度拆成两部分：

- `T_motion[i]`：由 `state_err[0..3]` 产生的运动相关扭矩。
  包括 `x_b / x_b_d / phi / phi_d`。
- `T_balance[i]`：由 `state_err[4..9]` 产生的平衡/腿部/机身姿态扭矩。
  包括左右腿角度、左右腿角速度、机身俯仰、机身俯仰角速度。

对应代码：

```c
for (int j = 0; j < 4; j++) {
  pc->T_motion[0] -= chassis->LQR_K[2][j] * chassis->state_err[j];
  pc->T_motion[1] -= chassis->LQR_K[3][j] * chassis->state_err[j];
}
for (int j = 4; j < 10; j++) {
  pc->T_balance[0] -= chassis->LQR_K[2][j] * chassis->state_err[j];
  pc->T_balance[1] -= chassis->LQR_K[3][j] * chassis->state_err[j];
}
```

然后用当前拍的期望总扭矩估算电流和功率：

```c
current_I = t2i(pc->T_motion[i] + pc->T_balance[i]);
current_w = chassis->leg[i]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;
pc->P[i] = MotorEstimatePower(pc->k, pc->I[i], pc->w[i]);
pc->P_total = pc->P[0] + pc->P[1];
```

当前平衡路径功率上限：

```c
pc->P_total_ref = 200.f;
```

如果要让平衡路径跟随超电/裁判系统功率，应改回：

```c
pc->P_total_ref = chassis->chassis_ctrl_cmd.max_power;
```

但这会受到主循环时序影响：`PowerControl()` 读取到的是上一帧计算出的
`max_power`，除非调整调用顺序。

## 刹车功率放开逻辑

平衡路径中额外计算 motion 分量的机械功率方向：

```c
P_motion_mech += pc->T_motion[i] * -pc->w[i];
```

这里使用 `-pc->w[i]`，因为当前底盘模型中 `speed_aps` 与底盘前进方向相反。

- `P_motion_mech >= 0`：motion 分量整体在做正功，理解为加速/驱动。
- `P_motion_mech < 0`：motion 分量整体在做负功，理解为刹车/回收。

功控只在“超功率且 motion 不是净刹车”时介入：

```c
if (pc->P_total > pc->P_total_ref && P_motion_mech >= -1e-3f) {
  ...
}
```

也就是说，平衡模式下如果当前运动需求是在刹车，即使估算总功率超过
`P_total_ref`，也不会缩小 `vx / target_yaw / wz` 对应的运动控制量。这样可以避
免功控把刹车力一起削掉。

这个判断只放开 motion 净刹车。若平衡/姿态分量本身仍然消耗功率，当前实现也不会
因为它在这一拍压缩 motion 刹车需求，这是有意偏向保留制动能力的取舍。

## 超功率时的缩放方式

当满足限功率条件时，`PowerControl()` 按当前两个轮子的估算功率占比分配功率预算：

```c
pc->P_ref[i] = pc->P[i] / pc->P_total * pc->P_total_ref;
pc->I_ref[i] = MotorEstimateCurrent(pc->k, pc->P_ref[i], pc->w[i], pc->I[i]);
pc->T_ref[i] = i2t(pc->I_ref[i]);
pc->T_motion_ref[i] = pc->T_ref[i] - pc->T_balance[i];
```

然后把每个轮子的 motion 扭矩缩放为 `0..1`：

```c
float s = pc->T_motion_ref[i] / pc->T_motion[i];
VAL_LIMIT(s, 0.0f, 1.0f);
pc->scale_motion[i] = s;
```

两个轮子的缩放系数取平均：

```c
pc->scale_combined = (pc->scale_motion[0] + pc->scale_motion[1]) * 0.5f;
```

最后只缩放 `state_err[1..3]` 并反写运动指令：

```c
chassis->state_err[1] *= pc->scale_combined;
chassis->state_err[2] *= pc->scale_combined;
chassis->state_err[3] *= pc->scale_combined;

chassis->chassis_ctrl_cmd.vx = sv->x_b_d - chassis->state_err[1];
chassis->chassis_ctrl_cmd.target_yaw = sv->phi - chassis->state_err[2];
chassis->chassis_ctrl_cmd.wz = sv->phi_d - chassis->state_err[3];
```

`state_err[0]` 参与了 `T_motion` 估算，但最终没有被缩放回写，因为 `x_b` 参考仍
保持为 0。

## 卧倒路径 PowerControl_Prostrate()

卧倒模式下，`ChassisProstrate()` 先生成 `wheel_speed_ref[]`。在
`LimitChassisOutput()` 中：

```text
wheel_speed_ref 限幅到 +/-53000
DJIMotorSetPIDRef() 运行轮电机速度 PID
PowerControl_Prostrate() 根据 final_output 限流
EnableJointMotor()
```

`PowerControl_Prostrate()` 使用速度 PID 已经算出的 `final_output` 估算电流：

```c
current_I = final_output * DJI_CURRENT_SCALE;
current_w = speed_aps * DEGREE_2_RAD;
pc->P[i] = MotorEstimatePower(pc->k, pc->I[i], pc->w[i]);
pc->P_total_ref = chassis->chassis_ctrl_cmd.max_power;
```

超功率时同样按当前功率占比分配 `P_ref[i]`，再反解 `I_ref[i]`：

```c
pc->P_ref[i] = pc->P[i] / pc->P_total * pc->P_total_ref;
pc->I_ref[i] = MotorEstimateCurrent(pc->k, pc->P_ref[i], pc->w[i], pc->I[i]);
```

最后直接覆盖轮电机输出：

```c
final_output = (int16_t)(pc->I_ref[i] / DJI_CURRENT_SCALE);
```

所以卧倒路径是直接限本帧电流输出，和非卧倒的“改运动指令”不同。

## 功率模型

`MotorEstimatePower()` 使用 6 参数模型：

```c
P = k0 + k1 * I + k2 * w + k3 * I * w + k4 * I * I + k5 * w * w;
P = fmaxf(P, 0.0f);
```

当前系数定义在 `power_control.h`：

```c
#define WHEEL_K0 0.6641993412640775f
#define WHEEL_K1 0.006444284468539646f
#define WHEEL_K2 0.0001423857226262331f
#define WHEEL_K3 0.017644430204543864f
#define WHEEL_K4 0.1650143850678086f
#define WHEEL_K5 3.096721772539512e-05f
```

电流/扭矩换算：

```c
#define DJI_CURRENT_SCALE (20.0f / 16384.0)
#define t2i(torque) ((torque) * (3591.0f / 187.0f) / (268.0f / 17.0f) / 0.3f)
#define i2t(current) ((current) * 0.3f * (268.0f / 17.0f) / (3591.0f / 187.0f))
```

`MotorEstimateCurrent()` 把目标功率反解为允许电流，当前反解式为：

```c
A = k[4];
B = k[1] + k[3] * w;
C = k[0] + k[2] * w + k[5] * w * w - fabsf(P_target);
```

再根据 `I_current` 的符号选择更接近当前电流的根，并限幅到约 `+/-20A`。

## 超级电容功率上限

`SuperCapModeControl(super_cap, power_limit)` 输出底盘内部 `max_power`。当前状态机：

| 当前状态 | 条件 | 下一状态 | `max_power` |
| --- | --- | --- | --- |
| 任意 | `error_detect != 0` | `SAFETY_MODE` | `power_limit` |
| `SAFETY_MODE` | `ctrl_cmd == BOOST` | `ACTIVE_MODE` | `180` |
| `SAFETY_MODE` | `cap_v > 18V` | `PASSIVE_MODE` | `power_limit + 40` |
| `SAFETY_MODE` | 其他 | 保持 | `power_limit` |
| `CHARGING_MODE` | `cap_v < 10V` | 保持 | `power_limit * 0.8` |
| `CHARGING_MODE` | `cap_v > 18V` | `PASSIVE_MODE` | `power_limit + 40` |
| `CHARGING_MODE` | 其他 | 保持 | `power_limit * 0.9` |
| `PASSIVE_MODE` | `ctrl_cmd == BOOST` | `ACTIVE_MODE` | `180` |
| `PASSIVE_MODE` | `cap_v < 15V` | `CHARGING_MODE` | `power_limit - power_limit^2 * 0.0025` |
| `PASSIVE_MODE` | `15V <= cap_v <= 18V` | `SAFETY_MODE` | `power_limit` |
| `PASSIVE_MODE` | `cap_v > 18V` | 保持 | `power_limit + 40` |
| `ACTIVE_MODE` | `cap_v < 15V` | `CHARGING_MODE` | `power_limit * 0.9` |
| `ACTIVE_MODE` | `ctrl_cmd == NORMAL` | `PASSIVE_MODE` | `cap_v > 18V ? power_limit + 40 : power_limit` |
| `ACTIVE_MODE` | 其他 | 保持 | `180` |

`SuperCapSendMessage()` 当前每 10ms 发送一次裁判系统功率限制、缓冲能量和底盘电源输出状态。

## 当前实现注意点

1. 平衡路径 `P_total_ref` 目前硬编码为 `200.f`，没有读取 `chassis_ctrl_cmd.max_power`。

2. 平衡路径功控通过改写 `chassis_ctrl_cmd` 间接生效，会受到 planner 斜坡影响；它不是直接本帧限轮端扭矩。

3. 刹车放开逻辑只看 motion 净机械功率。它会优先保留制动能力，可能允许总估算功率在刹车瞬间超过 `P_total_ref`。

4. `MotorEstimatePower()` 和 `MotorEstimateCurrent()` 当前使用带符号的 `I`、`w`，而函数注释里仍有 `|I|`、`|w|` 的描述，二者没有完全统一。

5. `MotorEstimatePower(float k[6], ...)` 内部会对传入的 `k[]` 乘以 `power_ctrl_k_coff`。当前 `power_ctrl_k_coff = 1.0f`，没有实际影响；如果之后改成其他值，需要避免每次调用反复缩放系数。

6. `Power_Ctrl_t` 头文件大段注释仍偏向旧版“PI/速度限制”思路，和当前“LQR motion 缩放 + 卧倒限流”实现不完全一致。

