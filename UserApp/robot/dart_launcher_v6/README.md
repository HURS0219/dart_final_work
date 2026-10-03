# dart_launcher_v6 · 制导飞镖发射架 (分层解耦版)

严格按 `UserApp/application.md` 与 `UserApp/APP层应用编写指引.md` 的分层思想，
并参考 `UserApp/robot/infantry_six_wheel_example2`、`hero_mecanum_example1` 的
**组件容器风格**重构 `dart_launcher_web_v5_HIK`。

> 与旧版 `dart_launcher_web_v5_HIK`(平面: robot.c 直接 include 各 dart_*.c + 全局变量)的区别：
> 本版把机构拆成**组件**，`RobotInstance` 作为组合根持有它们；`ctrl`(robot_cmd) 是唯一大脑，
> 只写各组件的 `ctrl_cmd`；组件之间**互不 include、不共享可写全局变量**。

## 结构

```
robot.c / robot.h      组合根: RobotInstance{motor,servo,vision} + RobotInit/Task; 装配遥测
robot_config.h         全部配置(电机 ID/PID 宏/舵机/UART/视觉/自瞄) —— 沿用 v5_HIK
robot_def.h            共享枚举/结构(pack(1)): Launcher_Cmd_s(解析指令) + Launcher_Telemetry_s
ctrl/ctrl.{c,h}        大脑: 指令->目标; 自动发射时序; 自瞄 yaw; 急停/失联保护
launcher/motor.{c,h}   4×DJI 电机(拉簧A/B + 丝杆/扳机 + yaw); ★含 DWT 兜底
launcher/servo.{c,h}   扳机 PWM 舵机 (bsp_pwm)
launcher/vision.{c,h}  视觉坐标(像素误差)
launcher/link.{c,h}    上位机协议(USART6 + J-Link RTT ch1); 只解析成 Launcher_Cmd_s
launcher/store.{c,h}   掉电保存(内部 Flash 双 bank + FNV 校验) —— 沿用 v5_HIK
ui.h / robot.cmake     占位 / 板型映射(GIMBAL_BOARD -> stm32-f4)
```

数据流：`link`(收包解析) → `robot` 传 `Launcher_Cmd_s` → `ctrl`(填 motor/servo 的 ctrl_cmd)
→ `motor/servo` Task 执行；`robot` 每周期装配 `Launcher_Telemetry_s` 交 `link` 发 F 帧。

## 关键修复 (相对 dart_launcher_final)

`launcher/motor.c` 的 `MotorsTask()` 保留了 v5_HIK 的 **DWT 兜底**：
```c
if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
```
调试器连接/断开会清 `TRCENA`，导致 DJI PID 的 `dt=0` → 电机乱转/不动。旧分层版
(`dart_launcher_final`)缺这段，是其实机电机异常的主因。

## 协议 (与 v5_HIK 完全一致)

下行 `PING/H/S/Z/M/P/R/D/W/Y/A/C/V/G/SAVE`；上行 `F,...` 遥测。
因此 **`dart_launcher_web_v5_HIK/esp32` 网页** 与 **`Tools/dart_launcher_web_pc`
(J-Link RTT PC 工具)** 都无需改动即可配合本固件。

## 构建 / 烧录
```powershell
powershell -File make_one\build.ps1 -Robot dart_launcher_v6 -Board GIMBAL_BOARD
powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_launcher_v6 -Board GIMBAL_BOARD
```

## 待办
- 实机验证：CAN 通信、拉簧保持、舵机标定、时序。
- 参数值(PID/行程/舵机角度)需按实机在 `robot_config.h` 调整。
