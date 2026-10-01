/*
 * launcher_cfg.h — dart_launcher_final 全部可调参数 (现场调参入口: 改源码 + 烧录)
 * =============================================================================
 * 【单位约定】本工程统一"角度制":
 *   - 全部角度为**输出侧 deg**(相对各自零点), 不再使用"圈";
 *   - 速度为输出侧 rpm; 速率为 deg/s; 时间为 s(ms 显式标注); 脉宽为 us。
 *
 * 【为什么不用"圈"】圈是角度的 1/360, 用圈意味着每次都要 ×360 或 ÷360,
 *   多一次换算就多一个出错点; 直接用角度更直观, 也省掉了换算代码。
 *
 * 【为什么逐路独立】拉簧 A/B 是两根不同的弹簧, 行程/刚度/预紧都可能不同,
 *   因此 ID/方向/行程角度/PID/归零速率全部逐路独立, 互不牵连。
 *
 * 【调参分工(遵循 AGENTS.md §6, 避免"两头调糊")】
 *   整机行程不对         -> LAUNCH_S?_DEG_PREP (该路预备位角度)
 *   某路到位慢/顶不住    -> LAUNCH_S?_ANGLE_KI / _ILIMIT  (积分消静差)
 *   某路抖动/冲击        -> LAUNCH_S?_ANGLE_KP / _KD
 *   急停归零太快/太慢    -> LAUNCH_SPRING_HOMING_SEC
 *   方向反               -> LAUNCH_S?_REVERSE
 *
 * 【安全红线(务必阅读)】
 *   1. 拉簧 A/B **非自锁**: 卸力(电流=0)会让弹簧瞬间释放, 绝不允许直接 DJIMotorStop()。
 *      急停必须走"锁位保持 -> 缓慢归零"(见 LAUNCH_S?_ESTOP_MODE)。
 *   2. 保持模式下 LAUNCH_S?_ANGLE_DEADBAND **必须为 0**:
 *      PID 在死区内会把输出电流清零(见 controller.c 的 f_PID 死区分支),
 *      死区非 0 会导致"松开-拉回"锯齿振荡, 持续冲击机械。
 *   3. 保持模式下 LAUNCH_S?_ANGLE_KI **必须 > 0**:
 *      纯 PD 面对恒定弹簧拉力必然存在稳态误差, 顶不到目标位置。
 *      但 Ki > 0 必须配足够的 LAUNCH_S?_ANGLE_ILIMIT, 否则堵转时积分吹爆。
 *   4. LAUNCH_S?_ANGLE_MAXOUT 是"顶得住"与"烧电机"的界限, 须按实测保持电流留 1.5~2 倍余量。
 * =============================================================================
 */
#pragma once

/* bsp_can.h: 提供 CAN_Init_Config_s(经 motor_def.h 间接需要) */
#include "bsp_can.h"
#include "motor_def.h" /* MOTOR_DIRECTION_NORMAL / REVERSE */

/* ============================================================================
 *            0. 丝杆电机 —— 【须按实机填写, 现为占位】
 * ==========================================================================*/
/*
 * 丝杆 (M3508, CAN1 ID4) 的特性与拉簧完全不同:
 *   - **自锁**: 传动不可逆, 断电后位置由摩擦锁住 -> 不需要耗电保持;
 *   - **不参与状态机**: 转到设定角度后即完成使命, 后续不再管;
 *   - 比赛时可把 LAUNCH_SC_DEG_PREP 设为 0 = 全程保持不动。
 *
 * ⚠ TODO(须实测后填写, 现为保守占位值, 贸然上电可能撞机械限位):
 *   - LAUNCH_SC_RATIO : 电机输出轴到丝杆的**总**传动比。
 *       直连(联轴器)时 = M3508 减速比 19.2032;
 *       若中间还有齿轮/同步带, 须乘上外加传动比。
 *   - LAUNCH_SC_DEG_PREP : 丝杆行程对应的**输出侧角度**。
 *       算法: 输出侧角度 = 丝杆行程(mm) / 导程(mm每圈) * 360
 *       例:  行程 50mm, 导程 5mm/圈 -> 10 圈 -> 3600 deg
 *   - LAUNCH_SC_REVERSE : 方向, 按实机确认(正转对应丝杆前进还是后退)。
 *   - 是否需要"到位后卸力": 见 LAUNCH_SC_STOP_AT_TARGET。
 */
#define LAUNCH_SC_ID 4
#define LAUNCH_SC_RATIO 19.2032f /* ⚠ TODO 直连=19.2032, 有外加传动须重算 */
#define LAUNCH_SC_REVERSE MOTOR_DIRECTION_NORMAL /* ⚠ TODO 按实机确认 */
#define LAUNCH_SC_DEG_PREP 0                     /* ⚠ TODO 丝杆行程角; 比赛不动填 0 */
#define LAUNCH_SC_TOL_DEG 5                      /* 到位容差(输出侧 deg) */

/*
 * 丝杆到位后是否卸力(0=继续用角度环撑着; 1=到位后 DJIMotorStop 卸力)。
 * 自锁丝杆推荐 1: 省电且不发热, 自锁保证位置不变。
 * 注意: 仅当走到目标后才卸力; 运动过程中仍是角度环闭环。
 */
#define LAUNCH_SC_STOP_AT_TARGET 1

/* 丝杆 PID (M3508 角度环)
 * 说明: 丝杆负载基本恒定(不像弹簧越拉越紧), 积分需求比拉簧小, 故 Ki 可取小些。 */
#define LAUNCH_SC_ANGLE_KP 5.0f
#define LAUNCH_SC_ANGLE_KI 0.10f
#define LAUNCH_SC_ANGLE_KD 0.30f
#define LAUNCH_SC_ANGLE_ILIMIT 6000.0f
#define LAUNCH_SC_ANGLE_MAXOUT 8000.0f
/* 丝杆急停: 自锁 -> 减速到零后卸力(不能瞬间停, 避免自锁硬抓造成冲击) */
#define LAUNCH_SC_ESTOP_MODE LAUNCH_ESTOP_RAMP_STOP

/* ============================================================================
 *            1. 拉簧 A (M3508, CAN1 ID2) —— 非自锁, 危险
 * ==========================================================================*/
#define LAUNCH_SA_ID 2
#define LAUNCH_SA_RATIO 19.2032f                   /* M3508 行星减速比 */
#define LAUNCH_SA_REVERSE MOTOR_DIRECTION_NORMAL   /* 1=反向(改后需重新取零) */

/*
 * 拉簧 A 的行程角度(输出侧 deg), 相对该路零点:
 *   _DEG_ZERO: 起点(弹簧自然松弛位), 通常 0
 *   _DEG_PREP: 预备位(上膛到位), 按实机弹簧行程填写
 * 例: 输出轴要转 5 圈 -> 1800 deg
 */
#define LAUNCH_SA_DEG_ZERO 0
#define LAUNCH_SA_DEG_PREP 1800 /* ⚠ 按 A 弹簧实际行程填写 */

/* 到位容差(输出侧 deg): 状态机靠它判断"该步完成, 可推进下一步" */
#define LAUNCH_SA_TOL_DEG 5

/* 拉簧 A 角度环 PID —— 保持弹簧专用
 * ⚠ DEADBAND 必须 0; ⚠ KI 必须 >0; ⚠ ILIMIT 须按实测保持电流的 1.5~2 倍 */
#define LAUNCH_SA_ANGLE_KP 5.0f
#define LAUNCH_SA_ANGLE_KI 0.10f
#define LAUNCH_SA_ANGLE_KD 0.30f
#define LAUNCH_SA_ANGLE_DEADBAND 0.0f     /* ★ 必须 0, 否则保持电流被死区清零 */
#define LAUNCH_SA_ANGLE_ILIMIT 6000.0f    /* ★ 积分限幅, 按实测保持电流调整 */
#define LAUNCH_SA_ANGLE_MAXOUT 8000.0f    /* ★ 保持力矩上限, 防烧电机 */

/* 拉簧 A 急停策略: 锁位保持 -> 缓慢归零(卸掉弹簧储能) */
#define LAUNCH_SA_ESTOP_MODE LAUNCH_ESTOP_HOLD_AND_HOME

/* ============================================================================
 *            2. 拉簧 B (M3508, CAN1 ID3) —— 非自锁, 危险
 * ==========================================================================*/
/* 与 A 完全独立: B 是另一根弹簧, 行程/参数可各不相同 */
#define LAUNCH_SB_ID 3
#define LAUNCH_SB_RATIO 19.2032f
#define LAUNCH_SB_REVERSE MOTOR_DIRECTION_NORMAL

#define LAUNCH_SB_DEG_ZERO 0
#define LAUNCH_SB_DEG_PREP 1980 /* ⚠ 按 B 弹簧实际行程填写(与 A 不同) */

#define LAUNCH_SB_TOL_DEG 5

#define LAUNCH_SB_ANGLE_KP 5.0f
#define LAUNCH_SB_ANGLE_KI 0.10f
#define LAUNCH_SB_ANGLE_KD 0.30f
#define LAUNCH_SB_ANGLE_DEADBAND 0.0f  /* ★ 必须 0 */
#define LAUNCH_SB_ANGLE_ILIMIT 6000.0f /* ★ 按实测调整 */
#define LAUNCH_SB_ANGLE_MAXOUT 8000.0f /* ★ 按实测调整 */

#define LAUNCH_SB_ESTOP_MODE LAUNCH_ESTOP_HOLD_AND_HOME

/* ============================================================================
 *            3. Yaw 轴 (M2006, CAN1 ID1)
 * ==========================================================================*/
#define LAUNCH_YAW_ID 1
#define LAUNCH_YAW_RATIO 36.0f /* M2006 减速比 */
/*
 * 方向: **实测确认需反置**(台架验证)。
 *   现象: 目标在画面右侧(像素误差 +180)时 yaw 往左转, 两个方向都反 -> 必须取反。
 *   反向同时影响反馈符号(库对 total_angle/speed_aps 取负)与 CAN 发送值;
 *   改后需重新取零(零点定义随方向变化)。
 */
#define LAUNCH_YAW_REVERSE MOTOR_DIRECTION_REVERSE
#define LAUNCH_YAW_TOL_DEG 5

/* Yaw 速度环 PID(自瞄/手动都是用速度环跟踪) */
#define LAUNCH_YAW_SPEED_KP 3.0f
#define LAUNCH_YAW_SPEED_KI 0.30f
#define LAUNCH_YAW_SPEED_KD 0.0f
#define LAUNCH_YAW_SPEED_ILIMIT 1000.0f
#define LAUNCH_YAW_SPEED_MAXOUT 10000.0f
#define LAUNCH_YAW_SPEED_DEADBAND 3.0f

/* Yaw 角度环 PID(引导模式) */
#define LAUNCH_YAW_ANGLE_KP 5.0f
#define LAUNCH_YAW_ANGLE_KI 0.0f
#define LAUNCH_YAW_ANGLE_KD 0.30f
#define LAUNCH_YAW_ANGLE_DEADBAND 2.0f
#define LAUNCH_YAW_ANGLE_MAXOUT 25920.0f

/* Yaw 无储能负载: 急停直接卸力 */
#define LAUNCH_YAW_ESTOP_MODE LAUNCH_ESTOP_COAST

/* ============================================================================
 *            4. 舵机 (复用 Modules/motor/servo_motor)
 * ==========================================================================*/
/*
 * 舵机参数分两层(见 servo_motor_cfg.h):
 *   [信号层] 由 servo_motor_cfg.h 的 SERVO_CFG_* 提供(PTK7350 规格, 一般不碰)
 *   [逻辑层] 本处给出标准位/预备位的**逻辑角**(deg), 现场按需要改
 */
#define LAUNCH_SV_DEG_STD 0   /* 标准位逻辑角(deg) */
#define LAUNCH_SV_DEG_PREP 90 /* 预备位逻辑角(deg) */

/* 舵机速率限幅(deg/s): >0 时 ServoTask 会把逻辑角以该速率平滑逼近目标 */
#define LAUNCH_SV_RATE_DPS 300.0f

/* 舵机标定(逐项: 单位 / 含义 / 怎么调)
 *   scale      : 逻辑角 -> 机械角 增益, 1.0=1:1。指令 30° 实际偏 33° 时按 30/33 缩小。
 *   trim_deg   : 零点机械偏移(deg)。指令 0° 时位置偏了改它。
 *   reverse    : 0 正常 / 1 反向。方向反了置 1(改后需重新对零)。
 *   limit_deg  : 逻辑角对称限位(deg), 保护机构不越程。
 *   pulse_min/max_us : 脉宽硬限位, 防舵机翻转损坏。 */
#define LAUNCH_SV_SCALE 1.0f
#define LAUNCH_SV_TRIM_DEG 0.0f
#define LAUNCH_SV_REVERSE 0
#define LAUNCH_SV_LIMIT_DEG 120.0f

/* ============================================================================
 *            5. 急救归零(仅拉簧 A/B) —— 用"时长"语义, 速率自动推导
 * ==========================================================================*/
/*
 * 为什么用"时长"而不是直接写速率:
 *   真正想控制的安全量是"卸能需要多久"; 速率只是推导结果。
 *   这样改行程时速率自动跟随, 归零时长恒定, 不会忘记同步修改。
 *
 * 为什么必须显式 (float) 转换:
 *   行程角度是整数, 若写成 LAUNCH_SA_DEG_PREP / LAUNCH_SPRING_HOMING_SEC,
 *   C 会做**整数除法**并静默截断(如 1810/20 = 90 而非 90.5),
 *   导致归零时长悄悄变长。故显式转 float。
 *
 * 注意: 丝杆/yaw **不适用**此参数(丝杆自锁不需要归零, yaw 无储能)。
 */
#define LAUNCH_SPRING_HOMING_SEC 20 /* 急救归零时长(s): 从预备位缓慢回到零位 */

/* 归零速率(输出侧 deg/s), 由行程与时长自动推导 */
#define LAUNCH_SA_HOMING_RATE_DPS ((float)(LAUNCH_SA_DEG_PREP) / (float)(LAUNCH_SPRING_HOMING_SEC))
#define LAUNCH_SB_HOMING_RATE_DPS ((float)(LAUNCH_SB_DEG_PREP) / (float)(LAUNCH_SPRING_HOMING_SEC))

/* ============================================================================
 *            6. 状态机时序参数
 * ==========================================================================*/
/* 舵机到位等待(ms): 舵机无反馈, 靠固定延时等它到位 */
#define LAUNCH_SERVO_SETTLE_MS 400u
/* 单步超时(ms): 某步长时间没等到到位判定则强制推进, 防止卡死 */
#define LAUNCH_STEP_TIMEOUT_MS 8000u

/* ============================================================================
 *            7. 通信 / 链路
 * ==========================================================================*/
/* 上位机(ESP32)串口 —— 与旧版 dart_launcher_web_v5_HIK 一致: USART6 */
#define LAUNCH_LINK_UART (&huart6)
#define LAUNCH_LINK_RECV_SIZE 128 /* 单包最大字节数(定长缓冲) */

/* 链路超时(ms): 超过该时间未收到任何上位机数据 -> 视为失联, 由 cmd 做"失联停车"
 * 说明: 旧版为 2000ms; ESP32 每 500ms 发一次 H 心跳, 故 2000ms 有 4 次容错。 */
#define LAUNCH_LINK_TIMEOUT_MS 2000u

/* 遥测周期(ms): 上行 F,... 帧的发送间隔(旧版 150ms) */
#define LAUNCH_FB_PERIOD_MS 150u

/* ============================================================================
 *            8. 视觉 (PC/Jetson -> STM32)
 * ==========================================================================*/
/* 视觉超时(ms): 超过该时间未收到坐标 -> aim.ok=0(目标丢失) */
#define LAUNCH_VIS_TIMEOUT_MS 500u
/* 画面中心像素 x(默认值; 上位机可用 C,x,center 覆盖) 1440x1080 -> 720 */
#define LAUNCH_VIS_CENTER_DEFAULT 720

/* 自瞄参数(应用层 PI: 像素误差 -> 目标转速) */
#define LAUNCH_YAW_AIM_KP 3.0f        /* 每 1 像素误差对应的输出转速(rpm) */
#define LAUNCH_YAW_AIM_KI 5.0f        /* 积分消静差, 加速收敛 */
#define LAUNCH_YAW_AIM_I_LIMIT 200.0f /* 积分限幅(px*s) */
#define LAUNCH_YAW_AIM_DEADBAND 2     /* 自瞄死区(像素) */
#define LAUNCH_YAW_AIM_RPM 200.0f     /* 自瞄最大输出转速(rpm), 可由 A 命令改 */
#define LAUNCH_YAW_AIM_MIN_RPM 30.0f  /* 最小转速: 克服静摩擦, 防末端蠕动 */
#define LAUNCH_YAW_GUIDE_DEG 0        /* 引导模式固定角度(输出侧 deg) */

/* ============================================================================
 *            9. 协议操作码(与 esp32/ 上位机约定, 改动需同步前端)
 * ==========================================================================*/
/* V,a[,b] 舵机操作码 —— a 为下列值 */
#define LAUNCH_SERVO_OP_NONE 0xFF  /* 无操作(默认值) */
#define LAUNCH_SERVO_OP_SET_STD 0  /* V,0,deg10 : 设标准位 */
#define LAUNCH_SERVO_OP_SET_PREP 1 /* V,1,deg10 : 设预备位 */
#define LAUNCH_SERVO_OP_GO_STD 2   /* V,2       : 去标准位 */
#define LAUNCH_SERVO_OP_GO_PREP 3  /* V,3       : 去预备位 */
#define LAUNCH_SERVO_OP_SET_DEG 4  /* V,4,deg10 : 直接给逻辑角 */
#define LAUNCH_SERVO_OP_ZERO 5     /* V,5       : 当前位置记为 0° */

/* G,cmd 时序/急停操作码 —— cmd 为下列值 */
#define LAUNCH_FSM_OP_SPRING_ZERO 0 /* G,0  : 拉簧回零 */
#define LAUNCH_FSM_OP_SPRING_PREP 1 /* G,1  : 拉簧到预备位 */
#define LAUNCH_FSM_OP_SERVO_STD 2   /* G,2  : 舵机到标准位 */
#define LAUNCH_FSM_OP_SERVO_PREP 3  /* G,3  : 舵机到预备位 */
#define LAUNCH_FSM_OP_AUTO_START 10 /* G,10 : 启动自动流程 */
#define LAUNCH_FSM_OP_AUTO_STOP 11  /* G,11 : 停止自动流程 */
#define LAUNCH_FSM_OP_ESTOP_ON 12   /* G,12 : 急停置位 */
#define LAUNCH_FSM_OP_ESTOP_OFF 13  /* G,13 : 解除急停 */

/* ============================================================================
 *            10. 状态机每一步的角度表(核心调参处)
 * ==========================================================================*/
/*
 * 【重要】"某状态下某机构应转到什么角度"全部集中在这里, 改完烧录即可,
 *   不需要动任何逻辑代码。丝杆不参与状态机, 故不出现在下表中。
 *
 * 时序(与旧版一致, 便于对照):
 *   IDLE -> [SERVO_STD1] -> [SPRING_STD1] -> [SPRING_PREP] -> [SERVO_PREP]
 *        -> [SPRING_BACK] -> [SERVO_STD2] -> DONE
 *
 * 语义:
 *   SERVO_STD1   : 舵机先回标准位(确保安全起始姿态)
 *   SPRING_STD1  : 拉簧回零(从当前任意位置回到起点)
 *   SPRING_PREP  : 拉簧拉到预备位(上膛)
 *   SERVO_PREP   : 舵机到预备位(待发)
 *   SPRING_BACK  : 拉簧回零(释放/击发)
 *   SERVO_STD2   : 舵机回标准位(收尾)
 */
#define LAUNCH_STEP_SERVO_STD1_DEG LAUNCH_SV_DEG_STD   /* 标准位 */
#define LAUNCH_STEP_SPRING_STD1_A_DEG LAUNCH_SA_DEG_ZERO /* A 回零 */
#define LAUNCH_STEP_SPRING_STD1_B_DEG LAUNCH_SB_DEG_ZERO /* B 回零 */
#define LAUNCH_STEP_SPRING_PREP_A_DEG LAUNCH_SA_DEG_PREP /* A 上膛 */
#define LAUNCH_STEP_SPRING_PREP_B_DEG LAUNCH_SB_DEG_PREP /* B 上膛 */
#define LAUNCH_STEP_SERVO_PREP_DEG LAUNCH_SV_DEG_PREP    /* 预备位 */
#define LAUNCH_STEP_SPRING_BACK_A_DEG LAUNCH_SA_DEG_ZERO /* A 释放 */
#define LAUNCH_STEP_SPRING_BACK_B_DEG LAUNCH_SB_DEG_ZERO /* B 释放 */
#define LAUNCH_STEP_SERVO_STD2_DEG LAUNCH_SV_DEG_STD     /* 收尾 */
