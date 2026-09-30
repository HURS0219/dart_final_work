/*
 * robot_def.h — dart_launcher_final 共享定义: 话题名 / 错误位 / 跨 app 数据结构
 * =============================================================================
 * 严格遵循 UserApp/application.md 与 APP层应用编写指引.md:
 *   本文件只放"应用之间共享"的东西 —— 话题名、健康位、通信结构体;
 *   不放任何实现, 不放任何可写全局变量。
 *
 * 通信结构体统一用 #pragma pack(1) 包裹, 取消字节对齐(框架约定), 便于 memcpy 传输。
 *
 * 数据流(全部经 message_center 伪 pub-sub, app 之间零 include):
 *
 *   app/link ──"launch_cmd"──> app/cmd ──"fsm_cmd"────> app/fsm
 *      ^                          │                        │
 *      │                          │         ┌──────────────┴──────────────┐
 *      │                          │         v                             v
 *      │                          │   "motor_cmd"                   "servo_cmd"
 *      │                          │         │                             │
 *      │                          │         v                             v
 *      │                          │    app/motor ──> 4x DJImotor     app/trigger ──> 舵机
 *      │                          │    (CAN1)                        (servo_motor)
 *      │                          v
 *      │                    "launch_state"
 *      │                          │
 *      │                          v
 *      │                     app/yaw (自瞄轴)
 *      │                          ^
 *      │                          │
 *      │                    "aim_cmd"
 *      │                          │
 *      │                    app/vision  <──"vision_cmd"── app/link
 *      │
 *      └<── "motor_fb" / "servo_fb" / "yaw_fb" / "launch_state" ── (各 app 反馈)
 *
 *   robot.c: 只建话题 + 周期调用各 app + Monitor + 顶层状态机(不含业务)
 *
 * 【重要】角度单位约定: 全工程"输出侧 deg"(整数), 不再使用"圈"。
 *   底层 DJImotor 的 measure.total_angle 是"转子侧多圈角",
 *   换算: 输出侧角 = (total_angle - zero) / ratio * sign  (在 app/motor 内完成)
 * =============================================================================
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ============================ message_center 话题名 ============================ */
/* 命名约定: "<机构>_cmd" 为下行指令(大脑 -> 机构), "<机构>_fb" 为上行反馈(机构 -> 大脑) */
#define TOPIC_LAUNCH_CMD "launch_cmd"     /* link   -> cmd      操作员指令 (Launcher_Cmd_s)     */
#define TOPIC_LAUNCH_STATE "launch_state" /* cmd    -> link/yaw 整机状态   (Launcher_State_s)   */
#define TOPIC_FSM_CMD "fsm_cmd"           /* cmd    -> fsm      时序触发   (Launcher_FsmCmd_s)  */
#define TOPIC_FSM_FB "fsm_fb"             /* fsm    -> cmd      时序反馈   (Launcher_FsmFb_s)   */
#define TOPIC_MOTOR_CMD "motor_cmd"       /* cmd/fsm-> motor    电机指令   (Launcher_MotorCmd_s)*/
#define TOPIC_MOTOR_FB "motor_fb"         /* motor  -> cmd/link 电机反馈   (Launcher_MotorFb_s) */
#define TOPIC_MOTOR_PARAM "motor_param"   /* motor  -> link     参数回显   (Launcher_MotorParam_s)*/
#define TOPIC_SERVO_CMD "servo_cmd"       /* cmd/fsm-> trigger  舵机指令   (Launcher_ServoCmd_s)*/
#define TOPIC_SERVO_FB "servo_fb"         /* trigger-> cmd/link 舵机反馈   (Launcher_ServoFb_s) */
#define TOPIC_AIM_CMD "aim_cmd"           /* vision -> yaw      自瞄偏差   (Launcher_Aim_s)     */
#define TOPIC_VISION_CMD "vision_cmd"     /* link   -> vision   注入/配置  (Launcher_VisCmd_s)  */
#define TOPIC_YAW_CMD "yaw_cmd"           /* cmd    -> yaw      yaw 指令   (Launcher_YawCmd_s)  */
#define TOPIC_YAW_FB "yaw_fb"             /* yaw    -> cmd/link yaw 反馈   (Launcher_YawFb_s)   */

/* ================================ 健康/错误位 ================================= */
/* Monitor 汇总各 app 的 err; 每位一个故障, 便于"一眼定位是哪个 app 异常" */
#define LAUNCH_ERR_NONE 0x0000u
#define LAUNCH_ERR_MOTOR_OFF 0x0001u   /* 电机 app 卡死 / 全部电机掉线 */
#define LAUNCH_ERR_TRIGGER_OFF 0x0002u /* 舵机 app 卡死 / 注册失败 */
#define LAUNCH_ERR_YAW_OFF 0x0004u     /* yaw app 卡死 / yaw 电机掉线 */
#define LAUNCH_ERR_VISION_OFF 0x0008u  /* 视觉超时(自瞄模式下无坐标) */
#define LAUNCH_ERR_LINK_OFF 0x0010u    /* 上位机链路超时(遥控失联) */
#define LAUNCH_ERR_CMD_OFF 0x0020u     /* 大脑 app 卡死 */
#define LAUNCH_ERR_FSM_OFF 0x0040u     /* 时序 app 卡死 / 单步超时 */
#define LAUNCH_ERR_SPRING_OFF 0x0080u  /* 拉簧电机掉线(危险: 掉线=弹簧失控) */

/* 致命故障掩码: 命中即整机 ESTOP 并停机构
 * 说明:
 *   - 链路超时不列入致命(操作员拔线是常见操作), 由 app/cmd 内部做"失联停车"降级;
 *   - 视觉超时不列入致命(发射前无目标是正常状态), 由 app/yaw 走本地降级。 */
#define LAUNCH_FATAL_MASK                                                                            \
  (LAUNCH_ERR_MOTOR_OFF | LAUNCH_ERR_TRIGGER_OFF | LAUNCH_ERR_YAW_OFF | LAUNCH_ERR_CMD_OFF |         \
   LAUNCH_ERR_FSM_OFF | LAUNCH_ERR_SPRING_OFF)

/* ================================ 机构枚举 ==================================== */
/* 电机槽位: 与上位机 ESP32 的 motor slot 一一对应, 改序需同步 esp32/www/js/motor.js
 *
 * 【注意】yaw 轴**不在**本表内!
 *   yaw 有自己的 app(app/yaw), 由它独占注册 M2006(ID1)并做视觉闭环。
 *   早期版本曾把 yaw 同时放进 app/motor 与 app/yaw, 导致同一 CAN ID 被注册两次,
 *   DJIMotorInit 会进入 IDcrash 死循环(现场表现为 RTT 刷屏 "ID crash id [513]")。
 *   故这里只保留"由 app/motor 统一管理"的 3 路。 */
typedef enum {
  LAUNCH_M_SPRING_A = 0, /* 拉簧 A (M3508, CAN1 ID2, 非自锁) */
  LAUNCH_M_SPRING_B,     /* 拉簧 B (M3508, CAN1 ID3, 非自锁) */
  LAUNCH_M_SCREW,        /* 丝杆   (M3508, CAN1 ID4, 自锁, 不参与状态机) */
  LAUNCH_M_COUNT,        /* = 3 */
} Launcher_MotorSlot_e;

/* 电机控制模式: 数值即上位机协议值, 勿随意改动
 * 注: 原协议有 MODE_TURNS=3(圈), 本工程统一改角度制后已废弃, 保留编号空洞防误用。 */
typedef enum {
  LAUNCH_MODE_STOP = 0,  /* 停止(卸力, 自由) */
  LAUNCH_MODE_SPEED = 1, /* 速度环, 目标 = 输出 rpm */
  LAUNCH_MODE_ANGLE = 2, /* 角度环, 目标 = 输出 deg (相对零点) */
  /* 3 = 已废弃的 TURNS(圈), 不再使用 */
} Launcher_MotorMode_e;

/* 电机可调参数 id: 数值即上位机协议值(P,slot,id,value), 勿随意改动 */
typedef enum {
  LAUNCH_PID_SPEED_KP = 1,
  LAUNCH_PID_SPEED_KI = 2,
  LAUNCH_PID_SPEED_KD = 3,
  LAUNCH_PID_SPEED_ILIMIT = 4,
  LAUNCH_PID_SPEED_MAXOUT = 5,
  LAUNCH_PID_ANGLE_KP = 6,
  LAUNCH_PID_ANGLE_KI = 7,
  LAUNCH_PID_ANGLE_KD = 8,
  LAUNCH_PID_ANGLE_DEADBAND = 9,
  LAUNCH_PID_ANGLE_MAXOUT = 10,
  LAUNCH_PID_GEAR_RATIO = 11,
  LAUNCH_PID_COUNT = 11,
} Launcher_ParamId_e;

/* 舵机位: 标准位 / 预备(上膛)位 */
typedef enum {
  LAUNCH_SERVO_STD = 0,
  LAUNCH_SERVO_PREP = 1,
} Launcher_ServoPos_e;

/* yaw 轴工作模式 */
typedef enum {
  LAUNCH_YAW_MANUAL = 0, /* 手动: 由上位机直接给目标角度 */
  LAUNCH_YAW_VISION = 1, /* 自瞄: 按视觉像素误差闭环 */
  LAUNCH_YAW_GUIDE = 2,  /* 引导: 固定角度 */
} Launcher_YawMode_e;

/* 自动发射时序步(与旧版 dart_fsm 一致, 便于对照)
 * 数值即协议上报值, 改动需同步 esp32/www/js/task.js 的 STEPS 文案表。 */
typedef enum {
  LAUNCH_TASK_IDLE = 0,      /* 空闲 */
  LAUNCH_TASK_SERVO_STD1,    /* 舵机 -> 标准位 */
  LAUNCH_TASK_SPRING_STD1,   /* 拉簧 A/B -> 零位 */
  LAUNCH_TASK_SPRING_PREP,   /* 拉簧 A/B -> 预备位(上膛) */
  LAUNCH_TASK_SERVO_PREP,    /* 舵机 -> 预备位(待发) */
  LAUNCH_TASK_SPRING_BACK,   /* 拉簧 A/B -> 零位(释放) */
  LAUNCH_TASK_SERVO_STD2,    /* 舵机 -> 标准位(收尾) */
  LAUNCH_TASK_DONE,          /* 完成 */
} Launcher_TaskStep_e;

/* ============================ 急停策略(逐路可配) ============================= */
/* 每个机构的物理特性不同, 急停动作必须区别对待:
 *   - 拉簧: 非自锁, 卸力即弹回 → 锁位保持 + 缓慢归零卸能;
 *   - 自锁丝杆: 传动不可逆, 断电位置不变 → 减速到零后卸力(不耗电);
 *   - yaw: 无储能负载 → 直接卸力。
 * 【安全红线】拉簧绝不允许直接 DJIMotorStop(), 那会让弹簧瞬间释放。 */
typedef enum {
  LAUNCH_ESTOP_HOLD_AND_HOME = 0, /* 锁位保持 -> 以 cfg 速率缓慢归零(仅拉簧 A/B) */
  LAUNCH_ESTOP_RAMP_STOP = 1,     /* 减速到零 -> 卸力(自锁丝杆; 不能瞬间停) */
  LAUNCH_ESTOP_COAST = 2,         /* 直接卸力(无储能轴, 如 yaw) */
} Launcher_EstopMode_e;

/* ============================== 跨 app 数据结构 =============================== */
/* 约定: 所有"发布/订阅"的负载都以此定义; 每个结构末尾带 tick(发布时刻 ms)用于判活。*/

#pragma pack(1)

/* ---------------- 下行: 操作员指令 (link -> cmd) ---------------- *
 * link 只负责"把 ASCII 解析成结构体", 不做任何机构语义解释;
 * 具体怎么动由 cmd / fsm 决定 —— 这是 link 与业务解耦的关键。 */
typedef struct {
  /* 单次动作请求(上升沿有效): 由 link 置 1, cmd 消费后清 0 */
  uint8_t req_ping; /* PING: 请求回 PONG */
  uint8_t req_heartbeat; /* H: 心跳(刷新链路活性, 不回包) */
  uint8_t req_zero; /* Z[,slot]: 取零; slot<0 表示全部 */
  int8_t req_zero_slot;
  uint8_t req_reset; /* R,slot: 恢复该路 PID 默认值 */
  int8_t req_reset_slot;
  uint8_t req_dir; /* D,slot: 反转该路方向 */
  int8_t req_dir_slot;
  uint8_t req_scan; /* S: 扫描在线电机 ID -> 回 "S,<n>,<id...>" */

  /* 持续设定(每次收到即整帧刷新, 值保持不变直到下次修改) */
  uint8_t set_motor; /* M,slot,mode,value 有效标志 */
  int8_t motor_slot;
  uint8_t motor_mode;    /* Launcher_MotorMode_e */
  float motor_target;    /* 单位随 mode: rpm(速度) / deg(角度) */

  uint8_t set_param; /* P,slot,id,value 有效标志 */
  int8_t param_slot;
  uint8_t param_id; /* Launcher_ParamId_e */
  float param_value;

  /* N,slot,deg : 设某路的目标角度(整数 deg, 输出侧) —— 取代旧的 W,turns100 */
  uint8_t set_angle; /* N,<slot>,<deg> 有效标志 */
  int8_t angle_slot;
  int16_t angle_deg;

  uint8_t set_yaw_mode; /* Y,mode */
  uint8_t yaw_mode;     /* Launcher_YawMode_e */

  uint8_t set_aim_rpm; /* A,rpm100: 自瞄最大转速 */
  float aim_rpm;

  uint8_t servo_op; /* V,a[,b]: 舵机操作码(见 launcher_cfg.h 的 LAUNCH_SERVO_OP_*) */
  float servo_arg1;
  float servo_arg2;

  uint8_t fsm_op; /* G,cmd: 时序/急停操作码(见 launcher_cfg.h 的 LAUNCH_FSM_OP_*) */

  uint8_t vis_op; /* C,x,center: 视觉注入 */
  int16_t vis_x;
  int16_t vis_center;

  uint32_t tick; /* 发布时刻(ms) */
} Launcher_Cmd_s;

/* ---------------- 上行: 整机状态 (cmd -> link/yaw) ---------------- */
typedef struct {
  uint8_t estop;        /* 1=急停已触发 */
  uint8_t homing;       /* 1=正在缓慢归零(急救卸能中) */
  uint8_t task_step;    /* 时序步(Launcher_TaskStep_e, 由 fsm 经 cmd 汇总) */
  uint8_t yaw_mode;   /* Launcher_YawMode_e */
  int16_t spring_a_deg; /* 当前 A 拉簧预备位设定(deg) */
  int16_t spring_b_deg; /* 当前 B 拉簧预备位设定(deg) */
  float aim_rpm;      /* 当前自瞄最大转速(输出 rpm) */
  uint8_t link_ok;    /* 1=上位机链路正常 */
  uint32_t tick;
} Launcher_State_s;

/* ---------------- 下行: 时序触发 (cmd -> fsm) ---------------- */
typedef struct {
  uint8_t start;    /* 1=启动自动流程(上升沿) */
  uint8_t stop;     /* 1=停止自动流程(上升沿) */
  uint8_t estop;    /* 1=急停置位 */
  uint8_t estop_clr;/* 1=急停解除(上升沿) */
  uint8_t spring_go;/* 0=拉簧回零, 1=拉簧到预备位 (边沿触发) */
  uint8_t servo_go; /* Launcher_ServoPos_e (边沿触发) */
  uint32_t tick;
} Launcher_FsmCmd_s;

/* ---------------- 时序反馈 (fsm -> cmd) ---------------- */
typedef struct {
  uint8_t task_step; /* Launcher_TaskStep_e */
  uint8_t done;      /* 1=流程完成 */
  uint8_t error;     /* 1=某步超时 */
  uint32_t tick;
} Launcher_FsmFb_s;

/* ---------------- 下行: 电机指令 (cmd/fsm -> motor) ---------------- */
typedef struct {
  uint8_t mode[LAUNCH_M_COUNT]; /* 每路 Launcher_MotorMode_e */
  float target[LAUNCH_M_COUNT]; /* 每路目标: 速度=rpm, 角度=输出侧 deg */
  uint8_t estop;                /* 1=急停(按逐路策略执行) */
  uint8_t estop_clr;            /* 1=解除急停 */
  uint8_t zero_req[LAUNCH_M_COUNT];  /* 1=请求该路取零 */
  uint8_t reset_req[LAUNCH_M_COUNT]; /* 1=请求该路 PID 复位到 cfg 默认 */
  uint8_t dir_req[LAUNCH_M_COUNT];   /* 1=请求该路方向翻转 */
  /* 参数设定(P,slot,id,value): 走同一话题, 避免额外通道 */
  uint8_t set_param;   /* 1=本次帧含参数设定 */
  int8_t param_slot;   /* 目标 slot */
  uint8_t param_id;    /* Launcher_ParamId_e */
  float param_value;   /* 参数新值 */
  /* 控制权归属: 1=本帧来自 cmd, 0=来自 fsm。
   * app/motor 只认"最后写入"的那一帧, 便于 cmd 与 fsm 共存而不打架。 */
  uint8_t from_fsm;
  uint32_t tick;
} Launcher_MotorCmd_s;

/* ---------------- 上行: 电机反馈 (motor -> cmd/link) ----------------
 * 【重要约束】message_center 的 data_len 是 uint8_t(上限 255 字节)!
 *   本结构体**必须 <= 255 字节**, 否则会被静默截断, 导致话题数据损坏。
 *   为满足该约束, 参数回显拆到独立的 "motor_param" 话题(仅 link 用),
 *   本话题只保留"实时控制必读"的字段。
 *   当前大小核算: 12(状态字节) + 80(5 组 float 数组) + 12(标志字节) + 4(tick) = 108 字节 ✅ */
typedef struct {
  uint8_t online[LAUNCH_M_COUNT];    /* 1=该路 CAN 有回传 */
  uint8_t reverse[LAUNCH_M_COUNT];   /* 1=方向已反转 */
  uint8_t mode[LAUNCH_M_COUNT];      /* 当前模式 */
  float target[LAUNCH_M_COUNT];      /* 当前目标(角度=输出deg, 速度=rpm) */
  float rpm[LAUNCH_M_COUNT];         /* 输出转速(rpm) */
  float angle_deg[LAUNCH_M_COUNT];   /* 输出角度(deg, 相对零点) */
  float temperature[LAUNCH_M_COUNT]; /* 电机温度(摄氏度) */
  float current[LAUNCH_M_COUNT];     /* 实际转矩电流(原始值) */
  uint8_t at_target[LAUNCH_M_COUNT]; /* 1=已到位(角度模式, 在容差内) */
  uint8_t holding[LAUNCH_M_COUNT];   /* 1=正在锁位保持(急停) */
  uint8_t homing[LAUNCH_M_COUNT];    /* 1=正在缓慢归零 */
  uint32_t tick;
} Launcher_MotorFb_s;

/* ---------------- 上行: 电机参数回显 (motor -> link) ----------------
 * 与反馈分开: 参数(11 项 × 4 路 × 4B = 176B)会让 MotorFb 超 255 字节上限,
 * 故单列一话题。只有 app/link(组遥测帧)订阅它, 其他 app 不关心。 */
typedef struct {
  float param[LAUNCH_M_COUNT][LAUNCH_PID_COUNT]; /* 顺序同 Launcher_ParamId_e */
  uint32_t tick;
} Launcher_MotorParam_s;

/* ---------------- 下行: 舵机指令 (cmd/fsm -> trigger) ---------------- */
typedef struct {
  int16_t std_deg;   /* 标准位逻辑角(deg) */
  int16_t prep_deg;  /* 预备位逻辑角(deg) */
  int16_t target_deg;/* 本周期目标逻辑角(deg) */
  uint8_t state;     /* 当前应在的位: Launcher_ServoPos_e */
  uint8_t zero;      /* 1=请求把当前位置记为逻辑 0° */
  uint8_t set_std;   /* 1=更新标准位设定值 */
  uint8_t set_prep;  /* 1=更新预备位设定值 */
  uint8_t estop;     /* 1=急停(回标准位) */
  /* V,a[,b] 的直接操作码(见 launcher_cfg.h 的 LAUNCH_SERVO_OP_*) */
  uint8_t op;        /* 操作码; 0xFF=无操作 */
  int16_t op_arg;    /* 操作参数(deg) */
  uint32_t tick;
} Launcher_ServoCmd_s;

/* ---------------- 上行: 舵机反馈 (trigger -> cmd/link) ---------------- */
typedef struct {
  int16_t cur_deg;  /* 当前逻辑角(deg) */
  int16_t target_deg;/* 当前目标逻辑角(deg) */
  int16_t pulse_us; /* 当前输出脉宽(us) */
  int16_t std_deg;  /* 标准位(deg) */
  int16_t prep_deg; /* 预备位(deg) */
  uint8_t state;    /* Launcher_ServoPos_e */
  uint32_t tick;
} Launcher_ServoFb_s;

/* ---------------- 下行: 自瞄指令 (vision -> yaw) ---------------- */
typedef struct {
  int16_t x;      /* 绿光中心像素 x */
  int16_t center; /* 画面中心像素 x */
  int16_t err;    /* 像素误差 = x - center(由 vision 算好, yaw 直接用) */
  uint8_t ok;     /* 1=本帧识别到, 0=丢失 */
  uint32_t tick;
} Launcher_Aim_s;

/* ---------------- 下行: 视觉配置/注入 (link -> vision) ---------------- */
typedef struct {
  uint8_t inject; /* 1=本次是上位机注入坐标(联调用) */
  int16_t x;
  int16_t center;
  uint32_t tick;
} Launcher_VisCmd_s;

/* ---------------- 下行: yaw 指令 (cmd -> yaw) ---------------- */
typedef struct {
  uint8_t mode;      /* Launcher_YawMode_e */
  int16_t manual_deg;/* 手动模式目标角度(输出侧 deg) */
  int16_t guide_deg; /* 引导模式目标角度(输出侧 deg) */
  float aim_rpm;     /* 自瞄最大输出转速(rpm) */
  uint8_t estop;     /* 1=急停卸力 */
  uint32_t tick;
} Launcher_YawCmd_s;

/* ---------------- 上行: yaw 反馈 (yaw -> cmd/link) ---------------- */
typedef struct {
  float cur_rpm;  /* 当前输出转速(rpm) */
  float aim_rpm;  /* 自瞄转速上限(输出 rpm) */
  int16_t err;    /* 最近一次像素误差 */
  int16_t angle_deg;/* 当前输出角度(deg) */
  uint8_t mode;   /* Launcher_YawMode_e */
  uint8_t vis_ok; /* 1=视觉可用 */
  uint32_t tick;
} Launcher_YawFb_s;

#pragma pack()

/* ==================== 话题负载大小静态检查(编译期) ====================
 * message_center 的 data_len 是 uint8_t, 所有话题负载必须 <= 255 字节,
 * 否则会被静默截断并损坏数据。这里用负数组技巧在编译期拦住。 */
#define LAUNCH_TOPIC_MAX_LEN 255

#define LAUNCH_ASSERT_FITS(type)                                                                     \
  typedef char launch_size_ok_##type[(sizeof(type) <= LAUNCH_TOPIC_MAX_LEN) ? 1 : -1]

LAUNCH_ASSERT_FITS(Launcher_Cmd_s);
LAUNCH_ASSERT_FITS(Launcher_State_s);
LAUNCH_ASSERT_FITS(Launcher_FsmCmd_s);
LAUNCH_ASSERT_FITS(Launcher_FsmFb_s);
LAUNCH_ASSERT_FITS(Launcher_MotorCmd_s);
LAUNCH_ASSERT_FITS(Launcher_MotorFb_s);
LAUNCH_ASSERT_FITS(Launcher_MotorParam_s);
LAUNCH_ASSERT_FITS(Launcher_ServoCmd_s);
LAUNCH_ASSERT_FITS(Launcher_ServoFb_s);
LAUNCH_ASSERT_FITS(Launcher_Aim_s);
LAUNCH_ASSERT_FITS(Launcher_VisCmd_s);
LAUNCH_ASSERT_FITS(Launcher_YawCmd_s);
LAUNCH_ASSERT_FITS(Launcher_YawFb_s);

/* ===================== 每个 app 的健康状态 (Monitor 读取) ===================== */
/* 不走话题, 由 robot.c 通过各 app 的 *_GetStatus() 直接读取(与 dart_final 一致)。
 * 这是框架约定: 健康状态是"编排层"关注的元信息, 不属于业务话题。 */
typedef struct {
  uint32_t hb;  /* 心跳: app 每执行一次 +1; Monitor 比对是否更新 */
  uint16_t err; /* 错误位(LAUNCH_ERR_* 或运算) */
  float dt_us;  /* 上次任务耗时(us), 用于排查卡顿 */
  float key;    /* 关键量(调试观察用, 含义见各 app 注释) */
} Launcher_AppStatus_s;
