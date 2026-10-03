/*
 * robot_def.h — dart_launcher_v6 共享定义 (跨组件通信结构 / 枚举)
 * =============================================================================
 * 分层(组件容器风格, 参考 infantry_six_wheel_example2 / hero_mecanum_example1):
 *   robot.c 组合根: 持有各组件实例, RobotTask 依次调用各组件 Task;
 *   ctrl (robot_cmd): 大脑, 解析 link 指令 -> 填各组件 ctrl_cmd;
 *   组件 (motor/servo/vision/link): 只暴露 Init/Task + ctrl_cmd/反馈;
 *   组件之间互不 include, 不共享可写全局变量。
 *
 * 通信结构用 #pragma pack(1) 包裹(框架约定)。
 * =============================================================================
 */
#pragma once

#include <stdint.h>

/* ===================== 自瞄 yaw 模式 ===================== */
typedef enum {
  LAUNCH_YAW_MANUAL = 0, /* 手动: 由上位机给目标(本机 yaw 直接速度0/角度由 cmd 决定) */
  LAUNCH_YAW_VISION = 1, /* 自瞄: 按视觉像素误差闭环 */
  LAUNCH_YAW_GUIDE = 2,  /* 引导: 固定角度 */
} Launcher_YawMode_e;

/* ===================== 自动发射时序步 ===================== */
typedef enum {
  LAUNCH_TASK_IDLE = 0,
  LAUNCH_TASK_SERVO_STD1,
  LAUNCH_TASK_SPRING_STD1,
  LAUNCH_TASK_SPRING_PREP,
  LAUNCH_TASK_SERVO_PREP,
  LAUNCH_TASK_SPRING_BACK,
  LAUNCH_TASK_SERVO_STD2,
  LAUNCH_TASK_DONE,
} Launcher_TaskStep_e;

/* ===================== 舵机位 ===================== */
typedef enum { LAUNCH_SERVO_STD = 0, LAUNCH_SERVO_PREP = 1 } Launcher_ServoPos_e;

/* ===================== 协议操作码(V / G) ===================== */
#define LAUNCH_SV_OP_NONE 0xFF
#define LAUNCH_SV_OP_SET_STD 0
#define LAUNCH_SV_OP_SET_PREP 1
#define LAUNCH_SV_OP_GO_STD 2
#define LAUNCH_SV_OP_GO_PREP 3
#define LAUNCH_SV_OP_SET_DEG 4
#define LAUNCH_SV_OP_ZERO 5

#define LAUNCH_FSM_OP_SPRING_ZERO 0
#define LAUNCH_FSM_OP_SPRING_PREP 1
#define LAUNCH_FSM_OP_SERVO_STD 2
#define LAUNCH_FSM_OP_SERVO_PREP 3
#define LAUNCH_FSM_OP_AUTO_START 10
#define LAUNCH_FSM_OP_AUTO_STOP 11
#define LAUNCH_FSM_OP_ESTOP_ON 12
#define LAUNCH_FSM_OP_ESTOP_OFF 13

/* ===================== 上位机下行指令(解析结果) =====================
 * link 只做"文本 -> 字段"翻译, 不做机构语义; ctrl 负责解释。 */
#pragma pack(1)
typedef struct {
  /* 单次动作(上升沿), ctrl 消费后由 ctrl 清? 由 link 每帧重置 */
  uint8_t req_ping;      /* PING -> 由 link 直接回 PONG */
  uint8_t req_heartbeat; /* H */
  uint8_t req_scan;      /* S */
  uint8_t req_save;      /* SAVE: 请求持久化参数(由 ctrl 触发 store) */
  uint8_t req_zero;      /* Z[,slot] */
  int8_t req_zero_slot;  /* <-1=全部, >=0=指定 */
  uint8_t req_reset;     /* R,slot */
  int8_t req_reset_slot;
  uint8_t req_dir; /* D,slot */
  int8_t req_dir_slot;

  /* 持续设定 */
  uint8_t set_motor;  /* M,slot,mode,value */
  int8_t motor_slot;
  uint8_t motor_mode;
  float motor_value;  /* 单位随 mode */
  uint8_t set_param;  /* P,slot,id,value */
  int8_t param_slot;
  uint8_t param_id;
  float param_value;
  uint8_t set_turns; /* W,turns100 : 设拉簧预备位圈数 */
  float turns;
  uint8_t set_yaw_mode; /* Y,mode */
  uint8_t yaw_mode;
  uint8_t set_aim_rpm; /* A,rpm100 */
  float aim_rpm;
  uint8_t vis_set;    /* C,x,center */
  int16_t vis_x;
  int16_t vis_center;
  uint8_t servo_op;   /* V,a[,b] */
  float servo_arg1;
  float servo_arg2;
  uint8_t fsm_op;     /* G,cmd */
  uint32_t tick;
} Launcher_Cmd_s;

/* ===================== 上行遥测(供 link 组 F 帧) ===================== */
typedef struct {
  int n;
  struct {
    int slot, type, id, online, dir, mode, target100, rpm, angle100, turns100, temp, cur;
  } motor[4];
  struct {
    int cur10, state, std10, prep10;
  } servo;
  struct {
    int spring100, step, yaw, estop, aim100;
  } task;
  struct {
    int x, ok, center, err;
  } vis;
  int param[4][11];
} Launcher_Telemetry_s;
#pragma pack()

/* 电机槽(统一在 launcher/motor.h 使用, 这里给出便于 ctrl 引用) */
enum { LM_SPRING_A = 0, LM_SPRING_B, LM_TRIGGER, LM_YAW, LM_COUNT = 4 };
