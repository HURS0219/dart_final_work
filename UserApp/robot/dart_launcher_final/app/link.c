/*
 * link.c — 通信 app 实现(上位机串口协议)
 * =============================================================================
 * 【下行解析】串口回调里按 '\n' 切分, 逐条解析成 Launcher_Cmd_s 后发布话题。
 *   一次空闲帧中可能含多条命令(上位机会连发), 故循环处理。
 *
 * 【上行遥测】字段顺序与旧版 dart_launcher_web_v5_HIK 完全一致,
 *   保证 esp32/proto.py 的 _parse_state() 无需改动:
 *     F,<n>,
 *       每电机 12 项 × n : slot,type,id,online,dir,mode,target100,
 *                         rpm,angle100,deg100,temp,cur
 *       舵机 4 项        : cur10,state,std10,prep10
 *       任务 5 项        : spring_a_deg,step,yaw,estop,aim100
 *       视觉 4 项        : x,ok,center,err
 *       每电机 11 参数 × n
 *   注: 原第 10 项语义是 turns100(圈×100), 现改角度制后为 deg100(角度×100),
 *       字段个数与位置不变, 故解析不会错位; 前端显示单位需同步改成 "°"。
 *
 * 【缓冲】使用 bsp_usart 的 DMA+IDLE 接收(与 dart_final 一致)。
 * =============================================================================
 */
#include "link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "SEGGER_RTT.h" /* RTT 命令控制台(下行通道 0) */
#include "launcher_cfg.h"
#include "message_center.h"
#include "motortest.h" /* T/K 命令直控最小电机 app(调试用) */
#include "usart.h" /* LAUNCH_LINK_UART 展开为 &huart6 */

static USARTInstance *s_usart = NULL;
static Publisher_t *s_pub_cmd = NULL;

static Subscriber_t *s_sub_mf = NULL;
static Subscriber_t *s_sub_mp = NULL;
static Subscriber_t *s_sub_sf = NULL;
static Subscriber_t *s_sub_yf = NULL;
static Subscriber_t *s_sub_state = NULL;
static Subscriber_t *s_sub_vis = NULL;

static Launcher_AppStatus_s s_st;
static Launcher_Cmd_s s_cmd; /* 本周期解析结果(发布用) */
static uint8_t s_cmd_ready = 0;

/* 反馈镜像(组遥测帧用) */
static Launcher_MotorFb_s s_mf;
static Launcher_MotorParam_s s_mp;
static Launcher_ServoFb_s s_sf;
static Launcher_YawFb_s s_yf;
static Launcher_State_s s_state;
static Launcher_Aim_s s_aim; /* 视觉只用于遥测显示, 从 yaw_fb 取误差即可 */

static uint32_t s_last_rx_ms = 0;

/* ============================== 下行解析 ============================== */

/** @brief 取 PONG 等短回复 */
static void Reply(const char *s) {
  if (s_usart == NULL || s == NULL) return;
  USARTSend(s_usart, (uint8_t *)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}

/** @brief 把 "a,b,c" 解析成 long 数组, 返回个数 */
static int SplitInts(const char *s, long *out, int max) {
  int n = 0;
  const char *p = s;
  while (*p && n < max) {
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) break;
    out[n++] = v;
    p = end;
    if (*p == ',') p++;
    else break;
  }
  return n;
}

/**
 * @brief 解析一条命令 -> 填入 s_cmd
 * @note  只做"文本->字段"的翻译, 不做机构语义(那是 app/cmd 的事)。
 *        凡是单次动作(取零/复位/换向/启动...)一律置 req_ 系列标志位, 由 cmd 消费。
 */
static void ProcessOne(char *buf) {
  char cmd = buf[0];
  long a[4] = {0};
  int n = 0;

  /* 无参数命令 */
  if (strncmp(buf, "PING", 4) == 0) {
    Reply("PONG\n");
    return;
  }
  if (buf[0] == 'H' && buf[1] == '\0') {
    s_last_rx_ms = (uint32_t)DWT_GetTimeline_ms(); /* 心跳: 仅刷新活性 */
    return;
  }
  if (strncmp(buf, "SAVE", 4) == 0) {
    /* SAVE: 本工程已改为"纯 cfg"(不用 Flash, 见 launcher_cfg.h 说明)。
     * 为兼容旧网页的"保存"按钮, 这里回 SAVED 但不做任何持久化 ——
     * 真正的持久化请把值写回 launcher_cfg.h 再烧录。
     * 不静默忽略: 否则网页会以为保存成功却不知情。 */
    LOGWARNING("[link] SAVE ignored (pure-cfg build; edit launcher_cfg.h instead)");
    Reply("SAVED\n");
    return;
  }
  if (buf[0] == 'S' && buf[1] == '\0') {
    /* 扫描在线电机: 由 motor_fb 的 online 位汇总 */
    char out[64];
    int len = snprintf(out, sizeof(out), "S,%d", 0);
    int cnt = 0;
    for (int i = 0; i < LAUNCH_M_COUNT; i++) {
      if (s_mf.online[i]) cnt++;
    }
    len = snprintf(out, sizeof(out), "S,%d", cnt);
    for (int i = 0; i < LAUNCH_M_COUNT; i++) {
      if (s_mf.online[i]) {
        len += snprintf(out + len, sizeof(out) - len, ",%d", i);
      }
    }
    len += snprintf(out + len, sizeof(out) - len, "\n");
    Reply(out);
    return;
  }
  /* 废弃命令: 旧的圈数设定, 明确拒绝而不是静默执行错误的角度 */
  if (cmd == 'W') {
    Reply("ERR W_DEPRECATED use_N,slot,deg\n");
    return;
  }

  if (buf[1] == ',') n = SplitInts(buf + 2, a, 4);

  switch (cmd) {
    case 'Z': /* Z[,slot] */
      s_cmd.req_zero = 1;
      s_cmd.req_zero_slot = (n > 0) ? (int8_t)a[0] : -1;
      break;

    case 'M': /* M,slot,mode,value(value 单位随 mode) */
      if (n >= 3) {
        s_cmd.set_motor = 1;
        s_cmd.motor_slot = (int8_t)a[0];
        s_cmd.motor_mode = (uint8_t)a[1];
        /* 兼容旧版上位机的缩放: mode=1/2 时 value 为 ×10, mode=3(圈)已废弃。
         * 新角度制下上位机应直接发整数角度; 为兼容旧前端, 仍按 ×10 解释。 */
        s_cmd.motor_target = (float)a[2] / 10.0f;
      }
      break;

    case 'N': /* N,slot,deg : 直接给整数角度 */
      if (n >= 2) {
        s_cmd.set_angle = 1;
        s_cmd.angle_slot = (int8_t)a[0];
        s_cmd.angle_deg = (int16_t)a[1];
      }
      break;

    case 'P': /* P,slot,id,value(value ×100) */
      if (n >= 3) {
        s_cmd.set_param = 1;
        s_cmd.param_slot = (int8_t)a[0];
        s_cmd.param_id = (uint8_t)a[1];
        s_cmd.param_value = (float)a[2] / 100.0f;
      }
      break;

    case 'R': /* R,slot */
      if (n >= 1) {
        s_cmd.req_reset = 1;
        s_cmd.req_reset_slot = (int8_t)a[0];
      }
      break;

    case 'D': /* D,slot */
      if (n >= 1) {
        s_cmd.req_dir = 1;
        s_cmd.req_dir_slot = (int8_t)a[0];
      }
      break;

    case 'Y': /* Y,mode */
      if (n >= 1) {
        s_cmd.set_yaw_mode = 1;
        s_cmd.yaw_mode = (uint8_t)a[0];
      }
      break;

    case 'A': /* A,rpm100 */
      if (n >= 1) {
        s_cmd.set_aim_rpm = 1;
        s_cmd.aim_rpm = (float)a[0] / 100.0f;
      }
      break;

    case 'C': /* C,x,center : 视觉注入 */
      if (n >= 1) {
        s_cmd.vis_op = 1;
        s_cmd.vis_x = (int16_t)a[0];
        s_cmd.vis_center = (n >= 2) ? (int16_t)a[1] : 0;
      }
      break;

    /* ================= 最小电机 app 直控(调试阶段) =================
     * T,<slot>,<deg>   : 让 slot 路转到 deg 度(输出侧, 相对零点), 走角度环
     * T,<slot>,-9999   : 停该路(卸力)
     * T,<slot>,Z       : 把当前位置记为 0 度(取零)
     * 说明: 这条通道绕过 cmd/fsm, 直接调用 motortest 的接口, 便于在 RTT 上
     *   把"方向/整定"单独验证出来, 不受状态机与话题耦合干扰。 */
    case 'T': {
      if (n >= 1) {
        int slot = (int)a[0];
        if (slot >= 0 && slot < LAUNCH_M_COUNT) {
          if (n >= 2) {
            Motortest_SetAngle(slot, (float)a[1]);
          } else {
            Motortest_Stop(slot);
          }
        }
      }
      break;
    }
    case 'K': { /* K,<slot> : 取零 */
      if (n >= 1) {
        int slot = (int)a[0];
        if (slot >= 0 && slot < LAUNCH_M_COUNT) Motortest_Zero(slot);
      }
      break;
    }

    case 'V': /* V,a[,b] : 舵机操作 */
      if (n >= 1) {
        s_cmd.servo_op = (uint8_t)a[0];
        s_cmd.servo_arg1 = (n >= 2) ? (float)a[1] / 10.0f : 0.0f;
        s_cmd.servo_arg2 = (n >= 3) ? (float)a[2] / 10.0f : 0.0f;
      }
      break;

    case 'G': /* G,cmd : 时序/急停 */
      /* 【重要】线上协议里 G,0/G,1/G,2/G,3 就是操作码本身(0-based);
       * 但内部枚举 LAUNCH_FSM_OP_* 用 0 表示"无操作"(因为非 G 命令不设 fsm_op,
       * 该字段 memset 后为 0)。故这里必须把线上升高 1 映射到内部值,
       * 否则 "G,0" 与"没有 fsm_op"无法区分 —— 曾导致每一条 N/M 命令都被
       * 误判成 G,0(拉簧回零), 把刚设的目标清掉。 */
      if (n >= 1) {
        switch ((int)a[0]) {
          case 0: s_cmd.fsm_op = LAUNCH_FSM_OP_SPRING_ZERO; break;
          case 1: s_cmd.fsm_op = LAUNCH_FSM_OP_SPRING_PREP; break;
          case 2: s_cmd.fsm_op = LAUNCH_FSM_OP_SERVO_STD; break;
          case 3: s_cmd.fsm_op = LAUNCH_FSM_OP_SERVO_PREP; break;
          case 10: s_cmd.fsm_op = LAUNCH_FSM_OP_AUTO_START; break;
          case 11: s_cmd.fsm_op = LAUNCH_FSM_OP_AUTO_STOP; break;
          case 12: s_cmd.fsm_op = LAUNCH_FSM_OP_ESTOP_ON; break;
          case 13: s_cmd.fsm_op = LAUNCH_FSM_OP_ESTOP_OFF; break;
          default: s_cmd.fsm_op = LAUNCH_FSM_OP_NONE; break; /* 未知操作码: 不动作 */
        }
      }
      break;

    default:
      Reply("ERR\n");
      break;
  }
  s_cmd_ready = 1;
}

/**
 * @brief 把一段(可能含多行)文本按行切分并逐条解析, 结果发布到 launch_cmd
 * @param buf 以 '\0' 结尾的文本(会被就地修改)
 * @note  串口回调与 RTT 控制台共用本函数, 保证两条输入路径行为完全一致。
 */
static void ProcessLines(char *buf) {
  char *p = buf;
  while (*p) {
    char *nl = strchr(p, '\n');
    if (nl != NULL) *nl = '\0';

    /* 去掉行尾 \r 与空格 */
    size_t len = strlen(p);
    while (len > 0 && (p[len - 1] == '\r' || p[len - 1] == ' ')) p[--len] = '\0';

    if (len > 0) ProcessOne(p);
    if (nl == NULL) break;
    p = nl + 1;
  }
}

/** @brief 把本周期解析出的指令发布出去(若有) */
static void PublishIfReady(void) {
  if (s_cmd_ready) {
    s_cmd.tick = (uint32_t)DWT_GetTimeline_ms();
    PubPushMessage(s_pub_cmd, &s_cmd);
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd_ready = 0;
  }
}

/**
 * @brief 串口接收完成回调(中断上下文): 按行切分并逐条解析
 * @note  bsp_usart 会在回调返回后清空 recv_buff(见 bsp_usart.c), 故此处只读不依赖。
 */
static void Link_RxCallback(void) {
  s_last_rx_ms = (uint32_t)DWT_GetTimeline_ms();
  ProcessLines((char *)s_usart->recv_buff);
  /* 解析结果立即发布(在中断里发话题: message_center 是纯内存拷贝, 无阻塞) */
  PublishIfReady();
}

/* ============================== RTT 命令控制台 ==============================
 * 【为什么需要】现场调试常常只有 J-Link/SWD, 没有空闲串口; 而本工程唯一的命令
 *   入口原本是 USART6。这里把 RTT 的**下行通道(0)** 也接进来, 使 RTT 与 USART6
 *   共用同一条解析链路:
 *       文本 -> ProcessLines() -> ProcessOne() -> launch_cmd 话题 -> app/cmd
 *   于是 N/M/P/R/D/Z/V/G/C/Y/A/H/S/SAVE 等全部命令都能从 SWD 直接发。
 *
 * 【为什么放在主循环而不是中断】SEGGER_RTT_Read 不是中断安全的, 且命令频率很低,
 *   在 Link_Task 里轮询即可(1kHz), 对实时性无影响。
 *
 * 【注意】RTT 下行缓冲默认仅 16 字节(SEGGER_RTT_Conf.h 的 BUFFER_SIZE_DOWN),
 *   故单条命令不要超过约 14 字节(留 \r\n)。
 * ==========================================================================*/
static char s_rtt_line[64];
static uint8_t s_rtt_len = 0;

static void RttConsolePoll(void) {
  char buf[32];
  unsigned n;
  unsigned i;

  if (!SEGGER_RTT_HasData(0)) return;
  n = SEGGER_RTT_Read(0, buf, sizeof(buf));
  for (i = 0; i < n; i++) {
    char c = buf[i];
    if (c == '\r' || c == '\n') {
      if (s_rtt_len > 0) {
        s_rtt_line[s_rtt_len] = '\0';
        /* 复用同一条解析链路 */
        ProcessLines(s_rtt_line);
        PublishIfReady();
        s_rtt_len = 0;
      }
    } else if (s_rtt_len < sizeof(s_rtt_line) - 1) {
      s_rtt_line[s_rtt_len++] = c;
    } else {
      s_rtt_len = 0; /* 溢出: 丢弃整行, 防止半条命令被误解析 */
    }
  }
}

/* ============================== 上行遥测 ============================== */

static void SendTelemetry(void) {
  static char buf[1100];
  int len = snprintf(buf, sizeof(buf), "F,%d", LAUNCH_M_COUNT);

  /* 每电机 12 项 */
  for (int i = 0; i < LAUNCH_M_COUNT; i++) {
    int mode = (int)s_mf.mode[i];
    int target100 = (mode == LAUNCH_MODE_STOP) ? 0 : (int)(s_mf.target[i] * 100.0f);
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", i,
                    (int)M3508, /* type: 本表 3 路均为 M3508(yaw 由 app/yaw 单独上报) */
                    0,          /* id: 占位(实际 CAN ID 见 cfg) */
                    (int)s_mf.online[i], (int)s_mf.reverse[i], mode, target100,
                    (int)s_mf.rpm[i], (int)(s_mf.angle_deg[i] * 100.0f),
                    (int)(s_mf.angle_deg[i] * 100.0f), /* ★ 原 turns100 -> 角度×100 */
                    (int)s_mf.temperature[i], (int)s_mf.current[i]);
  }

  /* 舵机 4 项 */
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", (int)(s_sf.cur_deg * 10),
                  s_sf.state, (int)(s_sf.std_deg * 10), (int)(s_sf.prep_deg * 10));

  /* 任务 5 项: spring_a_deg, step, yaw, estop, aim100
   * ★ 原 spring100(圈×100) -> 现为 A 拉簧预备位角度(deg) */
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d", (int)s_state.spring_a_deg,
                  s_state.task_step, s_state.yaw_mode, s_state.estop,
                  (int)(s_state.aim_rpm * 100.0f));

  /* 视觉 4 项 */
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", s_aim.x, s_aim.ok, s_aim.center,
                  s_yf.err);

  /* 每电机 11 参数 ×100(来自独立话题 motor_param) */
  for (int i = 0; i < LAUNCH_M_COUNT; i++) {
    for (int j = 0; j < LAUNCH_PID_COUNT; j++) {
      len += snprintf(buf + len, sizeof(buf) - len, ",%d", (int)(s_mp.param[i][j] * 100.0f));
    }
  }

  buf[len++] = '\n';
  buf[len] = '\0';
  Reply(buf);
}

/* ============================== 生命周期 ============================== */

void Link_Init(void) {
  USART_Init_Config_s cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.usart_handle = LAUNCH_LINK_UART;
  cfg.recv_buff_size = LAUNCH_LINK_RECV_SIZE;
  cfg.module_callback = Link_RxCallback;

  s_usart = USARTRegister(&cfg);
  if (s_usart == NULL) {
    LOGERROR("[link] USART register FAIL");
  } else {
    LOGINFO("[link] usart ready (recv=%d)", LAUNCH_LINK_RECV_SIZE);
  }

  memset(&s_cmd, 0, sizeof(s_cmd));
  memset(&s_mf, 0, sizeof(s_mf));
  memset(&s_mp, 0, sizeof(s_mp));
  memset(&s_sf, 0, sizeof(s_sf));
  memset(&s_yf, 0, sizeof(s_yf));
  memset(&s_state, 0, sizeof(s_state));
  memset(&s_aim, 0, sizeof(s_aim));
  memset(&s_st, 0, sizeof(s_st));

  s_pub_cmd = PubRegister(TOPIC_LAUNCH_CMD, sizeof(Launcher_Cmd_s));
  s_sub_mf = SubRegister(TOPIC_MOTOR_FB, sizeof(Launcher_MotorFb_s));
  s_sub_mp = SubRegister(TOPIC_MOTOR_PARAM, sizeof(Launcher_MotorParam_s));
  s_sub_sf = SubRegister(TOPIC_SERVO_FB, sizeof(Launcher_ServoFb_s));
  s_sub_yf = SubRegister(TOPIC_YAW_FB, sizeof(Launcher_YawFb_s));
  s_sub_state = SubRegister(TOPIC_LAUNCH_STATE, sizeof(Launcher_State_s));
  s_sub_vis = SubRegister(TOPIC_AIM_CMD, sizeof(Launcher_Aim_s));
}

void Link_Task(void) {
  uint32_t t0 = (uint32_t)DWT_GetTimeline_us();
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  uint16_t err = LAUNCH_ERR_NONE;

  /* ---- 取各 app 反馈(用于组帧) ---- */
  {
    Launcher_MotorFb_s mf;
    Launcher_MotorParam_s mp;
    Launcher_ServoFb_s sf;
    Launcher_YawFb_s yf;
    Launcher_State_s st;
    Launcher_Aim_s am;
    if (SubGetMessage(s_sub_mf, &mf)) s_mf = mf;
    if (SubGetMessage(s_sub_mp, &mp)) s_mp = mp;
    if (SubGetMessage(s_sub_sf, &sf)) s_sf = sf;
    if (SubGetMessage(s_sub_yf, &yf)) s_yf = yf;
    if (SubGetMessage(s_sub_state, &st)) s_state = st;
    if (SubGetMessage(s_sub_vis, &am)) s_aim = am;
  }

  /* ---- RTT 命令控制台(SWD 调试用): 与 USART6 共用解析链路 ---- */
  RttConsolePoll();

  /* ---- 周期发遥测 ---- */
  static uint32_t last_fb = 0;
  if (now - last_fb >= LAUNCH_FB_PERIOD_MS) {
    last_fb = now;
    SendTelemetry();
  }

  /* ---- 链路活性 ---- */
  if (s_last_rx_ms != 0 && (now - s_last_rx_ms) > LAUNCH_LINK_TIMEOUT_MS) {
    err |= LAUNCH_ERR_LINK_OFF;
  }

  s_st.hb++;
  s_st.dt_us = (float)(DWT_GetTimeline_us() - t0);
  s_st.key = (float)s_state.task_step; /* 关键量: 时序步 */
  s_st.err = err;
}

const Launcher_AppStatus_s *Link_GetStatus(void) { return &s_st; }
