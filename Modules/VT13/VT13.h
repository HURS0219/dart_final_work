#pragma once

#include <stdint.h>

#include "main.h"
#include "usart.h"


/* ------------------- 帧格式常量 ------------------- */
#define VT13_SOF_1 0xA9u     ///< 帧头第一字节
#define VT13_SOF_2 0x53u     ///< 帧头第二字节
#define VT13_FRAME_SIZE 21u  ///< 完整帧字节数：SOF(2) + 有效载荷(17) + CRC16(2)

/* ------------------- 有效载荷偏移 ------------------- */
// 有效载荷从 buf[2] 开始，共 17 字节
// 摇杆/功能键/拨轮/扳机段：buf[2..9]，8 字节，内含若干 11bit/1bit 位域
// 鼠标+键盘段：buf[10..17]（跳过 buf[9] 中 mode_switch 那个字节）
//   鼠标 x  : buf[9..10]   int16_t
//   鼠标 y  : buf[11..12]  int16_t
//   鼠标 z  : buf[13..14]  int16_t
//   鼠标按键: buf[15]       1 字节 (press_l, press_r, press_m 各 2bit)
//   键盘    : buf[16..17]   uint16_t
//   以上 9 字节可作为整体 memcpy

/* ------------------- 摇杆/拨轮量程 ------------------- */
#define VT13_CH_MID 1024u  ///< 通道中值（原始 11bit）
#define VT13_CH_MIN 364u   ///< 通道最小原始值
#define VT13_CH_MAX 1684u  ///< 通道最大原始值
/// 将原始 11bit 无符号值转换为有符号偏移量 [-660, +660]
#define VT13_CH_TO_SIGNED(raw) ((int16_t)(raw) - (int16_t)VT13_CH_MID)

#define RC_SW_L ((uint16_t)0)  // 开关向上时的值
#define RC_SW_M ((uint16_t)1)  // 开关中间时的值
#define RC_SW_R ((uint16_t)2)  // 开关向下时的值

#define switch_left(s) (s == RC_SW_L)
#define switch_middle(s) (s == RC_SW_M)
#define switch_right(s) (s == RC_SW_R)

/* ------------------- 鼠标按键常量 ------------------- */
#define VT13_MOUSE_NOT_PRESSED 0u
#define VT13_MOUSE_PRESSED 1u

/* ------------------- 键盘按键 bit 位 ------------------- */
#define VT13_KEY_W (1u << 0)
#define VT13_KEY_S (1u << 1)
#define VT13_KEY_A (1u << 2)
#define VT13_KEY_D (1u << 3)
#define VT13_KEY_SHIFT (1u << 4)
#define VT13_KEY_CTRL (1u << 5)
#define VT13_KEY_Q (1u << 6)
#define VT13_KEY_E (1u << 7)
#define VT13_KEY_R (1u << 8)
#define VT13_KEY_F (1u << 9)
#define VT13_KEY_G (1u << 10)
#define VT13_KEY_Z (1u << 11)
#define VT13_KEY_X (1u << 12)
#define VT13_KEY_C (1u << 13)
#define VT13_KEY_V (1u << 14)
#define VT13_KEY_B (1u << 15)

/* ================================================================
 *  VT13 原始帧有效载荷布局（供 Decode 内部使用，外部无需关心）
 *
 *  字节偏移（相对 buf[0]，SOF 在 buf[0..1]，有效载荷从 buf[2]）：
 *
 *  buf[2..8]  — 摇杆 / 功能键 / 拨轮 / 扳机（7 字节位域）
 *    bits [0..10]   ch_0  : 右摇杆水平（11bit，原始 364~1684）
 *    bits [11..21]  ch_1  : 右摇杆垂直（11bit）
 *    bits [22..32]  ch_2  : 左摇杆垂直（11bit）
 *    bits [33..43]  ch_3  : 左摇杆水平（11bit）
 *    bits [44..45]  s1    : 左拨杆（2bit，mode_switch）
 *    bits [46]      pause : 暂停键（1bit）
 *    bits [47]      fn_1  : 功能键 1（1bit）
 *    bits [48]      fn_2  : 功能键 2（1bit）
 *    bits [49..59]  dial  : 拨轮（11bit）
 *    bits [60]      trigger：扳机（1bit）
 *    bits [61..63]  保留（3bit，对齐至 8 字节）
 *    → 共 64bit = 8 字节，buf[2..9]
 *
 *  buf[10..18] — 鼠标 + 键盘（9 字节，可整体 memcpy 进 VT13_MouseKey_t）
 *    buf[10..11] mouse.x       int16_t  小端
 *    buf[12..13] mouse.y       int16_t  小端
 *    buf[14..15] mouse.z       int16_t  小端
 *    buf[16]     mouse_buttons uint8_t  [1:0]=press_l [3:2]=press_r [5:4]=press_m
 *    buf[17..18] keyboard      uint16_t 小端，bit 定义见 VT13_KEY_* 宏
 *
 *  buf[19..20] — CRC16（decode 时可选校验，已移出结构体）
 * ================================================================ */

/**
 * @brief VT13 遥控器数据结构体（packed，与串口接收缓冲区内存布局一致）
 * @note  总帧长 21 字节，波特率 921600，每 14ms 发送一帧
 *        ch_0~ch_3、wheel 为原始 11bit 值 [364, 1684]，读取时用 VT13_CH_TO_SIGNED() 转换
 */

/**
 * @brief 鼠标 + 键盘子结构体（packed，9 字节）
 * @note  与 buf[10..18] 内存布局完全一致，可直接 memcpy
 */
#pragma pack(push, 1)
typedef struct {
  struct {
    int16_t x;            ///< 鼠标 X 轴移动速度
    int16_t y;            ///< 鼠标 Y 轴移动速度
    int16_t z;            ///< 鼠标滚轮移动速度
    uint8_t press_l : 2;  ///< 鼠标左键，1=按下
    uint8_t press_r : 2;  ///< 鼠标右键，1=按下
    uint8_t press_m : 2;  ///< 鼠标中键，1=按下
    uint8_t _pad : 2;     ///< 对齐填充，不使用
  } mouse;

  struct {
    uint16_t w : 1;      ///< W 键
    uint16_t s : 1;      ///< S 键
    uint16_t a : 1;      ///< A 键
    uint16_t d : 1;      ///< D 键
    uint16_t shift : 1;  ///< Shift 键
    uint16_t ctrl : 1;   ///< Ctrl 键
    uint16_t q : 1;      ///< Q 键
    uint16_t e : 1;      ///< E 键
    uint16_t r : 1;      ///< R 键
    uint16_t f : 1;      ///< F 键
    uint16_t g : 1;      ///< G 键
    uint16_t z : 1;      ///< Z 键
    uint16_t x : 1;      ///< X 键
    uint16_t c : 1;      ///< C 键
    uint16_t v : 1;      ///< V 键
    uint16_t b : 1;      ///< B 键
  } keyboard;
} VT13_MouseKey_t; /* sizeof == 9 */
#pragma pack(pop)

/**
 * @brief VT13 遥控器完整数据结构体
 * @note  摇杆 / 拨轮字段已在 Decode 内部转换为 [-660, +660] 的有符号值后赋入；
 *        mouse 和 keyboard 字段通过一次 memcpy 自 buf[10] 直接写入。
 */
typedef struct {
  struct {
    /* -------- 摇杆 / 拨轮 -------- */
    int16_t rocker_l_;  ///< 左摇杆水平轴（ch_3），已转换，范围 [-660, +660]，正=右
    int16_t rocker_l1;  ///< 左摇杆垂直轴（ch_2），已转换，范围 [-660, +660]，正=上
    int16_t rocker_r_;  ///< 右摇杆水平轴（ch_0），已转换，范围 [-660, +660]，正=右
    int16_t rocker_r1;  ///< 右摇杆垂直轴（ch_1），已转换，范围 [-660, +660]，正=上
    int16_t dial;       ///< 拨轮，已转换，范围 [-660, +660]

    /* -------- 拨杆 / 功能键 / 扳机 -------- */
    uint8_t mode_switch;  ///< 左拨杆档位（s1），2bit 原始值，1=上 3=中 2=下
    uint8_t pause;        ///< 暂停按键，1=按下
    uint8_t fn_1;         ///< 自定义功能键 1，1=按下
    uint8_t fn_2;         ///< 自定义功能键 2，1=按下
    uint8_t trigger;      ///< 扳机键，1=按下
  } rc;

  /* -------- 鼠标 + 键盘（整体 memcpy 自原始帧）-------- */
  VT13_MouseKey_t mouse_key;  ///< 鼠标移动/按键 + 键盘状态，与 buf[10..18] 布局一致

  /* -------- 按钮状态跟踪 -------- */
  struct {
    /* 状态标志位（toggle模式，0/1切换） */
    uint8_t pause_flag;    ///< 暂停键状态标志
    uint8_t fn_1_flag;     ///< 功能键1状态标志
    uint8_t fn_2_flag;     ///< 功能键2状态标志
    uint8_t trigger_flag;  ///< 扳机键状态标志
    uint8_t mouse_l_flag;  ///< 鼠标左键状态标志
    uint8_t mouse_r_flag;  ///< 鼠标右键状态标志
    uint8_t mouse_m_flag;  ///< 鼠标中键状态标志

    /* 按键计数器 */
    uint32_t pause_count;    ///< 暂停键按下次数
    uint32_t fn_1_count;     ///< 功能键1按下次数
    uint32_t fn_2_count;     ///< 功能键2按下次数
    uint32_t trigger_count;  ///< 扳机键按下次数
    uint32_t mouse_l_count;  ///< 鼠标左键按下次数
    uint32_t mouse_r_count;  ///< 鼠标右键按下次数
    uint32_t mouse_m_count;  ///< 鼠标中键按下次数

    /* 上一次按键状态（用于检测边沿） */
    uint8_t pause_last;    ///< 上一次暂停键状态
    uint8_t fn_1_last;     ///< 上一次功能键1状态
    uint8_t fn_2_last;     ///< 上一次功能键2状态
    uint8_t trigger_last;  ///< 上一次扳机键状态
    uint8_t mouse_l_last;  ///< 上一次鼠标左键状态
    uint8_t mouse_r_last;  ///< 上一次鼠标右键状态
    uint8_t mouse_m_last;  ///< 上一次鼠标中键状态
  } button_status;
} VT13_RC_t;

VT13_RC_t* VT13RemoteInit(UART_HandleTypeDef* huart);
uint8_t VT13RemoteIsOnline(void);