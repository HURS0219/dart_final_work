//
// Created by zhouz on 2026/2/28.
//

#include <stdlib.h>
#include <string.h>

#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "VT13.h"

/* ================================================================
 *  CRC-16/CCITT-FALSE
 *  多项式 P(x) = x^16 + x^12 + x^5 + 1  (0x1021)
 *  初值 0xFFFF，输入/输出不反转，无最终 XOR
 * ================================================================ */
static const uint16_t vt13_crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief 计算 CRC-16/CCITT-FALSE 校验值
 * @param p_msg  数据指针
 * @param len    数据字节数（不含 CRC 本身）
 * @param crc    初始值（首次调用传入 0xFFFF）
 * @return       计算得到的 CRC16
 */
static uint16_t vt13_crc16_calc(const uint8_t *p_msg, uint16_t len, uint16_t crc)
{
    if (p_msg == NULL) return 0xFFFFu;
    while (len--)
    {
        crc = (uint16_t)(crc >> 8) ^ vt13_crc16_tab[((uint16_t)(crc) ^ (uint16_t)(*p_msg++)) & 0x00FFu];
    }
    return crc;
}

/**
 * @brief 校验一帧数据的 CRC16
 * @param p_msg  完整帧数据（含末尾 2 字节 CRC）
 * @param len    帧总字节数（含 CRC）
 * @return       1=校验通过，0=失败
 */
static uint8_t vt13_verify_crc16(const uint8_t *p_msg, uint16_t len)
{
    if (p_msg == NULL || len <= 2u) return 0u;
    uint16_t expected = vt13_crc16_calc(p_msg, (uint16_t)(len - 2u), 0xFFFFu);
    return ((expected & 0xFFu) == p_msg[len - 2u] &&
            ((expected >> 8u) & 0xFFu) == p_msg[len - 1u]) ? 1u : 0u;
}

/* ================================================================
 *  模块私有变量
 * ================================================================ */
static VT13_RC_t        vt13_rc;           // 解析后的遥控器数据（单例）
static USARTInstance   *vt13_usart;        // 串口实例
static DaemonInstance  *vt13_daemon;       // 守护进程实例
static uint8_t          vt13_init_flag = 0;// 初始化标志

/* ================================================================
 *  内部函数
 * ================================================================ */

/**
 * @brief 遥控器离线回调：清空数据并尝试重新启动串口接收
 */
static void VT13LostCallback(void *id)
{
    (void)id;
    // memset(&vt13_rc, 0, sizeof(vt13_rc));
    USARTServiceInit(vt13_usart);
    LOGWARNING("[VT13] remote control lost");
}

/**
 * @brief 更新按钮状态：处理toggle标志和按下计数
 */
static void VT13UpdateButtonStatus(void)
{
    /* 暂停键处理 */
    if (vt13_rc.rc.pause && !vt13_rc.button_status.pause_last) {
        vt13_rc.button_status.pause_flag = !vt13_rc.button_status.pause_flag;
        vt13_rc.button_status.pause_count++;
    }
    vt13_rc.button_status.pause_last = vt13_rc.rc.pause;

    /* 功能键1处理 */
    if (vt13_rc.rc.fn_1 && !vt13_rc.button_status.fn_1_last) {
        vt13_rc.button_status.fn_1_flag = !vt13_rc.button_status.fn_1_flag;
        vt13_rc.button_status.fn_1_count++;
    }
    vt13_rc.button_status.fn_1_last = vt13_rc.rc.fn_1;

    /* 功能键2处理 */
    if (vt13_rc.rc.fn_2 && !vt13_rc.button_status.fn_2_last) {
        vt13_rc.button_status.fn_2_flag = !vt13_rc.button_status.fn_2_flag;
        vt13_rc.button_status.fn_2_count++;
    }
    vt13_rc.button_status.fn_2_last = vt13_rc.rc.fn_2;

    /* 扳机键处理 */
    if (vt13_rc.rc.trigger && !vt13_rc.button_status.trigger_last) {
        vt13_rc.button_status.trigger_flag = !vt13_rc.button_status.trigger_flag;
        vt13_rc.button_status.trigger_count++;
    }
    vt13_rc.button_status.trigger_last = vt13_rc.rc.trigger;

    /* 鼠标左键处理 */
    if (vt13_rc.mouse_key.mouse.press_l && !vt13_rc.button_status.mouse_l_last) {
        vt13_rc.button_status.mouse_l_flag = !vt13_rc.button_status.mouse_l_flag;
        vt13_rc.button_status.mouse_l_count++;
    }
    vt13_rc.button_status.mouse_l_last = vt13_rc.mouse_key.mouse.press_l;

    /* 鼠标右键处理 */
    if (vt13_rc.mouse_key.mouse.press_r && !vt13_rc.button_status.mouse_r_last) {
        vt13_rc.button_status.mouse_r_flag = !vt13_rc.button_status.mouse_r_flag;
        vt13_rc.button_status.mouse_r_count++;
    }
    vt13_rc.button_status.mouse_r_last = vt13_rc.mouse_key.mouse.press_r;

    /* 鼠标中键处理 */
    if (vt13_rc.mouse_key.mouse.press_m && !vt13_rc.button_status.mouse_m_last) {
        vt13_rc.button_status.mouse_m_flag = !vt13_rc.button_status.mouse_m_flag;
        vt13_rc.button_status.mouse_m_count++;
    }
    vt13_rc.button_status.mouse_m_last = vt13_rc.mouse_key.mouse.press_m;
}

/**
 * @brief 解析一帧 VT13 原始数据
 *        自动由串口接收回调调用，外部通常无需手动调用
 * @param buf  接收缓冲区指针，长度须 >= VT13_FRAME_SIZE
 * @param len  数据长度
 * @return     1=解析成功(有效帧), 0=失败
 */
static uint8_t VT13Decode(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < VT13_FRAME_SIZE) return 0u;

    /* 1. 验证帧头 */
    if (buf[0] != VT13_SOF_1 || buf[1] != VT13_SOF_2){
        LOGWARNING("[VT13] bad SOF: 0x%02X 0x%02X", buf[0], buf[1]);
        return 0u;
    }

    /* 2. CRC 校验（对前 19 字节计算，比对第 20、21 字节） */
    if (!vt13_verify_crc16(buf, VT13_FRAME_SIZE)){
        LOGWARNING("[VT13] CRC error");
        return 0u;
    }

    /* 3. 将 buf[2..9] 以小端方式加载到 64bit 整数，后续统一移位提取各字段 */
    uint64_t raw64 = 0;
    memcpy(&raw64, buf + 2, sizeof(uint64_t));

    /* 摇杆/拨轮：提取 11bit 原始值后减去中值，转换为 [-660, +660] */
    vt13_rc.rc.rocker_r_ = VT13_CH_TO_SIGNED((raw64 >>  0) & 0x7FFu); ///< ch_0 右摇杆水平
    vt13_rc.rc.rocker_r1 = VT13_CH_TO_SIGNED((raw64 >> 11) & 0x7FFu); ///< ch_1 右摇杆垂直
    vt13_rc.rc.rocker_l1 = VT13_CH_TO_SIGNED((raw64 >> 22) & 0x7FFu); ///< ch_2 左摇杆垂直
    vt13_rc.rc.rocker_l_ = VT13_CH_TO_SIGNED((raw64 >> 33) & 0x7FFu); ///< ch_3 左摇杆水平

    /* 功能键 / 拨杆（直接取 1~2 bit，无需偏移） */
    vt13_rc.rc.mode_switch = (uint8_t)((raw64 >> 44) & 0x3u);  ///< s1 左拨杆，2bit
    vt13_rc.rc.pause       = (uint8_t)((raw64 >> 46) & 0x1u);  ///< 暂停键
    vt13_rc.rc.fn_1        = (uint8_t)((raw64 >> 47) & 0x1u);  ///< 功能键 1
    vt13_rc.rc.fn_2        = (uint8_t)((raw64 >> 48) & 0x1u);  ///< 功能键 2
    vt13_rc.rc.dial        = VT13_CH_TO_SIGNED((raw64 >> 49) & 0x7FFu); ///< 拨轮
    vt13_rc.rc.trigger     = (uint8_t)((raw64 >> 60) & 0x1u);  ///< 扳机

    /* 鼠标 + 键盘：直接 memcpy，buf[10..18] → mouse_key */
    memcpy(&vt13_rc.mouse_key, &buf[10], sizeof(VT13_MouseKey_t));
    
    /* 更新按钮状态和计数器 */
    VT13UpdateButtonStatus();
    return 1u;
}

/**
 * @brief 串口接收完成回调：解析成功后喂狗
 */
static void VT13RxCallback(void)
{
  if (VT13Decode(vt13_usart->recv_buff, VT13_FRAME_SIZE)) {
    DaemonReload(vt13_daemon);
  }
}

/**
 * @brief 初始化 VT13 遥控器，注册串口实例和守护进程
 * @param huart  遥控器所使用的 UART 外设句柄指针（如 &huart6）
 * @return       指向内部 VT13_RC_t 数据的指针
 */
VT13_RC_t *VT13RemoteInit(UART_HandleTypeDef *huart)
{
    /* 注册串口实例 */
    USART_Init_Config_s usart_conf = {
        .usart_handle    = huart,
        .recv_buff_size  = VT13_FRAME_SIZE,
        .module_callback = VT13RxCallback,
    };
    vt13_usart = USARTRegister(&usart_conf);

    /* 注册守护进程：遥控器约 14ms 发一帧，100ms 未收到视为离线 */
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 70,           // 30 * 10ms = 300ms 超时
        .init_count   = 0,
        .callback     = VT13LostCallback,
        .owner_id     = NULL,
    };
    vt13_daemon = DaemonRegister(&daemon_conf);

    /* 初始化按钮状态 */
    memset(&vt13_rc.button_status, 0, sizeof(vt13_rc.button_status));

    vt13_init_flag = 1;
    return &vt13_rc;
}

uint8_t VT13RemoteIsOnline(void)
{
    if (vt13_init_flag && vt13_daemon != NULL) {
        return DaemonIsOnline(vt13_daemon);
    }
    return 0u;
}
