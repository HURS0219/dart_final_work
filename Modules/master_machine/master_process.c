/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */

#include "master_process.h"

#include "bsp_log.h"
#include "daemon.h"
#include "ins_task.h"
#include "navigator.h"
#include "referee.h"
#include "seasky_protocol.h"
#include "srm_protocol.h"
#define VISION_USE_VCP

#ifdef VISION_USE_VCP
static DaemonInstance *vision_daemon_instance;

static Vision_Receive_s recv_data;  // 接收数据
static Vision_Send_s send_data;     // 发送数据
static INS_t *current_attitude;
static referee_info_t *referee_info;

// 打包，注册
static Message receive;
static Message send;

static float vision_bullet_speed = 22.0f;  // 默认初速度，防止0值异常
static uint8_t vision_robot_id = 0;

uint8_t custom_data[] = {0x40, 0x50, 0x60, 0x70};
uint16_t packed_length;
void InitParam(void) {
#define RIGISTER_ID(data, id, packet) \
  data.ptr_list[id] = &(packet);      \
  data.size_list[id] = sizeof(packet);

  RIGISTER_ID(receive, 1, recv_data.gimbal_receive);
  RIGISTER_ID(receive, 2, recv_data.shoot_receive);

  RIGISTER_ID(send, 1, send_data.gimbal_send);
  RIGISTER_ID(send, 2, send_data.shoot_send);
}

/**
 * @brief 由双板通信将底盘回传的裁判系统数据注入视觉发送模块
 * @note  云台板上裁判系统未直连，需通过此接口从底盘板获取数据
 * @param bullet_speed 弹丸初速度
 * @param robot_id     本机器人ID
 */
void VisionSetRefereeData(float bullet_speed, uint8_t robot_id) {
  vision_bullet_speed = bullet_speed;
  vision_robot_id = robot_id;
}

void UpdateGimbalAttitude(Vision_Send_s *vision_send) {
  vision_send->gimbal_send.yaw = current_attitude->Yaw;
  vision_send->gimbal_send.pitch = current_attitude->Pitch;
  vision_send->gimbal_send.roll = current_attitude->Roll;
  vision_send->gimbal_send.mode = 0;
  vision_send->shoot_send.bullet_speed = vision_bullet_speed;
  vision_send->gimbal_send.color = vision_robot_id;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id) {
#ifdef VISION_USE_UART
  USARTServiceInit(vision_usart_instance);
#endif  // !VISION_USE_UART
  LOGWARNING("[vision] vision offline, restart communication.");
}

#endif

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;
static DaemonInstance *vision_daemon_instance;
/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision() {
  uint16_t flag_register;
  DaemonReload(vision_daemon_instance);  // 喂狗
                                         // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle) {
  USART_Init_Config_s conf;
  conf.module_callback = DecodeVision;
  conf.recv_buff_size = VISION_RECV_SIZE;
  conf.usart_handle = _handle;
  vision_usart_instance = USARTRegister(&conf);

  // 为master process注册daemon,用于判断视觉通信是否离线
  Daemon_Init_Config_s daemon_conf = {
      .callback = VisionOfflineCallback,  // 离线时调用的回调函数,会重启串口接收
      .owner_id = vision_usart_instance,
      .reload_count = 10,
  };
  vision_daemon_instance = DaemonRegister(&daemon_conf);

  return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend() {
  // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
  // 析构后的陷阱需要特别注意!
  static uint16_t flag_register;
  static uint8_t send_buff[VISION_SEND_SIZE];
  static uint16_t tx_len;
  // TODO: code to set flag_register
  flag_register = 30 << 8 | 0b00000001;
  // 将数据转化为seasky协议的数据包
  get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
  USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA);  // 和视觉通信使用IT,防止和接收使用的DMA冲突
  // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
  // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
  // 若使用了daemon,则也可以使用DMA发送.
}

#endif  // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"

static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len) {
  // uint16_t flag_register;
  get_srm_protocol_info(vis_recv_buff, &receive);
  // TODO: code to resolve flag_register;
}

/* 视觉通信初始化 */
Vision_Receive_s *VisionInit(IMU_Init_Config_s *imu_init_config) {
  current_attitude = INS_Init(imu_init_config);
  referee_info = GetReferee();
  USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
  vis_recv_buff = USBInit(conf);
  InitParam();
  // 为master process注册daemon,用于判断视觉通信是否离线
  Daemon_Init_Config_s daemon_conf = {
      .callback = VisionOfflineCallback,  // 离线时调用的回调函数,会重启串口接收
      .owner_id = NULL,
      .reload_count = 5,  // 50ms
  };
  vision_daemon_instance = DaemonRegister(&daemon_conf);

  return &recv_data;
}

void VisionSend() {
  static uint8_t send_buff[256];
  static uint16_t tx_len;
  UpdateGimbalAttitude(&send_data);
  // 使用新添加的打包函数打包发送数据
  srm_protocol_pack_send_data(&send, send_buff, &tx_len);

  // 发送数据
  if (tx_len > 0) {
    USBTransmit(send_buff, tx_len);
  }
}

#endif  // VISION_USE_VCP
