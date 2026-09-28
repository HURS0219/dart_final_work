#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "ins_task.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u

#pragma pack(1)
// typedef enum
// {
//   NO_FIRE = 0,
//   AUTO_FIRE = 1,
//   AUTO_AIM = 2
// } Fire_Mode_e;
//
// typedef enum
// {
//   NO_TARGET = 0,
//   TARGET_CONVERGING = 1,
//   READY_TO_FIRE = 2
// } Target_State_e;
//
// typedef enum
// {
//   NO_TARGET_NUM = 0,
//   HERO1 = 1,
//   ENGINEER2 = 2,
//   INFANTRY3 = 3,
//   INFANTRY4 = 4,
//   INFANTRY5 = 5,
//   OUTPOST = 6,
//   SENTRY = 7,
//   BASE = 8
// } Target_Type_e;

typedef struct
{
  float yaw;
  float pitch;
} Gimbal_Receive_s;

typedef struct
{
  int fire_flag;
} Shoot_Receive_s;

typedef struct
{
  Gimbal_Receive_s gimbal_receive;
  Shoot_Receive_s shoot_receive;

} Vision_Receive_s;

typedef struct {
  float yaw;
  float pitch;
  float roll;
  int mode;
  int color;
}Gimbal_Send_s;

typedef struct {
  float bullet_speed;
}Shoot_Send_s;

typedef struct
{
  Gimbal_Send_s gimbal_send;
  Shoot_Send_s shoot_send;
} Vision_Send_s;

// typedef enum
// {
//   COLOR_NONE = 0,
//   COLOR_BLUE = 1,
//   COLOR_RED = 2,
// } Enemy_Color_e;
//
// typedef enum
// {
//   VISION_MODE_AIM = 0,
//   VISION_MODE_SMALL_BUFF = 1,
//   VISION_MODE_BIG_BUFF = 2
// } Work_Mode_e;
//
// typedef enum
// {
//   BULLET_SPEED_NONE = 0,
//   BIG_AMU_10 = 10,
//   SMALL_AMU_15 = 15,
//   BIG_AMU_16 = 16,
//   SMALL_AMU_18 = 18,
//   SMALL_AMU_30 = 30,
// } Bullet_Speed_e;
//
// typedef struct
// {
//   Enemy_Color_e enemy_color;
//   Work_Mode_e work_mode;
//   Bullet_Speed_e bullet_speed;
//
//   float yaw;
//   float pitch;
//   float roll;
// } Vision_Send_s;


#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Receive_s *VisionInit(IMU_Init_Config_s* imu_init_config);

// /**
//  * @brief 发送视觉数据
//  *
//  */
// void VisionSend();
//
// /**
//  * @brief 设置视觉发送标志位
//  *
//  * @param enemy_color
//  * @param work_mode
//  * @param bullet_speed
//  */
// void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);
//
// /**
//  * @brief 设置发送数据的姿态部分
//  *
//  * @param yaw
//  * @param pitch
//  */
// void VisionSetAltitude(float yaw, float pitch, float roll);

void InitParam(void);

void VisionSend();

void navigator_send();

void VisionSetRefereeData(float bullet_speed, uint8_t robot_id);
#endif // !MASTER_PROCESS_H