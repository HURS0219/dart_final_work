/*
 * simple_motor.h — 最简通用 DJI 电机多机 CAN 总线 (GM6020 / M3508 / M2006)
 *
 * 一条 CAN 总线上最多挂 4 个电机, 同一目标同时驱动全部。
 * 控制帧: GM6020 用 0x1FF/0x2FF; M3508/M2006 用 0x200/0x1FF; 8字节=4电机x2字节
 * 反馈帧: GM6020 0x204+id; M3508/M2006 0x200+id
 */
#ifndef WIFI_GM6020_SIMPLE_MOTOR_H
#define WIFI_GM6020_SIMPLE_MOTOR_H

#include <stdint.h>

#define SIMPLE_MOTOR_MAX 4

typedef enum {
  SIMPLE_MOTOR_GM6020 = 0,
  SIMPLE_MOTOR_M3508,
  SIMPLE_MOTOR_M2006,
} SimpleMotorType_e;

typedef struct {
  SimpleMotorType_e type;
  uint8_t id;
  uint8_t used;
  /* 反馈 */
  uint16_t ecd;
  int16_t rpm;
  int16_t current;
  uint8_t temp;
  uint32_t fb_count;
  float total_angle;  // 多圈累计角度(电机侧, 度)
  int32_t round;
  uint8_t inited;
  /* 输出(待发送的值) */
  int16_t out;
  /* 角度模式 */
  float zero;         // 角度零点(电机总角度)
  float speed_integral;
  float angle_out;
} SimpleMotor_t;

extern SimpleMotor_t g_motors[SIMPLE_MOTOR_MAX];
extern uint8_t g_motor_num;

void SimpleMotorBusInit(void);                              // 清空 + 配置 CAN
void SimpleMotorClear(void);                                // 清空电机列表
int SimpleMotorAdd(SimpleMotorType_e type, uint8_t id);    // 添加电机, 返回下标/-1
void SimpleMotorSet(int idx, int16_t value);               // 按类型限幅并存入 out
void SimpleMotorReadAll(void);                             // 轮询 CAN, 解码所有电机
void SimpleMotorSendAll(void);                             // 打包并发送所有电机控制帧

/* 扫描: 全通滤波监听, 识别总线上的反馈 ID */
void SimpleMotorScanBegin(void);
void SimpleMotorScanTask(void);
uint8_t SimpleMotorScanResult(uint16_t* out, uint8_t max);
void SimpleMotorReconfigFilters(void);  // 扫描后恢复各电机滤波器

#endif  // WIFI_GM6020_SIMPLE_MOTOR_H
