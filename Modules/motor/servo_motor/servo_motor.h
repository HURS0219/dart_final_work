/**
 * @file servo_motor.h
 * @author ai (extracted & merged from UserApp/robot/dart_servo_v0.2)
 * @brief 舵机通用模块: PWM 舵机(PTK7350 / SG90 等) + 串口总线舵机
 * @version 1.0
 * @date 2026-09-28
 *
 * @attention
 *  1. PWM 舵机: 50Hz(周期 20ms), 高电平脉宽 pulse_min_us ~ pulse_max_us 线性对应
 *     机械角 0 ~ range_deg。逻辑角 <-> 机械角 的换算包含:
 *         center_deg(逻辑 0 对应机械角) / trim_deg(微调) / scale(比例) / reverse(方向)
 *     并可用 ServoTask() 做速率限幅。
 *  2. 总线舵机: 串口行协议(0x55 0x55 ...), 支持角度设定与位置回读。
 *  3. 本模块只做“驱动 + 标定 + 速率限幅”; 状态机(待机/手动/混控/自检)等控制逻辑
 *     由 app 层实现, 以保持 module 层通用。
 *
 *  典型 PTK7350: 50Hz, 500~2500us; 180° 型 range_deg=180, 270° 型 range_deg=270(实测约279)。
 */
#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "stdint.h"

#define SERVO_MOTOR_CNT 7  // 最多注册的舵机实例数

/* ------------------------------- 总线舵机协议 ------------------------------- */
#define Servo_Frame_First 0x55    // 帧头1
#define Servo_Frame_Second 0x55   // 帧头2
#define Servo_MAX_BUFF 10         // 单包最大长度
#define SERVO_MOVE_CMD 0x03       // 角度控制命令
#define SERVO_UNLOAD_CMD 0x14     // 失力命令
#define SERVO_POS_READ_CMD 0x15   // 读取位置命令

/* 舵机类型 */
typedef enum {
  Servo_None_Type = 0,
  Bus_Servo = 1,  // 串口总线舵机
  PWM_Servo = 2,  // PWM 舵机
} ServoType_e;

/**
 * @brief PWM 舵机标定参数(脉宽 <-> 机械角 线性; 逻辑角含中性/微调/比例/方向)
 *        脉宽 pulse_min_us 对应机械 0°, pulse_max_us 对应机械 range_deg°。
 *        逻辑角 = (机械角 - center_deg - trim_deg) / (reverse * scale)
 */
typedef struct {
  float pulse_min_us;    // 机械 0° 对应脉宽 (如 500)
  float pulse_max_us;    // 机械 range_deg 对应脉宽 (如 2500)
  float range_deg;       // 机械量程 (180 / 270 / 279 ...)
  float center_deg;      // 逻辑 0° 对应的机械角 (默认 range_deg/2)
  float trim_deg;        // 中性微调
  float scale;           // 比例 (默认 1)
  int8_t reverse;        // 方向: +1 / -1 (默认 +1)
  float rate_limit_dps;  // 速率限幅 deg/s (<=0 表示不限速, 立即到位)
} Servo_Calib_Config_s;

/* 用于初始化不同舵机的结构体, 各类舵机通用 */
typedef struct {
  ServoType_e servo_type;             // 舵机类型
  uint8_t servo_id;                   // 舵机编号
  UART_HandleTypeDef *_handle;        // Bus_Servo 使用的串口
  PWM_Init_Config_s pwm_init_config;  // PWM_Servo 使用的 PWM 配置(周期建议 0.02s)
  Servo_Calib_Config_s calib;         // PWM_Servo 使用的标定参数
} Servo_Init_Config_s;

/* 舵机实例 */
typedef struct {
  uint8_t servo_id;  // 舵机编号
  ServoType_e servo_type;
  PWMInstance *pwm_instance;      // PWM 舵机底层实例
  USARTInstance *usart_instance;  // 总线舵机的串口实例

  /* --- PWM 舵机 --- */
  Servo_Calib_Config_s calib;  // 运行时标定(可在线修改)
  float target_deg;            // 目标逻辑角
  float angle;                 // 当前逻辑角(速率限幅后)
  float raw_deg;               // 当前机械角
  float pulse_us;              // 当前脉宽
  float last_time_s;           // 上次 ServoTask 的时间戳

  /* --- 总线舵机 --- */
  uint16_t recv_angle;  // 回读的角度
} ServoInstance;

/**
 * @brief 注册一个舵机
 * @param config 初始化配置(见 Servo_Init_Config_s)
 * @return ServoInstance* 成功返回实例, 失败返回 NULL
 */
ServoInstance *ServoInit(Servo_Init_Config_s *config);

/**
 * @brief 设置逻辑角(度)
 *        PWM: 设置目标角; 若未启用速率限幅则立即输出, 否则由 ServoTask() 平滑推进
 *        Bus: 直接下发角度命令
 */
void ServoSetAngle(ServoInstance *servo, float angle);

/**
 * @brief 直接设置脉宽(us), 自动按 pulse_min/max 限幅(绕过逻辑角映射, 供标定/测试用)
 */
void ServoSetPulseUs(ServoInstance *servo, float pulse_us);

/**
 * @brief 舵机周期任务: 对所有启用速率限幅的 PWM 舵机, 把 angle 平滑逼近 target 并输出
 *        建议放入 app 的周期任务(如 100Hz~1kHz)
 */
void ServoTask(void);

/**
 * @brief (总线舵机) 失力: 舵机不再保持力矩
 */
void ServoUnload(ServoInstance *servo);

/**
 * @brief (总线舵机) 请求位置回读; 收到应答后由 ServoGetRecvAngle() 读取
 */
void ServoRequestAngle(ServoInstance *servo);

/* ------------------------------- 标定接口 ------------------------------- */
void ServoSetNeutral(ServoInstance *servo, float center_deg);  // 设置逻辑 0° 对应机械角
void ServoSetTrim(ServoInstance *servo, float trim_deg);       // 微调
void ServoSetScale(ServoInstance *servo, float scale);         // 比例
void ServoSetReverse(ServoInstance *servo, int8_t reverse);    // 方向
void ServoSetRateLimit(ServoInstance *servo, float dps);       // 速率限幅
void ServoZero(ServoInstance *servo);                          // 当前位置记为逻辑 0°
void ServoResetCal(ServoInstance *servo);                      // 清除标定(回默认)

/* ------------------------------- 使能/失能 ------------------------------- */
void ServoEnable(ServoInstance *servo);   // PWM: 启动输出
void ServoDisable(ServoInstance *servo);  // PWM: 停止输出(失去保持力)

/* ------------------------------- 查询 ------------------------------- */
float ServoGetAngle(ServoInstance *servo);      // 逻辑角
float ServoGetRawDeg(ServoInstance *servo);     // 机械角
float ServoGetPulseUs(ServoInstance *servo);    // 脉宽
uint16_t ServoGetRecvAngle(ServoInstance *servo);  // 总线舵机回读角

#endif  // !SERVO_MOTOR_H
