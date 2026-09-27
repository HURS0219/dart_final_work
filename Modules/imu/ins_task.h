/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @author  annotation and modification by NeoZeng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention INS任务的初始化不要放入实时系统!应该由application拥有实例,随后在
 *            应用层调用初始化函数.
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "BMI088driver.h"
#include "QuaternionEKF.h"
#include "stdint.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

typedef struct {
  float Gyro[3];   // 三个轴的角速度数据 [0]-Pitch方向 [1]-Roll方向 [2]-Yaw方向 单位: rad/s
  float Accel[3];  // 三个轴的加速度数据 [0]-X方向 [1]-Y方向 [2]-Z方向 单位: m/s^2
  // float Ang_accel[3]; // 加角速度数据

  float Roll;           // 横滚角(绕X轴旋转) 单位: °
  float Pitch;          // 俯仰角(绕Y轴旋转) 单位: °
  float Yaw;            // 偏航角(绕Z轴旋转) 单位: °
  float YawTotalAngle;  // Yaw轴累计转过的总角度，可用于多圈控制 单位: °
} attitude_t;  // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)。姿态信息包括三轴角速度、三轴加速度和欧拉角姿态

typedef struct {
  float q[4];  // 四元数估计值 [0]-实部 [1~3]-虚部(i,j,k)

  float MotionAccel_b[3];  // 机体坐标系下加速度 [0]-X方向 [1]-Y方向 [2]-Z方向 单位: m/s^2
  float MotionAccel_n[3];  // 导航坐标系(绝对系)下的加速度 [0]-X方向 [1]-Y方向 [2]-Z方向 单位: m/s^2

  float AccelLPF;  // 加速度低通滤波系数，用于滤除高频噪声

  // bodyframe在绝对系的向量表示（就是机体坐标系各轴基向量在导航坐标系中的表示）
  float xn[3];  // 机体坐标系X轴在导航坐标系中的方向余弦
  float yn[3];  // 机体坐标系Y轴在导航坐标系中的方向余弦
  float zn[3];  // 机体坐标系Z轴在导航坐标系中的方向余弦

  // 加速度在机体系和XY两轴的夹角
  // float atanxz;
  // float atanyz;

  // IMU原始测量值
  float Gyro[3];   // 陀螺仪原始测量角速度 [0]-Pitch方向 [1]-Roll方向 [2]-Yaw方向 单位:  rad/s
  float Accel[3];  // 加速度计原始测量加速度 [0]-X方向 [1]-Y方向 [2]-Z方向 单位: m/s^2

  // 姿态解算结果
  float Roll;           // 解算得到的横滚角 单位: °
  float Pitch;          // 解算得到的俯仰角 单位: °
  float Yaw;            // 解算得到的偏航角 单位: °
  float YawTotalAngle;  // Yaw轴累计转过的总角度 单位: °

  uint8_t init;  // 初始化标志位 0-未初始化 1-已初始化
} INS_t;

/**
 * @brief 用于修正IMU安装误差的参数（修改方法见ins的md）
 */
typedef struct {
  uint8_t flag;  // 参数更新标志位，当参数发生变化时置1
  uint8_t offset_flag; //是否采用离线标注参数，0-在线标定 1-离线标定

  float scale[3];  // 三轴标度因数修正系数 [0]-X轴 [1]-Y轴 [2]-Z轴

  float Yaw;    // Yaw轴安装偏角修正 单位: °
  float Pitch;  // Pitch轴安装偏角修正 单位: °
  float Roll;   // Roll轴安装偏角修正 单位: °
  float GyroOffset[3];
  // ==================== 【新增】 ====================
  // IMU中心相对于底盘旋转中心的物理偏移量 (单位: 米)
  // X: 前后偏移 (前正后负)
  // Y: 左右偏移 (左正右负 或 视你的坐标系而定)
  // Z: 上下偏移 (通常Z轴向心力影响较小，可填0)
  float CenterOffset[3];
  // =================================================
} IMU_Init_Config_s;

/**
 * @brief 初始化惯导解算系统，使用默认参数
 *
 */
INS_t *INS_Init(IMU_Init_Config_s *imu_init_config);

/**
 * @brief 此函数放入实时系统中,以1kHz频率运行
 *        p.s. osDelay(1);
 *
 */
void INS_Task(void);

/**
 * @brief INS任务主函数
 *
 */
void StartINSTASK(void const *argument);

uint8_t INS_GetAttitude(attitude_t *attitude);

/**
 * @brief 四元数更新函数,即实现dq/dt=0.5Ωq
 *
 * @param q  四元数
 * @param gx
 * @param gy
 * @param gz
 * @param dt 距离上次调用的时间间隔
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);

/**
 * @brief 四元数转换成欧拉角 ZYX
 *
 * @param q
 * @param Yaw
 * @param Pitch
 * @param Roll
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);

/**
 * @brief ZYX欧拉角转换为四元数
 *
 * @param Yaw
 * @param Pitch
 * @param Roll
 * @param q
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);

/**
 * @brief 机体系到惯性系的变换函数
 *
 * @param vecBF body frame
 * @param vecEF earth frame
 * @param q
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

/**
 * @brief 惯性系转换到机体系
 *
 * @param vecEF
 * @param vecBF
 * @param q
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

/**
 * @brief 【新增】IMU杆臂补偿 (消除高速旋转时的向心加速度)
 * @param accel 当前三轴加速度 (m/s^2) [将被就地修改]
 * @param gyro  当前三轴角速度 (rad/s)
 * @param offset IMU中心到旋转中心的偏移量 (m)
 */
// static void IMU_LeverArm_Compensation(float accel[3], float gyro[3], float offset[3]);

#endif
