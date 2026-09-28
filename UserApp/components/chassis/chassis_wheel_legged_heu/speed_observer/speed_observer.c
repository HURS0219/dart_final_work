/**
******************************************************************************
* @file    speed_observer.c
* @author  Enhao
* @date    2025/11/9
* @brief   None
******************************************************************************
* @attention
* None
*
******************************************************************************
*/

#include "speed_observer.h"

#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "kalman_filter.h"

KalmanFilter_t vaEstimateKF;  // 卡尔曼滤波器结构体

static float vaEstimateKF_F[4] = {1.0f, 0.003f, 0.0f, 1.0f};  // 状态转移矩阵，控制周期为0.001s

static float vaEstimateKF_P[4] = {1.0f, 0.0f, 0.0f, 1.0f};  // 后验估计协方差初始值

static float vaEstimateKF_Q[4] = {0.5f, 0.0f, 0.0f, 0.5f};  // Q矩阵初始值

static float vaEstimateKF_R[4] = {100.0f, 0.0f, 0.0f, 100.0f};

static float vaEstimateKF_K[4];

static float vaEstimateKF_H[4] = {1.0f, 0.0f, 0.0f, 1.0f};  // 设置矩阵H为常量

#define OBSERVE_TIME 3  // 任务周期是3ms

void xvEstimateKF_Init(KalmanFilter_t* EstimateKF) {
  Kalman_Filter_Init(EstimateKF, 2, 0, 2);  // 状态向量2维 没有控制量 测量向量2维

  memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
  memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
  memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
  memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
  memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t* EstimateKF, float acc, float vel) {
  // 卡尔曼滤波器测量值更新
  EstimateKF->MeasuredVector[0] = vel;  // 测量速度
  EstimateKF->MeasuredVector[1] = acc;  // 测量加速度

  // 卡尔曼滤波器更新函数
  Kalman_Filter_Update(EstimateKF);
  // todo:更新频率需要能够控制
}