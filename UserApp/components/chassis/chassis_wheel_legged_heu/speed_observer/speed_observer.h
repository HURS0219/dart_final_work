/**
 ******************************************************************************
 * @file    speed_observer.h
 * @author  Enhao
 * @date    2025/11/9
 * @brief   None
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#pragma once
#include "kalman_filter.h"
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel);