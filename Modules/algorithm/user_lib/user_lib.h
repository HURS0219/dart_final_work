/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "arm_math.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdint.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// ???????,????q31??f32,???????
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32
void MatInit(mat* m, uint8_t row, uint8_t col);

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
  do {                           \
    if ((val) <= (min)) {        \
      (val) = (min);             \
    } else if ((val) >= (max)) { \
      (val) = (max);             \
    }                            \
  } while (0)

#define ANGLE_LIMIT_360(val, angle) \
  do {                              \
    (val) = (angle) - (int)(angle); \
    (val) += (int)(angle) % 360;    \
  } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
  do {                              \
    if ((val) > 180) (val) -= 360;  \
  } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct {
  // --- 状态变量 ---
  float planning_v;  // 规划速度
  float expected_a;  // 期望加速度
  // --- 参数配置 ---
  float max_v;             // 物理最大速度 (如 3.0 m/s)
  float max_accel;         // 最大转矩区加速度 (如 5.0 m/s^2)
  float min_accel;         // 加速度下限 (如 0.05 m/s^2)，防止高转速下无法收敛
  float accel_base_speed;  // 基速 (转折点速度) (如 1.0 m/s)
  // 意味着 0~1m/s 期间你可以满加速度，超过 1m/s 后加速度开始按 1/v 衰减

  float max_decel;         // 低速刹车加速度 (如 4.5 m/s^2)
  float min_decel;         // 高速刹车加速度下限 (如 1.0 m/s^2)
  float decel_base_speed;  // 减速衰减转折速度，低于此速度用 max_decel，高于后按 1/v 衰减
  float k_p_vel;           // 速度闭环比例系数 (实际速度落后时的补偿)
} Ramp_Controller_t;

/**
 * @brief ??????????,??????????????????
 *
 * @param size ????
 * @return void*
 */
void* zmalloc(size_t size);

// ???????
float Sqrt(float x);
// ????????
float abs_limit(float num, float Limit);
// ???????
float sign(float value);
// ????????
float float_deadband(float Value, float minValue, float maxValue);
// ???????
float float_constrain(float Value, float minValue, float maxValue);
// ???????
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// ??????????
float loop_float_constrain(float Input, float minValue, float maxValue);
// ??? ????? 180 ~ -180
float theta_format(float Ang);

// 计算当前角度与目标角度的最小差值（[-180, 180]），返回格式化后的当前角度
float wrap180(float angle_deg);

int float_rounding(float raw);

float soft_limit(float x, float lim);

void slope_following(float target, float* set, float acc_d);

float ramp_controller_update(Ramp_Controller_t* ramp, float input_v, float actual_v, float dt);

float* Norm3d(float* v);

float NormOf3d(float* v);

void Cross3d(float* v1, float* v2, float* res);

float Dot3d(float* v1, float* v2);

float AverageFilter(float new_data, float* buf, uint8_t len);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
