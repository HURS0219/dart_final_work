/**
 ******************************************************************************
 * @file    user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "user_lib.h"

#include "main.h"
#include "math.h"
#include "memory.h"
#include "stdlib.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

void* zmalloc(size_t size) {
  void* ptr = malloc(size);
  memset(ptr, 0, size);
  return ptr;
}

// 快速开方
float Sqrt(float x) {
  float y;
  float delta;
  float maxError;

  if (x <= 0) {
    return 0;
  }

  // initial guess
  y = x / 2;

  // refine
  maxError = x * 0.001f;

  do {
    delta = (y * y) - x;
    y -= delta / (2 * y);
  } while (delta > maxError || delta < -maxError);

  return y;
}

// 绝对值限制
float abs_limit(float num, float Limit) {
  if (num > Limit) {
    num = Limit;
  } else if (num < -Limit) {
    num = -Limit;
  }
  return num;
}

// 判断符号位
float sign(float value) {
  if (value >= 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0.0f;
  }
  return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue) {
  if (maxValue < minValue) {
    return Input;
  }

  if (Input > maxValue) {
    float len = maxValue - minValue;
    while (Input > maxValue) {
      Input -= len;
    }
  } else if (Input < minValue) {
    float len = maxValue - minValue;
    while (Input < minValue) {
      Input += len;
    }
  }
  return Input;
}

void slope_following(float target, float __packed* set, float acc_d) {
  if (target > *set) {
    *set = *set + acc_d;
    if (*set >= target) *set = target;
  } else if (target < *set) {
    *set = *set - acc_d;
    if (*set <= target) *set = target;
  }
}

/**
 * @brief 基于恒功率模型的动态斜坡更新
 */
float ramp_controller_update(Ramp_Controller_t* ramp, float input_v, float actual_v, float dt) {
  // 1. 输入限幅
  if (input_v > ramp->max_v) input_v = ramp->max_v;
  if (input_v < -ramp->max_v) input_v = -ramp->max_v;
  // 2. 判断加速还是减速
  uint8_t is_accelerating = 0;
  if ((input_v * ramp->planning_v >= 0) && (fabsf(input_v) > fabsf(ramp->planning_v))) {
    is_accelerating = 1;
  }
  // 3. 计算当前物理允许的最大加速度
  float current_limit;
  if (is_accelerating) {
    float abs_v = fabsf(ramp->planning_v);
    if (abs_v <= ramp->accel_base_speed) {
      // --- 恒转矩区 (Constant Torque Region) ---
      // 速度没起来之前，用最大能力加速
      current_limit = ramp->max_accel;
    } else {
      // --- 恒功率区 (Constant Power Region) ---
      // 速度起来了，加速度按 1/v 衰减
      // Acc_limit = P_const / v  =>  Acc_limit = (Max_Acc * Base_V) / Current_V
      current_limit = (ramp->max_accel * ramp->accel_base_speed) / abs_v;
      // 选填：为了防止加速度衰减得太狠（比如速度极高时加速度趋近0导致无法达到满速）
      // 可以设置一个最低下限
      if (current_limit < ramp->min_accel) current_limit = ramp->min_accel;
    }
  } else {
    // 减速区：高速时衰减刹车力度，防止轮腿前倾翻车
    float abs_v = fabsf(ramp->planning_v);
    if (abs_v <= ramp->decel_base_speed) {
      current_limit = ramp->max_decel;
    } else {
      current_limit = (ramp->max_decel * ramp->decel_base_speed) / abs_v;
      if (current_limit < ramp->min_decel) current_limit = ramp->min_decel;
    }
  }
  float max_step = current_limit * dt;
  // 4. 更新规划速度
  float err = input_v - ramp->planning_v;

  if (fabsf(err) > max_step) {
    float step = (err > 0) ? max_step : -max_step;
    ramp->planning_v += step;
    ramp->expected_a = step / dt;  // 这里的 expected_a 会自动画出一条漂亮的双曲线
  } else {
    ramp->expected_a = (input_v - ramp->planning_v) / dt;
    ramp->planning_v = input_v;
  }
  
  // 5. 增加实际速度静差前馈补偿
  float output_v = ramp->planning_v;
  if (ramp->k_p_vel > 0.0f) {
    float vel_err = ramp->planning_v - actual_v;
    output_v += ramp->k_p_vel * vel_err;
  }

  // 6. 输出限幅
  if (output_v > ramp->max_v) output_v = ramp->max_v;
  if (output_v < -ramp->max_v) output_v = -ramp->max_v;
  
  return output_v;
}

float soft_limit(float x, float lim) {
    if (fabsf(x) <= lim) return x;
    float sign = (x > 0) ? 1.0f : -1.0f;
    return sign * (lim + (fabsf(x) - lim) / (1.0f + (fabsf(x) - lim)));
}

// 角度格式化为-180~180
float theta_format(float Ang) { return loop_float_constrain(Ang, -180.0f, 180.0f); }

// 将角度限制在 [-180, 180] 范围内, O(1)时间复杂度, 效率高于 theta_format
float wrap180(float angle_deg) {
  float diff = fmodf(angle_deg, 360.0f);
  diff = diff - 360.0f * floorf(diff / 360.0f + 0.5f);
  return diff;
}

int float_rounding(float raw) {
  static int integer;
  static float decimal;
  integer = (int)raw;
  decimal = raw - integer;
  if (decimal > 0.5f) integer++;
  return integer;
}

// 三维向量归一化
float* Norm3d(float* v) {
  float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
  return v;
}

// 计算模长
float NormOf3d(float* v) { return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

// 三维向量叉乘v1 x v2
void Cross3d(float* v1, float* v2, float* res) {
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float* v1, float* v2) { return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]; }

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float* buf, uint8_t len) {
  float sum = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    buf[i] = buf[i + 1];
    sum += buf[i];
  }
  buf[len - 1] = new_data;
  sum += new_data;
  return sum / len;
}

void MatInit(mat* m, uint8_t row, uint8_t col) {
  m->numCols = col;
  m->numRows = row;
  m->pData = (float*)zmalloc(row * col * sizeof(float));
}
