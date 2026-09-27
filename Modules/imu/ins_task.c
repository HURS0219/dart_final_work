/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"

#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "controller.h"
#include "general_def.h"
#include "spi.h"
#include "tim.h"
#include "user_lib.h"

static INS_t INS;
static IMU_Init_Config_s IMU_Param;
// BMI088Instance* BMI;
static PIDInstance TempCtrl = {0};
static osThreadId insTaskHandle;

// body2earth
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 40;  // 恒温设定温度

static void IMU_Param_Correction(IMU_Init_Config_s *param, float gyro[3], float accel[3]);

static void IMUPWMSet(uint16_t pwm) {
#ifdef STM32F407xx
  __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
#elifdef STM32H7
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
#endif
}
/**
 * @brief 温度控制
 *
 */
static void IMU_Temperature_Ctrl(void) {
  PIDCalculate(&TempCtrl, BMI088.Temperature, 40);
  IMUPWMSet(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4) {
  float acc_init[3] = {0};
  float gravity_norm[3] = {0, 0, 1};  // 导航系重力加速度矢量,归一化后为(0,0,1)
  float axis_rot[3] = {0};            // 旋转轴
  // 读取100次加速度计数据,取平均值作为初始值
  for (uint8_t i = 0; i < 100; ++i) {
    BMI088_Read(&BMI088);
    IMU_Param_Correction(&IMU_Param, BMI088.Gyro, BMI088.Accel);
    acc_init[X] += BMI088.Accel[X];  // X轴不变
    acc_init[Y] += BMI088.Accel[Y];  // Y轴取反
    acc_init[Z] += BMI088.Accel[Z];  // Z轴取反
    DWT_Delay(0.001);
  }
  for (uint8_t i = 0; i < 3; ++i) acc_init[i] /= 100;
  Norm3d(acc_init);
  // 计算原始加速度矢量和导航系重力加速度矢量的夹角
  float angle = acosf(Dot3d(acc_init, gravity_norm));
  Cross3d(acc_init, gravity_norm, axis_rot);
  Norm3d(axis_rot);
  init_q4[0] = cosf(angle / 2.0f);
  for (uint8_t i = 0; i < 2; ++i) init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f);  // 轴角公式,第三轴为0(没有z轴分量)
}

__attribute__((noreturn)) void StartINSTASK(void const *argument) {
  static float ins_start;
  static float ins_dt;
  LOGINFO("[freeRTOS] INS Task Start");
  for (;;) {
    // 1kHz
    ins_start = DWT_GetTimeline_ms();
    INS_Task();
    ins_dt = DWT_GetTimeline_ms() - ins_start;
    if (ins_dt > 1) LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
    // VisionSend();  // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
    osDelay(1);
  }
}

/**
 * @brief 【新增】IMU杆臂补偿 (消除高速旋转时的向心加速度)
 * @param accel 当前三轴加速度 (m/s^2) [将被就地修改]
 * @param gyro  当前三轴角速度 (rad/s)
 * @param offset IMU中心到旋转中心的偏移量 (m)
 */
static void IMU_LeverArm_Compensation(float accel[3], float gyro[3], float offset[3]) {
  // 如果没有设置偏移量，则不消耗算力
  if (offset[X] == 0.0f && offset[Y] == 0.0f && offset[Z] == 0.0f) {
    return;
  }

  // 1. 计算叉乘: V = w x r (角速度 叉乘 偏置向量 -> 得到线速度)
  float cross_wx_r[3];
  cross_wx_r[X] = gyro[Y] * offset[Z] - gyro[Z] * offset[Y];
  cross_wx_r[Y] = gyro[Z] * offset[X] - gyro[X] * offset[Z];
  cross_wx_r[Z] = gyro[X] * offset[Y] - gyro[Y] * offset[X];

  // 2. 计算向心加速度: a_c = w x (w x r) = w x V
  float a_c[3];
  a_c[X] = gyro[Y] * cross_wx_r[Z] - gyro[Z] * cross_wx_r[Y];
  a_c[Y] = gyro[Z] * cross_wx_r[X] - gyro[X] * cross_wx_r[Z];
  a_c[Z] = gyro[X] * cross_wx_r[Y] - gyro[Y] * cross_wx_r[X];

  // 3. 从原始加速度中剔除向心加速度
  // (注意：此处的accel和gyro必须已经是对齐了机体坐标系的数据)
  accel[X] -= a_c[X];
  accel[Y] -= a_c[Y];
  accel[Z] -= a_c[Z];
}

/**
 * @brief 调试用陀螺仪校准函数，用于测量陀螺仪零偏值
 * @param sample_count 采样次数
 * @return void
 */
static void INS_CalibrateGyroForDebug(uint16_t sample_count) {
  float gyro_sum[3] = {0.0f, 0.0f, 0.0f};

  // 重置陀螺仪偏差值
  for (uint8_t i = 0; i < 3; i++) {
    BMI088.GyroOffset[i] = 0.0f;
  }

  // 采集指定次数的数据
  for (uint16_t i = 0; i < sample_count; i++) {

    do {
      BMI088_Read(&BMI088);
      IMU_Temperature_Ctrl();
      DWT_Delay(0.001);
    } while (BMI088.Temperature <= 39.0f || BMI088.Temperature >= 41.0f);

    // 累加陀螺仪读数
    for (uint8_t j = 0; j < 3; j++) {
      gyro_sum[j] += BMI088.Gyro[j];
    }
    DWT_Delay(0.001);  // 1ms延时
  }

  // 计算平均值作为零偏
  for (uint8_t i = 0; i < 3; i++) {
    BMI088.GyroOffset[i] = gyro_sum[i] / sample_count;
  }
}

INS_t *INS_Init(IMU_Init_Config_s *imu_init_config) {
  if (!INS.init)
    INS.init = 1;
  else
    return &INS;

#ifdef STM32F407xx
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
#elifdef STM32H7
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
#endif
#ifdef STM32F407xx
  while (BMI088Init(&hspi1, 0) != BMI088_NO_ERROR);
#elifdef STM32H7
  while (BMI088Init(&hspi2, 0) != BMI088_NO_ERROR);
#endif
  // 使用我们的调试校准函数来测量陀螺仪零偏值，绕过预定义值

  // imu heat init
  PID_Init_Config_s config = {.MaxOut = 2000,
                              .IntegralLimit = 300,
                              .DeadBand = 0,
                              .Kp = 1000,
                              .Ki = 20,
                              .Kd = 0,
                              .Improve = 0x01};  // enable integratiaon limit
  PIDInit(&TempCtrl, &config);

  for (int i=0;i<1000;i++) {
    BMI088_Read(&BMI088);
    IMU_Temperature_Ctrl();
    DWT_Delay(0.001);
  }
  //是否在线标定
  if (imu_init_config->offset_flag==1) {
    for (uint8_t i=0;i<3;i++)
      BMI088.GyroOffset[i]=imu_init_config->GyroOffset[i];
  }
  else {
    INS_CalibrateGyroForDebug(5000);
  }

  // 手动计算加速度缩放因子，因为我们跳过了完整的校准过程
  BMI088.AccelScale = 9.81f / BMI088.gNorm;
  IMU_Param.scale[X] = imu_init_config->scale[X];
  IMU_Param.scale[Y] = imu_init_config->scale[Y];
  IMU_Param.scale[Z] = imu_init_config->scale[Z];
  IMU_Param.Yaw = imu_init_config->Yaw;
  IMU_Param.Pitch = imu_init_config->Pitch;
  IMU_Param.Roll = imu_init_config->Roll;
  IMU_Param.flag = imu_init_config->flag;
  // ==================== 【新增】 ====================
  IMU_Param.CenterOffset[X] = imu_init_config->CenterOffset[X];
  IMU_Param.CenterOffset[Y] = imu_init_config->CenterOffset[Y];
  IMU_Param.CenterOffset[Z] = imu_init_config->CenterOffset[Z];

  // BMI088CalibrateGyroForDebug(BMI,1000);
  float init_quaternion[4] = {0};
  InitQuaternion(init_quaternion);
  // 改进的初始化方式：使用更稳定的四元数初始化
  // float init_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // 单位四元数
  IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001f, 10000000, 0.9996f,
                         0.0085f);  // 增加测量噪声，启用渐消因子和低通滤波
  // noise of accel is relatively big and of high freq,thus lpf is used
  INS.AccelLPF = 0.0085;
  DWT_GetDeltaT(&INS_DWT_Count);
  // 创建INS任务
  osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
  insTaskHandle = osThreadCreate(osThread(instask), NULL);
  return &INS;  // @todo: 这里偷懒了,不要这样做! 修改INT_t结构体可能会导致异常,待修复.
}

/* 注意以1kHz的频率运行此任务 */
void INS_Task(void) {
  static uint32_t count = 0;
  const float gravity[3] = {0, 0, 9.81f};  // 使用本地重力加速度值

  dt = DWT_GetDeltaT(&INS_DWT_Count);
  t += dt;

  // ins update
  if ((count % 1) == 0) {
    BMI088_Read(&BMI088);

    INS.Accel[X] = BMI088.Accel[X];
    INS.Accel[Y] = BMI088.Accel[Y];
    INS.Accel[Z] = BMI088.Accel[Z];
    INS.Gyro[X] = BMI088.Gyro[X];
    INS.Gyro[Y] = BMI088.Gyro[Y];
    INS.Gyro[Z] = BMI088.Gyro[Z];

    // 修正安装误差
    IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
    // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;
    // ==================== 【新增】 ====================
    // 扣除小陀螺时的向心加速度，防止姿态解算发飘导致平移
    // 必须放在 IMU_Param_Correction 之后，EKF_Update 之前
    IMU_LeverArm_Compensation(INS.Accel, INS.Gyro, IMU_Param.CenterOffset);
    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; ++i)  // 同样过一个低通滤波
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
                             INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);  // 转换回导航系n

    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    // VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);
  }

  // temperature control
  if ((count % 2) == 0) {
    // 500hz
    IMU_Temperature_Ctrl();
  }

  if ((count++ % 1000) == 0) {
    // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
  }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
  vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                     (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

  vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                     (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

  vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
  vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                     (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

  vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                     (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

  vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Init_Config_s *param, float gyro[3], float accel[3]) {
  static float lastYawOffset, lastPitchOffset, lastRollOffset;
  static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

  if (fabsf(param->Yaw - lastYawOffset) > 0.001f || fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
      fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
    cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
    cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
    cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
    sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
    sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
    sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

    // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
    c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
    c_12 = cosPitch * sinYaw;
    c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
    c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
    c_22 = cosYaw * cosPitch;
    c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
    c_31 = -cosPitch * sinRoll;
    c_32 = sinPitch;
    c_33 = cosPitch * cosRoll;
    param->flag = 0;
  }
  float gyro_temp[3];
  for (uint8_t i = 0; i < 3; ++i) gyro_temp[i] = gyro[i] * param->scale[i];

  gyro[X] = c_11 * gyro_temp[X] + c_12 * gyro_temp[Y] + c_13 * gyro_temp[Z];
  gyro[Y] = c_21 * gyro_temp[X] + c_22 * gyro_temp[Y] + c_23 * gyro_temp[Z];
  gyro[Z] = c_31 * gyro_temp[X] + c_32 * gyro_temp[Y] + c_33 * gyro_temp[Z];

  float accel_temp[3];
  for (uint8_t i = 0; i < 3; ++i) accel_temp[i] = accel[i] * param->scale[i];

  accel[X] = c_11 * accel_temp[X] + c_12 * accel_temp[Y] + c_13 * accel_temp[Z];
  accel[Y] = c_21 * accel_temp[X] + c_22 * accel_temp[Y] + c_23 * accel_temp[Z];
  accel[Z] = c_31 * accel_temp[X] + c_32 * accel_temp[Y] + c_33 * accel_temp[Z];

  lastYawOffset = param->Yaw;
  lastPitchOffset = param->Pitch;
  lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this
// demo-------------------------------------------------
//----------------------------------you can read them for learning or
// programming-----------------------------------------------
//----------------------------------they could also be helpful for further
// design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt) {
  float qa, qb, qc;

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll) {
  *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
  *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
  *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q) {
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
  Yaw /= 57.295779513f;
  Pitch /= 57.295779513f;
  Roll /= 57.295779513f;
  cosPitch = arm_cos_f32(Pitch / 2);
  cosYaw = arm_cos_f32(Yaw / 2);
  cosRoll = arm_cos_f32(Roll / 2);
  sinPitch = arm_sin_f32(Pitch / 2);
  sinYaw = arm_sin_f32(Yaw / 2);
  sinRoll = arm_sin_f32(Roll / 2);
  q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
  q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
  q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
  q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

uint8_t INS_GetAttitude(attitude_t *attitude) {
  if (attitude == NULL || !INS.init) {
    return 0;
  }

  // 复制姿态数据
  attitude->Yaw = INS.Yaw;
  attitude->Pitch = INS.Pitch;
  attitude->Roll = INS.Roll;
  attitude->YawTotalAngle = INS.YawTotalAngle;
  // 如果需要角速度数据也可以复制
  attitude->Gyro[0] = INS.Gyro[0];
  attitude->Gyro[1] = INS.Gyro[1];
  attitude->Gyro[2] = INS.Gyro[2];

  return 1;
}