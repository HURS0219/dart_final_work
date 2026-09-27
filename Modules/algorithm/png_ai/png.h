/**
 ******************************************************************************
 * @file    png.h
 * @author  ai (png_ai)
 * @version V1.0.0
 * @date    2026/09/27
 * @brief   视线比例引导(Proportional Navigation Guidance, PNG)算法对外接口
 ******************************************************************************
 * @attention
 *  - 本模块只实现“制导律”，即由视线角速率 d_lambda 解算横向加速度指令 a_cmd；
 *  - 视线角速率 d_lambda 由调用方(视觉/卡尔曼滤波)直接给出，模块内部不做微分、
 *    不做滤波，对应需求“只接受 d_lambda”；
 *  - 单位统一为国际单位制(SI)：角度 rad，角速率 rad/s，距离 m，速度 m/s，
 *    加速度 m/s^2，导航常数 N 无量纲；
 *  - 本模块不依赖任何 HAL/BSP，仅依赖标准库，便于主机端自测与后续接入 app。
 *
 *  控制律(与需求“输出 a_cmd = n*v*d_lambda”一致)：
 *    PPN(纯比例导引): a_cmd = N * v * d_lambda
 *    APN(增广比例导引): a_cmd = N * v * d_lambda + a_o_gain * N * a_o
 *  其中 v 由 vel_src 选择为接近速度 v_c 或导弹惯性系速度 v_m。
 ******************************************************************************
 */
#ifndef _PNG_H
#define _PNG_H

#include "png_cfg.h"
#include "stdint.h"
#include "math.h"

/* ============================================================================
 * 引导模式
 * -------------------------------------------------------------------------- */
typedef enum
{
    PNG_MODE_PPN = 0, // 纯比例导引: 末端静态靶, 实现最简单
    PNG_MODE_APN      // 增广比例导引: 末端横向随机移动靶, 叠加目标加速度前馈
} PNG_Mode_e;

/* ============================================================================
 * 比例项使用的速度来源
 * -------------------------------------------------------------------------- */
typedef enum
{
    PNG_VEL_USE_VC = 0, // 使用接近速度 v_c (纯比例导引的经典形式)
    PNG_VEL_USE_VM      // 使用导弹惯性系速度 v_m
} PNG_VelSrc_e;

/* ============================================================================
 * 默认引导模式 / 速度来源 (枚举依赖, 故定义于本头文件而非 png_cfg.h)
 * -------------------------------------------------------------------------- */
#define PNG_DEFAULT_MODE        (PNG_MODE_PPN)    // 默认: 纯比例导引
#define PNG_DEFAULT_VEL_SRC     (PNG_VEL_USE_VC)  // 默认: 用接近速度 v_c

/* ============================================================================
 * 初始化配置结构体
 *   注意: 其字段顺序必须与 PNGInstance 的“配置块”严格一致(PNGInit 用 memcpy)。
 * -------------------------------------------------------------------------- */
typedef struct
{
    PNG_Mode_e mode;     // 引导模式 PPN / APN
    PNG_VelSrc_e vel_src;// 比例项速度来源 v_c / v_m
    float N;             // 导航常数 n (无量纲)
    float MaxOut;        // 横向加速度输出限幅 (m/s^2)
    float DeadBand;      // 输出死区 (m/s^2)
    float a_o_gain;      // APN 目标加速度项系数 (经典取 0.5)
} PNG_Init_Config_s;

/* ============================================================================
 * 每周期输入结构体
 *   对应需求列出的全部输入; 当前制导律未用到的项可随意填 0, 仅作接口预留。
 * -------------------------------------------------------------------------- */
typedef struct
{
    float d_lambda; // 视线角速率 [rad/s]         (核心输入)
    float r;        // 弹目相对距离 [m]           (预留, 当前不使用)
    float v_m;      // 导弹惯性系速度 [m/s]        (当 vel_src=VM 时使用)
    float v_c;      // 导弹接近速度 [m/s]          (当 vel_src=VC 时使用)
    float v_t;      // 目标速度在视线上的分量 [m/s](预留, 当前不使用)
    float a_o;      // 目标加速度 [m/s^2]           (APN 使用, 来自卡尔曼或人工值)
} PNG_Input_s;

/* ============================================================================
 * PNG 实例
 *   结构划分与 controller.h 的 PIDInstance 一致:
 *   前面是“配置块”, 后面是“运行时状态”, 便于调试器直接观察。
 * -------------------------------------------------------------------------- */
typedef struct
{
    /* ----------------------------- init config block ---------------------- */
    PNG_Mode_e mode;      // 引导模式
    PNG_VelSrc_e vel_src; // 比例项速度来源
    float N;              // 导航常数
    float MaxOut;         // 输出限幅
    float DeadBand;       // 输出死区
    float a_o_gain;       // 目标加速度项系数

    /* ----------------------------- runtime state -------------------------- */
    float d_lambda;  // 本周期视线角速率输入
    float v_eff;     // 本周期参与比例项的实际速度
    float a_prop;    // 比例项 N*v*d_lambda
    float a_aug;     // 增广项 a_o_gain*N*a_o
    float a_cmd_raw; // 限幅/死区前的原始指令
    float a_cmd;     // 最终输出指令 (横向加速度, m/s^2)
} PNGInstance;

/* ============================================================================
 * 对外接口
 * -------------------------------------------------------------------------- */

/**
 * @brief 初始化 PNG 实例
 * @param png    PNG 实例指针
 * @param config 初始化配置 (为 NULL 时全部采用 png_cfg.h 默认值)
 * @note  仿照 PIDInit: 先整体清零, 再拷贝配置块; 对未填写(N<=0 等)的关键项
 *        回退到 png_cfg.h 的默认值, 避免误用导致指令恒为 0。
 */
void PNGInit(PNGInstance *png, PNG_Init_Config_s *config);

/**
 * @brief 清除运行时状态(保留配置), 用于重新制导/切换目标
 * @param png PNG 实例指针
 */
void PNGClear(PNGInstance *png);

/**
 * @brief 执行一次制导解算
 * @param png PNG 实例指针
 * @param in  本周期输入(见 PNG_Input_s)
 * @return float 横向加速度指令 a_cmd [m/s^2]
 */
float PNGCalculate(PNGInstance *png, const PNG_Input_s *in);

/**
 * @brief 在线修改导航常数
 * @param png PNG 实例指针
 * @param N   新的导航常数
 */
void PNGSetNavConstant(PNGInstance *png, float N);

/**
 * @brief 在线切换引导模式(PPN/APN)
 * @param png  PNG 实例指针
 * @param mode 目标模式
 */
void PNGSetMode(PNGInstance *png, PNG_Mode_e mode);

/**
 * @brief 读取最近一次输出的指令
 * @param png PNG 实例指针
 * @return float 最近一次的 a_cmd [m/s^2]
 */
float PNGGetCommand(PNGInstance *png);

#endif /* _PNG_H */
