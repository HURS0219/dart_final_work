/*
 * dart_app.h — 在 control-2026 (STM32F407) 上运行 dart_fc 的制导/控制逻辑,
 *              并把最新状态暴露给 ST7735 可视化 UI。
 *
 * 数据全部来自 dart_fc 移植过来的 control/guidance/attitude/pid 模块:
 *   - 4 舵面角度     : control_out_t.servo_deg[4]
 *   - 制导状态机     : mode + guidance valid/started + 链路/视觉/失效标志
 *   - PID 参数       : dart_config.h 的 CFG_*_KP/KD (与 dart_fc 一致)
 *   - 加速度指令     : guidance_t.accel_x / accel_y
 *   - IMU 姿态       : dart_attitude_t (由板载 BMI088/INS 提供, 无则仿真)
 *   - 视觉           : 目标框 x/y/w/h + online (真图像流接口见 st7735_ui)
 */
#ifndef DART_APP_H
#define DART_APP_H

#include <stdint.h>
#include "dart_attitude.h"
#include "dart_guidance.h"
#include "dart_control.h"

typedef struct {
    /* 舵面 */
    float servo_deg[4];              /* 当前逻辑偏角 (deg) */
    float servo_raw[4];              /* 当前机械角 (deg) */
    int   surface_state;             /* surface_state_t */
    int   surface_failsafe;
    float mix_pitch, mix_yaw, mix_roll;

    /* 制导 / 状态机 */
    int   mode;          /* dart_mode_t */
    int   armed;
    int   link_ok;
    int   vision_ok;
    int   failsafe;
    int   guid_valid;
    int   guid_started;
    float lambda_x, lambda_y;        /* 视线角 (rad) */
    float lambda_dot_x, lambda_dot_y;/* 视线角速率 (rad/s) */

    /* 加速度指令 */
    float accel_x, accel_y;          /* m/s^2 */

    /* IMU */
    dart_attitude_t att;
    float accel_g[3];                /* 机体加速度 (g) */

    /* 视觉目标框 (像素) */
    int   target_x, target_y, target_w, target_h;
    int   vision_online;
} dart_state_t;

void DartAppInit(void);
void DartAppTask(void);                  /* 建议 >=500Hz 调用 */
const dart_state_t *DartAppGetState(void);

/* 注入视觉目标 (真图像流): x,y 为像素坐标, w/h 为目标框尺寸。
 * 注入后 CFG_VISION_TIMEOUT_MS 内视为在线, 超时回落到内置演示目标。 */
void DartAppInjectVision(int x, int y, int w, int h);

#endif /* DART_APP_H */
