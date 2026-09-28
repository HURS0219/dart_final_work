/*
 * dart_cfg.h — 制导飞镖运行时参数 (网页可调, 掉电保存)
 *
 * 所有可调量集中于此: 4 路舵机标定 / 混控矩阵 / 控制 PID / 比例导引参数 / 失效保护。
 * 网页通过 DartCfgSetParam(id,val) 修改, 改动去抖后写内部 Flash (双 Bank + 校验)。
 */
#ifndef DART_FC_CFG_H
#define DART_FC_CFG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DART_CFG_SERVO_CNT 4

/* ---- 参数 ID (网页/串口共用, 顺序不可随意改) ---- */
enum {
    DART_P_SERVO_NEUTRAL = 0,   /* 0..3  舵面机械中立角 (deg) */
    DART_P_SERVO_TRIM = 4,      /* 4..7  中立微调 (deg) */
    DART_P_SERVO_MAX = 8,       /* 8..11 最大偏角 (deg) */
    DART_P_SERVO_REVERSE = 12,  /* 12..15 方向 (-1/1) */
    DART_P_SERVO_PULSE_MIN = 16,
    DART_P_SERVO_PULSE_MAX = 17,
    DART_P_SERVO_RANGE = 18,
    DART_P_SERVO_RATE = 19,     /* 舵面速率限幅 (deg/s) */
    DART_P_MIX = 20,            /* 20..31 混控矩阵 4x3 (行优先) */
    DART_P_ROLL_KP = 32,
    DART_P_ROLL_KI = 33,
    DART_P_ROLL_KD = 34,
    DART_P_PITCH_KP = 35,
    DART_P_PITCH_KI = 36,
    DART_P_PITCH_KD = 37,
    DART_P_YAW_KP = 38,
    DART_P_YAW_KI = 39,
    DART_P_YAW_KD = 40,
    DART_P_NAV_RATIO = 41,
    DART_P_V_CLOSE = 42,
    DART_P_PNG_MAX_G = 43,
    DART_P_FAILSAFE_MS = 44,
    DART_P_COUNT = 45
};

typedef struct {
    /* 舵机标定 */
    float servo_neutral_deg[DART_CFG_SERVO_CNT];
    float servo_trim_deg[DART_CFG_SERVO_CNT];
    float servo_max_deg[DART_CFG_SERVO_CNT];
    float servo_reverse[DART_CFG_SERVO_CNT];   /* -1 / 1 (float 便于参数表) */
    float servo_pulse_min_us;
    float servo_pulse_max_us;
    float servo_range_deg;
    float servo_rate_limit;                    /* deg/s */

    /* 混控矩阵 [舵面][pitch,yaw,roll] */
    float mix[DART_CFG_SERVO_CNT][3];

    /* 控制 PID */
    float roll_kp, roll_ki, roll_kd;
    float pitch_kp, pitch_ki, pitch_kd;
    float yaw_kp, yaw_ki, yaw_kd;

    /* 比例导引 */
    float nav_ratio;
    float v_close;
    float png_max_g;

    /* 失效保护 */
    uint32_t failsafe_ms;
} DartCfg_t;

void DartCfgInit(void);
DartCfg_t *DartCfg(void);

/* 版本号: 任何参数变化(含上电加载)都会自增, 供驱动层检测重配置 */
uint32_t DartCfgVersion(void);

/* 用当前 cfg 重新初始化混控矩阵等派生量 */
void DartCfgApply(void);

/* 恢复出厂默认 */
void DartCfgReset(void);

/* 参数读写 (越界返回 0/无操作) */
int DartCfgSetParam(int id, float value);
float DartCfgGetParam(int id);

/* 掉电保存 */
void DartCfgMarkDirty(void);
void DartCfgSaveNow(void);
void DartCfgTask(void);       /* 周期调用: 去抖后写 Flash */
int DartCfgTakeSaved(void);   /* 返回并清除"已保存"标志 */

#ifdef __cplusplus
}
#endif

#endif /* DART_FC_CFG_H */
