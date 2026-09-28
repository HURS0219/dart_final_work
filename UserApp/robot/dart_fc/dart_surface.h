/*
 * dart_surface.h — 制导飞镖 4 路 X 型舵面 状态机 (重写版)
 *
 * 需求变化后重写: 原来把"模式"和"舵面输出"揉在一起; 现在拆成独立状态机,
 * 每个状态对应明确的舵面行为, 并与运行时参数 (DartCfg) 解耦:
 *
 *   BOOT    上电初始化, 舵面回中立
 *   NEUTRAL 标准位: 舵面锁定中立, 不参与制导 (测试/待发)
 *   ACTIVE  制导位: 接受混控量 (pitch/yaw/roll) 输出到 4 舵面
 *   TEST    调试位: 4 路同步正弦扫描, 用于单独验证舵机
 *   MANUAL  手动位: 每路直接给逻辑偏角 (网页逐路调试)
 *   FAULT   失效保护: 链路/视觉异常时回中立
 *
 * 舵面角由状态机统一做: 混控 -> 每路量程/方向/中立/微调 -> 速率限幅 -> 底层 PWM。
 */
#ifndef DART_FC_SURFACE_H
#define DART_FC_SURFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DART_SURFACE_CNT 4

typedef enum {
    SURFACE_BOOT = 0,
    SURFACE_NEUTRAL,
    SURFACE_ACTIVE,
    SURFACE_TEST,
    SURFACE_MANUAL,
    SURFACE_FAULT,
    SURFACE_STATE_CNT
} surface_state_t;

void DartSurfaceInit(void);

/* 周期调用 (建议 100Hz~500Hz): 状态机推进 + 舵面输出 */
void DartSurfaceTask(float dt);

void DartSurfaceSetState(surface_state_t st);
surface_state_t DartSurfaceGetState(void);
const char *DartSurfaceStateName(surface_state_t st);

/* ACTIVE 状态使用: 归一化混控指令 (-1..1) */
void DartSurfaceSetMix(float pitch, float yaw, float roll);

/* MANUAL 状态使用: 每路逻辑偏角 (deg, 相对中立, 正负) */
void DartSurfaceSetManualDeg(const float deg[DART_SURFACE_CNT]);
void DartSurfaceSetManualOne(int idx, float deg);

/* 失效保护: 置 1 时无论当前状态都强制回中立 */
void DartSurfaceSetFailsafe(int on);

/* 取零点: 把第 idx 路当前机械位置设为逻辑 0 (中立), 并标记保存 */
void DartSurfaceZero(int idx);

/* 查询 */
void DartSurfaceGetRawDeg(float out[DART_SURFACE_CNT]);   /* 机械角 (显示用) */
void DartSurfaceGetDeflDeg(float out[DART_SURFACE_CNT]);  /* 逻辑偏角 */
float DartSurfaceGetTargetDeg(int idx);                   /* 目标逻辑偏角 */
int DartSurfaceIsFailsafe(void);

#ifdef __cplusplus
}
#endif

#endif /* DART_FC_SURFACE_H */
