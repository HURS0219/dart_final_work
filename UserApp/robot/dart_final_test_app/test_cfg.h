/*
 * test_cfg.h — dart_final_test_app 测试参数与“话题↔生产者/消费者”映射
 * =============================================================================
 * 本 app 用于**逐个黑盒测试** dart_final 的各 app 服务(自底向上: fin -> guidance -> vision/imu)。
 * 监视器(方案①: 不改 message_center)自己订阅所有话题, 并借“映射表”推断 producer/sub 状态。
 * =============================================================================
 */
#pragma once

/* ---------- 测试使能(默认用注入, 避免无硬件死等/串口干扰) ---------- */
#define TEST_IMU_ENABLE 0    /* 0=不初始化真实 IMU(避免无 BMI088 时 INS_Init 死等), 用 ATT 注入 */
#define TEST_VISION_ENABLE 0 /* 0=不注册 OpenMV 串口, 用 TGT 注入 */

/* ---------- 周期 ---------- */
#define MON_PERIOD_MS 100u  /* 实时监视 UI 刷新周期(10Hz) */
#define LOG_PERIOD_MS 1000u /* 每秒写一条 log(供 MATLAB) */
#define DATA_FRESH_MS 500u  /* 话题数据“新鲜”判定窗口(ms): 超过则 data=0 */

/* ============================================================================
 *                      话题 ↔ 预期 生产者(pub) / 消费者(sub) 映射
 * ----------------------------------------------------------------------------
 * 方案① 下监视器看不到别的 app 的订阅, 故用本表推断:
 *   producer=1 <=> 该话题的生产者 app 正在运行(hb 心跳在涨, SWD/RTT 可读)
 *   sub     =1 <=> 该话题的消费者 app 正在运行(同上)
 * 值为 app 索引; -1 表示无。
 * ==========================================================================*/
/* app 索引 */
#define A_IMU 0
#define A_VISION 1
#define A_GUID 2
#define A_FIN 3
#define A_CNT 4

/* 话题索引 */
#define T_ATTITUDE 0
#define T_TARGET 1
#define T_MIX 2
#define T_SERVO_FB 3
#define T_CNT 4

/* 每个话题 的 生产者 / 消费者 (顺序与 T_* 一致) */
#define TOPIC_PRODUCER {A_IMU, A_VISION, A_GUID, A_FIN}
#define TOPIC_CONSUMER {A_GUID, A_GUID, A_FIN, -1}
#define TOPIC_NAME {"attitude", "target", "mix", "servo_fb"}
