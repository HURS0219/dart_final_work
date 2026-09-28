/*
 * servo_mixer.h — 四舵面解耦混控 (通用模块, 与具体业务无关)
 *
 * 作用: 把飞行控制量 (pitch/yaw/roll, -1..1) 解耦成 4 路舵面角度。
 * 属于 Module 层: 只做"控制量 → 硬件动作"的映射, 不含任何制导/PID 逻辑。
 *
 * 解耦矩阵: matrix[舵面][0=pitch, 1=yaw, 2=roll]
 * 舵面角 = neutral_deg + (Σ matrix·cmd) * max_angle_deg
 */
#ifndef SERVO_MIXER_H
#define SERVO_MIXER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pitch;   /* -1..1 */
    float yaw;     /* -1..1 */
    float roll;    /* -1..1 */
} mix_cmd_t;

/**
 * @brief 初始化: 设置解耦矩阵、中立角、最大偏角
 * @param matrix       4x3 解耦矩阵 (每行一个舵面)
 * @param neutral_deg  舵面中立角 (deg)
 * @param max_angle_deg 最大偏角 (deg)
 */
void servo_mixer_init(const float matrix[4][3], float neutral_deg, float max_angle_deg);

/** @brief 运行时修改解耦矩阵 */
void servo_mixer_set_matrix(const float matrix[4][3]);

/** @brief 解耦: pitch/yaw/roll 指令 → 4 路舵面角度 */
void servo_mixer_apply(const mix_cmd_t *in, float out_deg[4]);

/** @brief 全手动: 4 路摇杆直接映射到舵面 */
void servo_mixer_full_manual(const float sticks[4], float gain_deg, float out_deg[4]);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_MIXER_H */
