#include "servo_mixer.h"

/* 默认矩阵/参数: 使用前应调用 servo_mixer_init() 显式设置 */
static float s_matrix[4][3] = {
    {1.0f, 1.0f, 1.0f},
    {1.0f, 1.0f, 1.0f},
    {1.0f, 1.0f, 1.0f},
    {1.0f, 1.0f, 1.0f},
};
static float s_neutral_deg = 90.0f;
static float s_max_angle_deg = 35.0f;

void servo_mixer_init(const float matrix[4][3], float neutral_deg, float max_angle_deg)
{
    servo_mixer_set_matrix(matrix);
    s_neutral_deg = neutral_deg;
    s_max_angle_deg = max_angle_deg;
}

void servo_mixer_set_matrix(const float matrix[4][3])
{
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 3; j++) {
            s_matrix[i][j] = matrix[i][j];
        }
    }
}

void servo_mixer_apply(const mix_cmd_t *in, float out_deg[4])
{
    int i;
    for (i = 0; i < 4; i++) {
        float u = s_matrix[i][0] * in->pitch
                + s_matrix[i][1] * in->yaw
                + s_matrix[i][2] * in->roll;
        out_deg[i] = s_neutral_deg + u * s_max_angle_deg;
    }
}

void servo_mixer_full_manual(const float sticks[4], float gain_deg, float out_deg[4])
{
    out_deg[0] = s_neutral_deg + sticks[1] * gain_deg;
    out_deg[1] = s_neutral_deg + sticks[3] * gain_deg;
    out_deg[2] = s_neutral_deg + sticks[2] * gain_deg;
    out_deg[3] = s_neutral_deg + sticks[1] * gain_deg;
}
