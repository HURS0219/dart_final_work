#include "dart_control.h"
#include "dart_cfg.h"
#include "dart_config.h"

#define GRAVITY 9.80665f

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void set_neutral(control_out_t *out)
{
    out->mix.pitch = 0.0f;
    out->mix.yaw = 0.0f;
    out->mix.roll = 0.0f;
    servo_mixer_apply(&out->mix, out->servo_deg);
}

void control_init(void)
{
    /* 混控矩阵/中立/量程由 DartCfg 统一管理 (DartCfgApply 会初始化 mixer)。
     * 这里再 Apply 一次, 保证即使单独调用 control_init 也已就绪。 */
    DartCfgApply();
}

float control_stick_shape(int raw, int deadband, float expo)
{
    float x;
    float mag;
    float shaped;
    float sign;

    if (raw > 1000) raw = 1000;
    if (raw < -1000) raw = -1000;
    if (raw > -deadband && raw < deadband) {
        return 0.0f;
    }

    sign = (raw < 0) ? -1.0f : 1.0f;
    mag = (float)((raw < 0 ? -raw : raw) - deadband)
        / (float)(1000 - deadband);
    if (mag > 1.0f) mag = 1.0f;

    shaped = (1.0f - expo) * mag + expo * mag * mag * mag;
    x = sign * shaped;
    return clampf(x, -1.0f, 1.0f);
}

static void step_competition(const control_in_t *in, control_out_t *out)
{
    DartCfg_t *c = DartCfg();
    float max_g = c->png_max_g * GRAVITY;
    float pitch_cmd;
    float yaw_cmd;
    float roll_cmd;
    guidance_t g = in->guid;

    roll_cmd = -(c->roll_kp * in->att.roll_deg
                 + c->roll_kd * in->att.gx_dps) / CFG_SERVO_MAX_ANGLE;

    if (in->vision_ok) {
        float bx, by;
        guidance_png(&g, c->nav_ratio, c->v_close, CFG_PNG_SIGN, c->png_max_g);
        guidance_rotate_to_body(g.accel_x, g.accel_y,
                                in->att.roll_deg, CFG_DEROT_SIGN, &bx, &by);
        yaw_cmd   = bx / max_g;
        pitch_cmd = by / max_g;
    } else {
        pitch_cmd = in->sticks[1];
        yaw_cmd   = in->sticks[2];
    }

    out->mix.pitch = clampf(pitch_cmd, -CFG_MANUAL_CMD_LIMIT, CFG_MANUAL_CMD_LIMIT);
    out->mix.yaw   = clampf(yaw_cmd,   -CFG_MANUAL_CMD_LIMIT, CFG_MANUAL_CMD_LIMIT);
    out->mix.roll  = clampf(roll_cmd,  -CFG_MANUAL_CMD_LIMIT, CFG_MANUAL_CMD_LIMIT);
    servo_mixer_apply(&out->mix, out->servo_deg);
}

static void step_decoupled_manual(const control_in_t *in, control_out_t *out)
{
    DartCfg_t *c = DartCfg();
    float roll_sp  = in->sticks[3] * CFG_MANUAL_MAX_ROLL_DEG;
    float pitch_sp = in->sticks[1] * CFG_MANUAL_MAX_PITCH_DEG;
    float yaw_rate_sp = in->sticks[2] * CFG_MANUAL_MAX_YAW_RATE;

    float roll_cmd  = -(c->roll_kp  * (in->att.roll_deg  - roll_sp)
                        + c->roll_kd  * in->att.gx_dps);
    float pitch_cmd = -(c->pitch_kp * (in->att.pitch_deg - pitch_sp)
                        + c->pitch_kd * in->att.gy_dps);
    float yaw_cmd   = -(c->yaw_kp   * (yaw_rate_sp - in->att.gz_dps)
                        + c->yaw_kd   * in->att.gz_dps);

    roll_cmd  /= CFG_SERVO_MAX_ANGLE;
    pitch_cmd /= CFG_SERVO_MAX_ANGLE;
    yaw_cmd   /= CFG_SERVO_MAX_ANGLE;

    out->mix.pitch = clampf(pitch_cmd, -CFG_MANUAL_CMD_LIMIT, CFG_MANUAL_CMD_LIMIT);
    out->mix.yaw   = clampf(yaw_cmd,   -CFG_MANUAL_CMD_LIMIT, CFG_MANUAL_CMD_LIMIT);
    out->mix.roll  = clampf(roll_cmd,  -CFG_MANUAL_CMD_LIMIT, CFG_MANUAL_CMD_LIMIT);
    servo_mixer_apply(&out->mix, out->servo_deg);
}

static void step_full_manual(const control_in_t *in, control_out_t *out)
{
    servo_mixer_full_manual(in->sticks, CFG_FM_GAIN_DEG, out->servo_deg);
    out->mix.pitch = in->sticks[1];
    out->mix.yaw   = in->sticks[2];
    out->mix.roll  = in->sticks[3];
}

void control_step(const control_in_t *in, control_out_t *out)
{
    out->failsafe = 0;

    if (in->mode == MODE_FULL_MANUAL) {
        if (!in->link_ok) {
            out->failsafe = 1;
            set_neutral(out);
            return;
        }
        step_full_manual(in, out);
        return;
    }

    if (!in->armed || !in->link_ok) {
        out->failsafe = in->link_ok ? 0 : 1;
        set_neutral(out);
        return;
    }

    if (in->mode == MODE_DECOUPLED_MANUAL) {
        step_decoupled_manual(in, out);
    } else {
        step_competition(in, out);
    }
}
