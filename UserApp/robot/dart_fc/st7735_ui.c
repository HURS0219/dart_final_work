#include "st7735_ui.h"

#include <math.h>
#include <stdio.h>

#include "main.h"
#include "bsp_dwt.h"

#include "st7735.h"
#include "dart_app.h"
#include "dart_config.h"
#include "dart_surface.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define UI_SCREEN_CNT    3u
#define UI_KEY_DEBOUNCE  8u
#define UI_FRAME_DT      0.10f    /* 10Hz 刷新 (切屏立即) */

#define UI_TITLE_H       12
#define UI_LINE_H        12

#define C_BG     ST7735_BLACK
#define C_TITLE  ST7735_BLUE
#define C_TEXT   ST7735_WHITE
#define C_DIM    0x7BEF
#define C_ACC    ST7735_CYAN
#define C_WARN   ST7735_RED
#define C_OK     ST7735_GREEN
#define C_SKY    0x0410
#define C_GROUND 0x8A00

#define CAM_X  2
#define CAM_Y  (UI_TITLE_H + 2)
#define CAM_W  (ST7735_WIDTH - 4)
#define CAM_H  (ST7735_HEIGHT - CAM_Y - 14)

#define ATT_X  4
#define ATT_Y  (UI_TITLE_H + 2)
#define ATT_W  (ST7735_WIDTH - 8)
#define ATT_H  92

static uint8_t s_screen;
static uint8_t s_dirty;
static uint8_t s_need_clear;
static uint8_t s_inited;
static float   s_last_draw_s;

static uint8_t  s_key_raw_last = 1u;
static uint8_t  s_key_stable = 1u;
static uint32_t s_key_change_ms;

static const uint16_t *s_frame;
static uint16_t s_frame_w, s_frame_h;

/* IMU 地平仪离屏缓冲: 一次性刷, 避免清屏闪烁 */
static uint16_t s_att_box[ATT_W * ATT_H];

/* ---------- 数字格式化 (不用浮点 printf) ---------- */
static void fmtf(char *dst, int cap, float v, int dec)
{
    static const long p10[4] = {1, 10, 100, 1000};
    int neg;
    float av;
    long scale;
    long scaled;
    long whole;
    long frac;

    if (dec < 0) dec = 0;
    if (dec > 3) dec = 3;
    neg = (v < 0.0f);
    av = neg ? -v : v;
    scale = p10[dec];
    scaled = (long)(av * (float)scale + 0.5f);
    whole = scaled / scale;
    frac = scaled % scale;

    if (dec == 0) {
        snprintf(dst, (size_t)cap, "%s%ld", neg ? "-" : "", whole);
    } else {
        snprintf(dst, (size_t)cap, "%s%ld.%0*ld", neg ? "-" : "", whole, dec, frac);
    }
}

static void pid_line(char *dst, int cap, char axis, float kp, float kd)
{
    char a[10], b[10];
    fmtf(a, sizeof(a), kp, 1);
    fmtf(b, sizeof(b), kd, 2);
    snprintf(dst, (size_t)cap, "PID %c %s/%s", axis, a, b);
}

static void draw_title(const char *txt)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH, UI_TITLE_H, C_TITLE);
    ST7735_DrawString(2, 0, txt, C_TEXT, C_TITLE);
}

/* ---------- 按键 (PA0, 低有效) ----------
 * 基于时间消抖: 无论调用频率多少, 只要新电平稳定 >=30ms 就生效。 */
static void ui_key_scan(void)
{
    uint8_t raw = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ? 0u : 1u;
    uint32_t now = HAL_GetTick();

    if (raw != s_key_raw_last) {
        s_key_raw_last = raw;
        s_key_change_ms = now;
        return;
    }
    if ((now - s_key_change_ms) < 30u) {
        return;
    }
    if (raw != s_key_stable) {
        s_key_stable = raw;
        if (raw == 0u) {   /* 按下沿: 切屏 */
            s_screen = (uint8_t)((s_screen + 1u) % UI_SCREEN_CNT);
            s_dirty = 1u;
            s_need_clear = 1u;
        }
    }
}

/* ================= 界面1: 系统状态 (布局静态, 只重写数值) ================= */
static void draw_screen_status(const dart_state_t *s, uint8_t full)
{
    char b[26], t[12];
    const char *mn;
    uint16_t y = UI_TITLE_H + 2;

    if (full) draw_title("DART CTRL 1/3");

    mn = (s->mode == 0) ? "COMP" : (s->mode == 1) ? "DECOUP" : "FULL";
    snprintf(b, sizeof(b), "MODE %s   ", mn);
    ST7735_DrawString(2, y, b, C_ACC, C_BG); y += UI_LINE_H;

    snprintf(b, sizeof(b), "ARM%d LNK%d VIS%d FS%d",
             s->armed, s->link_ok, s->vision_online, s->failsafe);
    ST7735_DrawString(2, y, b, s->failsafe ? C_WARN : C_TEXT, C_BG); y += UI_LINE_H;

    snprintf(b, sizeof(b), "SURF %s   ", DartSurfaceStateName((surface_state_t)s->surface_state));
    ST7735_DrawString(2, y, b, s->surface_failsafe ? C_WARN : C_ACC, C_BG); y += UI_LINE_H;


    snprintf(b, sizeof(b), "GUID %s/%s  ",
             s->guid_valid ? "ok" : "--", s->guid_started ? "run" : "idle");
    ST7735_DrawString(2, y, b, C_DIM, C_BG); y += UI_LINE_H;

    fmtf(t, sizeof(t), s->servo_deg[0], 1);
    snprintf(b, sizeof(b), "S1 %s ", t);
    ST7735_DrawString(2, y, b, C_TEXT, C_BG);
    fmtf(t, sizeof(t), s->servo_deg[1], 1);
    snprintf(b, sizeof(b), "S2 %s ", t);
    ST7735_DrawString(66, y, b, C_TEXT, C_BG); y += UI_LINE_H;

    fmtf(t, sizeof(t), s->servo_deg[2], 1);
    snprintf(b, sizeof(b), "S3 %s ", t);
    ST7735_DrawString(2, y, b, C_TEXT, C_BG);
    fmtf(t, sizeof(t), s->servo_deg[3], 1);
    snprintf(b, sizeof(b), "S4 %s ", t);
    ST7735_DrawString(66, y, b, C_TEXT, C_BG); y += UI_LINE_H;

    pid_line(b, sizeof(b), 'R', CFG_ROLL_KP, CFG_ROLL_KD);
    ST7735_DrawString(2, y, b, C_DIM, C_BG); y += UI_LINE_H;
    pid_line(b, sizeof(b), 'P', CFG_PITCH_KP, CFG_PITCH_KD);
    ST7735_DrawString(2, y, b, C_DIM, C_BG); y += UI_LINE_H;
    pid_line(b, sizeof(b), 'Y', CFG_YAW_KP, CFG_YAW_KD);
    ST7735_DrawString(2, y, b, C_DIM, C_BG); y += UI_LINE_H;

    {
        char ax[10], ay[10];
        fmtf(ax, sizeof(ax), s->accel_x, 2);
        fmtf(ay, sizeof(ay), s->accel_y, 2);
        snprintf(b, sizeof(b), "ACC %s %s   ", ax, ay);
        ST7735_DrawString(2, y, b, C_ACC, C_BG); y += UI_LINE_H;
    }
    {
        char lx[10], ly[10];
        fmtf(lx, sizeof(lx), s->lambda_x, 2);
        fmtf(ly, sizeof(ly), s->lambda_y, 2);
        snprintf(b, sizeof(b), "LOS %s %s   ", lx, ly);
        ST7735_DrawString(2, y, b, C_DIM, C_BG); y += UI_LINE_H;
    }
    {
        char lx[10], ly[10];
        fmtf(lx, sizeof(lx), s->lambda_dot_x, 1);
        fmtf(ly, sizeof(ly), s->lambda_dot_y, 1);
        snprintf(b, sizeof(b), "LDOT %s %s  ", lx, ly);
        ST7735_DrawString(2, y, b, C_DIM, C_BG);
    }
}

/* ================= 界面2: OpenMV ================= */
static uint32_t s_rand = 0x2468ACE1u;

static uint16_t ui_rand(void)
{
    s_rand = s_rand * 1103515245u + 12345u;
    return (uint16_t)(s_rand >> 13);
}

static void draw_noise(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    uint16_t row[ST7735_WIDTH];
    uint16_t j, i;
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            uint16_t r = ui_rand();
            row[i] = (uint16_t)(((r & 0x1F) << 11) | (((r >> 5) & 0x3F) << 5) | (r >> 11));
        }
        ST7735_Blit(x, (uint16_t)(y + j), w, 1, row);
    }
}

static void draw_frame_scaled(const uint16_t *buf, uint16_t sw, uint16_t sh,
                              uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    uint16_t row[ST7735_WIDTH];
    uint16_t j, i;
    for (j = 0; j < h; j++) {
        uint16_t sy = (uint16_t)((uint32_t)j * sh / h);
        const uint16_t *src = buf + (uint32_t)sy * sw;
        for (i = 0; i < w; i++) {
            uint16_t sx = (uint16_t)((uint32_t)i * sw / w);
            row[i] = src[sx];
        }
        ST7735_Blit(x, (uint16_t)(y + j), w, 1, row);
    }
}

static void draw_camera_scene(const dart_state_t *s)
{
    float sx = (float)CAM_W / (CFG_IMAGE_CX * 2.0f);
    float sy = (float)CAM_H / (CFG_IMAGE_CY * 2.0f);
    int cx = CAM_X + CAM_W / 2;
    int cy = CAM_Y + CAM_H / 2;
    int tx = CAM_X + (int)((float)s->target_x * sx);
    int ty = CAM_Y + (int)((float)s->target_y * sy);
    int tw = (int)((float)s->target_w * sx);
    int th = (int)((float)s->target_h * sy);
    int g;

    if (tw < 4) tw = 4;
    if (th < 4) th = 4;
    if (tx < CAM_X) tx = CAM_X;
    if (ty < CAM_Y) ty = CAM_Y;
    if (tx + tw > CAM_X + CAM_W) tw = CAM_X + CAM_W - tx;
    if (ty + th > CAM_Y + CAM_H) th = CAM_Y + CAM_H - ty;

    ST7735_FillRect(CAM_X, CAM_Y, CAM_W, CAM_H, 0x0000);
    for (g = CAM_X + 20; g < CAM_X + CAM_W; g += 20)
        ST7735_DrawLine((int16_t)g, CAM_Y, (int16_t)g, CAM_Y + CAM_H - 1, 0x0841);
    for (g = CAM_Y + 20; g < CAM_Y + CAM_H; g += 20)
        ST7735_DrawLine(CAM_X, (int16_t)g, CAM_X + CAM_W - 1, (int16_t)g, 0x0841);

    ST7735_DrawLine(cx - 6, cy, cx + 6, cy, C_DIM);
    ST7735_DrawLine(cx, cy - 6, cx, cy + 6, C_DIM);

    ST7735_DrawRect((uint16_t)tx, (uint16_t)ty, (uint16_t)tw, (uint16_t)th, ST7735_RED);
}

static void draw_screen_camera(const dart_state_t *s, uint8_t full)
{
    char b[26], t[10];

    if (full) {
        draw_title("OPENMV 2/3");
        ST7735_DrawRect(CAM_X - 1, CAM_Y - 1, CAM_W + 2, CAM_H + 2, C_DIM);
    }

    if (s_frame != 0 && s_frame_w > 0 && s_frame_h > 0) {
        draw_frame_scaled(s_frame, s_frame_w, s_frame_h, CAM_X, CAM_Y, CAM_W, CAM_H);
    } else if (s->vision_online) {
        draw_camera_scene(s);
    } else {
        draw_noise(CAM_X, CAM_Y, CAM_W, CAM_H);
    }

    if (s->vision_online) {
        fmtf(t, sizeof(t), (float)s->target_w, 0);
        snprintf(b, sizeof(b), "TGT %d,%d w%s   ", s->target_x, s->target_y, t);
        ST7735_DrawString(2, ST7735_HEIGHT - 12, b, C_OK, C_BG);
    } else {
        ST7735_DrawString(2, ST7735_HEIGHT - 12, "NO SIGNAL     ", C_WARN, C_BG);
    }
}

/* ================= 界面3: 惯导 ================= */
static void draw_screen_imu(const dart_state_t *s, uint8_t full)
{
    char b[26], t[12];
    int cx = ATT_X + ATT_W / 2;
    int cy = ATT_Y + ATT_H / 2;
    float roll = s->att.roll_deg;
    float pitch = s->att.pitch_deg;
    float slope = tanf(roll * (float)M_PI / 180.0f);
    int pitch_px = (int)(pitch);
    int col, row;
    uint16_t ty;

    if (full) draw_title("IMU ATT 3/3");

    /* 一次性合成地平仪到离屏缓冲, 再单次 Blit (无清屏闪烁) */
    for (row = 0; row < ATT_H; row++) {
        for (col = 0; col < ATT_W; col++) {
            int dx = col - ATT_W / 2;
            int hy = ATT_H / 2 + pitch_px + (int)((float)dx * slope);
            s_att_box[row * ATT_W + col] = (row >= hy) ? C_GROUND : C_SKY;
        }
    }
    ST7735_Blit(ATT_X, ATT_Y, ATT_W, ATT_H, s_att_box);

    {
        int yl = cy + pitch_px + (int)((float)(-ATT_W / 2) * slope);
        int yr = cy + pitch_px + (int)((float)(ATT_W - 1 - ATT_W / 2) * slope);
        ST7735_DrawLine(ATT_X, (int16_t)yl, ATT_X + ATT_W - 1, (int16_t)yr, ST7735_WHITE);
    }

    ST7735_DrawLine(cx - 16, cy, cx - 5, cy, ST7735_YELLOW);
    ST7735_DrawLine(cx + 5, cy, cx + 16, cy, ST7735_YELLOW);
    ST7735_DrawLine(cx, cy - 3, cx, cy + 3, ST7735_YELLOW);

    if (full) {
        ST7735_DrawRect(ATT_X - 1, ATT_Y - 1, ATT_W + 2, ATT_H + 2, C_DIM);
    }
    if (!s->att.valid) {
        ST7735_DrawString(ATT_X + ATT_W - 20, ATT_Y + 2, "SIM", C_WARN, C_SKY);
    }

    ty = ATT_Y + ATT_H + 4;
    fmtf(t, sizeof(t), roll, 1);
    snprintf(b, sizeof(b), "R %s ", t);
    ST7735_DrawString(2, ty, b, C_TEXT, C_BG);
    fmtf(t, sizeof(t), pitch, 1);
    snprintf(b, sizeof(b), "P %s ", t);
    ST7735_DrawString(66, ty, b, C_TEXT, C_BG);
    ty += UI_LINE_H;

    fmtf(t, sizeof(t), s->att.yaw_deg, 1);
    snprintf(b, sizeof(b), "Y %s ", t);
    ST7735_DrawString(2, ty, b, C_TEXT, C_BG);
    fmtf(t, sizeof(t), s->att.gz_dps, 1);
    snprintf(b, sizeof(b), "Gz %s ", t);
    ST7735_DrawString(66, ty, b, C_DIM, C_BG);
    ty += UI_LINE_H;

    {
        char gx[8], gy[8];
        fmtf(gx, sizeof(gx), s->att.gx_dps, 0);
        fmtf(gy, sizeof(gy), s->att.gy_dps, 0);
        snprintf(b, sizeof(b), "Gx%s Gy%s ", gx, gy);
        ST7735_DrawString(2, ty, b, C_DIM, C_BG);
        ty += UI_LINE_H;
    }
    {
        char ax[8], ay[8];
        fmtf(ax, sizeof(ax), s->accel_g[0], 2);
        fmtf(ay, sizeof(ay), s->accel_g[1], 2);
        snprintf(b, sizeof(b), "Ax%s Ay%s ", ax, ay);
        ST7735_DrawString(2, ty, b, C_DIM, C_BG);
    }
}

/* ================= 对外接口 ================= */
void ST7735_UI_Init(void)
{
    ST7735_Init();
    s_screen = 0u;
    s_dirty = 1u;
    s_need_clear = 1u;
    s_last_draw_s = 0.0f;
    s_frame = 0;
    s_frame_w = 0;
    s_frame_h = 0;
    s_inited = 1u;
}

void ST7735_UI_SetFrame(const uint16_t *rgb565, uint16_t w, uint16_t h)
{
    s_frame = rgb565;
    s_frame_w = w;
    s_frame_h = h;
    if (s_screen == 1u) s_dirty = 1u;
}

void ST7735_UI_Task(void)
{
    const dart_state_t *s;
    float now;
    uint8_t full;

    if (!s_inited) ST7735_UI_Init();

    ui_key_scan();

    now = DWT_GetTimeline_s();
    if (!s_dirty && (now - s_last_draw_s) < UI_FRAME_DT) return;
    s_last_draw_s = now;
    s_dirty = 0u;

    full = s_need_clear;
    if (full) {
        ST7735_FillScreen(C_BG);
        s_need_clear = 0u;
    }

    s = DartAppGetState();
    switch (s_screen) {
    case 0u: draw_screen_status(s, full); break;
    case 1u: draw_screen_camera(s, full); break;
    default: draw_screen_imu(s, full);    break;
    }
}
