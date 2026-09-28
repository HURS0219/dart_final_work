#include "st7735_test.h"

#if defined(STM32F407xx)

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "st7735.h"

#define ST7735_TEST_INTERVAL_S 1.0f
#define ST7735_TEST_STAGES     6u

static uint8_t s_inited;
static uint8_t s_stage;
static float s_last_s;

static void ST7735TestDrawStage(uint8_t stage)
{
    uint16_t i;
    uint16_t bar_w = ST7735_WIDTH / 8u;

    switch (stage) {
    case 0:
        ST7735_FillScreen(ST7735_RED);
        ST7735_DrawString(4, 4, "ST7735", ST7735_WHITE, ST7735_RED);
        ST7735_DrawString(4, 20, "SPI2 PB13/15", ST7735_WHITE, ST7735_RED);
        break;
    case 1:
        ST7735_FillScreen(ST7735_GREEN);
        ST7735_DrawString(4, 4, "128x160", ST7735_BLACK, ST7735_GREEN);
        break;
    case 2:
        ST7735_FillScreen(ST7735_BLUE);
        ST7735_DrawString(4, 4, "RGB565", ST7735_WHITE, ST7735_BLUE);
        break;
    case 3:
        ST7735_FillScreen(ST7735_WHITE);
        ST7735_DrawString(4, 4, "CS=PB12", ST7735_BLACK, ST7735_WHITE);
        ST7735_DrawString(4, 20, "DC=PB14", ST7735_BLACK, ST7735_WHITE);
        ST7735_DrawString(4, 36, "RST=PF1", ST7735_BLACK, ST7735_WHITE);
        ST7735_DrawString(4, 52, "BLK=PF0", ST7735_BLACK, ST7735_WHITE);
        break;
    case 4: { /* 8 段彩条 */
        static const uint16_t colors[8] = {
            ST7735_RED, ST7735_ORANGE, ST7735_YELLOW, ST7735_GREEN,
            ST7735_CYAN, ST7735_BLUE, ST7735_MAGENTA, ST7735_WHITE};
        for (i = 0; i < 8u; i++) {
            ST7735_FillRect((uint16_t)(i * bar_w), 0, bar_w, ST7735_HEIGHT, colors[i]);
        }
        break;
    }
    default: /* 5: 灰阶/边框 */
        ST7735_FillScreen(ST7735_BLACK);
        ST7735_FillRect(0, 0, ST7735_WIDTH, 8, ST7735_GRAY);
        ST7735_FillRect(0, ST7735_HEIGHT - 8, ST7735_WIDTH, 8, ST7735_GRAY);
        ST7735_DrawString(4, 76, "ST7735 READY", ST7735_CYAN, ST7735_BLACK);
        break;
    }
}

void ST7735TestInit(void)
{
    if (s_inited) return;
    ST7735_Init();
    s_last_s = DWT_GetTimeline_s();
    s_stage = 0u;
    ST7735TestDrawStage(s_stage);
    s_inited = 1u;
    LOGINFO("[st7735] bring-up test start");
}

void ST7735TestTask(void)
{
    float now;

    if (!s_inited) {
        ST7735TestInit();
    }

    now = DWT_GetTimeline_s();
    if ((now - s_last_s) < ST7735_TEST_INTERVAL_S) return;
    s_last_s = now;

    s_stage = (uint8_t)((s_stage + 1u) % ST7735_TEST_STAGES);
    ST7735TestDrawStage(s_stage);
}

#else

void ST7735TestInit(void) {}
void ST7735TestTask(void) {}

#endif /* STM32F407xx */
