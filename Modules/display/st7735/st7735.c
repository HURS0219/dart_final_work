#include "st7735.h"
#include "st7735_port.h"

#include "main.h"
#include "spi.h"

/* oled.c 中定义的 6x12 ASCII 字库 (oledfont.h), 这里只引用不重复定义 */
extern const unsigned char asc2_1206[95][12];

static uint8_t s_ready = 0;

/* ---------- 底层控制脚 ---------- */
static inline void cs_low(void)  { HAL_GPIO_WritePin(ST7735_CS_PORT,  ST7735_CS_PIN,  GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(ST7735_CS_PORT,  ST7735_CS_PIN,  GPIO_PIN_SET); }
static inline void dc_cmd(void)  { HAL_GPIO_WritePin(ST7735_DC_PORT,  ST7735_DC_PIN,  GPIO_PIN_RESET); }
static inline void dc_data(void) { HAL_GPIO_WritePin(ST7735_DC_PORT,  ST7735_DC_PIN,  GPIO_PIN_SET); }

static void st7735_write_cmd(uint8_t cmd)
{
    dc_cmd();
    cs_low();
    HAL_SPI_Transmit(&ST7735_SPI_HANDLE, &cmd, 1, 100);
    cs_high();
}

static void st7735_write_data(const uint8_t *data, uint16_t len)
{
    if (len == 0u) return;
    dc_data();
    cs_low();
    HAL_SPI_Transmit(&ST7735_SPI_HANDLE, (uint8_t *)data, len, 1000);
    cs_high();
}

static void st7735_write_cmd_data(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    st7735_write_cmd(cmd);
    st7735_write_data(data, len);
}

/* ---------- GPIO / SPI ---------- */
static void st7735_pin_out(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef g = {0};
    g.Pin = pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &g);
}

static void ST7735_GpioInit(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    st7735_pin_out(ST7735_CS_PORT,  ST7735_CS_PIN);
    st7735_pin_out(ST7735_DC_PORT,  ST7735_DC_PIN);
    st7735_pin_out(ST7735_RST_PORT, ST7735_RST_PIN);
    st7735_pin_out(ST7735_BLK_PORT, ST7735_BLK_PIN);

    cs_high();
    dc_data();
    HAL_GPIO_WritePin(ST7735_BLK_PORT, ST7735_BLK_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST7735_RST_PORT, ST7735_RST_PIN, GPIO_PIN_SET);
}

void ST7735_SetBacklight(uint8_t on)
{
    HAL_GPIO_WritePin(ST7735_BLK_PORT, ST7735_BLK_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ST7735_HardReset(void)
{
    HAL_GPIO_WritePin(ST7735_RST_PORT, ST7735_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(ST7735_RST_PORT, ST7735_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);
}

static void ST7735_RunInitSeq(void)
{
    static const uint8_t frmctr1[] = {0x01, 0x2C, 0x2D};
    static const uint8_t frmctr2[] = {0x01, 0x2C, 0x2D};
    static const uint8_t frmctr3[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
    static const uint8_t invctr[]  = {0x07};
    static const uint8_t pwctr1[]  = {0xA2, 0x02, 0x84};
    static const uint8_t pwctr2[]  = {0xC5};
    static const uint8_t pwctr3[]  = {0x0A, 0x00};
    static const uint8_t pwctr4[]  = {0x8A, 0x2A};
    static const uint8_t pwctr5[]  = {0x8A, 0xEE};
    static const uint8_t vmctr1[]  = {0x0E};
    static const uint8_t madctl[]  = {ST7735_MADCTL};
    static const uint8_t colmod[]  = {0x05};
    static const uint8_t gmctrp1[] = {0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
                                      0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};
    static const uint8_t gmctrn1[] = {0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                                      0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};

    st7735_write_cmd(0x01); /* SWRESET */
    HAL_Delay(150);
    st7735_write_cmd(0x11); /* SLPOUT */
    HAL_Delay(120);

    st7735_write_cmd_data(0xB1, frmctr1, sizeof(frmctr1));
    st7735_write_cmd_data(0xB2, frmctr2, sizeof(frmctr2));
    st7735_write_cmd_data(0xB3, frmctr3, sizeof(frmctr3));
    st7735_write_cmd_data(0xB4, invctr,  sizeof(invctr));
    st7735_write_cmd_data(0xC0, pwctr1,  sizeof(pwctr1));
    st7735_write_cmd_data(0xC1, pwctr2,  sizeof(pwctr2));
    st7735_write_cmd_data(0xC2, pwctr3,  sizeof(pwctr3));
    st7735_write_cmd_data(0xC3, pwctr4,  sizeof(pwctr4));
    st7735_write_cmd_data(0xC4, pwctr5,  sizeof(pwctr5));
    st7735_write_cmd_data(0xC5, vmctr1,  sizeof(vmctr1));

    st7735_write_cmd(0x20); /* INVOFF; 若颜色反相改成 0x21 (INVON) */

    st7735_write_cmd_data(0x36, madctl, sizeof(madctl));
    st7735_write_cmd_data(0x3A, colmod, sizeof(colmod));

    st7735_write_cmd_data(0xE0, gmctrp1, sizeof(gmctrp1));
    st7735_write_cmd_data(0xE1, gmctrn1, sizeof(gmctrn1));

    st7735_write_cmd(0x13); /* NORON */
    HAL_Delay(10);
    st7735_write_cmd(0x29); /* DISPON */
    HAL_Delay(100);
}

void ST7735_Init(void)
{
    /* 先把 SPI2 提速 (CubeMX 默认 256 分频只有 164kHz, 刷屏太慢)。
     * HAL_SPI_Init 会重配 PB13/14/15, 因此必须在它之后再配控制脚,
     * 否则借用的 MISO(PB14)=DC 会被重新设回 AF。 */
    ST7735_SPI_HANDLE.Init.BaudRatePrescaler = ST7735_SPI_PRESCALER;
    HAL_SPI_Init(&ST7735_SPI_HANDLE);

    ST7735_GpioInit();

    ST7735_HardReset();
    ST7735_RunInitSeq();
    ST7735_SetBacklight(1);
    ST7735_FillScreen(ST7735_BLACK);
    s_ready = 1u;
}

/* ---------- 绘制 ---------- */
void ST7735_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t d[4];

    x0 += ST7735_X_OFFSET;
    x1 += ST7735_X_OFFSET;
    y0 += ST7735_Y_OFFSET;
    y1 += ST7735_Y_OFFSET;

    d[0] = (uint8_t)(x0 >> 8); d[1] = (uint8_t)x0;
    d[2] = (uint8_t)(x1 >> 8); d[3] = (uint8_t)x1;
    st7735_write_cmd_data(0x2A, d, 4); /* CASET */

    d[0] = (uint8_t)(y0 >> 8); d[1] = (uint8_t)y0;
    d[2] = (uint8_t)(y1 >> 8); d[3] = (uint8_t)y1;
    st7735_write_cmd_data(0x2B, d, 4); /* RASET */

    st7735_write_cmd(0x2C); /* RAMWR */
}

void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint8_t line[ST7735_WIDTH * 2u];
    uint16_t i;

    if (!s_ready || w == 0u || h == 0u) return;
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    if (x + w > ST7735_WIDTH)  w = ST7735_WIDTH - x;
    if (y + h > ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    ST7735_SetWindow(x, y, x + w - 1u, y + h - 1u);

    for (i = 0; i < w; i++) {
        line[i * 2u]      = (uint8_t)(color >> 8);
        line[i * 2u + 1u] = (uint8_t)color;
    }

    dc_data();
    cs_low();
    for (i = 0; i < h; i++) {
        HAL_SPI_Transmit(&ST7735_SPI_HANDLE, line, (uint16_t)(w * 2u), 1000);
    }
    cs_high();
}

void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    uint8_t d[2];
    if (!s_ready || x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    d[0] = (uint8_t)(color >> 8);
    d[1] = (uint8_t)color;
    ST7735_SetWindow(x, y, x, y);
    st7735_write_data(d, 2);
}

void ST7735_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;

    for (;;) {
        ST7735_DrawPixel((uint16_t)x0, (uint16_t)y0, color);
        if (x0 == x1 && y0 == y1) break;
        {
            int16_t e2 = err * 2;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }
    }
}

void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (w == 0u || h == 0u) return;
    ST7735_DrawLine((int16_t)x, (int16_t)y, (int16_t)(x + w - 1u), (int16_t)y, color);
    ST7735_DrawLine((int16_t)x, (int16_t)(y + h - 1u), (int16_t)(x + w - 1u), (int16_t)(y + h - 1u), color);
    ST7735_DrawLine((int16_t)x, (int16_t)y, (int16_t)x, (int16_t)(y + h - 1u), color);
    ST7735_DrawLine((int16_t)(x + w - 1u), (int16_t)y, (int16_t)(x + w - 1u), (int16_t)(y + h - 1u), color);
}

void ST7735_Blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *colors)
{
    uint8_t buf[256];
    uint32_t total;
    uint32_t idx = 0;

    if (!s_ready || colors == 0 || w == 0u || h == 0u) return;
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    if (x + w > ST7735_WIDTH)  w = ST7735_WIDTH - x;
    if (y + h > ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    ST7735_SetWindow(x, y, x + w - 1u, y + h - 1u);

    total = (uint32_t)w * h;
    dc_data();
    cs_low();
    while (idx < total) {
        uint16_t n = (uint16_t)((total - idx > 128u) ? 128u : (total - idx));
        uint16_t i;
        for (i = 0; i < n; i++) {
            uint16_t c = colors[idx + i];
            buf[i * 2u]      = (uint8_t)(c >> 8);
            buf[i * 2u + 1u] = (uint8_t)c;
        }
        HAL_SPI_Transmit(&ST7735_SPI_HANDLE, buf, (uint16_t)(n * 2u), 1000);
        idx += n;
    }
    cs_high();
}

void ST7735_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg)
{
    uint8_t glyph[ST7735_CHAR_H][ST7735_CHAR_W];
    uint16_t pix[ST7735_CHAR_W * ST7735_CHAR_H];
    uint8_t idx, t, t1, cx = 0, cy = 0, r, c;

    if (!s_ready) return;
    if (ch < ' ' || ch > '~') ch = '?';
    idx = (uint8_t)(ch - ' ');

    /* 与 oled.c 相同的取模顺序: 12 字节 -> 6 列 x 12 行 */
    for (t = 0; t < 12u; t++) {
        uint8_t temp = asc2_1206[idx][t];
        for (t1 = 0; t1 < 8u; t1++) {
            glyph[cy][cx] = (temp & 0x80u) ? 1u : 0u;
            temp <<= 1;
            cy++;
            if (cy == ST7735_CHAR_H) { cy = 0; cx++; break; }
        }
    }

    for (r = 0; r < ST7735_CHAR_H; r++) {
        for (c = 0; c < ST7735_CHAR_W; c++) {
            pix[r * ST7735_CHAR_W + c] = glyph[r][c] ? fg : bg;
        }
    }
    ST7735_Blit(x, y, ST7735_CHAR_W, ST7735_CHAR_H, pix);
}

void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg)
{
    if (!s_ready || str == NULL) return;
    while (*str != '\0') {
        if (x + ST7735_CHAR_W > ST7735_WIDTH) break;
        ST7735_DrawChar(x, y, *str, fg, bg);
        x += ST7735_CHAR_W;
        str++;
    }
}
