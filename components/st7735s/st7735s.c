/*
 * st7735s.c - ST7735S 1.8" 128x160 TFT driver for ESP32-S3
 *
 * Modular port of the st7735s-tft-main reference project (STM32F103,
 * bit-banged SPI):
 *   - bit-banged SPI -> official esp_driver_spi (hardware SPI + DMA)
 *   - GPIO macros    -> official esp_driver_gpio (RST / D-CX / backlight)
 *   - delay_ms       -> vTaskDelay
 * The lcdInit() command sequence, RGB565 pixel format (0x3A = 0x05) and
 * MADCTL 0x36 = 0xC0 are kept verbatim from the reference.
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "st7735s.h"
#include "st7735s_pins.h"
#include "st7735s_font.h"

static const char *TAG = "st7735s";

/* The panel RAM is 132x162; the reference clears 130x162 from the origin to
 * cover the visible 128x160 area - keep that behaviour for clear(). */
#define CLEAR_X2    129
#define CLEAR_Y2    161

#define PX_CHUNK    256     /* pixels per SPI transaction when streaming */

struct st7735s_t {
    spi_host_device_t host;
    spi_device_handle_t spi;
    bool bus_inited;
    gpio_num_t dc_io;
    gpio_num_t rst_io;
    gpio_num_t backlight_io;
    uint16_t width;
    uint16_t height;
};

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[16];
} init_cmd_t;

/* Port of lcdInit() from the reference */
static const init_cmd_t s_init_seq[] = {
    { 0x11, 0, { 0 } },                                     /* sleep out (+120 ms) */
    { 0xB1, 3, { 0x05, 0x3C, 0x3C } },                      /* frame rate ctrl     */
    { 0xB2, 3, { 0x05, 0x3C, 0x3C } },
    { 0xB3, 6, { 0x05, 0x3C, 0x3C, 0x05, 0x3C, 0x3C } },
    { 0xB4, 1, { 0x03 } },                                  /* display inversion   */
    { 0xC0, 3, { 0x28, 0x08, 0x04 } },                      /* power control       */
    { 0xC1, 1, { 0xC0 } },
    { 0xC2, 2, { 0x0D, 0x00 } },
    { 0xC3, 2, { 0x8D, 0x2A } },
    { 0xC4, 2, { 0x8D, 0xEE } },
    { 0xC5, 1, { 0x1A } },                                  /* VCOM               */
    { 0x36, 1, { 0xC0 } },                                  /* MADCTL             */
    { 0xE0, 16, { 0x04, 0x22, 0x07, 0x0A, 0x2E, 0x30, 0x25, 0x2A,
                  0x28, 0x26, 0x2E, 0x3A, 0x00, 0x01, 0x03, 0x13 } }, /* gamma + */
    { 0xE1, 16, { 0x04, 0x16, 0x06, 0x0D, 0x2D, 0x26, 0x23, 0x27,
                  0x27, 0x25, 0x2D, 0x3B, 0x00, 0x01, 0x04, 0x13 } }, /* gamma - */
    { 0x3A, 1, { 0x05 } },                                  /* RGB565             */
    { 0x29, 0, { 0 } },                                     /* display on         */
};

/* ------------------------------------------------------------------ */
/* SPI primitives                                                      */
/* ------------------------------------------------------------------ */

/* Direct polling transmit. src must live in internal RAM and be 4-byte
 * aligned so the DMA engine can reach it (const tables live in flash). */
static esp_err_t xfer(st7735s_handle_t dev, bool is_data, const void *src, size_t bytes)
{
    gpio_set_level(dev->dc_io, is_data);
    spi_transaction_t t = {
        .length = bytes * 8,
        .tx_buffer = src,
    };
    return spi_device_polling_transmit(dev->spi, &t);
}

esp_err_t st7735s_write_cmd(st7735s_handle_t dev, uint8_t cmd)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    uint8_t buf[4] __attribute__((aligned(4))) = { cmd };
    return xfer(dev, false, buf, 1);
}

esp_err_t st7735s_write_data(st7735s_handle_t dev, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(dev && data, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    /* bounce through an aligned buffer: callers may pass flash/unaligned data */
    uint8_t buf[32] __attribute__((aligned(4)));
    while (len) {
        size_t n = len > sizeof(buf) ? sizeof(buf) : len;
        memcpy(buf, data, n);
        esp_err_t err = xfer(dev, true, buf, n);
        if (err != ESP_OK) {
            return err;
        }
        data += n;
        len -= n;
    }
    return ESP_OK;
}

esp_err_t st7735s_write_data16(st7735s_handle_t dev, uint16_t data)
{
    uint8_t buf[2] = { data >> 8, data & 0xFF };
    return st7735s_write_data(dev, buf, sizeof(buf));
}

/* Stream "count" pixels of one colour (port of lcdWriteDataU16 loops) */
static esp_err_t write_color(st7735s_handle_t dev, uint16_t color, uint32_t count)
{
    uint8_t buf[PX_CHUNK * 2] __attribute__((aligned(4)));
    for (size_t i = 0; i < PX_CHUNK * 2; i += 2) {
        buf[i] = color >> 8;
        buf[i + 1] = color & 0xFF;
    }
    while (count) {
        uint32_t n = count > PX_CHUNK ? PX_CHUNK : count;
        esp_err_t err = xfer(dev, true, buf, n * 2);
        if (err != ESP_OK) {
            return err;
        }
        count -= n;
    }
    return ESP_OK;
}

/* Stream "count" pixels from an already big-endian packed buffer */
static esp_err_t write_pixels(st7735s_handle_t dev, const uint8_t *px, uint32_t count)
{
    while (count) {
        uint32_t n = count > PX_CHUNK ? PX_CHUNK : count;
        esp_err_t err = xfer(dev, true, px, n * 2);
        if (err != ESP_OK) {
            return err;
        }
        px += n * 2;
        count -= n;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

esp_err_t st7735s_default_config(st7735s_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    memset(cfg, 0, sizeof(*cfg));
    cfg->spi_host = ST7735S_SPI_HOST;
    cfg->sclk_io = ST7735S_PIN_SCLK;
    cfg->mosi_io = ST7735S_PIN_MOSI;
    cfg->cs_io = ST7735S_PIN_CS;
    cfg->dc_io = ST7735S_PIN_DC;
    cfg->rst_io = ST7735S_PIN_RST;
    cfg->backlight_io = ST7735S_PIN_BACKLIGHT;
    cfg->spi_clock_hz = ST7735S_SPI_CLOCK_HZ;
    return ESP_OK;
}

esp_err_t st7735s_new(const st7735s_config_t *cfg, st7735s_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(cfg && out_handle, ESP_ERR_INVALID_ARG, TAG, "null pointer");

    struct st7735s_t *dev = calloc(1, sizeof(struct st7735s_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no mem");
    dev->dc_io = cfg->dc_io;
    dev->rst_io = cfg->rst_io;
    dev->backlight_io = cfg->backlight_io;
    dev->width = ST7735S_WIDTH;
    dev->height = ST7735S_HEIGHT;

    /* ---- GPIOs (D-CX, optional RST and backlight) ---- */
    uint64_t mask = 1ULL << cfg->dc_io;
    if (cfg->rst_io != GPIO_NUM_NC) {
        mask |= 1ULL << cfg->rst_io;
    }
    if (cfg->backlight_io != GPIO_NUM_NC) {
        mask |= 1ULL << cfg->backlight_io;
    }
    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* ---- SPI bus (official esp_driver_spi) ---- */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = cfg->mosi_io,
        .miso_io_num = -1,
        .sclk_io_num = cfg->sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PX_CHUNK * 2,
    };
    dev->host = cfg->spi_host;
    esp_err_t ret = spi_bus_initialize(cfg->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi bus init failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    dev->bus_inited = true;

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = cfg->spi_clock_hz ? cfg->spi_clock_hz : ST7735S_SPI_CLOCK_HZ,
        .mode = 0,                      /* SCL idles low, sample on rising edge */
        .spics_io_num = cfg->cs_io,
        .queue_size = 4,
    };
    ret = spi_bus_add_device(cfg->spi_host, &dev_cfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi add device failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    /* ---- Hardware reset (reference: high 10 ms, low 10 ms, high 200 ms) ---- */
    if (dev->rst_io != GPIO_NUM_NC) {
        gpio_set_level(dev->rst_io, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(dev->rst_io, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(dev->rst_io, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* ---- Init sequence (lcdInit port) ---- */
    for (size_t i = 0; i < sizeof(s_init_seq) / sizeof(s_init_seq[0]); i++) {
        ESP_GOTO_ON_ERROR(st7735s_write_cmd(dev, s_init_seq[i].cmd), fail, TAG, "write failed");
        if (s_init_seq[i].len) {
            ESP_GOTO_ON_ERROR(st7735s_write_data(dev, s_init_seq[i].data, s_init_seq[i].len),
                              fail, TAG, "write failed");
        }
        if (s_init_seq[i].cmd == 0x11) {
            vTaskDelay(pdMS_TO_TICKS(120));     /* sleep-out settle time */
        }
    }

    if (dev->backlight_io != GPIO_NUM_NC) {
        gpio_set_level(dev->backlight_io, 1);
    }

    *out_handle = dev;
    ESP_LOGI(TAG, "panel ready: %ux%u RGB565, SPI %u Hz",
             dev->width, dev->height, (unsigned)(cfg->spi_clock_hz ? cfg->spi_clock_hz : ST7735S_SPI_CLOCK_HZ));
    return ESP_OK;

fail:
    st7735s_del(dev);
    return ret;
}

esp_err_t st7735s_del(st7735s_handle_t dev)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    if (dev->spi) {
        spi_bus_remove_device(dev->spi);
    }
    if (dev->bus_inited) {
        spi_bus_free(dev->host);
    }
    free(dev);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Window / fills                                                      */
/* ------------------------------------------------------------------ */

esp_err_t st7735s_set_window(st7735s_handle_t dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    uint8_t col[4] = { x1 >> 8, x1 & 0xFF, x2 >> 8, x2 & 0xFF };
    uint8_t row[4] = { y1 >> 8, y1 & 0xFF, y2 >> 8, y2 & 0xFF };
    esp_err_t err = st7735s_write_cmd(dev, 0x2A);       /* column address */
    if (err == ESP_OK) {
        err = st7735s_write_data(dev, col, sizeof(col));
    }
    if (err == ESP_OK) {
        err = st7735s_write_cmd(dev, 0x2B);             /* row address */
    }
    if (err == ESP_OK) {
        err = st7735s_write_data(dev, row, sizeof(row));
    }
    if (err == ESP_OK) {
        err = st7735s_write_cmd(dev, 0x2C);             /* memory write */
    }
    return err;
}

esp_err_t st7735s_fill(st7735s_handle_t dev, uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_FALSE(ex >= sx && ey >= sy, ESP_ERR_INVALID_ARG, TAG, "empty area");
    esp_err_t err = st7735s_set_window(dev, sx, sy, ex, ey);
    if (err != ESP_OK) {
        return err;
    }
    return write_color(dev, color, (uint32_t)(ex - sx + 1) * (ey - sy + 1));
}

esp_err_t st7735s_clear(st7735s_handle_t dev, uint16_t color)
{
    return st7735s_fill(dev, 0, 0, CLEAR_X2, CLEAR_Y2, color);
}

esp_err_t st7735s_draw_bitmap(st7735s_handle_t dev, uint16_t x, uint16_t y,
                              uint16_t w, uint16_t h, const void *pixels_be)
{
    ESP_RETURN_ON_FALSE(dev && pixels_be && w && h, ESP_ERR_INVALID_ARG, TAG, "bad argument");
    esp_err_t err = st7735s_set_window(dev, x, y, x + w - 1, y + h - 1);
    if (err != ESP_OK) {
        return err;
    }
    return write_pixels(dev, pixels_be, (uint32_t)w * h);
}

esp_err_t st7735s_display_on(st7735s_handle_t dev)
{
    return st7735s_write_cmd(dev, 0x29);
}

esp_err_t st7735s_display_off(st7735s_handle_t dev)
{
    return st7735s_write_cmd(dev, 0x28);
}

/* ------------------------------------------------------------------ */
/* Graphics                                                            */
/* ------------------------------------------------------------------ */

esp_err_t st7735s_draw_point(st7735s_handle_t dev, uint16_t x, uint16_t y, uint16_t color)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    esp_err_t err = st7735s_set_window(dev, x, y, x, y);
    if (err != ESP_OK) {
        return err;
    }
    return write_color(dev, color, 1);
}

esp_err_t st7735s_draw_line(st7735s_handle_t dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    /* Bresenham, as in the reference LCD_DrawLine */
    int xerr = 0, yerr = 0;
    int delta_x = (int)x2 - (int)x1;
    int delta_y = (int)y2 - (int)y1;
    int uRow = x1, uCol = y1;

    int incx = (delta_x > 0) - (delta_x < 0);
    int incy = (delta_y > 0) - (delta_y < 0);
    if (delta_x < 0) {
        delta_x = -delta_x;
    }
    if (delta_y < 0) {
        delta_y = -delta_y;
    }
    int distance = (delta_x > delta_y) ? delta_x : delta_y;

    for (int t = 0; t <= distance; t++) {
        esp_err_t err = st7735s_draw_point(dev, uRow, uCol, color);
        if (err != ESP_OK) {
            return err;
        }
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
    return ESP_OK;
}

esp_err_t st7735s_draw_rectangle(st7735s_handle_t dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    esp_err_t err = st7735s_draw_line(dev, x1, y1, x2, y1, color);
    if (err == ESP_OK) {
        err = st7735s_draw_line(dev, x1, y1, x1, y2, color);
    }
    if (err == ESP_OK) {
        err = st7735s_draw_line(dev, x1, y2, x2, y2, color);
    }
    if (err == ESP_OK) {
        err = st7735s_draw_line(dev, x2, y1, x2, y2, color);
    }
    return err;
}

esp_err_t st7735s_draw_circle(st7735s_handle_t dev, uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    /* Bresenham circle, as in the reference LCD_Draw_Circle */
    int a = 0, b = r;
    int di = 3 - (r << 1);
    while (a <= b) {
        const int pts[8][2] = {
            { x0 + a, y0 - b }, { x0 + b, y0 - a }, { x0 + b, y0 + a }, { x0 + a, y0 + b },
            { x0 - a, y0 + b }, { x0 - b, y0 + a }, { x0 - a, y0 - b }, { x0 - b, y0 - a },
        };
        for (int i = 0; i < 8; i++) {
            esp_err_t err = st7735s_draw_point(dev, pts[i][0], pts[i][1], color);
            if (err != ESP_OK) {
                return err;
            }
        }
        a++;
        if (di < 0) {
            di += 4 * a + 6;
        } else {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
    return ESP_OK;
}

esp_err_t st7735s_fill_circle(st7735s_handle_t dev, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    /* Point-in-circle scan (reference LCD_DrawFullCircle, signed loop vars
     * so x0 - r cannot underflow) */
    for (int y = (int)y0 - (int)r; y < (int)y0 + (int)r; y++) {
        for (int x = (int)x0 - (int)r; x < (int)x0 + (int)r; x++) {
            if ((x - (int)x0) * (x - (int)x0) + (y - (int)y0) * (y - (int)y0) <= (int)r * (int)r) {
                esp_err_t err = st7735s_draw_point(dev, x, y, color);
                if (err != ESP_OK) {
                    return err;
                }
            }
        }
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* ASCII text                                                          */
/* ------------------------------------------------------------------ */

static const unsigned char *font_glyph(char ch, uint8_t size)
{
    unsigned num = (unsigned char)ch - ' ';
    switch (size) {
    case 12:
        return asc2_1206[num];
    case 16:
        return asc2_1608[num];
    case 24:
        return asc2_2412[num];
    default:
        return NULL;
    }
}

esp_err_t st7735s_show_char(st7735s_handle_t dev, uint16_t x, uint16_t y, char ch, uint8_t size,
                            bool transparent, uint16_t pen_color, uint16_t back_color)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_FALSE(ch >= ' ' && ch <= '~', ESP_ERR_INVALID_ARG, TAG, "char out of range");

    const unsigned char *glyph = font_glyph(ch, size);
    ESP_RETURN_ON_FALSE(glyph, ESP_ERR_INVALID_ARG, TAG, "size must be 12, 16 or 24");

    unsigned w = size / 2;                          /* glyph width in pixels */
    unsigned bytes_per_col = size / 8 + ((size % 8) ? 1 : 0);

    if (!transparent) {
        /* Decode column-major glyph into a row-major big-endian pixel buffer
         * and stream it in one window write */
        uint8_t px[12 * 24 * 2] __attribute__((aligned(4)));
        for (unsigned row = 0; row < size; row++) {
            for (unsigned col = 0; col < w; col++) {
                uint8_t byte = glyph[col * bytes_per_col + row / 8];
                bool on = byte & (0x80 >> (row % 8));
                uint16_t color = on ? pen_color : back_color;
                size_t i = (row * w + col) * 2;
                px[i] = color >> 8;
                px[i + 1] = color & 0xFF;
            }
        }
        esp_err_t err = st7735s_set_window(dev, x, y, x + w - 1, y + size - 1);
        if (err != ESP_OK) {
            return err;
        }
        return write_pixels(dev, px, w * size);
    }

    /* Transparent: draw pen pixels only (reference "mode 1") */
    for (unsigned row = 0; row < size; row++) {
        for (unsigned col = 0; col < w; col++) {
            uint8_t byte = glyph[col * bytes_per_col + row / 8];
            if (byte & (0x80 >> (row % 8))) {
                esp_err_t err = st7735s_draw_point(dev, x + col, y + row, pen_color);
                if (err != ESP_OK) {
                    return err;
                }
            }
        }
    }
    return ESP_OK;
}

esp_err_t st7735s_show_string(st7735s_handle_t dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                              uint8_t size, const char *str, uint16_t pen_color, uint16_t back_color)
{
    ESP_RETURN_ON_FALSE(dev && str, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    uint16_t x0 = x;
    width += x;
    height += y;
    while (*str && *str >= ' ' && *str <= '~') {
        if (x >= width) {                       /* wrap to the next row */
            x = x0;
            y += size;
        }
        if (y >= height) {
            break;
        }
        esp_err_t err = st7735s_show_char(dev, x, y, *str, size, false, pen_color, back_color);
        if (err != ESP_OK) {
            return err;
        }
        x += size / 2;
        str++;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* GB2312 Chinese text                                                 */
/* ------------------------------------------------------------------ */

/* Decode a row-major glyph bitmap into a big-endian pixel buffer and stream
 * it through one window write (much faster than the reference's per-pixel
 * LCD_Fast_DrawPoint loop) */
static esp_err_t show_glyph(st7735s_handle_t dev, uint16_t x, uint16_t y, const unsigned char *mask,
                            unsigned size, uint16_t pen, uint16_t back)
{
    uint8_t px[32 * 32 * 2] __attribute__((aligned(4)));
    unsigned bytes_per_row = size / 8;
    for (unsigned row = 0; row < size; row++) {
        for (unsigned col = 0; col < size; col++) {
            uint8_t byte = mask[row * bytes_per_row + col / 8];
            bool on = byte & (0x80 >> (col % 8));
            uint16_t color = on ? pen : back;
            size_t i = (row * size + col) * 2;
            px[i] = color >> 8;
            px[i + 1] = color & 0xFF;
        }
    }
    esp_err_t err = st7735s_set_window(dev, x, y, x + size - 1, y + size - 1);
    if (err != ESP_OK) {
        return err;
    }
    return write_pixels(dev, px, size * size);
}

esp_err_t st7735s_show_gb16(st7735s_handle_t dev, uint16_t x, uint16_t y, const uint8_t gb[2],
                            uint16_t pen_color, uint16_t back_color)
{
    ESP_RETURN_ON_FALSE(dev && gb, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    for (unsigned i = 0; i < code_GB_16_count; i++) {
        if (code_GB_16[i].Index[0] == gb[0] && code_GB_16[i].Index[1] == gb[1]) {
            return show_glyph(dev, x, y, code_GB_16[i].Msk, 16, pen_color, back_color);
        }
    }
    ESP_LOGW(TAG, "gb16 glyph %02X %02X not in font table", gb[0], gb[1]);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t st7735s_show_gb16_str(st7735s_handle_t dev, uint16_t x, uint16_t y, const uint8_t *str,
                                uint16_t pen_color, uint16_t back_color)
{
    ESP_RETURN_ON_FALSE(dev && str, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    /* One glyph occupies 16 px = two 8 px slots (reference spacing) */
    unsigned slot = 0;
    while (*str) {
        if (*str >= 0x80) {
            st7735s_show_gb16(dev, x + slot * 8, y, str, pen_color, back_color);
            str += 2;
            slot += 2;
        } else {
            str++;                              /* skip ASCII (reference hung here) */
            slot++;
        }
    }
    return ESP_OK;
}

esp_err_t st7735s_show_gb32(st7735s_handle_t dev, uint16_t x, uint16_t y, const uint8_t gb[2],
                            uint16_t pen_color, uint16_t back_color)
{
    ESP_RETURN_ON_FALSE(dev && gb, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    for (unsigned i = 0; i < code_GB_32_count; i++) {
        if (code_GB_32[i].Index[0] == gb[0] && code_GB_32[i].Index[1] == gb[1]) {
            return show_glyph(dev, x, y, code_GB_32[i].Msk, 32, pen_color, back_color);
        }
    }
    ESP_LOGW(TAG, "gb32 glyph %02X %02X not in font table", gb[0], gb[1]);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t st7735s_show_gb32_str(st7735s_handle_t dev, uint16_t x, uint16_t y, const uint8_t *str,
                                uint16_t pen_color, uint16_t back_color)
{
    ESP_RETURN_ON_FALSE(dev && str, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    /* One glyph occupies 32 px = four 8 px slots (reference spacing) */
    unsigned slot = 0;
    while (*str) {
        if (*str >= 0x80) {
            st7735s_show_gb32(dev, x + slot * 8, y, str, pen_color, back_color);
            str += 2;
            slot += 4;
        } else {
            str++;
            slot += 2;
        }
    }
    return ESP_OK;
}
