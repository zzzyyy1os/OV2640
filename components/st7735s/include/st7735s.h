/*
 * st7735s.h - ST7735S 1.8" 128x160 RGB TFT driver (SPI) for ESP-IDF
 *
 * Modular port of the st7735s-tft-main reference project (STM32F103,
 * bit-banged SPI): transport on the official esp_driver_spi / esp_driver_gpio,
 * same init sequence, RGB565 pixel format and drawing/text API.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ST7735S_WIDTH   128
#define ST7735S_HEIGHT  160

/* RGB565 colors (port of LCD_ST7735S.h) */
#define ST7735S_WHITE       0xFFFF
#define ST7735S_BLACK       0x0000
#define ST7735S_BLUE        0x001F
#define ST7735S_BRED        0xF81F
#define ST7735S_GRED        0xFFE0
#define ST7735S_GBLUE       0x07FF
#define ST7735S_RED         0xF800
#define ST7735S_MAGENTA     0xF81F
#define ST7735S_GREEN       0x07E0
#define ST7735S_CYAN        0x7FFF
#define ST7735S_YELLOW      0xFFE0
#define ST7735S_BROWN       0xBC40
#define ST7735S_BRRED       0xFC07
#define ST7735S_GRAY        0x8430
#define ST7735S_DARKBLUE    0x01CF
#define ST7735S_LIGHTBLUE   0x7D7C
#define ST7735S_GRAYBLUE    0x5458

typedef struct st7735s_t *st7735s_handle_t;

typedef struct {
    spi_host_device_t spi_host;     /* SPI2_HOST by default              */
    gpio_num_t sclk_io;
    gpio_num_t mosi_io;             /* SDA on the module                 */
    gpio_num_t cs_io;               /* GPIO_NUM_NC to tie CS low         */
    gpio_num_t dc_io;               /* A0 / D-CX                         */
    gpio_num_t rst_io;              /* GPIO_NUM_NC to skip the hw reset  */
    gpio_num_t backlight_io;        /* GPIO_NUM_NC = fixed backlight     */
    uint32_t spi_clock_hz;          /* 20 MHz by default                 */
} st7735s_config_t;

/* Fill cfg with the pins from st7735s_pins.h */
esp_err_t st7735s_default_config(st7735s_config_t *cfg);

/* Create the display: SPI bus + panel init sequence + backlight on */
esp_err_t st7735s_new(const st7735s_config_t *cfg, st7735s_handle_t *out_handle);
esp_err_t st7735s_del(st7735s_handle_t dev);

/* Low-level access (lcdSelectRegister / lcdWriteDataU8 / lcdWriteDataU16 port) */
esp_err_t st7735s_write_cmd(st7735s_handle_t dev, uint8_t cmd);
esp_err_t st7735s_write_data(st7735s_handle_t dev, const uint8_t *data, size_t len);
esp_err_t st7735s_write_data16(st7735s_handle_t dev, uint16_t data);

/* Set the active window and arm the GRAM write cursor (lcdSetAddress port) */
esp_err_t st7735s_set_window(st7735s_handle_t dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

esp_err_t st7735s_display_on(st7735s_handle_t dev);
esp_err_t st7735s_display_off(st7735s_handle_t dev);

/* Fills (lcdClear / LCD_Fill port) */
esp_err_t st7735s_clear(st7735s_handle_t dev, uint16_t color);
esp_err_t st7735s_fill(st7735s_handle_t dev, uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);

/* Stream a w*h RGB565 bitmap to (x, y). Pixels must be packed big-endian
 * (high byte first, as on the wire). The buffer has to live in internal
 * RAM and be 4-byte aligned (DMA is used behind the scenes). */
esp_err_t st7735s_draw_bitmap(st7735s_handle_t dev, uint16_t x, uint16_t y,
                              uint16_t w, uint16_t h, const void *pixels_be);

/* Graphics (LCD_DrawPoint / LCD_DrawLine / LCD_DrawRectangle /
 * LCD_Draw_Circle / LCD_DrawFullCircle port) */
esp_err_t st7735s_draw_point(st7735s_handle_t dev, uint16_t x, uint16_t y, uint16_t color);
esp_err_t st7735s_draw_line(st7735s_handle_t dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
esp_err_t st7735s_draw_rectangle(st7735s_handle_t dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
esp_err_t st7735s_draw_circle(st7735s_handle_t dev, uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
esp_err_t st7735s_fill_circle(st7735s_handle_t dev, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);

/* ASCII text (LCD_ShowChar / LCD_ShowString port).
 * size: 12, 16 or 24 (glyph height; width is size/2).
 * transparent: true = only draw the pen pixels (mode 1),
 *              false = paint pen+background (mode 0). */
esp_err_t st7735s_show_char(st7735s_handle_t dev, uint16_t x, uint16_t y, char ch, uint8_t size,
                            bool transparent, uint16_t pen_color, uint16_t back_color);
esp_err_t st7735s_show_string(st7735s_handle_t dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                              uint8_t size, const char *str, uint16_t pen_color, uint16_t back_color);

/* GB2312 Chinese text (GUI_sprintf_hz port). The reference font tables
 * only contain a handful of glyphs - unknown codes return ESP_ERR_NOT_FOUND.
 * The _str variants walk a GB2312 encoded string (>= 0x80 starts a glyph). */
esp_err_t st7735s_show_gb16(st7735s_handle_t dev, uint16_t x, uint16_t y, const uint8_t gb[2],
                            uint16_t pen_color, uint16_t back_color);
esp_err_t st7735s_show_gb16_str(st7735s_handle_t dev, uint16_t x, uint16_t y, const uint8_t *str,
                                uint16_t pen_color, uint16_t back_color);
esp_err_t st7735s_show_gb32(st7735s_handle_t dev, uint16_t x, uint16_t y, const uint8_t gb[2],
                            uint16_t pen_color, uint16_t back_color);
esp_err_t st7735s_show_gb32_str(st7735s_handle_t dev, uint16_t x, uint16_t y, const uint8_t *str,
                                uint16_t pen_color, uint16_t back_color);

#ifdef __cplusplus
}
#endif
