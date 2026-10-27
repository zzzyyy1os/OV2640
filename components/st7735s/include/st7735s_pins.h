/*
 * st7735s_pins.h - ST7735S TFT wiring for the ESP32-S3 target
 *
 * The camera already occupies GPIO 4..18 and 21 (see camera_pins.h) and
 * the UART0 console uses GPIO 43/44, so the display lives on GPIO 38..42.
 * Any GPIOs work (SPI signals go through the GPIO matrix); change freely.
 */
#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define ST7735S_SPI_HOST        SPI2_HOST
#define ST7735S_SPI_CLOCK_HZ    (20000000)

#define ST7735S_PIN_SCLK        GPIO_NUM_41
#define ST7735S_PIN_MOSI        GPIO_NUM_42
#define ST7735S_PIN_CS          GPIO_NUM_40
#define ST7735S_PIN_DC          GPIO_NUM_39    /* A0 / D-CX on the module  */
#define ST7735S_PIN_RST         GPIO_NUM_38
#define ST7735S_PIN_BACKLIGHT   GPIO_NUM_NC    /* module has a fixed backlight */
