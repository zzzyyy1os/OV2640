/*
 * ov2640.h - OV2640 camera sensor driver (SCCB / register configuration)
 *
 * Modular ESP-IDF port of the OV2640 driver from the STM32 reference project
 * (STM32-OV2640-master, SimpleMethod, MIT). The STM32 HAL I2C calls are
 * replaced with the official esp_driver_i2c (i2c_master) driver, and the
 * DCMI capture logic lives in the separate "camera" component.
 *
 * Prerequisite: the sensor XCLK must already be running when ov2640_new()
 * is called (the camera component starts it before SCCB init).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** OV2640 SCCB 7-bit slave address (0x60 write / 0x61 read in the reference) */
#define OV2640_SCCB_ADDR        0x30
/** Expected chip product ID (register 0x0A, sensor bank) */
#define OV2640_PID_VALUE        0x26
/** Expected chip version (register 0x0B, sensor bank) */
#define OV2640_VER_VALUE        0x42

typedef struct ov2640_t *ov2640_handle_t;

/** Special effects, same order/values as the reference enum */
typedef enum {
    OV2640_EFFECT_ANTIQUE = 0,
    OV2640_EFFECT_BLUISH = 1,
    OV2640_EFFECT_GREENISH = 2,
    OV2640_EFFECT_REDDISH = 3,
    OV2640_EFFECT_BLACKWHITE = 4,
    OV2640_EFFECT_NEGATIVE = 5,
    OV2640_EFFECT_NEGATIVE_BLACKWHITE = 6,
    OV2640_EFFECT_NORMAL = 7,
} ov2640_effect_t;

/** JPEG output resolutions, same order as the reference tables */
typedef enum {
    OV2640_RES_160X120 = 0,
    OV2640_RES_320X240 = 1,
    OV2640_RES_640X480 = 2,
    OV2640_RES_800X600 = 3,
    OV2640_RES_1024X768 = 4,
    OV2640_RES_1280X960 = 5,
} ov2640_res_t;

/** Light mode / white balance presets */
typedef enum {
    OV2640_LIGHT_AUTO = 0,
    OV2640_LIGHT_SUNNY = 1,
    OV2640_LIGHT_CLOUDY = 2,
    OV2640_LIGHT_OFFICE = 3,
    OV2640_LIGHT_HOME = 4,
} ov2640_light_mode_t;

/** Sensor wiring configuration */
typedef struct {
    i2c_port_num_t i2c_port;    ///< I2C port used for SCCB
    int sda_io_num;             ///< SCCB SIOD (data) GPIO
    int scl_io_num;             ///< SCCB SIOC (clock) GPIO
    uint32_t sccb_freq_hz;      ///< SCCB clock, 0 -> 100 kHz
    gpio_num_t pwdn_io;         ///< Power-down GPIO (active high), GPIO_NUM_NC if not wired
    gpio_num_t reset_io;        ///< Reset GPIO (active low),  GPIO_NUM_NC if not wired
} ov2640_config_t;

/**
 * @brief Create sensor instance: power/reset sequence, SCCB bus setup, soft reset
 *
 * The reference init sequence is kept: PWDN low, hardware reset pulse,
 * SCCB soft reset (bank 1, reg 0x12 = 0x80) and PID/VER identification.
 *
 * @note XCLK must be running before calling this function.
 */
esp_err_t ov2640_new(const ov2640_config_t *config, ov2640_handle_t *ret_handle);

/** Release the sensor instance (SCCB bus + GPIOs) */
esp_err_t ov2640_del(ov2640_handle_t dev);

/** Read one sensor register (SCCB_Read port). Bank is whatever the tables selected. */
esp_err_t ov2640_read_reg(ov2640_handle_t dev, uint8_t reg, uint8_t *pdata);

/** Write one sensor register (SCCB_Write port) */
esp_err_t ov2640_write_reg(ov2640_handle_t dev, uint8_t reg, uint8_t data);

/**
 * @brief Apply a {reg, val} table terminated by {0xff, 0xff} (OV2640_Configuration port)
 *
 * Every write is read-back verified; mismatches are logged but do not abort
 * (some registers do not read back exactly, same behaviour as the reference).
 */
esp_err_t ov2640_write_reg_list(ov2640_handle_t dev, const uint8_t arr[][2]);

/** Read chip PID/VER (switches to sensor bank 1 first) */
esp_err_t ov2640_read_id(ov2640_handle_t dev, uint8_t *pid, uint8_t *ver);

/**
 * @brief Configure the JPEG pipeline and output resolution
 * (OV2640_ResolutionConfiguration port: JPEG_INIT + YUV422 + JPEG + resolution table)
 */
esp_err_t ov2640_set_jpeg_resolution(ov2640_handle_t dev, ov2640_res_t res);
esp_err_t ov2640_set_yuv422_resolution(ov2640_handle_t dev, ov2640_res_t res);

/** Get the pixel width/height of a JPEG resolution option */
esp_err_t ov2640_get_jpeg_resolution_size(ov2640_res_t res, uint32_t *width, uint32_t *height);

/** Image controls (port of the reference effect/adjust functions) */
esp_err_t ov2640_set_effect(ov2640_handle_t dev, ov2640_effect_t effect);
esp_err_t ov2640_set_contrast(ov2640_handle_t dev, int level);    ///< level: -2 .. +2
esp_err_t ov2640_set_saturation(ov2640_handle_t dev, int level);  ///< level: -2 .. +2
esp_err_t ov2640_set_brightness(ov2640_handle_t dev, int level);  ///< level: -2 .. +2
esp_err_t ov2640_set_light_mode(ov2640_handle_t dev, ov2640_light_mode_t mode);

/**
 * @brief White balance selection (port of the reference functions)
 * @param simple true  -> simple WB   (OV2640_SimpleWhiteBalance,   reg 0xc7 = 0x10)
 *               false -> advanced WB (OV2640_AdvancedWhiteBalance, reg 0xc7 = 0x00)
 */
esp_err_t ov2640_set_white_balance(ov2640_handle_t dev, bool simple);

#ifdef __cplusplus
}
#endif
