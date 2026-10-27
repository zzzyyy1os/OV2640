/*
 * camera.h - OV2640 DVP capture module for ESP32-S3
 *
 * Glues together:
 *   - the official esp_driver_cam (esp_cam_ctlr DVP / LCD_CAM) capture engine
 *   - the modular ov2640 sensor driver (SCCB + register tables ported from
 *     the STM32 reference project)
 *
 * Output format is either a JPEG stream (the driver's frame-done path scans
 * the SOI/EOI markers and reports the actual JPEG length) or raw YUV422
 * pixels - see camera_output_t. Frames arrive via camera_grab_frame().
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "soc/gpio_num.h"
#include "ov2640.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Sensor output format */
typedef enum {
    CAMERA_OUTPUT_JPEG,     ///< compressed JPEG stream (SOI..EOI per frame)
    CAMERA_OUTPUT_YUV422,   ///< raw YUV422 pixels (2 bytes per pixel)
} camera_output_t;

/** Camera wiring / behaviour configuration */
typedef struct {
    /* SCCB + power */
    i2c_port_num_t i2c_port;    ///< I2C port for SCCB
    int sda_io_num;             ///< SCCB SIOD GPIO
    int scl_io_num;             ///< SCCB SIOC GPIO
    gpio_num_t pwdn_io;         ///< Power-down GPIO (active high), GPIO_NUM_NC if not wired
    gpio_num_t reset_io;        ///< Reset GPIO (active low),  GPIO_NUM_NC if not wired

    /* DVP bus */
    gpio_num_t xclk_io;         ///< Master clock output to the sensor; GPIO_NUM_NC if the
                                ///< module has its own crystal (external XTAL mode)
    gpio_num_t pclk_io;         ///< Pixel clock input from the sensor
    gpio_num_t vsync_io;        ///< VSYNC input
    gpio_num_t href_io;         ///< HREF / DE input
    gpio_num_t data_io[8];      ///< DVP data lines D0..D7

    /* Behaviour */
    uint32_t xclk_freq_hz;      ///< XCLK frequency (0 -> 20 MHz)
    camera_output_t output;     ///< JPEG stream or raw YUV422
    ov2640_res_t resolution;    ///< sensor output resolution
    size_t frame_buffer_count;      ///< 1 or 2 (2 recommended, 0 -> 2)
} camera_config_t;

/**
 * @brief Fill config with the default wiring from camera_pins.h
 */
esp_err_t camera_default_config(camera_config_t *cfg);

/**
 * @brief Initialize XCLK, DVP capture engine and the OV2640 sensor
 *
 * Call this once before grabbing. Sensor init (SCCB) runs after the DVP
 * controller is created so that XCLK is already running.
 */
esp_err_t camera_init(const camera_config_t *cfg);

/**
 * @brief Wait for the next complete frame
 *
 * @param[out] out_buf  Pointer to the frame data inside an internal frame
 *                      buffer: the JPEG stream (SOI..EOI) in JPEG mode, or the
 *                      raw YUV422 pixel data in YUV422 mode. Valid until the
 *                      buffer is reused by the driver - copy the data before
 *                      the next grab if needed.
 * @param[out] out_len   Frame length in bytes
 * @param[in]  timeout_ms  Max wait for a frame
 */
esp_err_t camera_grab_frame(const uint8_t **out_buf, size_t *out_len, uint32_t timeout_ms);

/**
 * @brief Access the sensor handle for image controls
 * (ov2640_set_brightness / contrast / effect / light_mode ...)
 */
ov2640_handle_t camera_get_sensor(void);

/** Stop capture and release all resources */
esp_err_t camera_deinit(void);

#ifdef __cplusplus
}
#endif
