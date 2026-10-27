/*
 * camera_pins.h - OV2640 DVP wiring for ESP32-S3
 *
 * Pin assignment is centralized here - adjust it to match your wiring.
 *
 * All GPIOs below are safe on plain ESP32-S3 modules (DevKitC-1 etc.):
 * they avoid the flash pins (26-32), optional PSRAM pins (33-37),
 * strap pins (0, 3, 45, 46) and the USB pins (19, 20).
 */
#pragma once

/* ---- SCCB (I2C) ---- */
#define CAM_PIN_SIOD            (16)    /* sensor I2C data  (SIOD / SDA) */
#define CAM_PIN_SIOC            (17)    /* sensor I2C clock (SIOC / SCL) */
#define CAM_I2C_PORT            (I2C_NUM_0)
#define CAM_SCCB_FREQ_HZ        (100000)

/* ---- Sensor control ---- */
#define CAM_PIN_XCLK            (15)    /* 20 MHz master clock to the sensor */
#define CAM_PIN_PWDN            (18)    /* power-down, active high; GPIO_NUM_NC if tied low */
#define CAM_PIN_RESET           (21)    /* reset, active low;           GPIO_NUM_NC if tied high */

/* ---- DVP 8-bit data bus ---- */
#define CAM_PIN_D0              (4)
#define CAM_PIN_D1              (5)
#define CAM_PIN_D2              (6)
#define CAM_PIN_D3              (7)
#define CAM_PIN_D4              (8)
#define CAM_PIN_D5              (9)
#define CAM_PIN_D6              (10)
#define CAM_PIN_D7              (11)

/* ---- DVP sync signals ---- */
#define CAM_PIN_PCLK            (12)    /* pixel clock from the sensor */
#define CAM_PIN_VSYNC           (13)    /* vertical sync / frame valid */
#define CAM_PIN_HREF            (14)    /* horizontal reference / data enable */

/* ---- Clock ---- */
#define CAM_XCLK_FREQ_HZ        (20000000)
