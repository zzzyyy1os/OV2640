/*
 * ov2640_regs.h - OV2640 register configuration tables
 *
 * Ported from the STM32 reference project (STM32-OV2640-master, SimpleMethod):
 *   ov2640.c / ov2640.h  ->  Src/ov2640.c (Copyright 2021 SimpleMethod, MIT License)
 *
 * Each table is a list of {register, value} pairs, terminated by the
 * OV2640_REG_TABLE_END sentinel {0xff, 0xff}.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** End-of-table sentinel: {0xff, 0xff} */
#define OV2640_REG_TABLE_END    { 0xff, 0xff }

/* ---- Basic JPEG pipeline (must be applied first) ---- */
extern const uint8_t OV2640_JPEG_INIT[][2];     /* full sensor + DSP init      */
extern const uint8_t OV2640_YUV422[][2];        /* select YUV422 output        */
extern const uint8_t OV2640_JPEG[][2];          /* enable JPEG encoder output  */
extern const uint8_t OV2640_YUV422_RAW[][2];    /* raw YUV422 on the DVP bus
                                                 * (official esp_cam_sensor;
                                                 * IMAGE_MODE 0x00, not 0x10)  */

/* ---- JPEG output window / resolution ---- */
extern const uint8_t OV2640_160x120_JPEG[][2];
extern const uint8_t OV2640_320x240_JPEG[][2];
extern const uint8_t OV2640_640x480_JPEG[][2];
extern const uint8_t OV2640_800x600_JPEG[][2];
extern const uint8_t OV2640_1024x768_JPEG[][2];
extern const uint8_t OV2640_1280x960_JPEG[][2];

/* ---- Image adjustments (level -2 .. +2) ---- */
extern const uint8_t OV2640_CONTRAST2[][2];     /* +2 */
extern const uint8_t OV2640_CONTRAST1[][2];     /* +1 */
extern const uint8_t OV2640_CONTRAST0[][2];     /*  0 */
extern const uint8_t OV2640_CONTRAST_1[][2];    /* -1 */
extern const uint8_t OV2640_CONTRAST_2[][2];    /* -2 */

extern const uint8_t OV2640_SATURATION2[][2];   /* +2 */
extern const uint8_t OV2640_SATURATION1[][2];   /* +1 */
extern const uint8_t OV2640_SATURATION0[][2];   /*  0 */
extern const uint8_t OV2640_SATURATION_1[][2];  /* -1 */
extern const uint8_t OV2640_SATURATION_2[][2];  /* -2 */

extern const uint8_t OV2640_BRIGHTNESS2[][2];   /* +2 */
extern const uint8_t OV2640_BRIGHTNESS1[][2];   /* +1 */
extern const uint8_t OV2640_BRIGHTNESS0[][2];   /*  0 */
extern const uint8_t OV2640_BRIGHTNESS_1[][2];  /* -1 */
extern const uint8_t OV2640_BRIGHTNESS_2[][2];  /* -2 */

/* ---- Special effects ---- */
extern const uint8_t OV2640_SPECIAL_EFFECTS_ANTIQUE[][2];
extern const uint8_t OV2640_SPECIAL_EFFECTS_BLUISH[][2];
extern const uint8_t OV2640_SPECIAL_EFFECTS_GREENISH[][2];
extern const uint8_t OV2640_SPECIAL_EFFECTS_REDDISH[][2];
extern const uint8_t OV2640_SPECIAL_EFFECTS_BLACK[][2];           /* black & white */
extern const uint8_t OV2640_SPECIAL_EFFECTS_NEGATIVE[][2];
extern const uint8_t OV2640_SPECIAL_EFFECTS_BLACK_NEGATIVE[][2];  /* negative b&w  */
extern const uint8_t OV2640_SPECIAL_EFFECTS_NORMAL[][2];

/* ---- Light mode / white balance presets ---- */
extern const uint8_t OV2640_LIGHT_MODE_AUTO[][2];
extern const uint8_t OV2640_LIGHT_MODE_SUNNY[][2];
extern const uint8_t OV2640_LIGHT_MODE_CLOUDY[][2];
extern const uint8_t OV2640_LIGHT_MODE_OFFICE[][2];
extern const uint8_t OV2640_LIGHT_MODE_HOME[][2];

#ifdef __cplusplus
}
#endif
