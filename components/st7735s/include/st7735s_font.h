/*
 * st7735s_font.h - font table declarations for the ST7735S display module
 *
 * Tables are ported from the st7735s-tft-main reference project
 * (code/font/font.h, hz_16x16.h, hz_32x32.h).
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ASCII glyphs " "-"~" (PC2LCD2002, column-major bitmaps).
 * The digit in the name is the glyph height; width is size / 2. */
extern const unsigned char asc2_1206[95][12];   /* 6x12  */
extern const unsigned char asc2_1608[95][16];   /* 8x16  */
extern const unsigned char asc2_2412[95][36];   /* 12x24 */

/* GB2312 Chinese glyphs (row-major bitmaps, MSB = leftmost pixel).
 * The reference font files only carry the few glyphs its demo needs;
 * the *_count variables bound the tables (the reference hard-coded
 * oversized loop counts and read past the arrays). */
typedef struct {
    unsigned char Index[2];     /* GB2312 code of the glyph */
    unsigned char Msk[32];      /* 16x16 bitmap: 2 bytes per row */
} typFNT_GB16;

typedef struct {
    unsigned char Index[2];     /* GB2312 code of the glyph */
    unsigned char Msk[128];     /* 32x32 bitmap: 4 bytes per row */
} typFNT_GB32;

extern const typFNT_GB16 code_GB_16[];
extern const unsigned int code_GB_16_count;
extern const typFNT_GB32 code_GB_32[];
extern const unsigned int code_GB_32_count;

#ifdef __cplusplus
}
#endif
