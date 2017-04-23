/*
 * This is the core graphics library for all our displays, providing a common
 * set of graphics primitives (points, lines, circles, etc.).  It needs to be
 * paired with a hardware-specific library for each display device we carry
 * (to handle the lower-level functions).
 *
 * Adafruit invests time and resources providing this open source code, please
 * support Adafruit & open-source hardware by purchasing products from Adafruit!
 *
 * Copyright (c) 2013 Adafruit Industries.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Modified the AdaFruit library to be a C library, changed the font and
 * generally munged it in a variety of ways, creating a reasonably quick
 * and dirty way to put something "interesting" on the LCD display.
 * --Chuck McManis (2013, 2014)
 *
 */
#ifndef PULSEOX_GFX_H
#define PULSEOX_GFX_H

#include <stdint.h>

#define swap(a, b) { int16_t t = a; a = b; b = t; }

void gfx_drawPixel(int x, int y, uint16_t color);
void gfx_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
      uint16_t color);
void gfx_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void gfx_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void gfx_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void gfx_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void gfx_fillScreen(uint16_t color);

void gfx_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void gfx_drawCircleHelper(int16_t x0, int16_t y0, int16_t r,
        uint8_t cornername, uint16_t color);
void gfx_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void gfx_init(void (*draw)(int, int, uint16_t), int, int);

void gfx_fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
        uint8_t cornername, int16_t delta, uint16_t color);
void gfx_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
          int16_t x2, int16_t y2, uint16_t color);
void gfx_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
          int16_t x2, int16_t y2, uint16_t color);
void gfx_drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
           int16_t radius, uint16_t color);
void gfx_fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
           int16_t radius, uint16_t color);
void gfx_drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap,
        int16_t w, int16_t h, uint16_t color);
void gfx_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color,
      uint16_t bg, uint8_t size);
void gfx_setCursor(int16_t x, int16_t y);
void gfx_setTextColor(uint16_t c, uint16_t bg);
void gfx_setTextSize(uint8_t s);
void gfx_setTextWrap(uint8_t w);
void gfx_setRotation(uint8_t r);
void gfx_puts(char *);
void gfx_write(uint8_t);

uint16_t gfx_height(void);
uint16_t gfx_width(void);

uint8_t gfx_getRotation(void);

#define GFX_WIDTH   128
#define GFX_HEIGHT  64

struct gfx_state {
  int16_t _width, _height, cursor_x, cursor_y;
  uint16_t textcolor, textbgcolor;
  uint8_t textsize, rotation;
  uint8_t wrap;
  void (*drawpixel)(int, int, uint16_t);
};

extern struct gfx_state __gfx_state;

#endif
