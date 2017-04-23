/*
 * Pulse oximeter firmware
 *
 * Copyright (C) 2017 Jernej Kos <jernej@kos.mx>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Affero General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "clock.h"
#include "uart.h"
#include "measurement.h"
#include "lcd.h"
#include "gfx.h"

/**
 * Entry point.
 */
int main()
{
  // Initialize subsystems.
  clock_init();
  // uart_init();
  measurement_init();
  lcd_init();
  gfx_init(lcd_draw_pixel, LCD_WIDTH, LCD_HEIGHT);

  // Draw some demo graphics.
  gfx_drawLine(0, 0, 128, 64, 0x80);
  gfx_drawLine(0, 64, 128, 0, 0x80);
  gfx_setTextSize(2);
  gfx_setTextColor(0x80, 0x00);
  gfx_setCursor(15, 25);
  gfx_puts("GliaX");

  for (;;) {
    // Update measurement buffer.
    measurement_update();

    // Refresh LCD.
    lcd_refresh();
  }

  return 0;
}
