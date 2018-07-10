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
#include "adc.h"
#include "uart.h"
#include "battery.h"
#include "measurement.h"
#include "lcd.h"
#include "gfx.h"

#ifndef PULSEOX_DEBUG
#include "gui.h"
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <stdlib.h>

#define DEBUG_BUTTON_PORT GPIOF
#define DEBUG_U6 GPIO1

#ifdef PULSEOX_DEBUG
void debug_init()
{
  rcc_periph_clock_enable(RCC_GPIOF);

  // Setup GPIOs.
  gpio_mode_setup(DEBUG_BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, DEBUG_U6);
}
#endif

/**
 * Entry point.
 */
int main()
{
  // Initialize subsystems.
  clock_init();
  adc_init();
  battery_init();
  lcd_init();
  gfx_init(lcd_draw_pixel, LCD_WIDTH, LCD_HEIGHT);
#ifdef PULSEOX_DEBUG
  measurement_init(NULL);
#else
  gui_init(LCD_WIDTH, LCD_HEIGHT);
  measurement_init(gui_measurement_update);
#endif

#ifdef PULSEOX_DEBUG
  debug_init();

  // Display UART console enable message.
  gfx_setCursor(0, 0);
  gfx_setTextSize(1);
  gfx_setTextColor(0x80, 0x00);
  gfx_puts("GliaX debug\n===========\nPress U6 for\nconsole.");
  lcd_refresh();

  uint8_t console_enabled = 0;
  uint32_t start = clock_millis();
  for (;;) {
    // Check if U6 is low.
    if (!gpio_get(DEBUG_BUTTON_PORT, DEBUG_U6)) {
      console_enabled = 1;
      break;
    }

    // Continue startup after 5 seconds.
    if (clock_millis() - start > 5000) {
      break;
    }
  }

  // Initialize UART when console requested.
  if (console_enabled) {
    gfx_fillScreen(0x00);
    gfx_setCursor(0, 0);
    gfx_puts("Debug console\nenabled.");
    lcd_refresh();
    clock_msleep(2000);

    uart_init();
    uart_puts("Debug console enabled.\r\n");
  } else {
    gfx_fillScreen(0x00);
    gfx_setCursor(0, 0);
    gfx_puts("Debug console\nNOT enabled.");
  }
#endif

  for (;;) {
    // Update measurement buffer.
    measurement_update();

    // Render GUI.
#ifndef PULSEOX_DEBUG
    gui_render();
#endif

    // Refresh LCD.
    lcd_refresh();

    // Update battery status.
    battery_update();
  }

  return 0;
}
