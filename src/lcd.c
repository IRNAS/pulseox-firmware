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
#include "lcd.h"
#include "clock.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

// LCD GPIO definitions.
#define LCD_PORT GPIOA
#define LCD_SCLK GPIO5
#define LCD_A0 GPIO6
#define LCD_MOSI GPIO7
#define LCD_RST GPIO9
#define LCD_CS GPIO10

// Screen buffer.
uint8_t lcd_screen_buffer[LCD_WIDTH * (LCD_HEIGHT / LCD_PAGE_SIZE)];
// Needs refresh.
uint8_t lcd_dirty = 0;

void lcd_init()
{
  rcc_periph_clock_enable(RCC_GPIOA);

  // Setup GPIOs.
  gpio_mode_setup(LCD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_SCLK);
  gpio_mode_setup(LCD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_MOSI);
  gpio_mode_setup(LCD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_A0);
  gpio_mode_setup(LCD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_CS);
  gpio_mode_setup(LCD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_RST);

  // Reset LCD.
  gpio_clear(LCD_PORT, LCD_RST);
  clock_msleep(500);
  gpio_set(LCD_PORT, LCD_RST);

  // LCD initialization sequence.
  lcd_command(CMD_SET_BIAS_9);
  lcd_command(CMD_SET_ADC_NORMAL);
  lcd_command(CMD_SET_COM_REVERSE);
  lcd_command(CMD_SET_DISP_NORMAL);
  lcd_command(CMD_RMW_CLEAR);
  lcd_command(CMD_SET_POWER_CONTROL | 0x07);
  lcd_command(CMD_SET_RESISTOR_RATIO | 0x05);
  lcd_command(CMD_SET_ALLPTS_NORMAL);
  lcd_set_brightness(0x18);
  lcd_command(CMD_SET_DISP_START_LINE);
  lcd_command(CMD_DISPLAY_ON);

  // Clear display.
  for (int y = 0; y < LCD_HEIGHT / LCD_PAGE_SIZE; y++) {
    for (int x = 0; x < LCD_WIDTH; x++) {
      lcd_screen_buffer[y * LCD_WIDTH + x] = 0;
    }
  }
  lcd_refresh();
}

void lcd_write(uint8_t value, uint8_t type)
{
  if (type == MESSAGE_COMMAND) {
    gpio_clear(LCD_PORT, LCD_A0);
  } else {
    gpio_set(LCD_PORT, LCD_A0);
  }

  gpio_clear(LCD_PORT, LCD_SCLK);
  gpio_clear(LCD_PORT, LCD_CS);

  // Send data bit by bit, most significant bit first.
  for(int x = 0; x < 8; x++){
    gpio_clear(LCD_PORT, LCD_SCLK);

    if (value & 0x80) {
      gpio_set(LCD_PORT, LCD_MOSI);
    } else {
      gpio_clear(LCD_PORT, LCD_MOSI);
    }

    gpio_set(LCD_PORT, LCD_SCLK);
    value <<= 1;
  }

  gpio_set(LCD_PORT, LCD_CS);
}

void lcd_command(uint8_t command)
{
  lcd_write(command, MESSAGE_COMMAND);
}

void lcd_data(uint8_t data)
{
  lcd_write(data, MESSAGE_DATA);
}

void lcd_set_brightness(uint8_t brightness)
{
  lcd_command(CMD_SET_VOLUME_FIRST);
  lcd_command(CMD_SET_VOLUME_SECOND | (brightness & 0x3F));
}

void lcd_draw_pixel(int x, int y, uint16_t color)
{
  if (x > LCD_WIDTH || y > LCD_HEIGHT) {
    return;
  }

  // Compute offset within buffer.
  uint16_t offset = (LCD_WIDTH - 1 - x) + ( (LCD_HEIGHT - 1 - y ) / LCD_PAGE_SIZE) * LCD_WIDTH;

  if (color) {
    lcd_screen_buffer[offset] |= 1 << ((LCD_HEIGHT - 1 - y ) % 8);
  } else {
    lcd_screen_buffer[offset] &= 0xFF ^ 1 << ((LCD_HEIGHT - 1 - y ) % 8);
  }

  lcd_dirty = 1;
}

void lcd_refresh()
{
  if (!lcd_dirty) {
    return;
  }
  lcd_dirty = 0;

  for (int y = 0; y < LCD_HEIGHT / LCD_PAGE_SIZE; y++) {
    // TODO: Only refresh dirty pages.

    lcd_command(CMD_SET_PAGE | y);
    lcd_command(CMD_SET_COLUMN_LOWER);
    lcd_command(CMD_SET_COLUMN_UPPER);

    for (int x = 0; x < LCD_WIDTH; x++) {
      lcd_data(lcd_screen_buffer[y * LCD_WIDTH + x]);
    }
  }
}
