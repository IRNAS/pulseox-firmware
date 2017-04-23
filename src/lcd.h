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
#ifndef PULSEOX_LCD_H
#define PULSEOX_LCD_H

#include <stdint.h>

#define LCD_WIDTH 128
#define LCD_HEIGHT 64
#define LCD_PAGE_SIZE 8

// LCD control protocol commands.
#define CMD_DISPLAY_OFF 0xAE
#define CMD_DISPLAY_ON 0xAF
#define CMD_SET_DISP_START_LINE 0x40
#define CMD_SET_PAGE 0xB0
#define CMD_SET_COLUMN_UPPER 0x10
#define CMD_SET_COLUMN_LOWER 0x00
#define CMD_SET_ADC_NORMAL 0xA0
#define CMD_SET_ADC_REVERSE 0xA1
#define CMD_SET_DISP_NORMAL 0xA6
#define CMD_SET_DISP_REVERSE 0xA7
#define CMD_SET_ALLPTS_NORMAL 0xA4
#define CMD_SET_ALLPTS_ON 0xA5
#define CMD_SET_BIAS_9 0xA2
#define CMD_SET_BIAS_7 0xA3
#define CMD_RMW 0xE0
#define CMD_RMW_CLEAR 0xEE
#define CMD_INTERNAL_RESET 0xE2
#define CMD_SET_COM_NORMAL 0xC0
#define CMD_SET_COM_REVERSE 0xC8
#define CMD_SET_POWER_CONTROL 0x28
#define CMD_SET_RESISTOR_RATIO 0x20
#define CMD_SET_VOLUME_FIRST 0x81
#define CMD_SET_VOLUME_SECOND 0
#define CMD_SET_STATIC_OFF 0xAC
#define CMD_SET_STATIC_ON 0xAD
#define CMD_SET_STATIC_REG 0x0
#define CMD_SET_BOOSTER_FIRST 0xF8
#define CMD_SET_BOOSTER_234 0
#define CMD_SET_BOOSTER_5 1
#define CMD_SET_BOOSTER_6 3
#define CMD_NOP 0xE3
#define CMD_TEST 0xF0

// LCD message types.
#define MESSAGE_COMMAND 0x00
#define MESSAGE_DATA 0x01

/**
 * Setup LCD peripheral.
 */
void lcd_init();

/**
 * Bit-banged 8-bit MSB-first SPI write.
 *
 * @param value 8-bit value to send
 * @param type Message type (either MESSAGE_COMMAND or MESSAGE_DATA)
 */
void lcd_write(uint8_t value, uint8_t type);

/**
 * Transmit command message to LCD.
 *
 * @param command Command to transmit
 */
void lcd_command(uint8_t command);

/**
 * Transmit data message to LCD.
 *
 * @param command Data to transmit
 */
void lcd_data(uint8_t data);

/**
 * Set LCD brightness.
 *
 * @param brightness Brightness value
 */
void lcd_set_brightness(uint8_t val);

/**
 * Draw a pixel on the screen.
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param color Pixel color
 */
void lcd_draw_pixel(int x, int y, uint16_t color);

/**
 * Refresh the LCD screen based on last buffer updates.
 */
void lcd_refresh();

#endif
