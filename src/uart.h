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
#ifndef PULSEOX_UART_H
#define PULSEOX_UART_H

/**
 * Initialize UART. This disables the programming port and prevents
 * the firmware from being updated, so it should be initialized only
 * conditionally.
 */
void uart_init();

/**
 * Writes a single character to UART.
 */
void uart_putc(char character);

/**
 * Writes a NULL-terminated string to UART.
 */
void uart_puts(char *text);

/**
 * Writes an integer to UART.
 */
void uart_puti(int number);

#endif
