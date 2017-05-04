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
#include "uart.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#define _GNU_SOURCE
#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <sys/types.h>

static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);

// Global UART handle.
static FILE *uart_handle = NULL;

void uart_init()
{
  // Enable clocks for UART communication.
  rcc_periph_clock_enable(RCC_USART1);
  rcc_periph_clock_enable(RCC_GPIOA);

  // Setup alternate function USART1_TX for PA14.
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO14);

  // Setup UART parameters.
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  // Enable UART.
  usart_enable(USART1);

  // Setup a custom I/O stream for the UART.
  cookie_io_functions_t stub = { _iord, _iowr, NULL, NULL };
  uart_handle = fopencookie(NULL, "rw+", stub);
  /* Do not buffer the serial line */
  setvbuf(uart_handle, NULL, _IONBF, 0);
}

FILE *uart_stream()
{
  return uart_handle;
}

static ssize_t _iord(void *_cookie, char *_buf, size_t _n)
{
  // TODO: Implement reading from UART.
  (void) _cookie;
  (void) _buf;
  (void) _n;
  return 0;
}

static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n)
{
  (void) _cookie;

  int written = 0;
  while (_n-- > 0) {
    usart_send_blocking(USART1, *_buf++);
    written++;
  };
  return written;
}
