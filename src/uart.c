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

// UART initialization flag.
uint8_t uart_initialized = 0;

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
  uart_initialized = 1;
}

void uart_putc(char character)
{
  if (!uart_initialized) {
    return;
  }

  usart_send_blocking(USART1, character);
}

void uart_puts(char *text)
{
  for (;;) {
    char c = *text++;
    if (!c) break;
    uart_putc(c);
  }
}

void uart_puti(int number)
{
  char buf[8 * sizeof(int) + 1];
  char *str = &buf[sizeof(buf) - 1];
  int base = 10;

  if (number < 0) {
    uart_putc('-');
    number = -number;
  }

  *str = '\0';
  do {
    int m = number;
    number /= base;
    char c = m - base * number;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(number);

  uart_puts(str);
}