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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

// Number of milliseconds since boot.
static volatile uint32_t system_millis;

void sys_tick_handler()
{
  system_millis++;
}

void clock_init()
{
  // Set STM32 clock to 48 MHz from HSI oscillator.
  rcc_clock_setup_in_hsi_out_48mhz();

  // Setup timer to fire each 48000 clocks to get 1ms interrupts.
  systick_set_reload(48000);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();

  // Enable interrupt to call our sys_tick_handler.
  systick_interrupt_enable();
}

void clock_msleep(uint32_t delay)
{
  uint32_t wake = system_millis + delay;
  while (wake > system_millis);
}

void clock_usleep(uint32_t delay)
{
  for (int i = 0; i < delay * 48; i++) {
    __asm__("nop");
  }
}

uint32_t clock_millis()
{
  return system_millis;
}
