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
#include "measurement.h"
#include "clock.h"
#include "gfx.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#define _GNU_SOURCE
#include <stdio.h>

#define LED_PORT GPIOA
#define LED_PIN1 GPIO4
#define LED_PIN2 GPIO3
#define LED_PIN3 GPIO2

// LED5 - Red (660nm).
#define LED_RED (LED_PIN1 | (LED_PIN2 << 16) | (LED_PIN3 << 16))
// LED3 - Orange (610nm).
#define LED_ORANGE ((LED_PIN1 << 16) | LED_PIN2 | LED_PIN3)
// LED2 - IR (940nm).
#define LED_IR ((LED_PIN1 << 16) | (LED_PIN2 << 16) | LED_PIN3)
// LED4 - Yellow (590nm).
#define LED_YELLOW (LED_PIN1 | LED_PIN2 | (LED_PIN3 << 16))

#define DETECTOR_PORT GPIOA
#define DETECTOR_PIN_ANALOG_IN GPIO0
#define DETECTOR_PIN_VDD GPIO1

#define DETECTOR_TIMINGS_RISE_TIME 10
#define DETECTOR_TIMINGS_FALL_TIME 10
#define DETECTOR_TIMINGS_STABILIZE 10

#define ADC_CHANNEL0 0x00

void detector_power_on();
void detector_power_off();
uint16_t detector_read();
void led_turn_off();
void led_turn_on(uint32_t led);
uint16_t measurement_read_wavelength(uint32_t led);

void measurement_init()
{
  rcc_periph_clock_enable(RCC_ADC);
  rcc_periph_clock_enable(RCC_GPIOA);

  // Setup LED GPIOs.
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN1);
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN2);
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN3);
  gpio_clear(LED_PORT, LED_PIN1 | LED_PIN2 | LED_PIN3);

  // Setup detector GPIOs.
  gpio_mode_setup(DETECTOR_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, DETECTOR_PIN_ANALOG_IN);
  gpio_mode_setup(DETECTOR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DETECTOR_PIN_VDD);
  gpio_clear(DETECTOR_PORT, DETECTOR_PIN_VDD);

  // Setup ADC for detector.
  uint8_t adc_channel_array[] = { ADC_CHANNEL0 };

  adc_power_off(ADC1);
  adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
  adc_calibrate(ADC1);
  adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_041DOT5);
  adc_set_regular_sequence(ADC1, 1, adc_channel_array);
  adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
  adc_disable_analog_watchdog(ADC1);
  adc_power_on(ADC1);

  // Wait for ADC to start up.
  clock_msleep(100);
}

void detector_power_on()
{
  gpio_set(DETECTOR_PORT, DETECTOR_PIN_VDD);
  clock_usleep(DETECTOR_TIMINGS_RISE_TIME);
}

void detector_power_off()
{
  gpio_clear(DETECTOR_PORT, DETECTOR_PIN_VDD);
  clock_usleep(DETECTOR_TIMINGS_FALL_TIME);
}

uint16_t detector_read()
{
  adc_start_conversion_regular(ADC1);
  while (!(adc_eoc(ADC1)));
  return adc_read_regular(ADC1);
}

void led_turn_off()
{
  gpio_clear(LED_PORT, LED_PIN1 | LED_PIN2 | LED_PIN3);
}

void led_turn_on(uint32_t led)
{
  GPIO_BSRR(LED_PORT) = led;
}

uint16_t measurement_read_wavelength(uint32_t led)
{
  uint16_t result;
  led_turn_on(led);
  clock_usleep(DETECTOR_TIMINGS_STABILIZE);
  result = detector_read();
  led_turn_off();
  clock_usleep(DETECTOR_TIMINGS_STABILIZE);
  return result;
}

void measurement_update()
{
  uint16_t wavelengths[4] = {0,};
  char buffer[64] = {0,};

  detector_power_on();
  wavelengths[0] = measurement_read_wavelength(LED_RED);
  wavelengths[1] = measurement_read_wavelength(LED_ORANGE);
  wavelengths[2] = measurement_read_wavelength(LED_YELLOW);
  wavelengths[3] = measurement_read_wavelength(LED_IR);
  detector_power_off();

  // Output measurements.
  snprintf(buffer, sizeof(buffer), "Red: %u\nOrange: %u\nYellow: %u\nIR: %u",
    wavelengths[0], wavelengths[1], wavelengths[2], wavelengths[3]);
  gfx_fillScreen(0x00);
  gfx_setTextSize(1);
  gfx_setTextColor(0x80, 0x00);
  gfx_setCursor(0, 0);
  gfx_puts(buffer);

  clock_msleep(1000);
}
