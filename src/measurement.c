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
#include "uart.h"

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

// Timings in microseconds.
#define DETECTOR_TIMINGS_RISE_TIME 10
#define DETECTOR_TIMINGS_FALL_TIME 10

#define ADC_CHANNEL0 0x00

void detector_power_on();
void detector_power_off();
uint16_t detector_read();
void led_turn_off();
void led_turn_on(uint32_t led);
void led_wait(int clocks);
uint16_t measurement_read_wavelength(uint32_t led);
raw_measurement_t measurement_read();

// LED timings.
static volatile int timing_led_ir_on = 3;
static volatile int timing_led_ir_wait = 4;

// Filters.
static dc_filter_t dc_filter_ir = {0.0, 0.0};
static mean_diff_filter_t mean_diff_ir = { .index = 0, .sum = 0.0, .count = 0 };

// Sampling frequency.
static uint32_t last_report = 0;
static uint32_t sample_count = 0;
static uint32_t measurement_delay = 0;

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

  detector_power_on();
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

// Ensure this is inlined for timings to be correct.
__attribute__((always_inline)) inline void led_turn_off()
{
  GPIO_BSRR(LED_PORT) = ((LED_PIN1 | LED_PIN2 | LED_PIN3) << 16);
}

// Ensure this is inlined for timings to be correct.
__attribute__((always_inline)) inline void led_turn_on(uint32_t led)
{
  GPIO_BSRR(LED_PORT) = led;
}

// Ensure this is inlined for timings to be correct.
__attribute__((always_inline)) inline void led_wait(volatile int clocks)
{
  for (; clocks; --clocks) {
    // No operation here to keep this as tight as possible.
  }
}

uint16_t measurement_read_wavelength(uint32_t led)
{
  led_turn_on(led);
  led_wait(timing_led_ir_on);
  led_turn_off();
  led_wait(timing_led_ir_wait);

  uint16_t result = detector_read();
  clock_usleep(DETECTOR_TIMINGS_FALL_TIME);

  // Reduce timings when detector is saturated.
  if (result > 3800 && timing_led_ir_on >= 1) {
    timing_led_ir_on--;
  } else if (result < 1000 && timing_led_ir_on < 10) {
    timing_led_ir_on++;
  }

  return result;
}

raw_measurement_t measurement_read()
{
  raw_measurement_t result;

  // TODO: Read other wavelengths.
  // result.red = measurement_read_wavelength(LED_RED);
  // result.orange = measurement_read_wavelength(LED_ORANGE);
  // result.yellow = measurement_read_wavelength(LED_YELLOW);
  result.ir = measurement_read_wavelength(LED_IR);

  return result;
}

dc_filter_t measurement_dc_removal(float x, float prev_w, float alpha)
{
  dc_filter_t filter;
  filter.w = x + alpha * prev_w;
  filter.result = filter.w - prev_w;

  return filter;
}

float measurement_mean_diff(float value, mean_diff_filter_t *filter)
{
  float avg = 0;

  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];

  filter->index++;
  filter->index = filter->index % MEAN_FILTER_SIZE;

  if (filter->count < MEAN_FILTER_SIZE) {
    filter->count++;
  }

  avg = filter->sum / filter->count;
  return avg - value;
}

void measurement_update()
{
  raw_measurement_t raw = measurement_read();

  dc_filter_ir = measurement_dc_removal((float) raw.ir, dc_filter_ir.w, DC_FILTER_ALPHA);
  float mean_ir = measurement_mean_diff(dc_filter_ir.result, &mean_diff_ir);
  clock_msleep(measurement_delay);

  uint32_t now = clock_millis();
  sample_count++;
  if (now - last_report > 1000) {
    // Adjust measurement delay to achieve a sampling rate of around 100Hz.
    if (sample_count > 110) {
      // Increase delay.
      measurement_delay += 1;
    } else if (sample_count < 90 && measurement_delay >= 1) {
      // Decrease delay.
      measurement_delay -= 1;
    }
    last_report = now;
  }

  // Output measurements.
  uart_printf("%d,%u,%d,%d\r\n",
    (int) now,
    raw.ir,
    (int) (dc_filter_ir.result * 100),
    (int) (mean_ir * 100)
  );
}
