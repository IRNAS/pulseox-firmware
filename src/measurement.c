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
#include <string.h>

#define LED_PORT GPIOA
#define LED_PIN1 GPIO4
#define LED_PIN2 GPIO3
#define LED_PIN3 GPIO2

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
uint16_t measurement_read_wavelength(uint8_t led_index);
raw_measurement_t measurement_read();

// LED configuration defaults.
led_config_t led_config[] = {
  // LED2 - IR (940nm).
  {
    .gpio = ((LED_PIN1 << 16) | (LED_PIN2 << 16) | LED_PIN3),
    .duty_on = 5000,
    .duty_wait = 4
  },
  // LED5 - Red (660nm).
  {
    .gpio = (LED_PIN1 | (LED_PIN2 << 16) | (LED_PIN3 << 16)),
    .duty_on = 1000,
    .duty_wait = 4
  },
  // LED3 - Orange (610nm).
  {
    .gpio = ((LED_PIN1 << 16) | LED_PIN2 | LED_PIN3),
    .duty_on = 1000,
    .duty_wait = 4
  },
  // LED4 - Yellow (590nm)
  {
    .gpio = (LED_PIN1 | LED_PIN2 | (LED_PIN3 << 16)),
    .duty_on = 1000,
    .duty_wait = 4
  }
};

// Indices into the above LED configuration array.
const uint8_t LED_IR = 0;
const uint8_t LED_RED = 1;
const uint8_t LED_ORANGE = 2;
const uint8_t LED_YELLOW = 3;

// Filters.
dc_filter_t dc_filter_ir = {0.0, 0.0};
dc_filter_t dc_filter_red = {0.0, 0.0};
mean_diff_filter_t mean_diff_ir = { .index = 0, .sum = 0.0, .count = 0 };
mean_diff_filter_t rolling_mean_ir = { .index = 0, .sum = 0.0, .count = 0 };

// Pulse detection state.
enum {
  PULSE_IDLE = 0,
  PULSE_RISING = 1,
  PULSE_FALLING = 2,
};

uint8_t pulse_state = PULSE_IDLE;
uint32_t pulse_current_timestamp = 0;
uint32_t pulse_last_timestamp = 0;
uint8_t pulse_beats = 0;
uint8_t pulse_present = 0;
float pulse_previous_value = 0.0;
float pulse_current_bpm = 0.0;
const float PULSE_THRESHOLD = 10.0;
const float PULSE_RESET_THRESHOLD = 500.0;

// Sampling frequency.
uint32_t last_measurement = 0;
uint32_t last_report = 0;
uint32_t sample_count = 0;
uint32_t measurement_delay = 0;

// Current measuremnt.
measurement_t current_measurement;

// Measurement callback.
measurement_update_callback_t callback_on_update = NULL;

void measurement_init(measurement_update_callback_t on_update)
{
  callback_on_update = on_update;

  memset(&current_measurement, 0, sizeof(current_measurement));

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

uint16_t measurement_read_wavelength(uint8_t led_index)
{
  led_config_t *config = &led_config[led_index];

  volatile int duty_on = config->duty_on;
  volatile int duty_wait = config->duty_wait;

  led_turn_on(config->gpio);
  led_wait(duty_on);
  led_turn_off();
  led_wait(duty_wait);

  uint16_t result = detector_read();
  clock_usleep(DETECTOR_TIMINGS_FALL_TIME);

  // Reduce duty cycle when detector is saturated.
  /*if (result > 3800 && config->duty_on >= 1) {
    config->duty_on--;
  } else if (result < 1000 && config->duty_on < 10) {
    config->duty_on++;
  }*/

  return result;
}

raw_measurement_t measurement_read()
{
  raw_measurement_t result;

  // TODO: Read other wavelengths.
  //result.orange = measurement_read_wavelength(LED_ORANGE);
  //result.yellow = measurement_read_wavelength(LED_YELLOW);

  result.ir = measurement_read_wavelength(LED_IR);
  result.red = measurement_read_wavelength(LED_RED);
  // TODO: Also read ambient noise to subtract from the signals.

  return result;
}

dc_filter_t measurement_dc_removal(float x, float prev_w, float alpha)
{
  dc_filter_t filter;
  filter.w = x + alpha * prev_w;
  filter.result = filter.w - prev_w;

  return filter;
}

float measurement_mean_diff(float value, mean_diff_filter_t *filter, int diff)
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
  if (diff) {
    // Difference from mean.
    return avg - value;
  } else {
    // Rolling mean.
    return avg;
  }
}

int measurement_detect_pulse(float value)
{
  if (value >= PULSE_RESET_THRESHOLD) {
    pulse_state = PULSE_IDLE;
    pulse_current_timestamp = 0;
    pulse_last_timestamp = 0;
    pulse_previous_value = 0;
    pulse_current_bpm = 0.0;
    pulse_beats = 0;
    pulse_present = 0;
    return 0;
  }

  // If no pulse detected for some time, reset.
  if (pulse_beats > 0 && clock_millis() - pulse_last_timestamp > 5000) {
    pulse_current_bpm = 0.0;
    pulse_beats = 0;
    pulse_present = 0;
  }

  switch (pulse_state) {
    case PULSE_IDLE: {
      // Idle state: we wait for the value to cross the threshold.
      if (value >= PULSE_THRESHOLD) {
        pulse_state = PULSE_RISING;
      }
      break;
    }
    case PULSE_RISING: {
      if (value > pulse_previous_value) {
        // Still rising.
        pulse_current_timestamp = clock_millis();
      } else {
        // Reached the peak.
        uint32_t beat_duration = pulse_current_timestamp - pulse_last_timestamp;
        pulse_last_timestamp = pulse_current_timestamp;

        // Compute BPM.
        float raw_bpm = 60000.0 / (float) beat_duration;
        // TODO: Moving average for BPM.
        if (raw_bpm > 10.0 && raw_bpm < 300.0) {
          pulse_beats++;

          if (pulse_beats > 3) {
            pulse_current_bpm = raw_bpm;
            pulse_beats = 3;
            pulse_present = 1;
          }
        }

        pulse_state = PULSE_FALLING;
        return 1;
      }
      break;
    }
    case PULSE_FALLING: {
      // Move into idle state when under the threshold.
      if (value < PULSE_THRESHOLD) {
        pulse_state = PULSE_IDLE;
      }
      break;
    }
  }

  pulse_previous_value = value;
  return 0;
}

void measurement_update()
{
  uint32_t now = clock_millis();

  if (now - last_measurement > measurement_delay) {
    raw_measurement_t raw = measurement_read();

    // IR.
    dc_filter_ir = measurement_dc_removal((float) raw.ir, dc_filter_ir.w, DC_FILTER_ALPHA);
    float mean_ir = measurement_mean_diff(dc_filter_ir.result, &mean_diff_ir, 1);
    mean_ir = measurement_mean_diff(mean_ir, &rolling_mean_ir, 0);

    // Red.
    dc_filter_red = measurement_dc_removal((float) raw.red, dc_filter_red.w, DC_FILTER_ALPHA);

    // Perform pulse detection.
    measurement_detect_pulse(mean_ir);

    // Compute derived measurements.
    if (pulse_present) {
      current_measurement.hr = (int) pulse_current_bpm;
    } else {
      // No pulse present, SpO2 should be ignored.
      current_measurement.hr = 0;
      current_measurement.spo2 = 0;
      current_measurement.waveform_hr = 0;
      current_measurement.waveform_hr_max = 100;
      current_measurement.waveform_spo2 = 0;
      current_measurement.waveform_spo2_max = 100;
    }

    // Notify subscribers.
    if (callback_on_update != NULL) {
      callback_on_update(&current_measurement);
    }

    last_measurement = now;
    sample_count++;

    // Output measurements.
    uart_printf("%d,%u,%d,%d\r\n",
      (int) now,
      raw.ir,
      (int) (dc_filter_ir.result * 100),
      (int) (mean_ir * 100)
    );
  }

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
    sample_count = 0;
  }
}
