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
#include "adc.h"
#include "gfx.h"
#include "uart.h"
#include "dsp.h"
#include "spo2.h"
#include "qfplib/qfplib.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>

#define LED_PORT GPIOA
#define LED_PIN1 GPIO4
#define LED_PIN2 GPIO3
#define LED_PIN3 GPIO2

#define DETECTOR_PORT GPIOA
#define DETECTOR_PIN_ANALOG_IN GPIO0
// XXX: Due to a recent change in the PCB, the detector is always powered.
// #define DETECTOR_PIN_VDD GPIO1

// Timings in microseconds.
#define DETECTOR_TIMINGS_RISE_TIME 10
#define DETECTOR_TIMINGS_FALL_TIME 10

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
  // LED4 - Yellow (590nm).
  {
    .gpio = (LED_PIN1 | LED_PIN2 | (LED_PIN3 << 16)),
    .duty_on = 1000,
    .duty_wait = 4
  },
  // Ambient light LED (e.g., all LEDs off).
  {
    .gpio = ((LED_PIN1 | LED_PIN2 | LED_PIN3) << 16),
    .duty_on = 1000,
    .duty_wait = 4
  }
};

// Indices into the above LED configuration array.
const uint8_t LED_IR = 0;
const uint8_t LED_RED = 1;
const uint8_t LED_ORANGE = 2;
const uint8_t LED_YELLOW = 3;
const uint8_t LED_AMBIENT = 4;

// Filters.
dc_filter_t dc_filter_ir = { 0.0, 0.0 };
dc_filter_t dc_filter_red = { 0.0, 0.0 };
mean_filter_t mean_diff_ir = { .index = 0, .sum = 0.0, .count = 0 };
mean_filter_t rolling_mean_ir = { .index = 0, .sum = 0.0, .count = 0 };
mean_filter_t rolling_mean_pulse = { .index = 0, .sum = 0.0, .count = 0 };
butterworth_filter_t butt_filter_ir = { .v = {0., 0., 0.} };
butterworth_filter_t butt_filter_spo2_ir = { .v = {0., 0., 0.} };
butterworth_filter_t butt_filter_spo2_red = { .v = {0., 0., 0.} };

// Pulse detection state.
enum {
  PULSE_IDLE = 0,
  PULSE_FALLING = 1,
  PULSE_RISING = 2,
};

uint8_t pulse_state = PULSE_IDLE;
uint32_t pulse_current_timestamp = 0;
uint32_t pulse_last_timestamp = 0;
uint8_t pulse_beats = 0;
uint8_t pulse_present = 0;
float pulse_previous_value = 0.0;
float pulse_current_bpm = 0.0;

// Red/IR ratio calculation.
float ac_sqsum_ir = 0.0;
float ac_sqsum_red = 0.0;
uint16_t spo2_samples = 0;
uint16_t spo2_beats = 0;

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

  rcc_periph_clock_enable(RCC_GPIOA);

  // Setup LED GPIOs.
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN1);
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN2);
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN3);
  gpio_clear(LED_PORT, LED_PIN1 | LED_PIN2 | LED_PIN3);

  // Setup detector GPIOs.
  gpio_mode_setup(DETECTOR_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, DETECTOR_PIN_ANALOG_IN);
  // XXX: Due to a recent change in the PCB, the detector is always powered.
  // gpio_mode_setup(DETECTOR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DETECTOR_PIN_VDD);
  // gpio_clear(DETECTOR_PORT, DETECTOR_PIN_VDD);

  detector_power_on();
}

void detector_power_on()
{
  // XXX: Due to a recent change in the PCB, the detector is always powered.
  // gpio_set(DETECTOR_PORT, DETECTOR_PIN_VDD);
  clock_usleep(DETECTOR_TIMINGS_RISE_TIME);
}

void detector_power_off()
{
  // XXX: Due to a recent change in the PCB, the detector is always powered.
  // gpio_clear(DETECTOR_PORT, DETECTOR_PIN_VDD);
  clock_usleep(DETECTOR_TIMINGS_FALL_TIME);
}

uint16_t detector_read()
{
  return adc_read(ADC_CHANNEL_MEASUREMENT);
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

  // TODO: Automatically adjust LED duty cycle.

  return result;
}

raw_measurement_t measurement_read()
{
  raw_measurement_t result;

  result.ir = measurement_read_wavelength(LED_IR);
  result.ambient = measurement_read_wavelength(LED_AMBIENT);
  result.red = measurement_read_wavelength(LED_RED);

#ifdef PULSEOX_DEBUG
  // TODO: Read other wavelengths.
  result.orange = measurement_read_wavelength(LED_ORANGE);
  result.yellow = measurement_read_wavelength(LED_YELLOW);
#endif

  return result;
}

int measurement_detect_pulse(uint16_t raw, float value)
{
  if (value >= PULSE_RESET_THRESHOLD || raw >= MEASUREMENT_THRESHOLD) {
    pulse_state = PULSE_IDLE;
    pulse_current_timestamp = 0;
    pulse_last_timestamp = 0;
    pulse_previous_value = 0;
    pulse_current_bpm = 0.0;
    pulse_beats = 0;
    pulse_present = 0;
    memset(&rolling_mean_pulse, 0, sizeof(rolling_mean_pulse));
    return 0;
  }

  // If no pulse detected for some time, reset.
  if (pulse_beats > 0 && clock_millis() - pulse_last_timestamp > PULSE_RESET_TIMEOUT) {
    pulse_current_bpm = 0.0;
    pulse_beats = 0;
    pulse_present = 0;
    memset(&rolling_mean_pulse, 0, sizeof(rolling_mean_pulse));
  }

  switch (pulse_state) {
    case PULSE_IDLE: {
      // Idle state: we wait for the value to cross the threshold.
      if (value <= PULSE_THRESHOLD) {
        pulse_state = PULSE_FALLING;
      }
      break;
    }
    case PULSE_FALLING: {
      if (value < pulse_previous_value) {
        // Still falling.
        pulse_current_timestamp = clock_millis();
      } else {
        // Reached the bottom.
        uint32_t beat_duration = pulse_current_timestamp - pulse_last_timestamp;
        pulse_last_timestamp = pulse_current_timestamp;

        // Compute BPM.
        float raw_bpm = 60000.0 / (float) beat_duration;
        if (raw_bpm > 10.0 && raw_bpm < 300.0) {
          pulse_beats++;
          float bpm = filter_mean(&rolling_mean_pulse, raw_bpm, 0);

          if (pulse_beats > PULSE_INITIAL_BEATS) {
            pulse_current_bpm = bpm;
            pulse_beats = PULSE_INITIAL_BEATS;
            pulse_present = 1;
          }
        }

        pulse_state = PULSE_RISING;
        return 1;
      }
      break;
    }
    case PULSE_RISING: {
      // Move into idle state when under the threshold.
      if (value > PULSE_THRESHOLD) {
        pulse_state = PULSE_IDLE;
      }
      break;
    }
  }

  pulse_previous_value = value;
  return 0;
}

#ifdef PULSEOX_BOARD_DIAGNOSTIC
#include "lcd.h"

void measurement_diagnostic(uint8_t led_index)
{
  led_config_t *config = &led_config[led_index];

  led_turn_on(config->gpio);
  clock_msleep(1000);
  led_turn_off();
}

void measurement_update()
{
  gfx_setTextSize(1);
  gfx_setTextColor(0x80, 0x00);

  gfx_fillScreen(0x00);
  gfx_setCursor(0, 0);
  gfx_puts("IR ON");
  lcd_refresh();
  measurement_diagnostic(LED_IR);

  gfx_fillScreen(0x00);
  gfx_setCursor(0, 0);
  gfx_puts("RED ON");
  lcd_refresh();
  measurement_diagnostic(LED_RED);

  gfx_fillScreen(0x00);
  gfx_setCursor(0, 0);
  gfx_puts("YELLOW ON");
  lcd_refresh();
  measurement_diagnostic(LED_YELLOW);

  gfx_fillScreen(0x00);
  gfx_setCursor(0, 0);
  gfx_puts("ORANGE ON");
  lcd_refresh();
  measurement_diagnostic(LED_ORANGE);
}
#else
void measurement_update()
{
  uint32_t now = clock_millis();

  if (now - last_measurement > measurement_delay) {
    raw_measurement_t raw = measurement_read();

    // IR.
    float dc_ir = filter_dc(&dc_filter_ir, (float) raw.ir, DC_FILTER_ALPHA);
    float mean_ir = filter_mean(&mean_diff_ir, dc_ir, 1);
    mean_ir = filter_mean(&rolling_mean_ir, mean_ir, 0);
    float butt_ir = filter_butterworth_lp(&butt_filter_ir, dc_ir);

    // Red.
    float dc_red = filter_dc(&dc_filter_red, (float) raw.red, DC_FILTER_ALPHA);

    // Normalize IR and red AC by their DC components.
    float norm_ir = dc_ir / dc_filter_ir.w;
    float norm_red = dc_red / dc_filter_red.w;

    float butt_norm_ir = filter_butterworth_lp(&butt_filter_spo2_ir, norm_ir);
    float butt_norm_red = filter_butterworth_lp(&butt_filter_spo2_red, norm_red);

    ac_sqsum_ir += butt_norm_ir * butt_norm_ir;
    ac_sqsum_red += butt_norm_red * butt_norm_red;
    spo2_samples++;

    // Perform pulse detection.
    if (measurement_detect_pulse(raw.ir, butt_ir)) {
      spo2_beats++;
    }

    // Compute derived measurements.
    static float ratio = 0.0;
    if (pulse_present) {
      current_measurement.hr = (int) pulse_current_bpm;

      if (spo2_beats >= SPO2_UPDATE_BEATS) {
        ratio = qfp_fsqrt(ac_sqsum_red / ac_sqsum_ir);
        current_measurement.spo2 = spo2_lookup(ratio);

        // Reset readings.
        ac_sqsum_ir = 0.0;
        ac_sqsum_red = 0.0;
        spo2_samples = 0;
        spo2_beats = 0;
      }
    } else {
      // No pulse present, SpO2 should be ignored.
      current_measurement.hr = 0;
      current_measurement.spo2 = 0;
    }

    // Provide data for the waveform.
    // TODO: Determine minimum and maximum based on some last samples?
    current_measurement.waveform_spo2_min = SPO2_WAVEFORM_MIN;
    current_measurement.waveform_spo2_max = SPO2_WAVEFORM_MAX;
    current_measurement.waveform_spo2 = mean_ir;
    if (current_measurement.waveform_spo2 < current_measurement.waveform_spo2_min) {
      current_measurement.waveform_spo2 = current_measurement.waveform_spo2_min;
    }
    if (current_measurement.waveform_spo2 > current_measurement.waveform_spo2_max) {
      current_measurement.waveform_spo2 = current_measurement.waveform_spo2_max;
    }

    // Notify subscribers.
    if (callback_on_update != NULL) {
      callback_on_update(&current_measurement);
    }

    last_measurement = now;
    sample_count++;

#ifdef PULSEOX_DEBUG
    // Output measurements.
    uart_puti((int) now);
    uart_putc(',');
    uart_puti(raw.ir);
    uart_putc(',');
    uart_puti((int) (dc_ir * 100));
    uart_putc(',');
    uart_puti((int) (mean_ir * 100));
    uart_putc(',');
    uart_puti((int) (butt_ir * 100));
    uart_putc(',');
    uart_puti((int) (dc_red * 100));
    uart_putc(',');
    uart_puti(raw.orange);
    uart_putc(',');
    uart_puti(raw.yellow);
    uart_putc(',');
    uart_puti((int) (norm_ir * 100000));
    uart_putc(',');
    uart_puti((int) (norm_red * 100000));
    uart_putc(',');
    uart_puti((int) (butt_norm_ir * 100000));
    uart_putc(',');
    uart_puti((int) (butt_norm_red * 100000));
    uart_putc(',');
    uart_puti((int) (ratio * 100));
    uart_putc(',');
    uart_puti(raw.ambient);
    uart_putc(',');
    uart_puti(raw.red);
    uart_puts("\r\n");
#endif
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
#endif
