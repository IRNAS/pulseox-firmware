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
#define LED_PIN1 GPIO2
#define LED_PIN2 GPIO3
#define LED_PIN3 GPIO4

#define DETECTOR_PORT GPIOA
#define DETECTOR_PIN_ANALOG_IN GPIO0

// Timings in microseconds.
#define DETECTOR_TIMINGS_RISE_TIME 10
#define DETECTOR_TIMINGS_FALL_TIME 10

// defined in and max brightness values
#define MIN_LED_IR 40
#define MIN_LED_RED 250
#define MAX_LED_IR 1500
#define MAX_LED_RED 11000

uint16_t detector_read();
void led_turn_off();
void led_turn_on(uint32_t led);
void led_wait(int clocks);
uint16_t measurement_read_wavelength(uint8_t led_index);
raw_measurement_t measurement_read();
void sqi_ir_loop();
void sqi_red_loop();
float calculate_mean_sqi(uint8_t led);
void empty_amp_buffer(uint8_t led);
void empty_std_buffer(uint8_t led);
void empty_sqi_buffer(uint8_t led);

// LED configuration defaults.
led_config_t led_config[] = {
  // LED2 - IR (940nm).
  {
    .gpio = ((LED_PIN1 << 16) | (LED_PIN2 << 16) | LED_PIN3),
    .duty_on = IR_DEFAULT,
    .duty_wait = 4
  },
  // LED5 - Red (660nm).
  {
    .gpio = (LED_PIN1 | (LED_PIN2 << 16) | (LED_PIN3 << 16)),
    .duty_on = RED_DEFAULT,
    .duty_wait = 4
  },
  // LED3 - Orange (610nm).
  {
    .gpio = ((LED_PIN1 << 16) | LED_PIN2 | LED_PIN3),
    .duty_on = 50,
    .duty_wait = 4
  },
  // LED4 - Yellow (590nm).
  {
    .gpio = (LED_PIN1 | LED_PIN2 | (LED_PIN3 << 16)),
    .duty_on = 50,
    .duty_wait = 4
  },
  // Ambient light LED (e.g., all LEDs off).
  {
    .gpio = ((LED_PIN1 | LED_PIN2 | LED_PIN3) << 16),
    .duty_on = 500,
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
butterworth_filter_t butt_filter_red = { .v = {0., 0., 0.} };
butterworth_filter_t butt_filter_spo2_ir = { .v = {0., 0., 0.} };
butterworth_filter_t butt_filter_spo2_red = { .v = {0., 0., 0.} };
butterworth_filter_t butt_filter_ir_h = { .v = {0., 0., 0.} };
butterworth_filter_t butt_filter_red_h = { .v = {0., 0., 0.} };
butterworth_filter_t butt_filter_ir_ambient = { .v = {0., 0., 0.} };
butterworth_filter_t butt_filter_red_ambient = { .v = {0., 0., 0.} };

// Pulse detection state.
enum {
  PULSE_IDLE = 0,
  PULSE_FALLING = 1,
  PULSE_RISING = 2,
};
// IR pulse variables
uint8_t pulse_state = PULSE_IDLE;
uint32_t pulse_current_timestamp = 0;
uint32_t pulse_last_timestamp = 0;
uint8_t pulse_beats = 0;
uint8_t pulse_present = 0;
bool ir_peaks_present = false;
float pulse_previous_value = 0.0;
float pulse_current_bpm = 0.0;

// RED peak detector variables
uint8_t pulse_state_red = PULSE_IDLE;
uint32_t red_current_timestamp = 0;
uint32_t red_last_timestamp = 0;
float red_previous_value = 0.0;
bool red_peaks_present = false;

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

// SQI variables
uint8_t cur_amp_element_ir = 0;
uint8_t cur_amp_element_red = 0;
uint8_t cur_std_element_ir = 0;
uint8_t cur_std_element_red = 0;
uint8_t cur_sqi_element_ir = 0;
uint8_t cur_sqi_element_red = 0;
float sqi_ir = 0.0;
float sqi_red = 0.0;
float mean_sqi_ir = 0.0;
float mean_sqi_red = 0.0;
bool std_ir_buf_full = false;
bool std_red_buff_full = false;
bool plus_step_ir = false;
bool minus_step_red = false;
bool ir_buff_ready = false;
bool red_buff_ready = false;
uint32_t last_time = 0;
uint32_t sqi_ir_last_timestamp = 0;
uint32_t sqi_ir_cur_timestamp = 0;
uint32_t sqi_red_last_timestamp = 0;
uint32_t sqi_red_cur_timestamp = 0;
uint32_t test_ir = 0;
uint32_t test_red = 0;

// Low and high borders
uint16_t clip_bright_ir = MAX_LED_IR;     // IR max brightness value
uint16_t clip_bright_red = MAX_LED_RED;   // RED max brightness value
uint16_t noise_bright_ir = MIN_LED_IR;     // IR min brightness value
uint16_t noise_bright_red = MIN_LED_RED;   // RED min brightness value

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

  clock_usleep(DETECTOR_TIMINGS_RISE_TIME);
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
    
    ir_peaks_present = false;
    red_peaks_present = false;
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
      } 
      else {
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
        
        // add IR data to AMP buffer
        butt_ir_buffer[cur_amp_element_ir] = value;
        cur_amp_element_ir++;
        if (cur_amp_element_ir == NUM_OF_PEAKS) {
          cur_amp_element_ir = 0;
          ir_buff_ready = true;
        }
        ir_peaks_present = true;
        if (ir_buff_ready) {
          sqi_ir_loop();
        }

        // add RED data to AMP buffer
        butt_red_buffer[cur_amp_element_red] = value;
        cur_amp_element_red++;
        if (cur_amp_element_red == NUM_OF_PEAKS) {
          cur_amp_element_red = 0;
          red_buff_ready = true;
        }
        red_peaks_present = true;
        if (red_buff_ready) {
          sqi_red_loop();
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

void red_detect_mins(uint16_t raw, float value) {
  if (raw >= MEASUREMENT_THRESHOLD) {
    pulse_state_red = PULSE_IDLE;
    red_current_timestamp = 0;
    red_last_timestamp = 0;
    red_previous_value = 0.0;
    
    red_peaks_present = false;
    return;
  }
  
  switch(pulse_state_red) {
    case PULSE_IDLE: {
      // Idle state: we wait for the value to cross the threshold.
      if (value <= RED_THRESHOLD) {
        pulse_state_red = PULSE_FALLING;
      }
      break;
    }
    case PULSE_FALLING: {
      if (value < red_previous_value) {
        // Still falling.
        red_current_timestamp = clock_millis();
      }
      else { 
        // Reached the bottom.
        red_last_timestamp = red_current_timestamp;
        // add data to AMP buffer
        butt_red_buffer[cur_amp_element_red] = value;
        cur_amp_element_red++;
        if (cur_amp_element_red == NUM_OF_PEAKS) {
          cur_amp_element_red = 0;
          red_buff_ready = true;
        }
        red_peaks_present = true;
        if (red_buff_ready) {
          sqi_red_loop();
        }

        pulse_state_red = PULSE_RISING;
        return;
      }
      break;
    }
    case PULSE_RISING: {
      // Move into idle state when under the threshold.
      if (value > RED_THRESHOLD) {
        pulse_state_red = PULSE_IDLE;
      }
      break;
    }
  }
  red_previous_value = value;
  return;
}
  
void change_brightness_ir() {
  // empty AMP in STD buffers (used by SQI)
  empty_amp_buffer(LED_IR);
  empty_std_buffer(LED_IR);

  // change direction of brightness change if needed
  if (led_config[0].duty_on < noise_bright_ir + 1) {  
    plus_step_ir = true;
  }
  else if (led_config[0].duty_on > clip_bright_ir - 1) {
    plus_step_ir = false;
  }

  // change brightness for defined step
  if (plus_step_ir) {
    led_config[0].duty_on += IR_STEP;
  }
  else {
    led_config[0].duty_on -= IR_STEP;
  }

  // noise detection
  if (led_config[0].duty_on <= (5*IR_STEP + MIN_LED_IR) && plus_step_ir == false) {
    if (sqi_ir != 0 && sqi_ir < SQI_NOISE_THRESHOLD) { 
      noise_bright_ir = led_config[0].duty_on;
    }
  }
}

void change_brightness_red() {
  // empty AMP in STD buffers (used by SQI)
  empty_amp_buffer(LED_RED);
  empty_std_buffer(LED_RED);

  // change direction of brightness change if needed
  if (led_config[1].duty_on > clip_bright_red - 1)  {
    minus_step_red = true;
  }
  else if (led_config[1].duty_on < noise_bright_red + 1){
    minus_step_red = false;
  }

  // change brightness for defined step
  if (minus_step_red) {
    led_config[1].duty_on -= RED_STEP;
  }
  else {
    led_config[1].duty_on += RED_STEP;
  }

   // noise detection
   if (led_config[1].duty_on <= (5*RED_STEP + MIN_LED_RED) && minus_step_red == true) {
     if (sqi_red != 0 && sqi_red < SQI_NOISE_THRESHOLD) { 
       noise_bright_red = led_config[1].duty_on;
     }
   }
}

void sqi_ir_loop() {
  // calcuate AMP - signal amplitude
  float amp_ir = 0.0;
  for (int i = 0; i < NUM_OF_PEAKS; i++) {
    amp_ir += butt_ir_buffer[i];
  }
  amp_ir = amp_ir / NUM_OF_PEAKS; // mean
  if (amp_ir < 0.0) { 
    amp_ir = -amp_ir; //abs
  }
  amp_ir = 2 * amp_ir;

  // calculate STD - signal standard deviation
  float mean_std_ir = 0.0;
  for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
    mean_std_ir += noise_ir_buffer[i];
  }
  mean_std_ir = mean_std_ir / RAW_BUFFER_SIZE;
  float std_ir = 0.0;
  for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
    std_ir += ((noise_ir_buffer[i] - mean_std_ir) * (noise_ir_buffer[i] - mean_std_ir));
  }
  std_ir = std_ir / RAW_BUFFER_SIZE;  // var
  std_ir = qfp_fsqrt_fast(std_ir);

  // calculate SQI
  sqi_ir = 1 - (std_ir / amp_ir);
  if (sqi_ir < 0.0) {
    sqi_ir = 0.0;
  }
  // add calculated SQI to buffer
  if (cur_sqi_element_ir < NUM_OF_SQIS) {
    sqi_ir_buffer[cur_sqi_element_ir] = sqi_ir;
    cur_sqi_element_ir++;
  }
  else {  // if we have full sqi buffer
    mean_sqi_ir = calculate_mean_sqi(LED_IR);
    empty_sqi_buffer(LED_IR);
    sqi_ir_cur_timestamp = clock_millis();
    if (mean_sqi_ir < SQI_IR_BORDER && (sqi_ir_cur_timestamp - sqi_ir_last_timestamp > SQI_BRIGHT_CHANGE_TIMEOUT)) {
      change_brightness_ir();
      sqi_ir_last_timestamp = sqi_ir_cur_timestamp;
    }
  }
  std_ir_buf_full = false;  // maybe doesn't need to happen every time
}

void sqi_red_loop() {
   // calcuate AMP - signal amplitude
  float amp_red = 0.0;
  for (int i = 0; i < NUM_OF_PEAKS; i++) {
    amp_red += butt_red_buffer[i];
  }
  amp_red = amp_red / NUM_OF_PEAKS; // mean
  if (amp_red < 0.0) {
    amp_red = -amp_red; // abs
  }
  amp_red = 2 * amp_red;

  // calculate STD - signal standard deviation
  float mean_std_red = 0.0;
  for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
    mean_std_red += noise_red_buffer[i];
  }
  mean_std_red = mean_std_red / RAW_BUFFER_SIZE;
  float std_red = 0.0;
  for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
    std_red += ((noise_red_buffer[i]- mean_std_red) * (noise_red_buffer[i] - mean_std_red));
  }
  std_red = std_red / RAW_BUFFER_SIZE;  // var
  std_red = qfp_fsqrt_fast(std_red);

  // calculate SQI
  sqi_red = 1 - (std_red / amp_red);
  if (sqi_red < 0.0) {
    sqi_red = 0.0;
  }
  // add calculated SQI to buffer
  if (cur_sqi_element_red < NUM_OF_SQIS) {
    sqi_red_buffer[cur_sqi_element_red] = sqi_red;
    cur_sqi_element_red++;
  }
  else { // if we have full sqi buffer
    mean_sqi_red = calculate_mean_sqi(LED_RED);
    empty_sqi_buffer(LED_RED);
    sqi_red_cur_timestamp = clock_millis();
    if (mean_sqi_red < SQI_RED_BORDER && (sqi_red_cur_timestamp - sqi_red_last_timestamp > SQI_BRIGHT_CHANGE_TIMEOUT)) {
      change_brightness_red();
      sqi_red_last_timestamp = sqi_red_cur_timestamp;
    }
  }
  std_red_buff_full = false;
}

void empty_amp_buffer(uint8_t led) {
  if (led == LED_IR) {  // empty IR buffer
    for (int i = 0; i < NUM_OF_PEAKS; i++) {
        butt_ir_buffer[i] = 0.0;
    }
    cur_amp_element_ir = 0;
    ir_buff_ready = false;
  }
  else if (led == LED_RED) {  // empty RED buffer
    for (int i = 0; i < NUM_OF_PEAKS; i++) {
        butt_red_buffer[i] = 0.0;
    }
    cur_amp_element_red = 0;
    red_buff_ready = false;
  }
}

void empty_std_buffer(uint8_t led) {
  if (led == LED_IR) {  // empty IR buffer
    for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
      noise_ir_buffer[i] = 0.0;
    }
    std_ir_buf_full = false;
    cur_std_element_ir = 0;
  }
  else if (led == LED_RED) {  // empty RED buffer
    for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
      noise_red_buffer[i] = 0.0;
    }
    std_red_buff_full = false;
    cur_std_element_red = 0;
  }
}

void empty_sqi_buffer(uint8_t led) {
  if (led == LED_IR) {  // empty IR buffer
    for (int i = 0; i < NUM_OF_SQIS; i++) {
      sqi_ir_buffer[i] = 0.0;
    }
    cur_sqi_element_ir = 0;
  }
  else if (led == LED_RED) {  // empty RED buffer
    for (int i = 0; i < NUM_OF_SQIS; i++) {
      sqi_red_buffer[i] = 0.0;
    }
    cur_sqi_element_red = 0;
  }
}

float calculate_mean_sqi(uint8_t led) {
  float mean_sqi = 0;
  if (led == LED_IR) {  // IR SQI
    for (int i = 0; i < NUM_OF_SQIS; i++) {
      mean_sqi += sqi_ir_buffer[i];
    }
    mean_sqi = mean_sqi / NUM_OF_SQIS;
  }
  else if (led == LED_RED) {  // RED SQI
    for (int i = 0; i < NUM_OF_SQIS; i++) {
      mean_sqi += sqi_red_buffer[i];
    }
    mean_sqi = mean_sqi / NUM_OF_SQIS;
  }
  return mean_sqi;
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
    
    // Intially set values to 0 - for debug output if finger is not detected
    float butt_ir = 0.0;
    float dc_red = 0.0;
    float butt_ambient_red = 0.0;
    float butt_red = 0.0;
    float norm_ir = 0.0;
    float norm_red = 0.0;
    float butt_norm_ir = 0.0;
    float butt_norm_red = 0.0;

    //Finger detect
    if (raw.ir < MEASUREMENT_THRESHOLD && raw.red < MEASUREMENT_THRESHOLD) {
      current_measurement.finger_in = 1;
      // Check status of brightness calibration
      if (sqi_ir > SQI_IR_BORDER && sqi_red > SQI_RED_BORDER) { 
        // both SQI-s are OK - display hr and sp02 values
        current_measurement.is_calibrating = 0;
      }
      else {  // Calibration loop
        current_measurement.is_calibrating = 1;
      }
    }
    else {  // Finger out
      current_measurement.finger_in = 0;
      current_measurement.is_calibrating = 0;
      pulse_present = 0;

      // reset SQI-s
      sqi_ir = 0.0;
      sqi_red = 0.0;
      mean_sqi_ir = 0.0;
      mean_sqi_red = 0.0;
      empty_sqi_buffer(LED_IR);
      empty_sqi_buffer(LED_RED);

      // reset buffers for SQI
      empty_amp_buffer(LED_IR);
      empty_std_buffer(LED_IR);
      led_config[0].duty_on = IR_DEFAULT;

      empty_amp_buffer(LED_RED);
      empty_std_buffer(LED_RED);
      led_config[1].duty_on = RED_DEFAULT;

      // reset hr and spo2 display
      current_measurement.hr = 0;
      current_measurement.spo2 = 0;
    }

    // IR signal - all time necessary variables
    float dc_ir = filter_dc(&dc_filter_ir, (float) raw.ir, DC_FILTER_ALPHA);
    float butt_ambient_ir = filter_butterworth_hp_ambient(&butt_filter_ir_ambient, dc_ir);
    float mean_ir = filter_mean(&mean_diff_ir, butt_ambient_ir, 1);
    mean_ir = filter_mean(&rolling_mean_ir, mean_ir, 0);
    static float ratio = 0.0;
    if (current_measurement.finger_in) {
      // IR.
      butt_ir = filter_butterworth_lp(&butt_filter_ir, butt_ambient_ir);
      // Red.
      dc_red = filter_dc(&dc_filter_red, (float) raw.red, DC_FILTER_ALPHA);
      butt_ambient_red = filter_butterworth_hp_ambient(&butt_filter_red_ambient, dc_red);
      butt_red = filter_butterworth_lp(&butt_filter_red, butt_ambient_red);

      // add IR data to STD buffer
      if (std_ir_buf_full == false) {
        float noise_ir = filter_butterworth_hp(&butt_filter_ir_h, butt_ambient_ir);
        noise_ir_buffer[cur_std_element_ir] = noise_ir;
        cur_std_element_ir++;
        if (cur_std_element_ir == RAW_BUFFER_SIZE) {
          std_ir_buf_full = true;
          cur_std_element_ir = 0;
        }
      }
      // add RED data to STD buffer
      if (std_red_buff_full == false) {
        float noise_red = filter_butterworth_hp(&butt_filter_red_h, butt_ambient_red);
        noise_red_buffer[cur_std_element_red] = noise_red;
        cur_std_element_red++;
        if (cur_std_element_red == RAW_BUFFER_SIZE) {
          std_red_buff_full = true;
          cur_std_element_red = 0;
        }
      }
      // Normalize IR and red AC by their DC components.
      norm_ir = dc_ir / dc_filter_ir.w;
      norm_red = dc_red / dc_filter_red.w;

      butt_norm_ir = filter_butterworth_lp(&butt_filter_spo2_ir, norm_ir);
      butt_norm_red = filter_butterworth_lp(&butt_filter_spo2_red, norm_red);

      ac_sqsum_ir += butt_norm_ir * butt_norm_ir;
      ac_sqsum_red += butt_norm_red * butt_norm_red;
      spo2_samples++;

      // Perform pulse detection.
      if (measurement_detect_pulse(raw.ir, butt_ir)) {
        spo2_beats++;
      }

      // Detect peaks in red signal
      //red_detect_mins(raw.red, butt_red);

      // Compute derived measurements.
      
      if (pulse_present) {
        current_measurement.hr = (int) pulse_current_bpm;

        if (spo2_beats >= SPO2_UPDATE_BEATS) {
          ratio = qfp_fsqrt_fast(ac_sqsum_red / ac_sqsum_ir);
          current_measurement.spo2 = spo2_lookup(ratio);

          // Reset readings.
          ac_sqsum_ir = 0.0;
          ac_sqsum_red = 0.0;
          spo2_samples = 0;
          spo2_beats = 0;
        }
      } 
      else {
        // No pulse present, SpO2 should be ignored.
        current_measurement.hr = 0;
        current_measurement.spo2 = 0;
      }
    }
    
    // Provide data for the waveform - displayed all the time
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
/*
    uart_puti((int) (butt_ambient_ir * 100));
    uart_putc(',');
    uart_puti((int) (mean_ir * 100));
    uart_putc(',');*/
    uart_puti((int) (butt_ir * 100));
    uart_putc(',');
    //uart_puti((int) (noise_ir * 100));
    //uart_putc(',');
    //uart_puti((int) (dc_red * 100));
    //uart_putc(',');
    uart_puti((int) (butt_red * 100));
    uart_putc(',');
    //uart_puti((int) (noise_red * 100));
    //uart_putc(',');
   /* 
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
    uart_putc(',');*/
    
    uart_puti((int) (ratio * 100));
    uart_putc(',');
    
    uart_puti(raw.red);
    uart_putc(',');

    //uart_puti(led_config[0].duty_on);
    //uart_putc(',');
    //uart_puti(led_config[1].duty_on);
    //uart_putc(',');
    uart_puti((int) (sqi_ir * 100));
    uart_putc(',');
    uart_puti((int) (sqi_red * 100));
    //uart_putc(',');
    //uart_puti(raw.ambient);
    //uart_putc(',');
    //uart_puti((int) (pulse_present));
    //uart_putc(',');
    //uart_puti((int) (red_peaks_present));
    //uart_puti((int) (mean_sqi_ir * 100));
    //uart_putc(',');
    //uart_puti((int) (mean_sqi_red * 100));
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
  
  // Setup loop
  if (current_measurement.finger_in == 1) {
    if (((now - last_time) > CHANGE_BRIGHT_DELAY)) {
      if (ir_peaks_present != true) {
        change_brightness_ir();
      }
      if (red_peaks_present != true) {
        change_brightness_red();
      }
      last_time = now;
    }
  }
}
#endif
