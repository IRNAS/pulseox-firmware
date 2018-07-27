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
#ifndef PULSEOX_MEASUREMENT_H
#define PULSEOX_MEASUREMENT_H

#include <stdint.h>

// LED default brightness values - CHANGE IF NEEDED
#define IR_DEFAULT 160           // IR value for initialization
#define RED_DEFAULT 250          // RED value for initialization
#define IR_MAX 350               // Maximum IR brightness value
#define RED_MAX 5000             // Maximum RED brightness value
#define IR_MIN 20               // Minimum IR brightness value
#define RED_MIN 250             // Minimum RED brightness value 

#define CHANGE_BRIGHT_DELAY 3000  // setup loop delay in ms
#define SQI_IR_BORDER 0.8f        // IR test loop border value
#define SQI_RED_BORDER 0.8f       // RED test loop border value
#define PULSE_THRESHOLD -5.0      // Pulse detection threshold
#define RED_THRESHOLD -5.0        // Peak Detector RED threshold
#define IR_STEP 20                // IR brightness change step
#define RED_STEP 250              // RED brightness change step

// Buffer size is number of samples needed
#define RAW_BUFFER_SIZE 200
// Number of signal periods needed + 1
#define NUM_OF_PERIODS 2
// If the raw signal is above this threshold, ignore measurements.
#define MEASUREMENT_THRESHOLD 2600
// DC filter alpha.
#define DC_FILTER_ALPHA 0.95

// Number of beats required for initial pulse detection.
#define PULSE_INITIAL_BEATS 3
// Pulse reset threshold.
#define PULSE_RESET_THRESHOLD 500.0
// Pulse timeout (in ms). If no pulse detected for this time, reset readings. This
// is usually not needed as PULSE_RESET_THRESHOLD resets early.
#define PULSE_RESET_TIMEOUT 5000

// Compute SpO2 every so many heart beats.
#define SPO2_UPDATE_BEATS 3
// Waveform display min/max values. These settings only affect the waveform
// visualization not any measurements.
#define SPO2_WAVEFORM_MIN -20
#define SPO2_WAVEFORM_MAX 20

typedef struct {
  uint16_t red;
  uint16_t orange;
  uint16_t yellow;
  uint16_t ir;
  uint16_t ambient;
} raw_measurement_t;

// Buffers for AMP computation
float butt_ir_buffer[NUM_OF_PERIODS];
float butt_red_buffer[NUM_OF_PERIODS];

// Buffers for STD computation
float noise_ir_buffer[RAW_BUFFER_SIZE];
float noise_red_buffer[RAW_BUFFER_SIZE];

typedef struct {
  // Derived measurements.
  int hr;
  int spo2;
  // Data for drawing the waveform;
  float waveform_spo2;
  float waveform_spo2_min;
  float waveform_spo2_max;
  uint8_t finger_in;
  uint8_t is_calibrating;
  
  uint16_t ir_brightness;
  uint16_t red_brightness;
  int time;
} measurement_t;

typedef struct {
  // GPIOs that turns the LED on.
  uint32_t gpio;
  // Duty cycle.
  uint16_t duty_on;
  uint16_t duty_wait;
} led_config_t;

typedef void (*measurement_update_callback_t)(const measurement_t*);

/**
 * Setup measurement peripherals.
 */
void measurement_init(measurement_update_callback_t on_update);

/**
 * Update measurement buffer.
 */
void measurement_update();


#endif
