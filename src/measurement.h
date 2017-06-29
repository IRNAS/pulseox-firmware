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

#define MEAN_FILTER_SIZE 15
#define DC_FILTER_ALPHA 0.95

typedef struct {
  uint16_t red;
  uint16_t orange;
  uint16_t yellow;
  uint16_t ir;
  uint16_t ambient;
} raw_measurement_t;

typedef struct {
  float w;
  float result;
} dc_filter_t;

typedef struct {
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
} mean_diff_filter_t;

typedef struct {
  // Derived measurements.
  int hr;
  int spo2;
  // Data for drawing the waveform;
  float waveform_spo2;
  float waveform_spo2_min;
  float waveform_spo2_max;
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
