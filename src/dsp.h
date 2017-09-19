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
#ifndef PULSEOX_DSP_H
#define PULSEOX_DSP_H

#include <stdint.h>

// Mean filter window size.
#define MEAN_FILTER_SIZE 15

typedef struct {
  float w;
  float result;
} dc_filter_t;

typedef struct {
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
} mean_filter_t;

typedef struct {
  float v[3];
} butterworth_filter_t;

/**
 * DC removal filter.
 */
float filter_dc(dc_filter_t *filter, float x, float alpha);

/**
 * Rolling mean filter.
 */
float filter_mean(mean_filter_t *filter, float x, int difference);

/**
 * Butterworth low-pass filter with cutoff at 1.9Hz.
 */
float filter_butterworth_lp(butterworth_filter_t *filter, float x);

#endif
