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
#include "dsp.h"

float filter_dc(dc_filter_t *filter, float x, float alpha)
{
  float prev_w = filter->w;
  filter->w = x + alpha * prev_w;
  filter->result = filter->w - prev_w;

  return filter->result;
}

float filter_mean(mean_filter_t *filter, float x, int difference)
{
  float avg = 0;

  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = x;
  filter->sum += filter->values[filter->index];

  filter->index++;
  filter->index = filter->index % MEAN_FILTER_SIZE;

  if (filter->count < MEAN_FILTER_SIZE) {
    filter->count++;
  }

  avg = filter->sum / filter->count;
  if (difference) {
    // Difference from mean.
    return avg - x;
  } else {
    // Rolling mean.
    return avg;
  }
}

float filter_butterworth_lp(butterworth_filter_t *filter, float x)  // 3,3 Hz, order 2
{
  filter->v[0] = filter->v[1];
  filter->v[1] = filter->v[2];
  filter->v[2] = (9.348688702492780056e-3f * x) // b1
                  + (-0.74586058058659876480f * filter->v[0]) // -a3
                  + (1.70846582577662764457f * filter->v[1]); // -a2

  return (filter->v[0] + filter->v[2]) + 2 * filter->v[1];
}

float filter_butterworth_hp(butterworth_filter_t *filter, float x) // 20 Hz, order 2
{
  filter->v[0] = filter->v[1];
  filter->v[1] = filter->v[2];
  filter->v[2] = (0.39133577f * x)   // b1
                  + (-0.19581571f * filter->v[0]) // -a3
                  + (0.36952738f * filter->v[1]); // -a2

  return (filter->v[0] + filter->v[2]) - 2 * filter->v[1];
}

float filter_butterworth_hp_ambient(butterworth_filter_t *filter, float x) // 0,35 Hz, order 2
{
  filter->v[0] = filter->v[1];
  filter->v[1] = filter->v[2];
  filter->v[2] = (0.98457018f * x)   // b1
                  + (-0.96937846f * filter->v[0]) // -a3
                  + (1.96890227f * filter->v[1]); // -a2

  return (filter->v[0] + filter->v[2]) - 2 * filter->v[1];
}
