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
#ifndef PULSEOX_SPO2_H
#define PULSEOX_SPO2_H

#include <stdint.h>

// TODO: Calibrate lookup table.
const uint8_t SPO2_LOOKUP_TABLE[] = {
  99,99,99,99,99,99,99,99,99,99,98,98,98,98,
  98,97,97,97,97,97,97,96,96,96,96,96,96,95,
  95,95,95,95,95,94,94,94,94,94,93,93,93,93,
  93
};

/**
 * Performs a SpO2 value lookup given a ratio between normalized
 * red and IR intensities.
 *
 * @param ratio Ratio
 * @return SpO2 in percent
 */
uint8_t spo2_lookup(float ratio)
{
  uint8_t lut_size = sizeof(SPO2_LOOKUP_TABLE) / sizeof(uint8_t);
  uint8_t index = (uint8_t) (100 * ratio);
  if (index > 66) {
    index -= 66;
  } else if (index > 50) {
    index -= 50;
  }

  // Final sanity check.
  if (index >= lut_size) {
    index = lut_size - 1;
  } else if (index < 0) {
    index = 0;
  }

  return SPO2_LOOKUP_TABLE[index];
}

#endif
