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
#ifndef PULSEOX_ADC_H
#define PULSEOX_ADC_H

#include <stdint.h>

// Supported ADC channels.
#define ADC_CHANNEL_MEASUREMENT 0
#define ADC_CHANNEL_BATTERY 1

/**
 * Setup ADC.
 */
void adc_init();

/**
 * Read from given channel.
 */
uint16_t adc_read(uint8_t channel);

#endif
