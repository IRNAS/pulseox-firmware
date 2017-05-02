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
#ifndef PULSEOX_CLOCK_H
#define PULSEOX_CLOCK_H

#include <stdint.h>

/**
 * Setup clock.
 */
void clock_init();

/**
 * Sleep for a specified number of milliseconds.
 *
 * @param delay Delay in milliseconds
 */
void clock_msleep(uint32_t delay);

/**
 * Sleep for a specified number of microseconds.
 *
 * @param delay Delay in microseconds
 */
void clock_usleep(uint32_t delay);

/**
 * Return the number of milliseconds since boot.
 */
uint32_t clock_millis();

#endif
