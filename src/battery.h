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
#ifndef PULSEOX_BATTERY_H
#define PULSEOX_BATTERY_H

// The percent charge when the battery is considered low.
#define BATTERY_LOW_PERCENT_CHARGE 30

/**
 * Setup battery managemnet.
 */
void battery_init();

/**
 * Perform battery voltage measurement.
 */
void battery_update();

/**
 * Return the percentage of battery charge.
 */
int battery_get_percent_charge();

/**
 * Return whether the battery charge is currently considered low.
 */
int battery_is_low();

#endif
