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
#ifndef PULSEOX_GUI_H
#define PULSEOX_GUI_H

#include <stdint.h>

#include "measurement.h"

// Waveform width (in milliseconds).
#define GUI_WAVEFORM_WIDTH 5000
// Waveform maximum height (in pixels).
#define GUI_WAVEFORM_HEIGHT 20

/**
 * Setup GUI.
 */
void gui_init(uint16_t width, uint16_t height);

/**
 * Update GUI after a measurement.
 */
void gui_measurement_update(const measurement_t *measurement);

/**
 * Render the GUI.
 */
void gui_render();

#endif
