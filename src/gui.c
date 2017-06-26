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
#include "gui.h"
#include "gfx.h"
#include "clock.h"

#define _GNU_SOURCE
#include <stdio.h>

/**
 * Current state of the GUI.
 */
struct gui_state {
  uint16_t width;
  uint16_t height;
  // Current display values.
  int display_hr;
  int display_spo2;
  // Current waveform X coordinate. Wraps around display width.
  uint16_t waveform_x;
  // Last waveform update.
  uint32_t waveform_last_update;
  // Amount of milliseconds that are represented by one pixel.
  uint32_t waveform_pixel_ms;
  // Current waveform bucket.
  float waveform_bucket;
  uint32_t waveform_bucket_size;
};

// GUI state.
struct gui_state state;
// Temporary text buffer;
char text_buffer[16] = {0,};

void gui_init(uint16_t width, uint16_t height)
{
  state.width = width;
  state.height = height;

  state.display_hr = -1;
  state.display_spo2 = -1;

  // Initialize waveform.
  state.waveform_x = 0;
  state.waveform_last_update = 0;
  state.waveform_pixel_ms = GUI_WAVEFORM_WIDTH / width;

  gfx_fillScreen(0x00);
}

void gui_measurement_update(const measurement_t *measurement)
{
  // Update current heart rate and SpO2 displays.
  if (measurement->hr != state.display_hr) {
    gfx_setTextSize(1);
    gfx_setTextColor(0x80, 0x00);
    gfx_setCursor(90, 0);
    gfx_puts("HR");

    if (measurement->hr) {
      snprintf(text_buffer, sizeof(text_buffer), "%d", measurement->hr);
    } else {
      snprintf(text_buffer, sizeof(text_buffer), "--");
    }

    gfx_setTextSize(2);
    gfx_setCursor(80, 14);
    gfx_puts(text_buffer);
    state.display_hr = measurement->hr;
  }

  if (measurement->spo2 != state.display_spo2) {
    gfx_setTextSize(1);
    gfx_setTextColor(0x80, 0x00);
    gfx_setCursor(20, 0);
    gfx_puts("SpO2%");

    if (measurement->spo2) {
      snprintf(text_buffer, sizeof(text_buffer), "%d", measurement->spo2);
    } else {
      snprintf(text_buffer, sizeof(text_buffer), "--");
    }

    gfx_setTextSize(3);
    gfx_setCursor(15, 14);
    gfx_puts(text_buffer);
    state.display_spo2 = measurement->spo2;
  }

  // Update current waveform bucket.
  state.waveform_bucket += measurement->waveform_spo2;
  state.waveform_bucket_size++;

  // Update current waveform if we have enough data.
  if (clock_millis() - state.waveform_last_update >= state.waveform_pixel_ms) {
    float value = (state.waveform_bucket / (float) state.waveform_bucket_size);
    value = (value * (float) GUI_WAVEFORM_HEIGHT) / measurement->waveform_spo2_max;
    if (value > (float) GUI_WAVEFORM_HEIGHT) {
      value = (float) GUI_WAVEFORM_HEIGHT;
    }

    // Erase previous and next buckets.
    gfx_fillRect(state.waveform_x, state.height - GUI_WAVEFORM_HEIGHT,
      GUI_WAVEFORM_GAP + 1, GUI_WAVEFORM_HEIGHT, 0x00);

    // Draw new bucket.
    gfx_drawLine(
      state.waveform_x, state.height,
      state.waveform_x, state.height - (int) value,
      0x80
    );

    state.waveform_bucket = 0.0;
    state.waveform_bucket_size = 0;
    state.waveform_x = (state.waveform_x + 1) % state.width;
  }
}

void gui_render()
{
  // TODO.
}
