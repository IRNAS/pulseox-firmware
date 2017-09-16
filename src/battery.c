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
#include "battery.h"
#include "adc.h"
#include "clock.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define VBAT_PORT GPIOA
#define VBAT_PIN_ANALOG_IN GPIO1

// Vbat divider resistors (in kOhm * 10).
#define VBAT_DIVIDER_R1 47
#define VBAT_DIVIDER_R2 100

// ADC reference voltage (in mV).
#define VBAT_ADC_VDDA 3010

// Vbat measurement interval (in ms).
#define VBAT_UPDATE_INTERVAL 10000

uint32_t battery_last_measurement = 0;
uint16_t battery_current_voltage = 0;
uint16_t battery_percent_charge = 0;

typedef struct {
  uint32_t v_bat;
  uint8_t percent_charge;
} battery_config_t;

// Battery percent charge lookup table.
const uint8_t VBAT_CHARGE_LOOKUP_TABLE_SIZE = 7;
const battery_config_t VBAT_CHARGE_LOOKUP_TABLE[] = {
  { .v_bat = 4200, .percent_charge = 100 },
  { .v_bat = 4030, .percent_charge = 76  },
  { .v_bat = 3860, .percent_charge = 52  },
  { .v_bat = 3830, .percent_charge = 42  },
  { .v_bat = 3790, .percent_charge = 30  },
  { .v_bat = 3700, .percent_charge = 11  },
  { .v_bat = 3600, .percent_charge = 0   },
};

void battery_init()
{
  rcc_periph_clock_enable(RCC_GPIOA);

  // Setup battery voltage GPIO.
  gpio_mode_setup(VBAT_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VBAT_PIN_ANALOG_IN);

  battery_update();
}

void battery_update()
{
  uint32_t now = clock_millis();
  if (battery_last_measurement && now - battery_last_measurement < VBAT_UPDATE_INTERVAL) {
    return;
  }

  uint32_t raw = adc_read(ADC_CHANNEL_BATTERY);
  uint32_t v_bat = (raw * VBAT_ADC_VDDA * (VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2)) / (4096 * VBAT_DIVIDER_R2);
  battery_current_voltage = (uint16_t) v_bat;

  // Determine battery charge.
  battery_percent_charge = 0;
  for (uint8_t i = 0; i < VBAT_CHARGE_LOOKUP_TABLE_SIZE; i++) {
    if (v_bat >= VBAT_CHARGE_LOOKUP_TABLE[i].v_bat) {
      if (i == 0) {
        battery_percent_charge = 100;
      } else {
        // Perform linear interpolation between points.
        uint32_t v0 = VBAT_CHARGE_LOOKUP_TABLE[i].v_bat;
        uint32_t v1 = VBAT_CHARGE_LOOKUP_TABLE[i - 1].v_bat;
        uint32_t c0 = VBAT_CHARGE_LOOKUP_TABLE[i].percent_charge;
        uint32_t c1 = VBAT_CHARGE_LOOKUP_TABLE[i - 1].percent_charge;

        battery_percent_charge = c0 + (v_bat - v0) * (c1 - c0) / (v1 - v0);
      }
      break;
    }
  }

  battery_last_measurement = now;
}

int battery_get_percent_charge()
{
  return (int) battery_percent_charge;
}

int battery_is_low()
{
  return battery_percent_charge < BATTERY_LOW_PERCENT_CHARGE ? 1 : 0;
}
