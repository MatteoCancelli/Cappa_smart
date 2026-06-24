/*
 * Cappa_smart - ESP32-based smart kitchen hood controller
 * Copyright (C) 2026 Matteo Cancelli
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef UNIT_TEST

#include "test/Globals_test.h"
#include "include/HalInterface.h"

HalInterface hal = { nullptr, nullptr, nullptr, nullptr };

#else

#include "include/HalInterface.h"
#include "esp32-hal-ledc.h"
#include <Arduino.h>

static void hal_ledc_write(uint8_t pin, uint32_t duty)   { ledcWrite(pin, duty); }
static int  hal_digital_read(uint8_t pin)                { return digitalRead(pin); }
static void hal_digital_write(uint8_t pin, uint8_t val)  { digitalWrite(pin, val); }
static int  hal_analog_read(uint8_t pin)                 { return analogRead(pin); }

HalInterface hal = {
  hal_ledc_write,
  hal_digital_read,
  hal_digital_write,
  hal_analog_read
};

#endif
