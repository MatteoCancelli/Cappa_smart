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

#pragma once
#include <stdint.h>

#ifndef UNIT_TEST
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#else
typedef void* SemaphoreHandle_t;
#endif

struct SystemState {
  float temperature;
  float humidity;
  float air_quality_pct;
  float iaq_score;
  uint8_t iaq_accuracy;
  bool is_motion_detected;
  bool is_manual_mode;
  uint8_t fan_speed_target;
  SemaphoreHandle_t fan_mutex;
};
