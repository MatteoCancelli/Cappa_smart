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

#include "include/Globals.h"
#include "include/PIR.h"

void task_motion_sensor(void* pvParameters)
{
  SystemState* state = (SystemState*)pvParameters;
  TickType_t last_motion_time = 0;
  bool lamp_on = false;

  for (;;)
  {
    int pir_state = gpio_get_level((gpio_num_t)GPIO_PIR);

    if (pir_state == 1)
    {
      last_motion_time = xTaskGetTickCount();
      if (!lamp_on)
      {
        gpio_set_level((gpio_num_t)GPIO_LAMP, 1);
        lamp_on = true;
      }
      state->is_motion_detected = true;
    }

    if (lamp_on &&
        (xTaskGetTickCount() - last_motion_time > pdMS_TO_TICKS(5000)))
    {
      gpio_set_level((gpio_num_t)GPIO_LAMP, 0);
      lamp_on = false;
      state->is_motion_detected = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
