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
#include "include/HalInterface.h"
#include "include/SystemState.h"

typedef int TickType_t;
#define pdTRUE  1
#define pdFALSE 0
#define pdMS_TO_TICKS(x) (x)
#define HIGH 1
#define LOW  0
#define GPIO_FAN_PWM             32
#define GPIO_BTN_FAN_CONTROLLER  18
#define GPIO_LED_FAN_CONTROLLER  19
#define GPIO_POTENZIOMETRO       36

inline void* xSemaphoreCreateMutex()               { return nullptr; }
inline int   xSemaphoreTake(SemaphoreHandle_t, int) { return pdTRUE; }
inline void  xSemaphoreGive(SemaphoreHandle_t)      {}
inline void  vTaskDelay(int)                        {}
inline unsigned long millis()                       { return 0; }
inline int   digitalRead(uint8_t)                   { return HIGH; }
inline int   map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct FakeSerial {
  void println(const char*) {}
  template<typename... Args>
  void printf(const char*, Args...) {}
};
extern FakeSerial Serial;
