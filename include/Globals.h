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
#include <Arduino.h>
#include <Wire.h>
#include "esp32-hal-ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/SystemState.h"
#include "include/Aspiration.h"
#include "include/BME680.h"
#include "include/Display.h"
#include "include/PIR.h"
#include "include/TaskMQTT.h"

#define GPIO_PIR                 15
#define GPIO_LAMP                5
#define GPIO_POTENZIOMETRO       36
#define GPIO_BTN_FAN_CONTROLLER  18
#define GPIO_LED_FAN_CONTROLLER  19
#define GPIO_FAN_PWM             32
#define GPIO_FAN_ENABLE          33
// SDA = 21, SCL = 22

#define FAN_PWM_CHANNEL    0
#define FAN_PWM_FREQ       25000
#define FAN_PWM_RESOLUTION 8

class Adafruit_SSD1306;

extern Adafruit_SSD1306 display;
extern String scrolling_message;
extern SystemState system_state;

void init_gpio();
