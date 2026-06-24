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

void setup()
{
  pinMode(GPIO_FAN_PWM, OUTPUT);
  digitalWrite(GPIO_FAN_PWM, LOW);

  init_gpio();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  system_state.fan_mutex = xSemaphoreCreateMutex();

  init_display();
  init_environment_sensor(&system_state);

  setup_wifi();

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(60);

  Serial.println("Avvio task...");

  xTaskCreate(task_environment_sensor, "EnvSensor",   4096, &system_state, 4, NULL);  
  xTaskCreate(task_display_update,     "Display",     4096, &system_state, 1, NULL);
  xTaskCreate(task_motion_sensor,      "MotionSensor",4096, &system_state, 3, NULL);
  xTaskCreate(task_fan_control,        "FanControl",  4096, &system_state, 2, NULL);
  xTaskCreate(task_mode_button,        "ModeButton",  4096, &system_state, 3, NULL);
  xTaskCreate(task_mqtt,               "MQTT",        4096, &system_state, 2, NULL);

  digitalWrite(GPIO_FAN_ENABLE, HIGH);
}

void loop()
{
  // vuoto: tutto gestito da FreeRTOS
}
