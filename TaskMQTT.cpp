#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "include/Globals.h"

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connessione a ");
  Serial.println(WIFI_SSID);
  wifi_try_msg();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println();
    Serial.println("WiFi connesso!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    wifi_ok_msg(WiFi.localIP());
    delay(2000);
  }
  else
  {
    Serial.println();
    Serial.println("ERRORE: WiFi non connesso!");
    wifi_error_msg();
    delay(3000);
  }
}

void reconnect_mqtt()
{
  if (!mqttClient.connected())
  {
    Serial.print("Connessione MQTT...");

    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
    {
      Serial.println(" OK!");
    }
    else
    {
      Serial.print(" FALLITA, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void task_mqtt_publish(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(2000);

  float last_temp = NAN;
  float last_hum = NAN;
  float last_gas = NAN;
  String last_air = "";

  String last_mode = "";
  uint8_t last_speed = 255;

  bool last_motion = false;

  char buffer[64];

  for (;;)
  {

    if (WiFi.status() != WL_CONNECTED)
    {
      setup_wifi();
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    if (!mqttClient.connected())
    {
      reconnect_mqtt();
    }

    mqttClient.loop();

    if (mqttClient.connected())
    {

      if (isnan(last_temp) || abs(temp - last_temp) > 0.05)
      {
        dtostrf(temp, 5, 2, buffer);
        mqttClient.publish(TOPIC_TEMP, buffer, true);
        last_temp = temp;
      }

      if (isnan(last_hum) || abs(hum - last_hum) > 0.1)
      {
        dtostrf(hum, 5, 2, buffer);
        mqttClient.publish(TOPIC_HUM, buffer, true);
        last_hum = hum;
      }

      if (isnan(last_gas) || abs(gas_index - last_gas) > 0.3)
      {
        dtostrf(gas_index, 5, 2, buffer);
        mqttClient.publish(TOPIC_GAS, buffer, true);
        last_gas = gas_index;
      }

      String air_msg = air_index_to_msg(gas_index);
      if (air_msg != last_air)
      {
        mqttClient.publish(TOPIC_AIR_QUALITY, air_msg.c_str(), true);
        last_air = air_msg;
      }

      if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(50)))
      {
        String mode_msg = mode_manual ? "manual" : "auto";

        if (mode_msg != last_mode)
        {
          mqttClient.publish(TOPIC_FAN_MODE, mode_msg.c_str(), true);
          last_mode = mode_msg;
        }

        if (target_fan_speed != last_speed)
        {
          sprintf(buffer, "%d", target_fan_speed);
          mqttClient.publish(TOPIC_FAN_SPEED, buffer, true);
          last_speed = target_fan_speed;
        }

        xSemaphoreGive(fan_mutex);
      }

      if (motion_detected != last_motion)
      {
        mqttClient.publish(
            TOPIC_PIR,
            motion_detected ? "detected" : "clear",
            true);
        last_motion = motion_detected;
      }
    }

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}

// void task_mqtt_publish(void *pvParameters)
// {
//   TickType_t last_wake_time = xTaskGetTickCount();
//   const TickType_t frequency = pdMS_TO_TICKS(5000);
//   for (;;)
//   {
//     if (WiFi.status() != WL_CONNECTED)
//     {
//       Serial.println("WiFi disconnesso, riconnessione...");
//       setup_wifi();
//       vTaskDelay(pdMS_TO_TICKS(5000));
//       continue;
//     }
//     if (!mqttClient.connected())
//     {
//       reconnect_mqtt();
//     }
//     mqttClient.loop();
//     if (mqttClient.connected())
//     {
//       char buffer[64];
//       dtostrf(temp, 5, 2, buffer);
//       mqttClient.publish(TOPIC_TEMP, buffer);
//       dtostrf(hum, 5, 2, buffer);
//       mqttClient.publish(TOPIC_HUM, buffer);
//       dtostrf(gas_index, 5, 2, buffer);
//       mqttClient.publish(TOPIC_GAS, buffer);
//       String air_msg = air_index_to_msg(gas_index);
//       mqttClient.publish(TOPIC_AIR_QUALITY, air_msg.c_str());
//       uint8_t current_speed = 0;
//       if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
//       {
//         current_speed = target_fan_speed;
//         mqttClient.publish(TOPIC_FAN_MODE, mode_manual ? "manual" : "auto");
//         xSemaphoreGive(fan_mutex);
//       }
//       sprintf(buffer, "%d", current_speed);
//       mqttClient.publish(TOPIC_FAN_SPEED, buffer);
//       float rpm = read_fan_rpm();
//       if (rpm >= 0)
//       {
//         dtostrf(rpm, 6, 0, buffer);
//         mqttClient.publish(TOPIC_FAN_RPM, buffer);
//       }
//       mqttClient.publish(TOPIC_PIR, motion_detected ? "detected" : "clear");
//       // Serial.println("Dati MQTT pubblicati");
//     }
//     vTaskDelayUntil(&last_wake_time, frequency);
//   }
// }
