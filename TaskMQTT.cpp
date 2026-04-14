#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "include/Globals.h"
#include "include/Logic.h"
#include "include/TaskMQTT.h"

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
      Serial.println(" OK!");
    else
    {
      Serial.print(" FALLITA, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void task_mqtt(void* pvParameters)
{
  SystemState* state = (SystemState*)pvParameters;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(2000);

  float    last_temperature     = NAN;
  float    last_humidity        = NAN;
  float    last_air_quality_pct = NAN;
  float    last_iaq_score       = NAN;
  String   last_air_msg         = "";
  String   last_fan_mode        = "";
  uint8_t  last_fan_speed       = 255;
  bool     last_motion          = false;

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
      reconnect_mqtt();

    mqttClient.loop();

    if (mqttClient.connected())
    {
      if (isnan(last_temperature) || abs(state->temperature - last_temperature) > 0.05f)
      {
        dtostrf(state->temperature, 5, 2, buffer);
        mqttClient.publish(TOPIC_TEMP, buffer, true);
        last_temperature = state->temperature;
      }

      if (isnan(last_humidity) || abs(state->humidity - last_humidity) > 0.1f)
      {
        dtostrf(state->humidity, 5, 2, buffer);
        mqttClient.publish(TOPIC_HUM, buffer, true);
        last_humidity = state->humidity;
      }

      if (isnan(last_air_quality_pct) || abs(state->air_quality_pct - last_air_quality_pct) > 0.3f)
      {
        dtostrf(state->air_quality_pct, 5, 2, buffer);
        mqttClient.publish(TOPIC_GAS, buffer, true);
        last_air_quality_pct = state->air_quality_pct;
      }

      if (isnan(last_iaq_score) || abs(state->iaq_score - last_iaq_score) > 0.3f)
      {
        dtostrf(state->iaq_score, 5, 2, buffer);
        mqttClient.publish(TOPIC_IAQ_RAW, buffer, true);
        last_iaq_score = state->iaq_score;
      }

      String air_msg = air_index_to_msg(state->iaq_score);
      if (air_msg != last_air_msg)
      {
        mqttClient.publish(TOPIC_AIR_QUALITY, air_msg.c_str(), true);
        last_air_msg = air_msg;
      }

      if (xSemaphoreTake(state->fan_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
      {
        String fan_mode = state->is_manual_mode ? "manual" : "auto";
        if (fan_mode != last_fan_mode)
        {
          mqttClient.publish(TOPIC_FAN_MODE, fan_mode.c_str(), true);
          last_fan_mode = fan_mode;
        }

        if (state->fan_speed_target != last_fan_speed)
        {
          sprintf(buffer, "%d", state->fan_speed_target);
          mqttClient.publish(TOPIC_FAN_SPEED, buffer, true);
          last_fan_speed = state->fan_speed_target;
        }

        xSemaphoreGive(state->fan_mutex);
      }

      if (state->is_motion_detected != last_motion)
      {
        mqttClient.publish(TOPIC_PIR,
          state->is_motion_detected ? "detected" : "clear", true);
        last_motion = state->is_motion_detected;
      }
    }

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}
