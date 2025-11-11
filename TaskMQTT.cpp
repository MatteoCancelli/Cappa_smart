#include "include/Globals.h"
#include "include/TaskMQTT.h"


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connessione a ");
  Serial.println(WIFI_SSID);

  // Mostra su display
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connessione WiFi...");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connesso!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi OK!");
    display.setCursor(0, 16);
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.display();
    delay(2000);
  } else {
    Serial.println();
    Serial.println("ERRORE: WiFi non connesso!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi ERRORE!");
    display.setCursor(0, 16);
    display.println("Controllo cred.");
    display.display();
    delay(3000);
  }
}

void reconnect_mqtt() {
  if (!mqttClient.connected()) {
    Serial.print("Connessione MQTT...");
    
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println(" OK!");
    } else {
      Serial.print(" FALLITA, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void task_mqtt_publish(void *pvParameters) {
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(5000);

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnesso, riconnessione...");
      setup_wifi();
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    if (!mqttClient.connected()) {
      reconnect_mqtt();
    }

    // Loop MQTT per gestire messaggi
    mqttClient.loop();

    // Pubblica dati se connesso
    if (mqttClient.connected()) {
      char buffer[32];
      dtostrf(temp, 5, 2, buffer);
      mqttClient.publish(TOPIC_TEMP, buffer);
      dtostrf(hum, 5, 2, buffer);
      mqttClient.publish(TOPIC_HUM, buffer);
      dtostrf(gas_index, 5, 2, buffer);
      mqttClient.publish(TOPIC_GAS, buffer);
      String air_msg = air_index_to_msg(gas_index);
      mqttClient.publish(TOPIC_AIR_QUALITY, air_msg.c_str());
      uint8_t current_speed = 0;
      if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_speed = target_fan_speed;
        mqttClient.publish(TOPIC_FAN_MODE, mode_manual ? "manual" : "auto");
        xSemaphoreGive(fan_mutex);
      }
      sprintf(buffer, "%d", current_speed);
      mqttClient.publish(TOPIC_FAN_SPEED, buffer);
      float rpm = read_fan_rpm();
      if (rpm >= 0) {
        dtostrf(rpm, 6, 0, buffer);
        mqttClient.publish(TOPIC_FAN_RPM, buffer);
      }
      //mqttClient.publish(TOPIC_PIR, motion_detected ? "detected" : "clear");

      Serial.println("Dati MQTT pubblicati");
    }

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}