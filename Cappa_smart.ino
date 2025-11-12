#include "include/Globals.h"
#include "include/TaskMQTT.h"
#include "include/Display.h"
#include "include/BME680.h"

// void check_bme(){
//   if (!bme.begin(0x76)) {
//     Serial.println("Errore: BME688 non trovato!");
//     // display.clearDisplay();
//     // display.setCursor(0, 0);
//     // display.println("ERRORE BME688");
//     // display.setCursor(0, 16);
//     // display.println("Controlla I2C!");
//     // display.display();
//     bme_error_msg();
//     while (true);
//   }
//   Serial.println("BME688 trovato!");
//   // display.clearDisplay();
//   // display.setCursor(0, 0);
//   // display.println("BME688 OK");
//   // display.display();
//   bme_ok_msg();
//   delay(1000);
// }

// void task_bme(void *pvParameters){
//   for(;;) {
//     if (!bme.performReading()) {
//       Serial.println("Lettura fallita!");
//       display.clearDisplay();
//       display.setCursor(0, 0);
//       display.println("Lettura fallita!");
//       display.display();
//       vTaskDelay(pdMS_TO_TICKS(1000));
//     }
//     else {
//       temp = bme.temperature;
//       hum  = bme.humidity;
//       gas_index = gas_to_AirQualityIndex(bme.gas_resistance);
//     }
//     vTaskDelay(pdMS_TO_TICKS(2000)); 
//   }
// }

void task_pir(void *pvParameters) {
  TickType_t last_motion_time = 0;
  bool lamp_on = false;

  for (;;) {
    int pir_state = gpio_get_level((gpio_num_t)GPIO_PIR);

    if (pir_state == 1) {
      last_motion_time = xTaskGetTickCount();
      if (!lamp_on) {
          gpio_set_level((gpio_num_t)GPIO_LAMP, 1);
          lamp_on = true;
      }
    }

    if (lamp_on &&
      (xTaskGetTickCount() - last_motion_time > pdMS_TO_TICKS(5000))) {
      gpio_set_level((gpio_num_t)GPIO_LAMP, 0);
      lamp_on = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void task_logica_automatica_ventola(void *pvParameters) {
  unsigned long start_time = millis();
  bool sensor_warmup_skipped = false;

  Serial.println("Ventola SPENTA - attendo riscaldamento sensore...");

  for (;;) {
    // Skip warmup se il sensore è già caldo (gas_index >= 40 -> ventola spenta)
    if (!sensor_warmup_skipped) {
      if (gas_index >= 40) {
        Serial.println("Sensore già caldo, skip warmup!");
        sensor_warmup_skipped = true;
      } else if (millis() - start_time >= 270000) {
        Serial.println("Warmup completato (4.5 min)");
        sensor_warmup_skipped = true;
      } else {
        // Ancora in warmup
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
    }

    // Determina se la ventola deve essere accesa (in modalità automatica)
    bool condizione_accensione = (gas_index < 40 || hum > 75 || temp > 50);

    if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!mode_manual) {
        // Modalità automatica: imposta velocità fissa 150 se condizioni soddisfatte
        target_fan_speed = condizione_accensione ? 150 : 0;
      }
      xSemaphoreGive(fan_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void task_controllo_manuale_velocita(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (mode_manual) {
        int pot_value = analogRead(GPIO_POTENZIOMETRO);
        target_fan_speed = map(pot_value, 0, 4095, 0, 255);
      }
      xSemaphoreGive(fan_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void task_toggle_mode(void *pvParameters){
  int button_last_state = HIGH;

  for (;;) {
    int button_state = digitalRead(GPIO_BTN_FAN_CONTROLLER);

    if (button_state == LOW && button_last_state == HIGH) {
      if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        mode_manual = !mode_manual;
        digitalWrite(GPIO_LED_FAN_CONTROLLER, mode_manual ? HIGH : LOW);
        Serial.printf("Modalità: %s\n", mode_manual ? "MANUALE" : "AUTOMATICA");
        xSemaphoreGive(fan_mutex);
      }
      vTaskDelay(pdMS_TO_TICKS(200)); // debounce
    }

    button_last_state = button_state;
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

void task_attuatore_ventola(void *pvParameters) {
  for (;;) {
    uint8_t speed_to_set = 0;

    if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      speed_to_set = target_fan_speed;
      xSemaphoreGive(fan_mutex);
      Serial.printf("Fan speed: %d PWM, %.0f RPM\n", speed_to_set, read_fan_rpm());
    }

    ledcWrite(GPIO_FAN_PWM, speed_to_set);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void setup(){
  setup_GPIO();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  fan_mutex = xSemaphoreCreateMutex();
  
  check_display();
  check_bme();

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150);

  setup_wifi();

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(60);

  Serial.println("Inizio Task");

  // Task sensori e display
  xTaskCreate(task_bme, "BME688", 4096, NULL, 2, NULL);
  xTaskCreate(task_display, "Display", 4096, NULL, 1, NULL);
  xTaskCreate(task_pir, "PIR", 4096, NULL, 3, NULL);
  
  // Task ventola: separazione responsabilità
  xTaskCreate(task_logica_automatica_ventola, "LogicaAuto", 4096, NULL, 1, NULL);
  xTaskCreate(task_controllo_manuale_velocita, "PotenzioMetro", 4096, NULL, 1, NULL);
  xTaskCreate(task_toggle_mode, "ToggleMode", 4096, NULL, 1, NULL);
  xTaskCreate(task_attuatore_ventola, "AttuatoreVentola", 4096, NULL, 2, NULL);

  xTaskCreate(task_mqtt_publish, "MQTT_Publish", 4096, NULL, 2, NULL);
}

void loop(){
  // Vuoto: tutto gestito da FreeRTOS
}