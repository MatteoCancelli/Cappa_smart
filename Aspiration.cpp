#include "include/Globals.h"


void IRAM_ATTR tachimetro_interrupt() {
  tach_pulse_count++;
}

float read_fan_rpm() {
  static unsigned long last_time = 0;
  static int last_pulses = 0;

  unsigned long now = millis();
  if (now - last_time >= 1000) {  
    int pulses = tach_pulse_count - last_pulses;
    last_pulses = tach_pulse_count;
    last_time = now;
    return (pulses / 2.0f) * 60.0f; // RPM
  }
  return -1;  // non ancora aggiornato
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
        //Serial.printf("Potenziometro: %d \n Target Fan speed: %d \n", pot_value, target_fan_speed);
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
