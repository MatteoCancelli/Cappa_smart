#include <Arduino.h>
#include "include/Globals.h"
#include "include/Logic.h"
#include "esp32-hal-ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define SOFT_START_STEP     5
#define SOFT_START_DELAY_MS 30

static void ensure_fan_mutex()
{
  if (fan_mutex == NULL)
  {
    fan_mutex = xSemaphoreCreateMutex();
    if (fan_mutex == NULL)
    {
      Serial.println("ERROR: failed to create fan_mutex");
    }
  }
}

void attuatore_ventola(int speed_wanted)
{
  ensure_fan_mutex();
  if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
  {
    if (target_fan_speed != speed_wanted)
    {
      int current = target_fan_speed;
      int step = (speed_wanted > current) ? SOFT_START_STEP : -SOFT_START_STEP;

      while (current != speed_wanted)
      {
        current += step;
        if ((step > 0 && current > speed_wanted) ||
            (step < 0 && current < speed_wanted))
          current = speed_wanted;

        ledcWrite(GPIO_FAN_PWM, current);
        xSemaphoreGive(fan_mutex);
        vTaskDelay(pdMS_TO_TICKS(SOFT_START_DELAY_MS));
        if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(200)) != pdTRUE)
          return;
      }

      target_fan_speed = speed_wanted;
    }
    xSemaphoreGive(fan_mutex);
  }
}

void task_toggle_mode(void *pvParameters)
{
  int last_state = digitalRead(GPIO_BTN_FAN_CONTROLLER);
  unsigned long last_debounce = 0;
  const unsigned long debounce_ms = 50;
  for (;;)
  {
    int st = digitalRead(GPIO_BTN_FAN_CONTROLLER);
    if (st != last_state)
    {
      last_debounce = millis();
    }
    if ((millis() - last_debounce) > debounce_ms)
    {
      static int stable_state = HIGH;
      if (st != stable_state)
      {
        if (stable_state == HIGH && st == LOW)
        {
          mode_manual = !mode_manual;
          digitalWrite(GPIO_LED_FAN_CONTROLLER, mode_manual ? HIGH : LOW);
          Serial.printf("Modalità: %s\n", mode_manual ? "MANUALE" : "AUTOMATICA");
        }
        stable_state = st;
      }
    }
    last_state = st;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void controllo_manuale_velocita()
{
  int pot = analogRead(GPIO_POTENZIOMETRO);
  int speed_wanted = map(pot, 0, 4095, 0, 255);
  attuatore_ventola(speed_wanted);
}

void task_logica_ventola(void *pvParameters)
{
  ensure_fan_mutex();
  Serial.println("Logica ventola avviata");

  const TickType_t loop_delay_normal = pdMS_TO_TICKS(200);

  for (;;)
  {
    if (mode_manual)
    {
      controllo_manuale_velocita();
      vTaskDelay(loop_delay_normal);
      continue;
    }

    int speed_wanted = calcola_velocita_automatica(gas_index, hum, temp);
    attuatore_ventola(speed_wanted);

    vTaskDelay(loop_delay_normal);
  }
}