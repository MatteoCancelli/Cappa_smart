#ifdef UNIT_TEST
  #include "test/Globals_test.h"
#else
  #include <Arduino.h>
  #include "include/Globals.h"
  #include "esp32-hal-ledc.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
#endif
#include "include/Logic.h"
#include "include/HalInterface.h"
#include "include/Aspiration.h"

#define SOFT_START_STEP     5
#define SOFT_START_DELAY_MS 30

static void init_fan_mutex(SystemState* state)
{
  if (state->fan_mutex == NULL)
  {
    state->fan_mutex = xSemaphoreCreateMutex();
    if (state->fan_mutex == NULL)
      Serial.println("ERROR: failed to create fan_mutex");
  }
}

void set_fan_speed(SystemState* state, int speed_wanted)
{
  init_fan_mutex(state);
  if (xSemaphoreTake(state->fan_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
  {
    if (state->fan_speed_target != (uint8_t)speed_wanted)
    {
      int current = state->fan_speed_target;
      int step = (speed_wanted > current) ? SOFT_START_STEP : -SOFT_START_STEP;

      while (current != speed_wanted)
      {
        current += step;
        if ((step > 0 && current > speed_wanted) ||
            (step < 0 && current < speed_wanted))
          current = speed_wanted;

        hal.ledcWrite(GPIO_FAN_PWM, current);
        xSemaphoreGive(state->fan_mutex);
        vTaskDelay(pdMS_TO_TICKS(SOFT_START_DELAY_MS));
        if (xSemaphoreTake(state->fan_mutex, pdMS_TO_TICKS(200)) != pdTRUE)
          return;
      }

      state->fan_speed_target = (uint8_t)speed_wanted;
    }
    xSemaphoreGive(state->fan_mutex);
  }
}

static void update_fan_from_potentiometer(SystemState* state)
{
  int pot = hal.analogRead(GPIO_POTENZIOMETRO);
  int speed_wanted = map(pot, 0, 4095, 0, 255);
  set_fan_speed(state, speed_wanted);
}

void task_mode_button(void* pvParameters)
{
  SystemState* state = (SystemState*)pvParameters;
  int last_state = hal.digitalRead(GPIO_BTN_FAN_CONTROLLER);
  unsigned long last_debounce = 0;
  const unsigned long debounce_ms = 50;

  for (;;)
  {
    int current_btn = hal.digitalRead(GPIO_BTN_FAN_CONTROLLER);
    if (current_btn != last_state)
      last_debounce = millis();

    if ((millis() - last_debounce) > debounce_ms)
    {
      static int stable_state = HIGH;
      if (current_btn != stable_state)
      {
        if (stable_state == HIGH && current_btn == LOW)
        {
          state->is_manual_mode = !state->is_manual_mode;
          hal.digitalWrite(GPIO_LED_FAN_CONTROLLER, state->is_manual_mode ? HIGH : LOW);
          Serial.printf("Modalità: %s\n", state->is_manual_mode ? "MANUALE" : "AUTOMATICA");
        }
        stable_state = current_btn;
      }
    }
    last_state = current_btn;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void task_fan_control(void* pvParameters)
{
  SystemState* state = (SystemState*)pvParameters;
  init_fan_mutex(state);
  Serial.println("Logica ventola avviata");

  const TickType_t loop_delay = pdMS_TO_TICKS(200);

  for (;;)
  {
    if (state->is_manual_mode)
    {
      update_fan_from_potentiometer(state);
      vTaskDelay(loop_delay);
      continue;
    }

    int speed = calcola_velocita_automatica(
      state->iaq_score,
      state->humidity,
      state->temperature
    );
    set_fan_speed(state, speed);

    vTaskDelay(loop_delay);
  }
}
