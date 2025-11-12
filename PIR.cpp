#include "include/Globals.h"

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
      motion_detected = true;
    }

    if (lamp_on &&
      (xTaskGetTickCount() - last_motion_time > pdMS_TO_TICKS(5000))) {
      gpio_set_level((gpio_num_t)GPIO_LAMP, 0);
      lamp_on = false;
      motion_detected = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}