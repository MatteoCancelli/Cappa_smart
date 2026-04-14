#pragma once
#include <stdint.h>
#include "include/HalInterface.h"
#include "include/SystemState.h"

typedef int TickType_t;
#define pdTRUE  1
#define pdFALSE 0
#define pdMS_TO_TICKS(x) (x)
#define HIGH 1
#define LOW  0
#define GPIO_FAN_PWM             32
#define GPIO_BTN_FAN_CONTROLLER  18
#define GPIO_LED_FAN_CONTROLLER  19
#define GPIO_POTENZIOMETRO       36

inline void* xSemaphoreCreateMutex()               { return nullptr; }
inline int   xSemaphoreTake(SemaphoreHandle_t, int) { return pdTRUE; }
inline void  xSemaphoreGive(SemaphoreHandle_t)      {}
inline void  vTaskDelay(int)                        {}
inline unsigned long millis()                       { return 0; }
inline int   digitalRead(uint8_t)                   { return HIGH; }
inline int   map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct FakeSerial {
  void println(const char*) {}
  template<typename... Args>
  void printf(const char*, Args...) {}
};
extern FakeSerial Serial;
