#pragma once
#include <stdint.h>

#ifndef UNIT_TEST
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#else
typedef void* SemaphoreHandle_t;
#endif

struct SystemState {
  float temperature;
  float humidity;
  float air_quality_pct;
  float iaq_score;
  uint8_t iaq_accuracy;
  bool is_motion_detected;
  bool is_manual_mode;
  uint8_t fan_speed_target;
  SemaphoreHandle_t fan_mutex;
};
