#pragma once
#include <stdint.h>

struct HalInterface {
  void (*ledcWrite)(uint8_t pin, uint32_t duty);
  int  (*digitalRead)(uint8_t pin);
  void (*digitalWrite)(uint8_t pin, uint8_t val);
  int  (*analogRead)(uint8_t pin);
};

extern HalInterface hal;
