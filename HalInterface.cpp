#ifdef UNIT_TEST

#include "test/Globals_test.h"
#include "include/HalInterface.h"

HalInterface hal = { nullptr, nullptr, nullptr, nullptr };

#else

#include "include/HalInterface.h"
#include "esp32-hal-ledc.h"
#include <Arduino.h>

static void hal_ledc_write(uint8_t pin, uint32_t duty) { ledcWrite(pin, duty); }
static int hal_digital_read(uint8_t pin) { return digitalRead(pin); }
static void hal_digital_write(uint8_t pin, uint8_t val) { digitalWrite(pin, val); }
static int hal_analog_read(uint8_t pin) { return analogRead(pin); }

HalInterface hal = {
  hal_ledc_write,
  hal_digital_read,
  hal_digital_write,
  hal_analog_read
};

#endif