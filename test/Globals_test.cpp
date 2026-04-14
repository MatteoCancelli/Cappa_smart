#include "Globals_test.h"

uint8_t target_fan_speed = 0;
bool mode_manual = false;
SemaphoreHandle_t fan_mutex = nullptr;
float gas_index = 0;
float hum = 0;
float temp = 0;
FakeSerial Serial;