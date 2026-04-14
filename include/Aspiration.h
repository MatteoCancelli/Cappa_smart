#pragma once

#ifdef UNIT_TEST
  #include "test/Globals_test.h"
#else
  #include "include/Globals.h"
#endif

void set_fan_speed(SystemState* state, int speed_wanted);
void task_fan_control(void* state);
void task_mode_button(void* state);
