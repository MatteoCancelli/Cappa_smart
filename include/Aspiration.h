#pragma once

#ifdef UNIT_TEST
  #include "test/Globals_test.h"
#else
  #include "include/Globals.h"
#endif

void attuatore_ventola(int speed_wanted);
void task_toggle_mode(void *pvParameters);
void task_logica_ventola(void *pvParameters);