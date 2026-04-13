#pragma once
#include "include/Globals.h"

void attuatore_ventola(int speed_wanted);
void task_toggle_mode(void *pvParameters);
void task_logica_ventola(void *pvParameters);